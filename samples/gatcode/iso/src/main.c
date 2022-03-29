#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/iso.h>
#include <sys/byteorder.h>
#include <io_coder.h>
#include <hw_info.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Sender Specific */
/* ------------------------------------------------------ */

#define BIG_TERMINATE_TIMEOUT 60 /* seconds */

#define BIS_ISO_CHAN_COUNT 1
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

static void iso_connected_send(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected_send(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan_ops iso_ops_send = {
	.connected	= iso_connected_send,
	.disconnected	= iso_disconnected_send,
};

static struct bt_iso_chan_io_qos iso_tx_qos_send = {
	.sdu = sizeof(uint32_t), /* bytes */
	.rtn = 2,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos_send = {
	.tx = &iso_tx_qos_send,
};

static struct bt_iso_chan bis_iso_chan_send = {
	.ops = &iso_ops_send,
	.qos = &bis_iso_qos_send,
};

static struct bt_iso_chan *bis_send[BIS_ISO_CHAN_COUNT] = { &bis_iso_chan_send };

static struct bt_iso_big_create_param big_create_param_send = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = bis_send,
	.interval = 10000, /* in microseconds */
	.latency = 10, /* milliseconds */
	.packing = 0, /* 0 - sequential, 1 - interleaved */
	.framing = 0, /* 0 - unframed, 1 - framed */
};

/* ------------------------------------------------------ */
/* Receiver Specific */
/* ------------------------------------------------------ */

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
					   BT_LE_SCAN_OPT_NONE, \
					   BT_GAP_SCAN_FAST_INTERVAL, \
					   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

static bool         per_adv_found;
static bool         per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint16_t     per_interval_ms;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, 1);
static K_SEM_DEFINE(sem_big_sync_lost, 0, 1);

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	// printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i %s "
	//        "C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
	//        "Interval: 0x%04x (%u ms), SID: %u\n",
	//        le_addr, info->adv_type, info->tx_power, info->rssi, name,
	//        (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	//        phy2str(info->primary_phy), phy2str(info->secondary_phy),
	//        info->interval, BT_CONN_INTERVAL_TO_MS(info->interval), info->sid);

	if (!per_adv_found && info->interval) {
		per_adv_found = true;

		per_sid = info->sid;
		per_interval_ms = BT_CONN_INTERVAL_TO_MS(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);

		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb_recv(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	//        "Interval 0x%04x (%u ms), PHY %s\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr,
	//        info->interval, info->interval * 5 / 4, phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

static void term_cb_recv(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb_recv(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char data_str[129];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	//        "RSSI %i, CTE %u, data length %u, data: %s\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	//        info->rssi, info->cte_type, buf->len, data_str);
}

static void biginfo_cb_recv(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(biginfo->addr, le_addr, sizeof(le_addr));

	// printk("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
	//        "num_bis %u, nse %u, interval 0x%04x (%u ms), "
	//        "bn %u, pto %u, irc %u, max_pdu %u, "
	//        "sdu_interval %u us, max_sdu %u, phy %s, "
	//        "%s framing, %sencrypted\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr, biginfo->sid,
	//        biginfo->num_bis, biginfo->sub_evt_count,
	//        biginfo->iso_interval,
	//        (biginfo->iso_interval * 5 / 4),
	//        biginfo->burst_number, biginfo->offset,
	//        biginfo->rep_count, biginfo->max_pdu, biginfo->sdu_interval,
	//        biginfo->max_sdu, phy2str(biginfo->phy),
	//        biginfo->framing ? "with" : "without",
	//        biginfo->encryption ? "" : "not ");


	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks_recv = {
	.synced = sync_cb_recv,
	.term = term_cb_recv,
	.recv = recv_cb_recv,
	.biginfo = biginfo_cb_recv,
};

#define BIS_ISO_CHAN_COUNT 1

static void iso_recv_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	char data_str[128];
	size_t str_len;
	uint32_t count = 0; /* only valid if the data is a counter */

	if (buf->len == sizeof(count)) {
		count = sys_get_le32(buf->data);
	}

	str_len = bin2hex(buf->data, buf->len, data_str, sizeof(data_str));
	printk("Incoming data channel %p len %u: %s (counter value %u)\n",
	       chan, buf->len, data_str, count);

	int err = write_8_bit(&io_encoder, count % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}

static void iso_connected_recv(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected_recv(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops_recv = {
	.recv		= iso_recv_recv,
	.connected	= iso_connected_recv,
	.disconnected	= iso_disconnected_recv,
};

static struct bt_iso_chan_io_qos iso_rx_qos_recv;

static struct bt_iso_chan_qos bis_iso_qos_recv = {
	.rx = &iso_rx_qos_recv,
};

static struct bt_iso_chan bis_iso_chan_recv = {
	.ops = &iso_ops_recv,
	.qos = &bis_iso_qos_recv,
};

static struct bt_iso_chan *bis_recv[BIS_ISO_CHAN_COUNT] = { &bis_iso_chan_recv };

static struct bt_iso_big_sync_param big_sync_param_recv = {
	.bis_channels = bis_recv,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT) << 1),
	.mse = 1,
	.sync_timeout = 100, /* in 10 ms units */
};

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	uint64_t id;
	err = get_device_id(&id);
	if(err) {
		printk("Error getting id (err %d)\n", err);
	}

	if(id == remote_116 /*local_42*/) { // sender
		struct bt_le_ext_adv *adv;
		struct bt_iso_big *big;
		int err;
		uint32_t iso_send_count = 0;
		uint8_t iso_data[sizeof(iso_send_count)] = { 0 };
		struct net_buf *buf;

		printk("Starting ISO Broadcast Demo\n");

		/* Initialize the Bluetooth Subsystem */
		err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
			return;
		}

		/* Create a non-connectable non-scannable advertising set */
		err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
		if (err) {
			printk("Failed to create advertising set (err %d)\n", err);
			return;
		}

		/* Set periodic advertising parameters */
		err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
		if (err) {
			printk("Failed to set periodic advertising parameters"
				" (err %d)\n", err);
			return;
		}

		/* Enable Periodic Advertising */
		err = bt_le_per_adv_start(adv);
		if (err) {
			printk("Failed to enable periodic advertising (err %d)\n", err);
			return;
		}

		/* Start extended advertising */
		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err) {
			printk("Failed to start extended advertising (err %d)\n", err);
			return;
		}

		/* Create BIG */
		err = bt_iso_big_create(adv, &big_create_param_send, &big);
		if (err) {
			printk("Failed to create BIG (err %d)\n", err);
			return;
		}

		printk("Waiting for BIG complete...");
		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("done.\n");

		while (true) {
			static uint8_t timeout = BIG_TERMINATE_TIMEOUT;
			int ret;

			k_sleep(K_SECONDS(1));

			buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
			net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
			sys_put_le32(++iso_send_count, iso_data);
			net_buf_add_mem(buf, iso_data, sizeof(iso_data));
			ret = bt_iso_chan_send(&bis_iso_chan_send, buf);
			if (ret < 0) {
				printk("Unable to broadcast data: %d", ret);
				net_buf_unref(buf);
				return;
			}
			printk("Sending value %u\n", iso_send_count);
			int err = write_8_bit(&io_encoder, iso_send_count % 256);
			if(err) {
				printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
			}

			timeout--;
			if (!timeout) {
				timeout = BIG_TERMINATE_TIMEOUT;

				printk("BIG Terminate...");
				err = bt_iso_big_terminate(big);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				printk("done.\n");

				printk("Waiting for BIG terminate complete...");
				err = k_sem_take(&sem_big_term, K_FOREVER);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				printk("done.\n");

				printk("Create BIG...");
				err = bt_iso_big_create(adv, &big_create_param_send, &big);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				printk("done.\n");

				printk("Waiting for BIG complete...");
				err = k_sem_take(&sem_big_cmplt, K_FOREVER);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				printk("done.\n");
			}
		}
	} else if(id == remote_117 /*local_56*/) { // receiver
		struct bt_le_per_adv_sync_param sync_create_param;
		struct bt_le_per_adv_sync *sync;
		struct bt_iso_big *big;
		uint32_t sem_timeout;
		int err;

		printk("Starting Synchronized Receiver Demo\n");

		/* Initialize the Bluetooth Subsystem */
		err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
			return;
		}

		printk("Scan callbacks register...");
		bt_le_scan_cb_register(&scan_callbacks);
		printk("success.\n");

		printk("Periodic Advertising callbacks register...");
		bt_le_per_adv_sync_cb_register(&sync_callbacks_recv);
		printk("Success.\n");

		do {
			per_adv_lost = false;

			printk("Start scanning...");
			err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("success.\n");

			printk("Waiting for periodic advertising...\n");
			per_adv_found = false;
			err = k_sem_take(&sem_per_adv, K_FOREVER);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("Found periodic advertising.\n");

			printk("Stop scanning...");
			err = bt_le_scan_stop();
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("success.\n");

			printk("Creating Periodic Advertising Sync...");
			bt_addr_le_copy(&sync_create_param.addr, &per_addr);
			sync_create_param.options = 0;
			sync_create_param.sid = per_sid;
			sync_create_param.skip = 0;
			sync_create_param.timeout = (per_interval_ms * PA_RETRY_COUNT) / 10;
			sem_timeout = per_interval_ms * PA_RETRY_COUNT;
			err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("success.\n");

			printk("Waiting for periodic sync...\n");
			err = k_sem_take(&sem_per_sync, K_MSEC(sem_timeout));
			if (err) {
				printk("failed (err %d)\n", err);

				printk("Deleting Periodic Advertising Sync...");
				err = bt_le_per_adv_sync_delete(sync);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				continue;
			}
			printk("Periodic sync established.\n");

			printk("Waiting for BIG info...\n");
			err = k_sem_take(&sem_per_big_info, K_MSEC(sem_timeout));
			if (err) {
				printk("failed (err %d)\n", err);

				if (per_adv_lost) {
					continue;
				}

				printk("Deleting Periodic Advertising Sync...");
				err = bt_le_per_adv_sync_delete(sync);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				continue;
			}
			printk("Periodic sync established.\n");

	big_sync_create:
			printk("Create BIG Sync...\n");
			err = bt_iso_big_sync(sync, &big_sync_param_recv, &big);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("success.\n");

			printk("Waiting for BIG sync...\n");
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				printk("failed (err %d)\n", err);

				printk("BIG Sync Terminate...");
				err = bt_iso_big_terminate(big);
				if (err) {
					printk("failed (err %d)\n", err);
					return;
				}
				printk("done.\n");

				goto per_sync_lost_check;
			}
			printk("BIG sync established.\n");

			printk("Waiting for BIG sync lost...\n");
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("BIG sync lost.\n");

	per_sync_lost_check:
			printk("Check for periodic sync lost...\n");
			err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
			if (err) {
				/* Periodic Sync active, go back to creating BIG Sync */
				goto big_sync_create;
			}
			printk("Periodic sync lost.\n");
		} while (true);
	}
}
