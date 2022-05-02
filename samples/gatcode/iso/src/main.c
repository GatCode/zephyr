#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/iso.h>
#include <sys/byteorder.h>
#include <io_coder.h>
#include <hw_info.h>
#include <math.h>
#include <hal/nrf_rtc.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* D-Cube Defines */
/* ------------------------------------------------------ */
#define REMOTE true
#define SENDER_REMOTE remote_213
#define SENDER_LOCAL local_42
#define SENDER_START_DELAY_MS 25000

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1

/* ------------------------------------------------------ */
/* Defines Sender */
/* ------------------------------------------------------ */
#define SDU_INTERVAL_US 8000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 10 // 5ms-4s
#define RETRANSMISSION_NUMBER 2
#define MAXIMUM_SUBEVENTS 10 // MSE | 1-31

/* ------------------------------------------------------ */
/* Defines Receiver */
/* ------------------------------------------------------ */
#define PRESENTATION_DELAY_US 10000

/* ------------------------------------------------------ */
/* Sender Specific */
/* ------------------------------------------------------ */

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
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan bis_iso_chan_send;

uint32_t iso_send_count = 0;
uint8_t iso_data[sizeof(iso_send_count)] = { 0 };
struct net_buf *buf;

static void iso_sent_cb(struct bt_iso_chan *chan)
{
	int ret;
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

	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(iso_send_count++, iso_data);
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));
}

static struct bt_iso_chan_ops iso_ops_send = {
	.connected = iso_connected_send,
	.disconnected = iso_disconnected_send,
	.sent = iso_sent_cb,
};

static struct bt_iso_chan_io_qos iso_tx_qos_send = {
	.sdu = sizeof(uint32_t), /* bytes */
	.rtn = RETRANSMISSION_NUMBER,
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
	.interval = SDU_INTERVAL_US,
	.latency = TRANSPORT_LATENCY_MS,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_FRAMED,
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

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
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
	k_sem_give(&sem_per_sync);
}

static void term_cb_recv(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
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

static void biginfo_cb_recv(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(biginfo->addr, le_addr, sizeof(le_addr));

	printk("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
	       "num_bis %u, nse %u, interval 0x%04x (%u ms), "
	       "bn %u, pto %u, irc %u, max_pdu %u, "
	       "sdu_interval %u us, max_sdu %u, phy %s, "
	       "%s framing, %sencrypted\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, biginfo->sid,
	       biginfo->num_bis, biginfo->sub_evt_count,
	       biginfo->iso_interval,
	       (biginfo->iso_interval * 5 / 4),
	       biginfo->burst_number, biginfo->offset,
	       biginfo->rep_count, biginfo->max_pdu, biginfo->sdu_interval,
	       biginfo->max_sdu, phy2str(biginfo->phy),
	       biginfo->framing ? "with" : "without",
	       biginfo->encryption ? "" : "not ");

	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks_recv = {
	.synced = sync_cb_recv,
	.term = term_cb_recv,
	.biginfo = biginfo_cb_recv,
};

static uint64_t last_timestamp = 0;
static uint32_t packet_id = 0;

void my_timer_handler(struct k_timer *dummy)
{
	uint64_t current_timestamp = k_cyc_to_us_near32(k_cycle_get_32());
    printk("current ts: %llu - last ts: %llu - diff: %llu - diff in ms: %llu\n", current_timestamp, last_timestamp, current_timestamp - last_timestamp, (uint64_t)((current_timestamp - last_timestamp) / 1000.0));
	last_timestamp = current_timestamp;

	int err = write_8_bit(&io_encoder, packet_id % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

static void iso_recv_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	uint32_t count = 0;
	if (buf->len == sizeof(count)) {
		count = sys_get_le32(buf->data);
	}
	packet_id = count;

	uint32_t info_ts = info->ts;
	uint32_t curr = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
	uint32_t delta = curr - info_ts;

	k_timer_start(&my_timer, K_USEC(PRESENTATION_DELAY_US - delta), K_NO_WAIT);

	struct bt_iso_info iso_chan_info;
	bt_iso_chan_get_info(chan, &iso_chan_info);
	printk("sync_delay: %u, pto: %u, pto: %u\n", iso_chan_info.broadcaster.sync_delay, iso_chan_info.broadcaster.pto, iso_chan_info.sync_receiver.pto);
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
	.mse = MAXIMUM_SUBEVENTS,//BT_ISO_SYNC_MSE_MAX,
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

	if(id == (REMOTE == true ? SENDER_REMOTE : SENDER_LOCAL)) { // sender
		struct bt_le_ext_adv *adv;
		struct bt_iso_big *big;
		int err;

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

		k_sleep(K_MSEC(SENDER_START_DELAY_MS));

		int ret;
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
		buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		sys_put_le32(iso_send_count++, iso_data);
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));

		printk("Sending value %u\n", iso_send_count);
		err = write_8_bit(&io_encoder, iso_send_count % 256);
		if(err) {
			printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
		}
	} else { // receiver
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
