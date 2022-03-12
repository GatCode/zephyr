/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/iso.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>

#include <throughput_explorer.h>

#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button0_cb_data;

static struct explorer_config cfg = {0};
static struct explorer_payload payload = {0};
static struct explorer_statistic statistic = {0};

void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	reset_config(&cfg);
	reset_statistic(&statistic);
}

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

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

// static bool data_cb(struct bt_data *data, void *user_data)
// {
// 	char *name = user_data;
// 	uint8_t len;

// 	switch (data->type) {
// 	case BT_DATA_NAME_SHORTENED:
// 	case BT_DATA_NAME_COMPLETE:
// 		len = MIN(data->data_len, NAME_LEN - 1);
// 		memcpy(name, data->data, len);
// 		name[len] = '\0';
// 		return false;
// 	default:
// 		return true;
// 	}
// }


static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	// bt_data_parse(buf, data_cb, name);

	// bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
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

static void sync_cb(struct bt_le_per_adv_sync *sync,
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

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
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

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
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

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb,
};

#define BIS_ISO_CHAN_COUNT 1

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	// printk("HELLO\n");

	memcpy(&payload, buf->data, sizeof(struct explorer_payload));
	// uint64_t id = 0;
	// u8_arr_to_u64(payload.id, &id);
	// printk("ID: %llu\n", id);
	update_statistic(&statistic, &cfg, &payload);
	print_statistic(&statistic, 25);
	
	// printk("Received: %llu/%llu (%.2f%%) - Latency %.2f ms\n",
	// 		statistic.recv, statistic.recv + statistic.lost,
	// 		(float)statistic.recv * 100 / (statistic.recv + statistic.lost),
    //         k_ticks_to_us_floor64(statistic.latency) / 1000.0);

	// char data_str[128];
	// size_t str_len;
	// uint32_t count = 0; /* only valid if the data is a counter */

	// if (buf->len == sizeof(count)) {
	// 	count = sys_get_le32(buf->data);
	// }

	// str_len = bin2hex(buf->data, buf->len, data_str, sizeof(data_str));
	// printk("Incoming data channel %p len %u: %s (counter value %u)\n",
	//        chan, buf->len, data_str, count);
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos = {
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.rx = &iso_rx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos,
};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT] = { &bis_iso_chan };

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT) << 1),
	.mse = 1,
	.sync_timeout = 100, /* in 10 ms units */
};

void main(void)
{
	int err;

	/* Initialize Sync Button */
	if (!device_is_ready(button0.port)) {
		printk("Error: button device is not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	gpio_init_callback(&button0_cb_data, sync_button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb_data);

	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout;
	


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
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");


	do {
		per_adv_lost = false;

		struct bt_le_scan_param param = {
			.type = BT_LE_SCAN_TYPE_PASSIVE,
			.options = /*BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M | */BT_LE_SCAN_OPT_FILTER_DUPLICATE,
			.interval = 0x0080  /* 80 ms */,
			.window = 0x0080 /* 80ms */,
			.timeout = 0,
			.interval_coded = 0,
			.window_coded = 0,
		};

		printk("Start scanning...");
		err = bt_le_scan_start(&param, NULL);
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
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
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
