#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* ------------------------------------------------------ */
/* Basic Definitions */
/* ------------------------------------------------------ */
#define SDU_INTERVAL_US 20000

/* ------------------------------------------------------ */
/* Global Controller Overwrites */
/* ------------------------------------------------------ */
extern int8_t txp_global_overwrite;

/* ------------------------------------------------------ */
/* Important Globals */
/* ------------------------------------------------------ */
static uint32_t seq_num = 0;
static uint32_t prev_seq_num = 0;
static bool buffer_timer_started = false;

/* ------------------------------------------------------ */
/* Defines Threads (main thread = prio 0) */
/* ------------------------------------------------------ */
#define STACKSIZE 1024
#define ACL_PRIORITY 10

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
static struct k_thread thread_acl_data;
static struct bt_conn *acl_conn;

#define DEVICE_NAME_ACL "nRF52840"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define BT_LE_ADV_FAST_CONN \
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2, \
		BT_GAP_ADV_FAST_INT_MIN_2, NULL)

static struct bt_gatt_indicate_params ind_params;

static K_SEM_DEFINE(acl_connected, 0, 1);

extern void buffer_timer_handler(struct k_timer *timer_id);
K_TIMER_DEFINE(buffer_timer, buffer_timer_handler, NULL);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_ACL, DEVICE_NAME_ACL_LEN),
};

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	// currently unused
}

BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void acl_connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("ACL Connection failed (err 0x%02x)\n", err);
	} else {
		acl_conn = conn;
		printk("ACL Connected\n");
		k_sem_give(&acl_connected);

		/* Start buffer timer */
		if (!buffer_timer_started) {
			k_timer_start(&buffer_timer, K_NO_WAIT, K_USEC(SDU_INTERVAL_US));
			buffer_timer_started = true;
		}
	}
}

static void acl_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("ACL Disconnected (reason 0x%02x)\n", reason);
	k_thread_start(&thread_acl_data);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = acl_connected_cb,
	.disconnected = acl_disconnected_cb,
};

void work_adv_start(struct k_work *item)
{
	int ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad, ARRAY_SIZE(ad), NULL, 0);

	if (ret) {
		printk("ACL advertising failed to start (ret %d)\n", ret);
	}
}
K_WORK_DEFINE(adv_work, work_adv_start);

void acl_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	txp_global_overwrite = 8;
	bt_conn_cb_register(&conn_callbacks);
	k_work_submit(&adv_work);
}

static int8_t per_adv_rssi = 0;
static uint8_t acl_rssi = 0;

void indicate_work_handler(struct k_work *item)
{
	/* Fetch ACL connection handle */
	uint16_t acl_handle = 0;
	int err = bt_hci_get_conn_handle(acl_conn, &acl_handle);
	if (err) {
		printk("Failed to fetch the ACL connection handle (err %d)\n", err);
		return;
	}

	// /* Set TXP */
	// err = ble_hci_vsc_set_conn_tx_pwr_index(acl_handle, per_adv_txp_idx);
	// if (err) {
	// 	printk("Failed to set tx power (err %d)\n", err);
	// 	return;
	// }

	// /* Determine Opcode */
	// uint8_t new_opc = 0;
	// uint32_t free_slots = ring_buf_space_get(&PacketBuffer) / DATA_SIZE_BYTE;
	// if (PACKET_BUFFER_SIZE - free_slots > PACKET_BUFFER_OCCUPIED_THRESHOLD_HIGH) {
	// 	new_opc = 10; // ready for downshift
	// } else if (PACKET_BUFFER_SIZE - free_slots > PACKET_BUFFER_OCCUPIED_THRESHOLD_LOW) {
	// 	new_opc = 11; // normal buffering speed - ready for next adjustment
	// } else if (PACKET_BUFFER_SIZE - free_slots < 20) {
	// 	new_opc = 12; // buffer is critical - let's suggest refill
	// } else {
	// 	new_opc = 13; // ignore
	// }

	/* Prepare Indication */
	// #define CONST_IND_DATA_SIZE 3
	// uint8_t ind_data_size = CONST_IND_DATA_SIZE;
	// static uint8_t ind_data[CONST_IND_DATA_SIZE];

	// if (last_indicated_opcode == new_opc) { // don't indicate the same opcode twice
	// 	ind_data_size = ind_data_size - 1;
	// } else {
	// 	last_indicated_opcode = new_opc;
	// }

	static uint8_t ind_data[1];

	acl_rssi = (uint8_t)-per_adv_rssi; // convert to uint8_t

	/* Create Indication */
	ind_data[0] = acl_rssi;
	// ind_data[1] = (uint8_t)prr;
	// ind_data[2] = new_opc;
	ind_params.attr = &hts_svc.attrs[2];
	ind_params.data = &ind_data;
	ind_params.len = sizeof(uint8_t);// * ind_data_size;
	(void)bt_gatt_indicate(NULL, &ind_params);
}
K_WORK_DEFINE(indicate_work, indicate_work_handler);

void acl_indicate()
{
	k_work_submit(&indicate_work);
}

void buffer_timer_handler(struct k_timer *timer_id)
{
	// uint32_t free_slots = ring_buf_space_get(&PacketBuffer);

	// // initial buffer fill as stream establishment
	// if (PACKET_BUFFER_SIZE - (free_slots / DATA_SIZE_BYTE) > PACKET_BUFFER_OCCUPIED_THRESHOLD_HIGH && iso_just_established) {
	// 	iso_just_established = false;
	// } 
	
	// if (iso_just_established) {
	// 	return;
	// }
	
	// if (!ring_buf_is_empty(&PacketBuffer)) {
	// 	uint8_t data[DATA_SIZE_BYTE];
	// 	ring_buf_get(&PacketBuffer, (uint8_t*)&data, DATA_SIZE_BYTE);

	// 	uint8_t count_arr[4];
	// 	for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
	// 		if(i < 4) {
	// 			count_arr[i] = data[i];
	// 		}
	// 	}
	// 	seq_num = sys_get_le32(count_arr);

	// 	gpio_pin_toggle_dt(&led2);

	// 	if (seq_num - 1 != prev_seq_num && prev_seq_num < seq_num) {
	// 		for (uint8_t i = 0; i < seq_num - prev_seq_num; i++) {
	// 			prr = RollingmAvg(0);
	// 		}
	// 	} else {
	// 		prr = RollingmAvg(1);
	// 	}

	// 	// acl_indicate(prr);

	// 	if (seq_num - 1 != prev_seq_num) {
	// 		printk("LOST - ");
	// 	}

	// 	prev_seq_num = seq_num;
	// } else {
	// 	prr = RollingmAvg(0);
	// }

	acl_indicate(0);

	// printk("Buffer occupied: %u out of %u - prr: %.02f%% - seq_num: %u - rssi: %d, tx_power index: %d, per_adv_txp: %d\n", PACKET_BUFFER_SIZE - free_slots / DATA_SIZE_BYTE, PACKET_BUFFER_SIZE, prr, seq_num, per_adv_rssi, per_adv_txp_idx, per_adv_txp);
}

/* ------------------------------------------------------ */
/* ISO Stuff */
/* ------------------------------------------------------ */
#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN 30
#define BIS_ISO_CHAN_COUNT 1

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

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
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

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{
	k_sem_give(&sem_per_big_info);
}

static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	per_adv_rssi = info->rssi;
} 

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
	.recv = recv_cb,
};

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	if(info->flags == (BT_ISO_FLAGS_VALID | BT_ISO_FLAGS_TS)) { // valid ISO packet
		uint8_t count_arr[4];
		for(uint8_t i = 0; i < 4; i++) {
			count_arr[i] = buf->data[i];
		}
		seq_num = sys_get_le32(count_arr);
		txp_global_overwrite = (int8_t)buf->data[4];

		// str_len = bin2hex(buf->data, buf->len, data_str, sizeof(data_str));
		printk("Incoming data channel %p flags 0x%x seq_num %u ts %u len %u: "
			" counter value %u txp %d", chan, info->flags, info->seq_num,
			info->ts, buf->len, seq_num, txp_global_overwrite);

		if (seq_num != prev_seq_num + 1 && seq_num != prev_seq_num) {
			printk(" - LOST\n");
		} else {
			printk("\n");
		}

		prev_seq_num = seq_num;
	}
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

static struct bt_iso_chan_io_qos iso_rx_qos;

static struct bt_iso_chan_qos bis_iso_qos = {
	.rx = &iso_rx_qos
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos
};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT] = {
	&bis_iso_chan,
};

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT) << 1),
	.mse = 31,
	.sync_timeout = 100, /* in 10 ms units */ // BT_ISO_SYNC_TIMEOUT_MAX
};

void main(void)
{
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

	/* Start ACL */
	k_thread_create(&thread_acl_data, thread_acl_stack_area,
			K_THREAD_STACK_SIZEOF(thread_acl_stack_area),
			acl_thread, NULL, NULL, NULL,
			ACL_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_acl_data, "acl_thread");
	k_thread_start(&thread_acl_data);

	err = k_sem_take(&acl_connected, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
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
