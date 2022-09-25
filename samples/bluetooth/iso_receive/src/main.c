#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <io_coder.h>

/* ------------------------------------------------------ */
/* ADAPTATION ALGORITHM SELECTION TOGGLE (only select 1)  */
/* ------------------------------------------------------ */
// #define NO_ALGORITHM // disables ACL connection
// NOTE: only activate this if broadcaster also uses this!

/* ------------------------------------------------------ */
/* Basic Definitions */
/* ------------------------------------------------------ */
#define SDU_INTERVAL_US 20000
#define DATA_SIZE_BYTE 50

/* ------------------------------------------------------ */
/* Dynamic Adaptation Algorithm Definitions */
/* ------------------------------------------------------ */
#define PRR_TREND_THRESHOLD 3 // last x packets lost

/* ------------------------------------------------------ */
/* Global Controller Overwrites */
/* ------------------------------------------------------ */
extern int8_t txp_global_overwrite;

/* ------------------------------------------------------ */
/* Important Globals */
/* ------------------------------------------------------ */
static double prr = 0.0;
static uint32_t seq_num = 0;
static uint32_t prev_seq_num = 0;
static uint8_t trend_looking_bad = 0;
static bool just_received_error_iso_packet = false;

/* ------------------------------------------------------ */
/* PRR Calculation */
/* ------------------------------------------------------ */
#define PRR_MAVG_WINDOW_SIZE 100

/* ------------------------------------------------------ */
/* Defines Threads (main thread = prio 0) */
/* ------------------------------------------------------ */
#define STACKSIZE 1024
#define ACL_PRIORITY 10

/* ------------------------------------------------------ */
/* Logging & IO Coder*/
/* ------------------------------------------------------ */
LOG_MODULE_REGISTER(ISOLogger, CONFIG_LOG_DEFAULT_LEVEL);
static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Windowed Moving Average */
/* ------------------------------------------------------ */
// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da

typedef struct {
	uint64_t *maverage_values;
	uint64_t maverage_current_position;
	uint64_t maverage_current_sum;
	uint64_t maverage_sample_length;
	uint64_t last_packets[PRR_TREND_THRESHOLD];
	uint64_t last_packets_position;
} MAVG;

void init_mavg(MAVG *mavg, uint64_t *maverage_values, uint8_t window_size)
{
	mavg->maverage_values = maverage_values;
	mavg->maverage_current_position = 0;
	mavg->maverage_current_sum = 0;
	mavg->maverage_sample_length = window_size;
}

double RollingMAvgDouble(MAVG *mavg, uint8_t newValue)
{
	((uint64_t*)mavg->last_packets)[mavg->last_packets_position] = newValue;
	mavg->last_packets_position++;
	if (mavg->last_packets_position >= PRR_TREND_THRESHOLD) { // Don't go beyond the size of the array...
		mavg->last_packets_position = 0;
	}

	mavg->maverage_current_sum = mavg->maverage_current_sum - ((uint64_t*)mavg->maverage_values)[mavg->maverage_current_position] + newValue;
	((uint64_t*)mavg->maverage_values)[mavg->maverage_current_position] = newValue;
	mavg->maverage_current_position++;
	if (mavg->maverage_current_position >= mavg->maverage_sample_length) { // Don't go beyond the size of the array...
		mavg->maverage_current_position = 0;
	}
	return (double)mavg->maverage_current_sum * 100 / (double)mavg->maverage_sample_length;
}

bool MAvgTrendLookingBad(MAVG *mavg)
{
	for (uint8_t i = 0; i < PRR_TREND_THRESHOLD; i++) {
		if (((uint64_t*)mavg->last_packets)[i] == 1) {
			return false;
		}
	}
	return true;
}

static MAVG prr_mavg;

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
static struct k_thread thread_acl_data;
static struct bt_conn *acl_conn;

K_THREAD_STACK_DEFINE(thread_ind_stack_area, STACKSIZE);
static struct k_thread thread_ind_data;

#define DEVICE_NAME_ACL "nRF52840"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define BT_LE_ADV_FAST_CONN \
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2, \
		BT_GAP_ADV_FAST_INT_MIN_2, NULL)

static struct bt_gatt_indicate_params ind_params;

static K_SEM_DEFINE(acl_connected, 0, 1);
static K_SEM_DEFINE(sem_indicate, 0, 1);

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

void ind_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while(1) {
		/* Fetch ACL connection handle */
		uint16_t acl_handle = 0;
		int err = bt_hci_get_conn_handle(acl_conn, &acl_handle);
		if (err) {
			printk("Failed to fetch the ACL connection handle (err %d)\n", err);
			return;
		}

		/* Take it easy */
		err = k_sem_take(&sem_indicate, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}

		uint8_t tmp = (uint8_t)prr;
		if (trend_looking_bad) {
			tmp |= 1UL << 7;
		}

		/* Create Indication */
		static uint8_t ind_data[1];
		ind_data[0] = tmp;
		ind_params.attr = &hts_svc.attrs[2];
		ind_params.data = &ind_data;
		ind_params.len = sizeof(uint8_t);
		(void)bt_gatt_indicate(NULL, &ind_params);
	}
}

void acl_indicate()
{
	k_sem_give(&sem_indicate);
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
		
		prr = RollingMAvgDouble(&prr_mavg, 1);

		if (seq_num - 1 != prev_seq_num) {
			printk("LOST - ");
		}

		trend_looking_bad = MAvgTrendLookingBad(&prr_mavg);
		txp_global_overwrite = (int8_t)buf->data[4];
		if (trend_looking_bad) {
			txp_global_overwrite = MIN(txp_global_overwrite + 1, 8);
		}

		prev_seq_num = seq_num;
		just_received_error_iso_packet = false;
	} else if(info->flags == (BT_ISO_FLAGS_TS | BT_ISO_FLAGS_ERROR)) { // possibly valid ISO packet
		/* Hack since the scheduling has problems (off by 1) */
		if (!just_received_error_iso_packet) {
			prr = RollingMAvgDouble(&prr_mavg, 1); // good one
		} else {
			prr = RollingMAvgDouble(&prr_mavg, 0); // yeah, this one is a fail
		}
		trend_looking_bad = MAvgTrendLookingBad(&prr_mavg);
		just_received_error_iso_packet = true;
	}

	if (trend_looking_bad) {
		txp_global_overwrite = MIN(txp_global_overwrite + 1, 8);
	}

	int err = write_8_bit(&io_encoder, (uint8_t)prr);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
	
	printk("Last seq_num: %u, PRR: %.02f%%, MAvgTrendLookingBad: %u\n", seq_num, prr, trend_looking_bad);
	acl_indicate();
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
	.sync_timeout = BT_ISO_SYNC_TIMEOUT_MAX,
};

void main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout;
	int err;

	/* Initialize the moving average filter */
	static uint64_t prr_mavg_values[PRR_MAVG_WINDOW_SIZE] = {0};
	init_mavg(&prr_mavg, prr_mavg_values, PRR_MAVG_WINDOW_SIZE);

	/* Initialize IO Coder */
	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Start ACL */
	#ifndef NO_ALGORITHM
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

	/* Start Indication Thread */
	k_thread_create(&thread_ind_data, thread_ind_stack_area,
			K_THREAD_STACK_SIZEOF(thread_ind_stack_area),
			ind_thread, NULL, NULL, NULL,
			ACL_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_ind_data, "ind_thread");
	k_thread_start(&thread_ind_data);
	#endif

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
