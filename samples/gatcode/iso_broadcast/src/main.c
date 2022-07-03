#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
#include <ble_hci_vsc.h>
#include <io_coder.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define SDU_INTERVAL_US 10000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 10 // 5ms-4s
#define RETRANSMISSION_NUMBER 2
#define BROADCAST_ENQUEUE_COUNT 2U // Guarantee always data to send
#define STACKSIZE 1024
#define ACL_PRIORITY 9
#define ACL_DATA_LEN 4
#define ACL_SCAN_INTERVAL 0x0050 // (N * 0.625 ms) - 50ms - sample 2x
#define RANGE_PRIORITY 5
#define RANGE_CALC_INTERVAL_MS 100

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static float pdr = 0.0;
static int8_t rssi = 0;
K_MUTEX_DEFINE(linkback_lock);
static struct ble_hci_vs_tx_pwr_setting tx_power_setting;

/* ------------------------------------------------------ */
/* ACL (beacon) */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
static struct k_thread thread_acl_data;

static bool data_cb(struct bt_data *data, void *user_data)
{
	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len == ACL_DATA_LEN) {
		memcpy(user_data, data->data, ACL_DATA_LEN);
		return false;
	}
	return true;
}

static void acl_scan_cb(const bt_addr_le_t *addr, int8_t rssi_, uint8_t adv_type, struct net_buf_simple *buf)
{
	uint8_t acl_data[ACL_DATA_LEN];
	(void)memset(acl_data, 0, sizeof(acl_data));
	bt_data_parse(buf, data_cb, acl_data);

	if (acl_data[0] != 0) { // received ACL data
		uint8_t d0 = acl_data[0] >> 4;
		uint8_t d1 = acl_data[0] & d0;
		uint8_t d2 = acl_data[1] >> 4;
		uint8_t d3 = acl_data[1] & d2;

		k_mutex_lock(&linkback_lock, K_FOREVER);
		if (d0 == 0xF) {
			pdr = 100.0;
		} else {
			pdr = d0 * 10 + d1 + (float)d2 / 10.0 + (float)d3 / 100.0;
		}
		rssi = rssi_;
		k_mutex_unlock(&linkback_lock);

		// printk("PDR: %.2f%% - RSSI: %d\n", pdr, rssi);
	}
}

void acl_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int err;

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = ACL_SCAN_INTERVAL,
		.window     = ACL_SCAN_INTERVAL,
	};

	err = bt_le_scan_start(&scan_param, acl_scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}
}

/* ------------------------------------------------------ */
/* Range Extension (Model) */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_range_stack_area, STACKSIZE);
static struct k_thread thread_range_data;

void range_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while(1) {
		k_mutex_lock(&linkback_lock, K_FOREVER);
		printk("--- PDR: %.2f%% - RSSI: %d\n", pdr, rssi);
		k_mutex_unlock(&linkback_lock);

		tx_power_setting.tx_power = -40;

		if (rssi < -50) {
			tx_power_setting.add_3dBm = true;
		} else {
			tx_power_setting.add_3dBm = false;
		}

		
		int err = ble_hci_vsc_set_tx_pwr(tx_power_setting);
		if (err) {
			printk("Failed to set tx power (err %d)\n", err);
			return;
		}

		k_sleep(K_MSEC(RANGE_CALC_INTERVAL_MS));
	}
}

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BROADCAST_ENQUEUE_COUNT * BIS_ISO_CHAN_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

static uint32_t seq_num;

static void iso_connected(struct bt_iso_chan *chan)
{
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan bis_iso_chan;

uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
struct net_buf *buf;

void gpio_work_handler(struct k_work *work)
{
    // printk("Sending value %u\n", seq_num);
	int err = write_8_bit(&io_encoder, seq_num % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}
K_WORK_DEFINE(gpio_work, gpio_work_handler);

static void iso_sent(struct bt_iso_chan *chan)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(++seq_num, iso_data);
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	// k_work_submit(&gpio_work); // FIXME: ENABLE IF NEEDED
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE, /* bytes */
	.rtn = RETRANSMISSION_NUMBER,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos,
};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT] = { &bis_iso_chan };

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = SDU_INTERVAL_US, /* in microseconds */
	.latency = TRANSPORT_LATENCY_MS, /* milliseconds */
	.packing = BT_ISO_PACKING_SEQUENTIAL, /* 0 - sequential, 1 - interleaved */
	.framing = BT_ISO_FRAMING_UNFRAMED, /* 0 - unframed, 1 - framed */
};

void main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	printk("Starting ISO Broadcast Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Initialize ACL Scanning */
	k_thread_create(&thread_acl_data, thread_acl_stack_area,
			K_THREAD_STACK_SIZEOF(thread_acl_stack_area),
			acl_thread, NULL, NULL, NULL,
			ACL_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_acl_data, "acl_thread");
	k_thread_start(&thread_acl_data);

	#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CUSTOM, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	#define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			BT_LE_PER_ADV_OPT_USE_TX_POWER)

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_CUSTOM);
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

	/* Set initial TX power */
	tx_power_setting.add_3dBm = true;
	tx_power_setting.tx_power = 0;
	err = ble_hci_vsc_set_tx_pwr(tx_power_setting);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
		return;
	}

	/* Initialize Range Extension */
	k_thread_create(&thread_range_data, thread_range_stack_area,
			K_THREAD_STACK_SIZEOF(thread_range_stack_area),
			range_thread, NULL, NULL, NULL,
			RANGE_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_range_data, "range_thread");
	k_thread_start(&thread_range_data);

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
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

	/* Prime TX buffer */
	printk("Initialize sending (fill buffer)\n");
	for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
		iso_sent(&bis_iso_chan);
	}
}
