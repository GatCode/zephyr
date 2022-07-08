#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
#include <nrfx_clock.h>
#include <ble_hci_vsc.h>
#include <io_coder.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define SDU_INTERVAL_US 10000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 20 // 5ms-4s
#define BROADCAST_ENQUEUE_COUNT 2U // Guarantee always data to send
#define STACKSIZE 1024
#define ACL_PRIORITY 9
#define ACL_DATA_LEN 4
#define ACL_SCAN_INTERVAL 0x0050 // (N * 0.625 ms) - 50ms - sample 2x
#define RANGE_PRIORITY 5
#define RANGE_CALC_INTERVAL_MS 100
#define WATCHDOG_INTERVAL_MS 1000

#define MAX_RTN 10 // also default rtn
#define MAX_TXP 13 // also default tx power (+3dBm)

#define ENABLE_RANGE_EXTENSION_ALGORITHM true

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static float pdr = 0.0;
static int8_t rssi = 0;
K_MUTEX_DEFINE(linkback_lock);
uint8_t tx_pwr_setting = 0;
uint32_t watchdog_timestamp = 0;

/* ------------------------------------------------------ */
/* ACL (beacon) */
// /* ------------------------------------------------------ */
// K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
// static struct k_thread thread_acl_data;

// static void on_connected_cb(struct bt_conn *conn, uint8_t err)
// {
// 	if (err) {
// 		printk("Connection failed (err 0x%02x)\n", err);
// 	} else {
// 		printk("Connected\n");
// 	}
// }

// static void on_disconnected_cb(struct bt_conn *conn, uint8_t reason)
// {
// 	printk("Disconnected (reason 0x%02x)\n", reason);
// }

// static struct bt_conn_cb conn_callbacks = {
// 	.connected = on_connected_cb,
// 	.disconnected = on_disconnected_cb,
// };

// #define BT_LE_ADV_FAST_CONN                                                                        \
// 	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1,                      \
// 			BT_GAP_ADV_FAST_INT_MAX_1, NULL)

// #define CONFIG_BLE_DEVICE_NAME_BASE "NRF5340_AUDIO"
// #define DEVICE_NAME_PEER_L CONFIG_BLE_DEVICE_NAME_BASE "_H_L"
// #define DEVICE_NAME_PEER_L_LEN (sizeof(DEVICE_NAME_PEER_L) - 1)

// /* Advertising data for peer connection */
// static const struct bt_data ad_peer_l[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_PEER_L, DEVICE_NAME_PEER_L_LEN),
// };

// void work_adv_start(struct k_work *item)
// {
// 	int ret;

// 	ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad_peer_l, ARRAY_SIZE(ad_peer_l), NULL, 0);

// 	if (ret) {
// 		printk("Advertising failed to start (ret %d)\n", ret);
// 	}
// }
// K_WORK_DEFINE(adv_work, work_adv_start);

// void acl_thread(void *dummy1, void *dummy2, void *dummy3)
// {
// 	ARG_UNUSED(dummy1);
// 	ARG_UNUSED(dummy2);
// 	ARG_UNUSED(dummy3);

// 	bt_conn_cb_register(&conn_callbacks);
// 	k_work_submit(&adv_work);
// }



// void pdr_watchdog_handler(struct k_timer *dummy)
// {
// 	uint32_t curr_ts = k_uptime_get_32();
// 	if(curr_ts - watchdog_timestamp > 1000) {
// 		k_mutex_lock(&linkback_lock, K_FOREVER);
// 		pdr = 0.0;
// 		rssi = -127;
// 		k_mutex_unlock(&linkback_lock);
// 	}
// }
// K_TIMER_DEFINE(pdr_watchdog, pdr_watchdog_handler, NULL);


static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;



#define CONFIG_BLE_DEVICE_NAME_BASE "NRF5340_AUDIO"
#define DEVICE_NAME_PEER_L CONFIG_BLE_DEVICE_NAME_BASE "_H_L"
#define DEVICE_NAME_PEER_L_LEN (sizeof(DEVICE_NAME_PEER_L) - 1)

#define CONFIG_BLE_ACL_CONN_INTERVAL 100
#define CONFIG_BLE_ACL_SLAVE_LATENCY 0
#define CONFIG_BLE_ACL_SUP_TIMEOUT 400

#define BT_LE_CONN_PARAM_MULTI                                                                     \
	BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL,               \
			 CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

static K_SEM_DEFINE(sem_acl_connected, 0, 1);

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if ((data_len == DEVICE_NAME_PEER_L_LEN) &&
	    (strncmp(DEVICE_NAME_PEER_L, data, DEVICE_NAME_PEER_L_LEN) == 0)) {
		bt_le_scan_stop();

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			printk("Could not init connection\n");
			return ret;
		} else {
			k_sem_give(&sem_acl_connected);
		}

		// ret = ble_acl_gateway_conn_peer_set(0, &conn); // TODO: check err

		return 0;
	}

	return -ENOENT;
}

/** @brief  BLE parse advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			printk("AD malformed\n");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* We're only interested in general connectable events */
	if (type == BT_HCI_ADV_IND) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

void work_scan_start(struct k_work *item)
{
	int ret;

	ret = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (ret) {
		printk("Scanning failed to start (ret %d)\n", ret);
		return;
	}

	printk("Scanning successfully started\n");
}

K_WORK_DEFINE(start_scan_work, work_scan_start);


static double pow(double x, double y)
{
	double result = 1;

	if (y < 0) {
		y = -y;
		while (y--) {
			result /= x;
		}
	} else {
		while (y--) {
			result *= x;
		}
	}

	return result;
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	double temperature;
	uint32_t mantissa;
	int8_t exponent;

	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	/* temperature value display */
	mantissa = sys_get_le24(&((uint8_t *)data)[1]);
	exponent = ((uint8_t *)data)[4];
	temperature = (double)mantissa * pow(10, exponent);

	printf("Temperature %gC.\n", temperature);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HTS)) {
		memcpy(&uuid, BT_UUID_HTS_MEASUREMENT, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HTS_MEASUREMENT)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_INDICATE;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}


static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("CONNECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	memcpy(&uuid, BT_UUID_HTS, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
		return;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

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
	.rtn = MAX_RTN,
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
		if(ENABLE_RANGE_EXTENSION_ALGORITHM) {
			uint8_t new_tx_pwr_setting = 0;

			if (rssi > -20) {
				new_tx_pwr_setting = 0;
				bis[0]->qos->tx->rtn = 2;
			} else if (rssi > -30) {
				new_tx_pwr_setting = 8;
				bis[0]->qos->tx->rtn = 4;
			} else if (rssi > -40) {
				new_tx_pwr_setting = 12;
				bis[0]->qos->tx->rtn = 6;
			} else if (rssi > -50) {
				new_tx_pwr_setting = 13;
				bis[0]->qos->tx->rtn = 8;
			} else {
				new_tx_pwr_setting = 13;
				bis[0]->qos->tx->rtn = 10;
			}

			if(tx_pwr_setting != new_tx_pwr_setting) {
				uint8_t err = ble_hci_vsc_set_tx_pwr(new_tx_pwr_setting);
				if (err) {
					printk("Failed to set tx power (err %d)\n", err);
					return;
				}
				tx_pwr_setting = new_tx_pwr_setting;
			}
		}
		
		k_mutex_lock(&linkback_lock, K_FOREVER);
		printk("PDR: %.2f%% - RSSI: %d - RTN: %u\n", pdr, rssi, bis[0]->qos->tx->rtn);
		k_mutex_unlock(&linkback_lock);

		k_sleep(K_MSEC(RANGE_CALC_INTERVAL_MS));
	}
}

/* ------------------------------------------------------ */
/* NRFX Clock */
/* ------------------------------------------------------ */
static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

void main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	err = hfclock_config_and_start();
	if (err) {
		printk("NRFX Clock init failed (err %d)\n", err);
		return;
	}

	printk("Starting ISO Broadcast Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Initialize ACL Scanning */
	// k_thread_create(&thread_acl_data, thread_acl_stack_area,
	// 		K_THREAD_STACK_SIZEOF(thread_acl_stack_area),
	// 		acl_thread, NULL, NULL, NULL,
	// 		ACL_PRIORITY, 0, K_FOREVER);
	// k_thread_name_set(&thread_acl_data, "acl_thread");
	// k_thread_start(&thread_acl_data);

	k_work_submit(&start_scan_work);

	#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	// k_timer_start(&pdr_watchdog, K_MSEC(WATCHDOG_INTERVAL_MS), K_MSEC(WATCHDOG_INTERVAL_MS));

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
	init_ble_hci_vsc_tx_pwr();
	err = ble_hci_vsc_set_tx_pwr(MAX_TXP);
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
