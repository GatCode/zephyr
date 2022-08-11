#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/byteorder.h>
#include <ble_hci_vsc.h>
#include <stdlib.h>

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define SDU_INTERVAL_US 20000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 20 // 5ms-4s
#define BROADCAST_ENQUEUE_COUNT 2U // Guarantee always data to send

#define STACKSIZE 1024
#define ACL_PRIORITY 9
#define RANGE_PRIORITY 5

#define PDR_WATCHDOG_FREQ_MS 1000

#define ENABLE_RANGE_EXTENSION_ALGORITHM true // MAIN ALGO TOGGLE!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ALGO_MAX_THROUGHPUT (1000 / (SDU_INTERVAL_US / 1000)) * DATA_SIZE_BYTE * 8 / 1000
#define ALGO_HARD_LIMIT 16 // >= 16kbps are needed for LC3
#define ALGO_SOFT_LIMIT_1 0.5 * ALGO_MAX_THROUGHPUT
#define ALGO_SOFT_LIMIT_2 0.8 * ALGO_MAX_THROUGHPUT

#define INTELLIGENT_ALGO true
#define INCREMEMT_ALGO false

#define START_RTN 2 // also permanent rtn if algo not activated
#define START_TXP 0 // -.- see available_vs_tx_pwr_settings for settings
#define START_PHY BT_GAP_LE_PHY_2M // also permanent [phy] if algo not activated

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static double pdr = 0.0;
static double prev_pdr = 0.0;
uint8_t tx_pwr_setting = 0;
uint8_t rtn_setting = 0;
uint8_t phy_setting = BT_GAP_LE_PHY_2M; // also default setting (ISO only) if algo activated
uint8_t param_setting = 0;
static bool LED_ON = true;

/* ------------------------------------------------------ */
/* LEDs */
/* ------------------------------------------------------ */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* ------------------------------------------------------ */
/* Button */
/* ------------------------------------------------------ */
#define SW0_NODE	DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LED_ON = false;
	gpio_pin_set_dt(&led1, 0);
	gpio_pin_set_dt(&led2, 0);
	gpio_pin_set_dt(&led3, 0);
	gpio_pin_set_dt(&led4, 0);
}

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static uint32_t last_recv_packet_ts = 0;

#define DEVICE_NAME_ACL "nRF5340"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define CONFIG_BLE_ACL_CONN_INTERVAL 8 // 10ms
#define CONFIG_BLE_ACL_SLAVE_LATENCY 0
#define CONFIG_BLE_ACL_SUP_TIMEOUT 100

#define BT_LE_CONN_PARAM_MULTI \
		BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL, \
		CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

static K_SEM_DEFINE(sem_pdr_recv, 0, 1);

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if ((data_len == DEVICE_NAME_ACL_LEN) &&
	    (strncmp(DEVICE_NAME_ACL, data, DEVICE_NAME_ACL_LEN) == 0)) {
		bt_le_scan_stop();

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			printk("Could not init ACL connection\n");
			return ret;
		}

		return 0;
	}

	return -ENOENT;
}

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
	int ret = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (ret) {
		printk("ACL scanning failed to start (ret %d)\n", ret);
		return;
	}

	printk("ACL scanning successfully started\n");
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
	double value_recv;
	uint32_t mantissa;
	int8_t exponent;

	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	mantissa = sys_get_le24(&((uint8_t *)data)[1]);
	exponent = ((uint8_t *)data)[4];
	value_recv = (double)mantissa * pow(10, exponent);

	pdr = value_recv;
	// printk("PDR: %.2f%%\n", value_recv);
	k_sem_give(&sem_pdr_recv);
	last_recv_packet_ts = k_uptime_get_32();

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
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("ACL connected - %s\n", addr);

	/* Start Service Discovery */
	memcpy(&uuid, BT_UUID_HTS, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	int err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("ACL discovery failed(err %d)\n", err);
		return;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("ACL disconnected: %s (reason 0x%02x)\n", addr, reason);
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
static struct bt_le_ext_adv *adv;
static struct bt_iso_big *big;

static void iso_connected(struct bt_iso_chan *chan)
{
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	// printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan bis_iso_chan;

uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
struct net_buf *buf;

static void iso_sent(struct bt_iso_chan *chan)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(++seq_num, iso_data);
	iso_data[4] = rtn_setting;
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		// printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE, /* bytes */
	.rtn = START_RTN,
	.phy = START_PHY,
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

void pdr_watchdog_handler(struct k_timer *dummy)
{
	// NOTE: since the ACL connection is the strongest link,
	// 		 all packets incl a 0 PDR should come in.

	// uint32_t curr = k_uptime_get_32();
	// if (curr - last_recv_packet_ts > 1000) { // > 1s
	// 	pdr = 0.0; // reset - no packets arrived in the last second
	// 	k_sem_give(&sem_pdr_recv);
	// }
	k_sem_give(&sem_pdr_recv);
}
K_TIMER_DEFINE(pdr_watchdog, pdr_watchdog_handler, NULL);

uint32_t get_current_kbps()
{
	return (pdr / 100) * ALGO_MAX_THROUGHPUT;
}

struct broadcast_parameters
{
    uint8_t txp;
    uint8_t rtn;
};
static struct broadcast_parameters params[3] = {{0,2}, {0,4}, {1,8}};
static uint8_t params_idx = 0;
static uint32_t last_switch_ts = 0;
static uint32_t last_decreased_ts = false;

void range_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int err;

	while(1) {
		err = k_sem_take(&sem_pdr_recv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}

		// if(ENABLE_RANGE_EXTENSION_ALGORITHM) {
		// 	uint32_t kbps = get_current_kbps();

		// 	if (INTELLIGENT_ALGO) {
		// 		uint32_t curr = k_uptime_get_32();

		// 		if (kbps < ALGO_MAX_THROUGHPUT * 0.90) {
		// 			// increase
		// 			if (curr - last_decreased_ts > 1000 || kbps < ALGO_HARD_LIMIT) {
		// 				params_idx = params_idx < 2 ? params_idx + 1 : 2;
		// 				last_switch_ts = curr;
		// 			}
		// 		} else if (kbps < ALGO_MAX_THROUGHPUT * (0.90 + 0.09)) {
		// 			// ignore
		// 		} else {
		// 			if (curr - last_switch_ts > 1000) { // > 10s
		// 				// decrease
		// 				params_idx = params_idx > 0 ? params_idx - 1 : 0;
		// 				last_decreased_ts = curr;
		// 			}
		// 		}
		// 	}

		// 	if(param_setting != params_idx) {
		// 		err = bt_iso_big_terminate(big);
		// 		if (err) {
		// 			printk("bt_iso_big_terminate failed (err %d)\n", err);
		// 			return;
		// 		}

		// 		int err = ble_hci_vsc_set_tx_pwr(params[params_idx].txp);
		// 		if (err) {
		// 			printk("Failed to set tx power (err %d)\n", err);
		// 			return;
		// 		}

		// 		bis[0]->qos->tx->rtn = params[params_idx].rtn;
		// 		// bis[0]->qos->tx->phy = new_phy_setting;

		// 		err = k_sem_take(&sem_big_term, K_FOREVER);
		// 		if (err) {
		// 			printk("sem_big_term failed (err %d)\n", err);
		// 			return;
		// 		}

		// 		err = bt_iso_big_create(adv, &big_create_param, &big);
		// 		if (err) {
		// 			printk("bt_iso_big_create failed (err %d)\n", err);
		// 			return;
		// 		}

		// 		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		// 		if (err) {
		// 			printk("sem_big_cmplt failed (err %d)\n", err);
		// 			return;
		// 		}

		// 		iso_sent(&bis_iso_chan);
		// 		tx_pwr_setting = params[params_idx].txp;
		// 		rtn_setting = params[params_idx].rtn;
		// 		param_setting = params_idx;
		// 	}
		// }
		
		printk("PDR: %.2f%% - RTN: %u - TXP: %u, PHY: %u\n", pdr, bis[0]->qos->tx->rtn, tx_pwr_setting, bis[0]->qos->tx->phy);
		prev_pdr = pdr;

		if (LED_ON) {
			if (pdr > 20) {
				gpio_pin_set_dt(&led1, 1);
			} else {
				gpio_pin_set_dt(&led1, 0);
			}

			if (pdr > 40) {
				gpio_pin_set_dt(&led2, 1);
			} else {
				gpio_pin_set_dt(&led2, 0);
			}

			if (pdr > 60) {
				gpio_pin_set_dt(&led3, 1);
			} else {
				gpio_pin_set_dt(&led3, 0);
			}

			if (pdr > 80) {
				gpio_pin_set_dt(&led4, 1);
			} else {
				gpio_pin_set_dt(&led4, 0);
			}
		}
	}
}

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	int err;

	/* Initialize the LED */
	if (!device_is_ready(led1.port) || !device_is_ready(led2.port) ||  \
		!device_is_ready(led3.port) || !device_is_ready(led4.port)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

	/* Initialize the Button */
	int ret;
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Initialize Params */
	rtn_setting = START_RTN;
	tx_pwr_setting = START_TXP;

	/* Start ACL Scanning */
	k_work_submit(&start_scan_work);

	#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	/* Start PDR Watchdog timer */
	k_timer_start(&pdr_watchdog, K_MSEC(PDR_WATCHDOG_FREQ_MS), K_MSEC(PDR_WATCHDOG_FREQ_MS));

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
	err = ble_hci_vsc_set_tx_pwr(START_TXP);
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
