#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <ble_hci_vsc.h>
#include <stdlib.h>

/* ------------------------------------------------------ */
/* Defines Iso */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define SDU_INTERVAL_US 20000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 20 // 5ms-4s
#define BROADCAST_ENQUEUE_COUNT 2U // guarantee always data to send

/* ------------------------------------------------------ */
/* Defines R3 Concept */
/* ------------------------------------------------------ */
#define RTN 2 // also permanent rtn if algo not activated
#define TXP 0 // -.- see available_vs_tx_pwr_settings for settings

/* ------------------------------------------------------ */
/* Defines Threads (main thread = prio 0) */
/* ------------------------------------------------------ */
#define STACKSIZE 1024
#define ACL_PRIORITY 10
#define ADJ_PRIORITY 20

/* ------------------------------------------------------ */
/* Defines Algorithm */
/* ------------------------------------------------------ */
#define ENABLE_RANGE_EXTENSION_ALGORITHM true

#define ALGO_MAX_THROUGHPUT (1000 / (SDU_INTERVAL_US / 1000)) * DATA_SIZE_BYTE * 8 / 1000
#define ALGO_HARD_LIMIT 16 // >= 16kbps are needed for LC3
#define ALGO_SOFT_LIMIT_1 ALGO_MAX_THROUGHPUT * 0.5
#define ALGO_SOFT_LIMIT_2 ALGO_MAX_THROUGHPUT * 0.8

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static double prr = 0.0;
static int8_t acl_rssi = 0;
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

#define DEVICE_NAME_ACL "nRF5340"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define CONFIG_BLE_ACL_CONN_INTERVAL 8 // 10ms
#define CONFIG_BLE_ACL_SLAVE_LATENCY 0
#define CONFIG_BLE_ACL_SUP_TIMEOUT 100

#define BT_LE_CONN_PARAM_MULTI \
		BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL, \
		CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

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

static bool double_sending_rate_activated = false;
static bool buffer_reached_A = false;
static bool buffer_reached_B = false;

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	uint32_t mantissa = sys_get_le24(&((uint8_t *)data)[1]);
	int8_t exponent = ((uint8_t *)data)[4];
	prr = (double)mantissa * pow(10, exponent);
	acl_rssi = -((uint8_t *)data)[5];

	if (length >= 7) { // Opcode Received
		uint8_t curr_opcode = ((uint8_t *)data)[6];
		if (curr_opcode == 10) { // buffer reached B
			buffer_reached_B = true;
		} else if (curr_opcode == 11) { // buffer reached A
			buffer_reached_A = true;
		} else {
			buffer_reached_A = false;
			buffer_reached_B = false;
		}
	}

	printk("Received PRR: %.2f%% - ACL RSSI: %d\n", prr, acl_rssi);
	
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

	bt_conn_unref(conn);

	/* Start ACL Scanning */
	k_work_submit(&start_scan_work);
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

static K_SEM_DEFINE(sem_send_handler_stopped, 0, 1);
static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

static uint32_t seq_num;
static struct bt_le_ext_adv *adv;
static struct bt_iso_big *big;
static struct bt_iso_chan bis_iso_chan;

static void iso_connected(struct bt_iso_chan *chan)
{
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	k_sem_give(&sem_big_term);
}

struct net_buf *buf;
uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
uint8_t iso_data_second[DATA_SIZE_BYTE] = { 0 };
uint8_t iso_data_double_buffer[2 * DATA_SIZE_BYTE] = { 0 };

void send_handler(struct k_timer *self)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	if (double_sending_rate_activated) {
		sys_put_le32(++seq_num, iso_data);
		sys_put_le32(++seq_num, iso_data_second);
		memcpy(iso_data_double_buffer, iso_data, DATA_SIZE_BYTE);
		memcpy(iso_data_double_buffer + DATA_SIZE_BYTE, iso_data_second, DATA_SIZE_BYTE);
		net_buf_add_mem(buf, iso_data_double_buffer, sizeof(iso_data_double_buffer));
		// printk("Sent seq_num: %u\n", seq_num - 1);
		// printk("Sent seq_num: %u\n", seq_num);
	} else {
		sys_put_le32(++seq_num, iso_data);
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));
		// printk("Sent seq_num: %u\n", seq_num);
	}

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		printk("Unable to broadcast data: %d\n", ret);
		net_buf_unref(buf);
		return;
	}
}

void send_handler_stopped(struct k_timer *dummy)
{
	// printk("TIMER STOPPED @ seq_num: %u\n", seq_num);
	k_sem_give(&sem_send_handler_stopped);
}

K_TIMER_DEFINE(send_timer, send_handler, send_handler_stopped);

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected = iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE,
	.rtn = RTN,
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
/* Adjustment Algorithm */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(adj_stack_area, STACKSIZE);
static struct k_thread adj_thread;

uint32_t get_current_kbps()
{
	return (prr / 100) * ALGO_MAX_THROUGHPUT;
}

struct broadcast_parameters
{
    uint8_t txp;
    uint8_t rtn;
};
static struct broadcast_parameters params[3] = {{0,2}, {0,4}, {1,8}};
static uint8_t params_idx = 0;
static uint32_t last_switch_ts = 0;
static uint32_t last_decreased_ts = 0;

void adj_thread_handler(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int err;

	while(1) {
		uint32_t curr = k_uptime_get_32();

		if (buffer_reached_B && double_sending_rate_activated) { // revert to normal speed

			k_timer_stop(&send_timer);
			err = k_sem_take(&sem_send_handler_stopped, K_FOREVER);
			if (err) {
				printk("sem_big_term failed (err %d)\n", err);
				return;
			}
			// k_timer_status_sync(&send_timer); // wait until stopped

			err = bt_iso_big_terminate(big);
			if (err) {
				printk("bt_iso_big_terminate failed (err %d)\n", err);
				return;
			}

			err = k_sem_take(&sem_big_term, K_FOREVER);
			if (err) {
				printk("sem_big_term failed (err %d)\n", err);
				return;
			}

			bis[0]->qos->tx->sdu = DATA_SIZE_BYTE;
			double_sending_rate_activated = false;

			err = bt_iso_big_create(adv, &big_create_param, &big);
			if (err) {
				printk("bt_iso_big_create failed (err %d)\n", err);
				return;
			}

			err = k_sem_take(&sem_big_cmplt, K_FOREVER);
			if (err) {
				printk("sem_big_cmplt failed (err %d)\n", err);
				return;
			}

			k_timer_start(&send_timer, K_NO_WAIT, K_USEC(SDU_INTERVAL_US));
			// printk("REVERT TO NORMAL SPEED!\n");
		}

		// uint32_t kbps = get_current_kbps();

		// if (kbps < ALGO_MAX_THROUGHPUT * 0.90) {
		// 	// increase
		// 	if ((curr - last_decreased_ts > 1000 && curr - last_decreased_ts > 5000) || kbps < ALGO_HARD_LIMIT) {
		// 		params_idx = params_idx < 2 ? params_idx + 1 : 2;
		// 		last_switch_ts = curr;
		// 	}
		// } else if (kbps < ALGO_MAX_THROUGHPUT * (0.90 + 0.09)) {
		// 	// ignore
		// } else {
		// 	if (curr - last_switch_ts > 5000 && curr - last_decreased_ts > 5000) { // > 10s
		// 		// decrease
		// 		params_idx = params_idx > 0 ? params_idx - 1 : 0;
		// 		last_decreased_ts = curr;
		// 	}
		// }

		// increase / decrease
		else if (curr - last_decreased_ts > 2000) { //(param_setting != params_idx) {
			last_decreased_ts = curr;
			// printk("params_idx %u\n", params_idx);

			k_timer_stop(&send_timer);
			err = k_sem_take(&sem_send_handler_stopped, K_FOREVER);
			if (err) {
				printk("sem_big_term failed (err %d)\n", err);
				return;
			}
			// k_timer_status_sync(&send_timer); // wait until stopped

			err = bt_iso_big_terminate(big);
			if (err) {
				printk("bt_iso_big_terminate failed (err %d)\n", err);
				return;
			}

			err = k_sem_take(&sem_big_term, K_FOREVER);
			if (err) {
				printk("sem_big_term failed (err %d)\n", err);
				return;
			}

			err = bt_le_ext_adv_stop(adv);
			if (err) {
				printk("Failed to start extended advertising (err %d)\n", err);
				return;
			}

			// int err = ble_hci_vsc_set_tx_pwr(params[params_idx].txp);
			// if (err) {
			// 	printk("Failed to set tx power (err %d)\n", err);
			// 	return;
			// }

			// bis[0]->qos->tx->rtn = params[params_idx].rtn;

			bis[0]->qos->tx->sdu = 2 * DATA_SIZE_BYTE; // double speed
			double_sending_rate_activated = true; // set global flag

			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				printk("Failed to start extended advertising (err %d)\n", err);
				return;
			}

			err = bt_iso_big_create(adv, &big_create_param, &big);
			if (err) {
				printk("bt_iso_big_create failed (err %d)\n", err);
				return;
			}

			err = k_sem_take(&sem_big_cmplt, K_FOREVER);
			if (err) {
				printk("sem_big_cmplt failed (err %d)\n", err);
				return;
			}

			k_timer_start(&send_timer, K_NO_WAIT, K_USEC(SDU_INTERVAL_US));
			param_setting = params_idx;
		}
		
		if (LED_ON) {
			if (prr > 20) {
				gpio_pin_set_dt(&led1, 1);
			} else {
				gpio_pin_set_dt(&led1, 0);
			}

			if (prr > 40) {
				gpio_pin_set_dt(&led2, 1);
			} else {
				gpio_pin_set_dt(&led2, 0);
			}

			if (prr > 60) {
				gpio_pin_set_dt(&led3, 1);
			} else {
				gpio_pin_set_dt(&led3, 0);
			}

			if (prr > 80) {
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

	/* Start ACL Scanning */
	k_work_submit(&start_scan_work);

	#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	/* Set initial TX power */
	init_ble_hci_vsc_tx_pwr();
	err = ble_hci_vsc_set_tx_pwr(TXP);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
		return;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CUSTOM, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	#define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(0x0010, 0x0010, \
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

	/* Start Adjustment Thread */
	k_thread_create(&adj_thread, adj_stack_area,
		K_THREAD_STACK_SIZEOF(adj_stack_area),
		adj_thread_handler, NULL, NULL, NULL,
		ADJ_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&adj_thread, "adj_thread_handler");
	k_thread_start(&adj_thread);

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

	/* Start ISO Transmission */
	k_timer_start(&send_timer, K_NO_WAIT, K_USEC(SDU_INTERVAL_US));
}