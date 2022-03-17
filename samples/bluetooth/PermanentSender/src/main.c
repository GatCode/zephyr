#include <device.h>
#include <bluetooth/bluetooth.h>
#include <drivers/gpio.h>
#include <bluetooth/hci.h>

#include <throughput_explorer.h>

#define SW1_NODE DT_ALIAS(sw1)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button1_cb_data;

static K_SEM_DEFINE(sem_sync, 0, 1);

// static struct explorer_config cfg = {0};
// static struct explorer_payload payload = {0};
static struct sync_payload sync = {0};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &sync, sizeof(struct sync_payload)),
};

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.num_events = 1,
};

void debug_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("%.2f ms\n", k_ticks_to_us_floor64(z_nrf_rtc_timer_read()) / 1000.0);
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	switch (data->type) {
	case BT_DATA_MANUFACTURER_DATA:
		memcpy(user_data, data->data, sizeof(struct sync_payload));
		return false;
	default:
		return true;
	}
}

void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	if(adv_type == BT_GAP_ADV_TYPE_EXT_ADV) {	
		bt_data_parse(buf, data_cb, &sync);
		u64_to_u8_arr(z_nrf_rtc_timer_read(), sync.t2);
		k_sem_give(&sem_sync);
	}
}

void main(void)
{
	int err;
	struct bt_le_ext_adv *adv;

	/* Initialize Debug Button */
	if (!device_is_ready(button1.port)) {
		printk("Error: button device is not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	gpio_init_callback(&button1_cb_data, debug_button_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);

	/* Enable and setup Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Enable Scanning */
	struct bt_le_adv_param adv_param = {
		.options =  BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED,
		.interval_min = 0x0020 /* 20ms */,
		.interval_max = 0x0020 /* 20ms */,
		.secondary_max_skip = 0,
	};

	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create extended advertisement (err %d)\n", err);
		return;
	}

	/* Enable Extended Advertising */
	struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M | BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0a80  /* 80 ms */,
        .window = 0x0a80 /* 80ms */,
		.timeout = 0,
		.interval_coded = 0,
        .window_coded = 0,
    };

	err = bt_le_scan_start(&scan_param, &scan_cb);
	if (err) {
		printk("Bluetooth scan start failed (err %d)\n", err);
		return;
	}

	/* Sync Loop */
	while(1) {
		err = k_sem_take(&sem_sync, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}

		u64_to_u8_arr(z_nrf_rtc_timer_read(), sync.t3);

		printk("-------------\n");
		uint64_t t1, t2, t3;
		u8_arr_to_u64(sync.t1, &t1);
		u8_arr_to_u64(sync.t2, &t2);
		u8_arr_to_u64(sync.t3, &t3);
		printk("T1: %llu\n", t1);
		printk("T2: %llu\n", t2);
		printk("T3: %llu\n", t3);
		printk("-------------\n");

		int err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			printk("Failed to set advertising data!\n");
			return;
		}

		err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
		if (err) {
			printk("Failed to start extended advertising!\n");
			return;
		}
	}
}
