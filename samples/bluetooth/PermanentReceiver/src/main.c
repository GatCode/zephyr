#include <device.h>
#include <bluetooth/bluetooth.h>
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

static bool data_cb(struct bt_data *data, void *user_data)
{
	switch (data->type) {
	case BT_DATA_MANUFACTURER_DATA:
		memcpy(user_data, data->data, sizeof(struct explorer_payload));
		return false;
	default:
		return true;
	}
}

void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	if(adv_type == BT_GAP_ADV_TYPE_EXT_ADV) {
		bt_data_parse(buf, data_cb, &payload);
		update_statistic(&statistic, &cfg, &payload);
		print_statistic(&statistic, 25);
	}
}

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
	
	/* Enable and setup Bluetooth*/
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	struct bt_le_scan_param param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M | BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0080  /* 80 ms */,
        .window = 0x0080 /* 80ms */,
		.timeout = 0,
		.interval_coded = 0,
        .window_coded = 0,
    };

	err = bt_le_scan_start(&param, &scan_cb);
	if (err) {
		printk("Bluetooth scan start failed (err %d)\n", err);
		return;
	}
}
