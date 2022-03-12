#include <device.h>
#include <bluetooth/bluetooth.h>
#include <drivers/gpio.h>

#include <throughput_explorer.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button0_cb_data;
static K_SEM_DEFINE(sem_synced, 0, 1);

static struct explorer_config cfg = {0};
static struct explorer_payload payload = {0};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &payload, sizeof(struct explorer_payload)),
};

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.num_events = 1,
};

void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	reset_config(&cfg);
	k_sem_give(&sem_synced);
}

void sent_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{
	update_payload(&cfg, &payload);
	
	int err = gpio_pin_toggle_dt(&led);
	if (err) {
		printk("Failed to toggle LED (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
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

struct bt_le_ext_adv_cb adv_callbacks = {
	.sent = sent_cb,
};

void main(void)
{
	int err;
	struct bt_le_ext_adv *adv;

	/* Initialize LED */
	if (!device_is_ready(led.port)) {
		printk("Error: LED device is not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		printk("Error %d: failed to configure LED\n", err);
		return;
	}

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

	struct bt_le_adv_param param = {
		.options =  BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED,
		.interval_min = 0x0020 /* 20ms */,
		.interval_max = 0x0020 /* 20ms */,
		.secondary_max_skip = 0,
	};

	err = bt_le_ext_adv_create(&param, &adv_callbacks, &adv);
	if (err) {
		printk("Failed to create extended advertisement (err %d)\n", err);
		return;
	}

	err = k_sem_take(&sem_synced, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}

	update_payload(&cfg, &payload);
	err = gpio_pin_toggle_dt(&led);
	if (err) {
		printk("Failed to toggle LED (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set ad (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return;
	}
}
