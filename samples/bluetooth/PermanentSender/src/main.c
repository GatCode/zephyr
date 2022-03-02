#include <bluetooth/bluetooth.h>
#include <device.h>
#include <drivers/counter.h>
#include <drivers/gpio.h>

static uint64_t packet_id = 0;

struct adv_payload {
	uint8_t id[8];
	uint8_t timestamp[4];
};

static struct adv_payload payload = { .id = {0,0,0,0,0,0,0,0} };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &payload, sizeof(struct adv_payload)),
};

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.timeout = 1,
	.num_events = 1,
};

#define TIMER DT_LABEL(DT_NODELABEL(timer2))
static const struct device *counter_dev;

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

void start_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	counter_start(counter_dev);
	packet_id = 0;
}

void status_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Packet ID: %llu\n", packet_id);
}

void sent_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{
	// split uint64_t in uint8_t and store in adv_data
	for (int i = 0; i < 8; i++) {
        payload.id[i] = packet_id >> (8 * i);
    }

	uint32_t timestamp = 0;
	counter_get_value(counter_dev, &timestamp);
	for (int i = 0; i < 4; i++) {
        payload.timestamp[i] = timestamp >> (8 * i);
    }

	int err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set ad (err %d)\n", err);
		return;
	}

	k_sleep(K_MSEC(10));
	printk("Sending Packet number %llu at TS: %u\n", packet_id, timestamp);

	// printk("Sending Packet: %lli\n", packet_id);
	packet_id++;

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
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

	counter_dev = device_get_binding(TIMER);
	if (counter_dev == NULL) {
		printk("Device not found\n");
		return;
	}

	/* Initialize the Buttons */
	if (!device_is_ready(button0.port) || !device_is_ready(button1.port)) {
		printk("Error: button device is not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	gpio_init_callback(&button0_cb_data, start_button_pressed, BIT(button0.pin));
	gpio_init_callback(&button1_cb_data, status_button_pressed, BIT(button1.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	gpio_add_callback(button1.port, &button1_cb_data);

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
