#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

/* ------------------------------------------------------ */
/* ISO Definitions */
/* ------------------------------------------------------ */
#define DATA_SIZE_BYTE 23
#define ISO_CHAN_COUNT 1
#define SDU_INTERVAL_US 10000
#define TRANSPORT_LATENCY_MS 20
#define RETRANSMISSION_NUMBER 2
#define PHY BT_GAP_LE_PHY_2M

/* ------------------------------------------------------ */
/* Light */
/* ------------------------------------------------------ */
typedef union {
	struct {
		bool brake;
		bool indicator_left;
		bool indicator_right;
		bool reverse;
		bool fog;
	} fields;
    uint32_t bits;
} light_status;

static light_status l_status = { 0 };

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, ISO_CHAN_COUNT,
		BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, ISO_CHAN_COUNT);

struct net_buf *buf;
static uint32_t seq_num;
uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
static struct bt_iso_chan bis_iso_chan;

static void iso_connected(struct bt_iso_chan *chan)
{
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(l_status.bits, iso_data);
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf, seq_num++, BT_ISO_TIMESTAMP_NONE);
	if (ret < 0) {
		printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE,
	.rtn = RETRANSMISSION_NUMBER,
	.phy = PHY,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos
};

static struct bt_iso_chan *bis[ISO_CHAN_COUNT] = {
	&bis_iso_chan
};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = SDU_INTERVAL_US,
	.latency = TRANSPORT_LATENCY_MS,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

/* ------------------------------------------------------ */
/* Buttons */
/* ------------------------------------------------------ */
static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static const struct gpio_dt_spec button_2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static const struct gpio_dt_spec button_3 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0});
static const struct gpio_dt_spec button_4 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0});

static struct gpio_callback button_1_cb_data;
static struct gpio_callback button_2_cb_data;
static struct gpio_callback button_3_cb_data;
static struct gpio_callback button_4_cb_data;

void button_1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get_dt(&button_1)) {
		l_status.fields.indicator_left = true;
    } else {
		l_status.fields.indicator_left = false;
    }
}

void button_2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&button_2)) {
		l_status.fields.indicator_right = true;
    } else {
		l_status.fields.indicator_right = false;
    }
}

void button_3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&button_3)) {
		l_status.fields.brake = true;
    } else {
		l_status.fields.brake = false;
    }
}

void button_4_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&button_4)) {
		l_status.fields.reverse = true;
    } else {
		l_status.fields.reverse = false;
    }
}

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	/* Initialize the Button */
	int ret;
	if (!device_is_ready(button_1.port) | !device_is_ready(button_2.port) | 
			!device_is_ready(button_3.port) | !device_is_ready(button_4.port)) {
		printk("Error: one of the buttons is not ready\n");
		return;
	}

	ret = gpio_pin_configure_dt(&button_1, GPIO_INPUT);
	ret |= gpio_pin_configure_dt(&button_2, GPIO_INPUT);
	ret |= gpio_pin_configure_dt(&button_3, GPIO_INPUT);
	ret |= gpio_pin_configure_dt(&button_4, GPIO_INPUT);
	if (ret != 0) {
		printk("Error: failed to configure one of the buttons\n");
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button_1, GPIO_INT_EDGE_BOTH);
	ret |= gpio_pin_interrupt_configure_dt(&button_2, GPIO_INT_EDGE_BOTH);
	ret |= gpio_pin_interrupt_configure_dt(&button_3, GPIO_INT_EDGE_BOTH);
	ret |= gpio_pin_interrupt_configure_dt(&button_4, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		printk("Error: failed to configure interrupt on one of the buttons\n");
		return;
	}

	gpio_init_callback(&button_1_cb_data, button_1_pressed, BIT(button_1.pin));
	gpio_init_callback(&button_2_cb_data, button_2_pressed, BIT(button_2.pin));
	gpio_init_callback(&button_3_cb_data, button_3_pressed, BIT(button_3.pin));
	gpio_init_callback(&button_4_cb_data, button_4_pressed, BIT(button_4.pin));

	gpio_add_callback(button_1.port, &button_1_cb_data);
	gpio_add_callback(button_2.port, &button_2_cb_data);
	gpio_add_callback(button_3.port, &button_3_cb_data);
	gpio_add_callback(button_4.port, &button_4_cb_data);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
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

	/* Start Broadcasting */
	iso_sent(&bis_iso_chan);
}
