#include <device.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/iso.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>

#include <throughput_explorer.h>

#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button0_cb_data;
static K_SEM_DEFINE(sem_synced, 0, 1);

static struct explorer_config cfg = {0};
static struct explorer_payload payload = {0};

void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	reset_config(&cfg);
	k_sem_give(&sem_synced);
}

#define BIG_TERMINATE_TIMEOUT 60 /* seconds */

#define BIS_ISO_CHAN_COUNT 1
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
			  BT_ISO_SDU_BUF_SIZE(sizeof(struct explorer_payload)), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = sizeof(struct explorer_payload),
	.rtn = 1,
	.phy = BT_GAP_LE_PHY_CODED,
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
	.interval = 10000, /* in microseconds */
	.latency = 10, /* milliseconds */
	.packing = 0, /* 0 - sequential, 1 - interleaved */
	.framing = 0, /* 0 - unframed, 1 - framed */
};

void main(void)
{
	int err;
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;

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
		.options =  BT_LE_ADV_OPT_EXT_ADV /*| BT_LE_ADV_OPT_CODED*/,
		.interval_min = 0x0020 /* 20ms */,
		.interval_max = 0x0020 /* 20ms */,
		.secondary_max_skip = 0,
	};

	err = bt_le_ext_adv_create(&param, NULL, &adv);
	if (err) {
		printk("Failed to create extended advertisement (err %d)\n", err);
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

	struct net_buf *buf;

	err = k_sem_take(&sem_synced, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}

	while (true) {
		// static uint8_t timeout = BIG_TERMINATE_TIMEOUT;
		int ret;

		update_payload(&cfg, &payload);

		// k_sleep(K_SECONDS(1));
		k_sleep(K_MSEC(10));

		buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		net_buf_add_mem(buf, &payload, sizeof(payload));
		ret = bt_iso_chan_send(&bis_iso_chan, buf);
		if (ret < 0) {
			printk("Unable to broadcast data: %d", ret);
			net_buf_unref(buf);
			return;
		}
		printk("Sending value %llu\n", cfg.packet_id);

		// timeout--;
		// if (!timeout) {
		// 	timeout = BIG_TERMINATE_TIMEOUT;

		// 	printk("BIG Terminate...");
		// 	err = bt_iso_big_terminate(big);
		// 	if (err) {
		// 		printk("failed (err %d)\n", err);
		// 		return;
		// 	}
		// 	printk("done.\n");

		// 	printk("Waiting for BIG terminate complete...");
		// 	err = k_sem_take(&sem_big_term, K_FOREVER);
		// 	if (err) {
		// 		printk("failed (err %d)\n", err);
		// 		return;
		// 	}
		// 	printk("done.\n");

		// 	printk("Create BIG...");
		// 	err = bt_iso_big_create(adv, &big_create_param, &big);
		// 	if (err) {
		// 		printk("failed (err %d)\n", err);
		// 		return;
		// 	}
		// 	printk("done.\n");

		// 	printk("Waiting for BIG complete...");
		// 	err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		// 	if (err) {
		// 		printk("failed (err %d)\n", err);
		// 		return;
		// 	}
		// 	printk("done.\n");
		// }
	}
}
