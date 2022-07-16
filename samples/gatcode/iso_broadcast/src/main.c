#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
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

#define LED_ON true
#define PWM_LED_ON true

#define ENABLE_RANGE_EXTENSION_ALGORITHM false

#define START_RTN 8 // also permanent rtn if algo not activated
#define START_TXP 13 // -.- see available_vs_tx_pwr_settings for settings
#define START_PHY BT_GAP_LE_PHY_2M // also permanent [phy] if algo not activated

/* ------------------------------------------------------ */
/* LEDs */
/* ------------------------------------------------------ */
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

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
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		// printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	printk("Send: %u\n", seq_num);
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
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	int err;

	/* Initialize the LED */
	if (!device_is_ready(led3.port) || !device_is_ready(led4.port) || !device_is_ready(pwm_led.dev)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

	err = pwm_set_pulse_dt(&pwm_led, 0);
	if (err) {
		printk("Error %d: failed to set pulse width\n", err);
	}
	
	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

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
	init_ble_hci_vsc_tx_pwr();
	err = ble_hci_vsc_set_tx_pwr(START_TXP);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
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

	/* Prime TX buffer */
	printk("Initialize sending (fill buffer)\n");
	for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
		iso_sent(&bis_iso_chan);
	}
}