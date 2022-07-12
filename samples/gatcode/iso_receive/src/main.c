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
#include <sync_timer.h>
#include <stdlib.h>

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define PRESENTATION_DELAY_US 10000
#define MAXIMUM_SUBEVENTS 31 // MSE | 1-31

#define STACKSIZE 1024
#define ACL_PRIORITY 9

#define FIFO_SIZE 100

#define PDR_WATCHDOG_FREQ_MS 1000
#define INDICATE_IF_PDR_CHANGED_BY 1 // send indication if changes > define

#define MAX_TXP 13 // set ACL TX power to max (+3dBm)

#define LED_ON false
#define PWM_LED_ON true

#define THINGY_53 // NOTE: IMPORTRANT - DISABLE THIS IF DEVICE CHANGES!!!!!!!!!!!!!!!!!!!!

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static double pdr = 0.0;
static double last_indicated_pdr = 0.0;
static uint16_t iso_interval = 0;

/* ------------------------------------------------------ */
/* LEDs */
/* ------------------------------------------------------ */
#ifndef THINGY_53
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;
#define LED1_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
static struct k_thread thread_acl_data;

#define DEVICE_NAME_ACL "nRF5340"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define BT_LE_ADV_FAST_CONN \
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_1, \
		BT_GAP_ADV_FAST_INT_MAX_1, NULL)

static struct bt_gatt_indicate_params ind_params;
static K_SEM_DEFINE(acl_connected, 0, 1);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME_ACL, DEVICE_NAME_ACL_LEN),
};

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	// currently unused
}

BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void acl_connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("ACL Connection failed (err 0x%02x)\n", err);
	} else {
		uint16_t handle = 0;
		err = bt_hci_get_conn_handle(conn, &handle);
		err |= ble_hci_vsc_set_conn_tx_pwr(handle, BLE_HCI_VSC_TX_PWR_0dBm);
		if (err) {
			printk("ERROR: Setting ACL TX power failed\n");
		}
		printk("ACL Connected\n");
		if (LED_ON) {
			#ifndef THINGY_53
			gpio_pin_set_dt(&led3, 1);
			#endif
		}
		k_sem_give(&acl_connected);
	}
}

static void acl_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("ACL Disconnected (reason 0x%02x)\n", reason);
	if (LED_ON) {
		#ifndef THINGY_53
		gpio_pin_set_dt(&led3, 0);
		gpio_pin_set_dt(&led4, 0);
		#endif
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = acl_connected_cb,
	.disconnected = acl_disconnected_cb,
};

void work_adv_start(struct k_work *item)
{
	int ret = bt_le_adv_start(BT_LE_ADV_FAST_CONN, ad, ARRAY_SIZE(ad), NULL, 0);

	if (ret) {
		printk("ACL advertising failed to start (ret %d)\n", ret);
	}
}
K_WORK_DEFINE(adv_work, work_adv_start);

void acl_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* Set MAX TX power */
	init_ble_hci_vsc_tx_pwr();
	int err = ble_hci_vsc_set_tx_pwr(MAX_TXP);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	k_work_submit(&adv_work);
}

void acl_indicate(double pdr)
{
	if (PWM_LED_ON) {
		int err = pwm_set_pulse_dt(&pwm_led, (pwm_led.period * pdr) / 100);
		if (err) {
			printk("Error %d: failed to set pulse width\n", err);
		}
		#ifdef THINGY_53
		if (pdr < 1) {
			gpio_pin_set_dt(&led_red, 1);
		} else {
			gpio_pin_set_dt(&led_red, 0);
		}
		#endif
	}
	if (abs(last_indicated_pdr - pdr) > INDICATE_IF_PDR_CHANGED_BY) {
		static uint8_t htm[5];
		uint32_t mantissa;
		uint8_t exponent;

		mantissa = (uint32_t)(pdr * 100);
		exponent = (uint8_t)-2;

		htm[0] = 0;
		sys_put_le24(mantissa, (uint8_t *)&htm[1]);
		htm[4] = exponent;

		ind_params.attr = &hts_svc.attrs[2];
		ind_params.data = &htm;
		ind_params.len = sizeof(htm);
		(void)bt_gatt_indicate(NULL, &ind_params);
		last_indicated_pdr = pdr;

		if (LED_ON) {
			#ifndef THINGY_53
			gpio_pin_set_dt(&led4, 1);
			#endif
		}
	}
}

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
#define TIMEOUT_SYNC_CREATE K_SECONDS(20)
#define NAME_LEN            30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, \
					   BT_LE_SCAN_OPT_NONE, \
					   BT_GAP_SCAN_FAST_INTERVAL, \
					   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

static bool         per_adv_found;
static bool         per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint16_t     per_interval_ms;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, 1);
static K_SEM_DEFINE(sem_big_sync_lost, 0, 1);

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	if (!per_adv_found && info->interval) {
		per_adv_found = true;

		per_sid = info->sid;
		per_interval_ms = BT_CONN_INTERVAL_TO_MS(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);
		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	printk("periodic advertising sync received!\n");
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	printk("periodic advertising sync has been terminated!\n");
	per_adv_lost = true;	
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	iso_interval = biginfo->iso_interval;
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da
uint8_t maverage_values[FIFO_SIZE] = {0}; // all are zero as a start
uint8_t maverage_current_position  = 0;
uint64_t maverage_current_sum = 0;
uint8_t maverage_sample_length = sizeof(maverage_values) / sizeof(maverage_values[0]);

double RollingmAvg(uint8_t newValue)
{
         //Subtract the oldest number from the prev sum, add the new number
        maverage_current_sum = maverage_current_sum - maverage_values[maverage_current_position] + newValue;

        //Assign the newValue to the position in the array
        maverage_values[maverage_current_position] = newValue;

        maverage_current_position++;
        
        if (maverage_current_position >= maverage_sample_length) { // Don't go beyond the size of the array...
            maverage_current_position = 0;
        }
                
        //return the average
        return (double)maverage_current_sum * 100.0 / (double)maverage_sample_length;
}

// static bool pdr_timer_started = false;
static uint32_t seq_num = 0;
static uint32_t prev_seq_num = 0;
static uint32_t last_recv_packet_ts = 0;

void pdr_watchdog_handler(struct k_timer *dummy)
{
	uint32_t curr = audio_sync_timer_curr_time_get();
	// printk("curr: %u, last_recv_packet_ts: %u, diff: %u\n", curr, last_recv_packet_ts, curr - last_recv_packet_ts);
	if (curr - last_recv_packet_ts > 1000000) { // > 1s
		pdr = 0.0;
		acl_indicate(pdr);
		printk("PDR: %.2f%%\n", pdr);
	}
}
K_TIMER_DEFINE(pdr_watchdog, pdr_watchdog_handler, NULL);

void recv_packet_handler(struct k_timer *dummy)
{
	// currently unused
}
K_TIMER_DEFINE(recv_packet, recv_packet_handler, NULL);

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	if(info->flags == (BT_ISO_FLAGS_VALID | BT_ISO_FLAGS_TS)) { // valid ISO packet
		uint8_t count_arr[4];

		// printk("Data: ");
		for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
			if(i < 4) {
				count_arr[i] = buf->data[i];
			}
			// uint8_t data = buf->data[i];
			// printk("%x", data);
		}
		seq_num = sys_get_le32(count_arr);
		// printk(" | Packet ID: %u\n", seq_num);

		if(prev_seq_num + 1 != seq_num) {
			// printk("\n------------------------- LOST PACKET -------------------------\n");
		}
		prev_seq_num = seq_num;

		uint32_t curr = audio_sync_timer_curr_time_get();
		uint32_t packet_delta = abs(last_recv_packet_ts - curr);
		uint32_t iso_interval_us = iso_interval * 1.25 * 1000.0;
		double lost_packets = (double)packet_delta / (double)iso_interval_us;
		// printk("lost packets:  %.2f, packet_delta: %u, Packet ID: %u\n", lost_packets, packet_delta, seq_num);
		if (lost_packets > 0) {
			for (uint8_t i = 0; i < (uint8_t)lost_packets; i++) {
				pdr = RollingmAvg(0);
			}
		}
		pdr = RollingmAvg(1);

		printk("PDR: %.2f%%\n", pdr);
		acl_indicate(pdr);
		last_recv_packet_ts = audio_sync_timer_curr_time_get();

		// uint32_t info_ts = info->ts;
		// uint32_t curr = audio_sync_timer_curr_time_get();
		// uint32_t delta = curr - info_ts;
		// k_timer_start(&recv_packet, K_USEC(delta), K_NO_WAIT);
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos;

static struct bt_iso_chan_qos bis_iso_qos = {
	.rx = &iso_rx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos,
};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT] = { &bis_iso_chan };

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT) << 1),
	.mse = MAXIMUM_SUBEVENTS,
	.sync_timeout = BT_ISO_SYNC_TIMEOUT_MIN, /* in 10 ms units */
};

/* ------------------------------------------------------ */
/* Thingy:53 specific */
/* ------------------------------------------------------ */
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	NVIC_SystemReset(); // reboot since the Thingy:53 has no reset button
}

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout;
	int err;

	/* Initialize the LED */
	#ifndef THINGY_53
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
	#else
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, button.port->name, button.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (!device_is_ready(led_red.port) || !device_is_ready(pwm_led.dev)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

	err = pwm_set_pulse_dt(&pwm_led, 0);
	if (err) {
		printk("Error %d: failed to set pulse width\n", err);
	}
	#endif

	/* Initialize USB Output (Thingy:53) */
	#ifdef CONFIG_USB_DEVICE_STACK
	err = usb_enable(NULL);
	if (err) {
		printk("Failed to initialize USB device\n");
	}
	#endif

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Start ACL */
	k_thread_create(&thread_acl_data, thread_acl_stack_area,
			K_THREAD_STACK_SIZEOF(thread_acl_stack_area),
			acl_thread, NULL, NULL, NULL,
			ACL_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_acl_data, "acl_thread");
	k_thread_start(&thread_acl_data);

	err = k_sem_take(&acl_connected, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}

	/* Start ISO */
	printk("Init ISO receiption...");
	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	/* Start PDR Watchdog timer */
	k_timer_start(&pdr_watchdog, K_MSEC(PDR_WATCHDOG_FREQ_MS), K_MSEC(PDR_WATCHDOG_FREQ_MS));

	do {
		per_adv_lost = false;

		printk("Start scanning...");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("Found periodic advertising.\n");

		printk("Stop scanning...");
		err = bt_le_scan_stop();
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = (per_interval_ms * PA_RETRY_COUNT) / 10;
		sem_timeout = per_interval_ms * PA_RETRY_COUNT;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_MSEC(sem_timeout));
		if (err) {
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			continue;
		}
		printk("Periodic sync established.\n");

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_MSEC(sem_timeout));
		if (err) {
			printk("failed (err %d)\n", err);

			if (per_adv_lost) {
				continue;
			}

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			continue;
		}
		printk("Periodic sync established.\n");

big_sync_create:
		printk("Create BIG Sync...\n");
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Waiting for BIG sync...\n");
		err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
		if (err) {
			printk("failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("done.\n");

			goto per_sync_lost_check;
		}
		printk("BIG sync established.\n");

		printk("Waiting for BIG sync lost...\n");
		err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("BIG sync lost.\n");

per_sync_lost_check:
		printk("Check for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			/* Periodic Sync active, go back to creating BIG Sync */
			goto big_sync_create;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}