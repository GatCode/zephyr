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
#include <sync_timer.h>
#include <stdlib.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

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

#define LED_ON true

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static float pdr = 0.0;
static uint16_t iso_interval = 0;

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
		printk("ACL Connected\n");
		if (LED_ON) {
			gpio_pin_set_dt(&led, 1);
		}
	}
}

static void acl_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("ACL Disconnected (reason 0x%02x)\n", reason);
	if (LED_ON) {
		gpio_pin_set_dt(&led, 0);
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

	bt_conn_cb_register(&conn_callbacks);
	k_work_submit(&adv_work);
}

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
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
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
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

static uint32_t seq_num = 0;
static uint32_t prev_seq_num = 0;

// void gpio_work_handler(struct k_work *work)
// {
//     int err = write_8_bit(&io_encoder, seq_num % 256);
// 	if(err) {
// 		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
// 	}
// }
// K_WORK_DEFINE(gpio_work, gpio_work_handler);

// void recv_packet_handler(struct k_timer *dummy)
// {
// 	k_work_submit(&gpio_work);
// }
// K_TIMER_DEFINE(recv_packet, recv_packet_handler, NULL);

static bool pdr_timer_started = false;
static bool received_packet = true;

// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da
uint8_t maverage_values[FIFO_SIZE] = {0}; // all are zero as a start
uint8_t maverage_current_position  = 0;
uint64_t maverage_current_sum = 0;
uint8_t maverage_sample_length = sizeof(maverage_values) / sizeof(maverage_values[0]);

float RollingmAvg(uint8_t newValue)
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
        return (float)maverage_current_sum * 100.0 / (float)maverage_sample_length;
}

// void recv_pdr_handler(struct k_timer *dummy)
// {
// 	uint8_t value = 0;

// 	if(received_packet) {
// 		value = 1;
// 	}

// 	pdr = RollingmAvg(value);

// 	// pdr_indicate();
// 	k_work_submit(&acl_work_indicate);

// 	received_packet = false; // reset
// }
// K_TIMER_DEFINE(recv_pdr, recv_pdr_handler, NULL);


static struct bt_gatt_indicate_params ind_params;

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	if(info->flags == (BT_ISO_FLAGS_VALID | BT_ISO_FLAGS_TS)) { // valid ISO packet
		uint8_t count_arr[4];

		printk("Data: ");
		for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
			if(i < 4) {
				count_arr[i] = buf->data[i];
			}
			uint8_t data = buf->data[i];
			printk("%x", data);
		}
		seq_num = sys_get_le32(count_arr);
		printk(" | Packet ID: %u\n", seq_num);

		// if(!pdr_timer_started) {
		// 	uint32_t iso_ival_ms = iso_interval * 1.25;
		// 	k_timer_start(&recv_pdr, K_MSEC(iso_ival_ms), K_MSEC(iso_ival_ms));
		// 	pdr_timer_started = true;
		// }

		received_packet = true;
		if(prev_seq_num + 1 != seq_num) {
			received_packet = false;
			printk("\n------------------------- LOST PACKET -------------------------\n");
		}
		prev_seq_num = seq_num;

		if (seq_num % 10 == 0) {
			static uint8_t htm[5];
			static double value_recv = 20U;
			uint32_t mantissa;
			uint8_t exponent;
			
			value_recv = seq_num;

			printk("PDR: %.2f%%\n", value_recv);

			mantissa = (uint32_t)(value_recv * 100);
			exponent = (uint8_t)-2;

			htm[0] = 0;
			sys_put_le24(mantissa, (uint8_t *)&htm[1]);
			htm[4] = exponent;

			ind_params.attr = &hts_svc.attrs[2];
			ind_params.data = &htm;
			ind_params.len = sizeof(htm);
			(void)bt_gatt_indicate(NULL, &ind_params);
		}

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
	.sync_timeout = BT_ISO_SYNC_TIMEOUT_MAX, /* in 10 ms units */
};

void main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout;
	int err;

	/* Initialize the LED */
	if (!device_is_ready(led.port)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

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

	/* Start ISO */
	printk("Init ISO receiption...");
	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

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