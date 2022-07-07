#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
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
#define MOVING_WINDOW_SIZE 20
#define STACKSIZE 1024
#define ACL_PRIORITY 9
#define ISO_PRIORITY 10
#define ACL_ADV_INTERVAL 0x00A0 // 0x0020 to 0x4000 (N * 0.625ms) - 100ms
#define ACL_UPDATE_FREQUENCY_MS 10
#define PDR_UPDATE_FREQUENCY_MS 1000

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static float pdr = 0.0;
static uint16_t iso_interval = 0;

/* ------------------------------------------------------ */
/* ACL (beacon) */
/* ------------------------------------------------------ */
// static volatile uint8_t acl_data[] = { 0x00, 0x00, 0x00, 0x00 };

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, acl_data, 4)
// };

// static const struct bt_data sd[] = {
// 	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1)
// };

// void acl_work_handler(struct k_work *work)
// {
// 	int err;

// 	#define BT_LE_ADV_NCONN_CUSTOM BT_LE_ADV_PARAM(0, ACL_ADV_INTERVAL, ACL_ADV_INTERVAL, NULL)

// 	err = bt_le_adv_start(BT_LE_ADV_NCONN_CUSTOM, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
// 	if (err) {
// 		printk("ACL advertising failed to start (err %d)\n", err);
// 		return;
// 	}

// 	printk("ACL advertising successfully started\n");
// }
// K_WORK_DEFINE(acl_work, acl_work_handler);

// void acl_update_handler(struct k_work *work)
// {
// 	uint16_t integer_pdr = pdr * 100.0;
// 	uint8_t pdr_splitted[4] = {0, 0, 0, 0};
// 	uint8_t acl_data_size = 4;

// 	if (pdr < 10) {
// 		acl_data_size = 3;
// 	}

// 	if (pdr == 0) {
// 		acl_data_size = 2;
// 	}

// 	for (uint8_t i = 4; i > 0; i--) {
// 		pdr_splitted[i - 1] = integer_pdr % 10;
// 		integer_pdr /= 10;
// 	}

// 	if (pdr == 100.0) {
// 		pdr_splitted[0] = 0xF;
// 	}

// 	acl_data[0] = pdr_splitted[0] << 4 | pdr_splitted[1];
// 	acl_data[1] = pdr_splitted[2] << 4 | pdr_splitted[3];

//     bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
// }
// K_WORK_DEFINE(acl_update, acl_update_handler);

// void acl_packet_handler(struct k_timer *dummy)
// {
// 	k_work_submit(&acl_update);
// }
// K_TIMER_DEFINE(acl_packet, acl_packet_handler, NULL);

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
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

K_THREAD_STACK_DEFINE(thread_iso_stack_area, STACKSIZE);
static struct k_thread thread_iso_data;

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

static bool pdr_timer_started = false;
static bool received_packet = true;

// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da
static uint8_t maverage_values[MOVING_WINDOW_SIZE] = {0}; // all are zero as a start
static uint8_t maverage_current_position  = 0;
static uint64_t maverage_current_sum = 0;
static uint8_t maverage_sample_length = sizeof(maverage_values) / sizeof(maverage_values[0]);

static float RollingmAvg(uint8_t newValue)
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
// 	printk("PDR:  %.2f%%\n", pdr);
// }
// K_TIMER_DEFINE(recv_pdr, recv_pdr_handler, NULL);

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
			// printk("\n------------------------- LOST PACKET -------------------------\n");
		}
		prev_seq_num = seq_num;
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

void iso_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout;
	int err;

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

void main(void)
{
	int err;

	/* Initialize the LED */
	if (!device_is_ready(led.port)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Start ACL */
	printk("Start ACL...");
	// k_work_submit(&acl_work);
	// k_timer_start(&acl_packet, K_MSEC(ACL_UPDATE_FREQUENCY_MS), K_MSEC(ACL_UPDATE_FREQUENCY_MS));

	/* Start ISO Thread */
	k_thread_create(&thread_iso_data, thread_iso_stack_area,
			K_THREAD_STACK_SIZEOF(thread_iso_stack_area),
			iso_thread, NULL, NULL, NULL,
			ISO_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_iso_data, "iso_thread");
	k_thread_start(&thread_iso_data);
}