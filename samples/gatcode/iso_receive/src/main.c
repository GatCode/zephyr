#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <ble_hci_vsc.h>
#include <sync_timer.h>
#include <stdlib.h>

/* ------------------------------------------------------ */
/* Defines Iso */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define SDU_INTERVAL_US 20000 // could also be extraced from big info
#define MAXIMUM_SUBEVENTS 31 // MSE | 1-31

/* ------------------------------------------------------ */
/* Defines Threads (main thread = prio 0) */
/* ------------------------------------------------------ */
#define STACKSIZE 1024
#define ACL_PRIORITY 10

/* ------------------------------------------------------ */
/* PRR Calculation */
/* ------------------------------------------------------ */
#define PRR_MAVG_WINDOW_SIZE 50

/* ------------------------------------------------------ */
/* Defines Algorithm */
/* ------------------------------------------------------ */
#define PACKET_BUFFER_SIZE 40 // 50 = 1s
#define PACKET_BUFFER_OCCUPIED_THRESHOLD_LOW 20 // must be min 13 slots to ensure no issues - 50% = 7 packets
#define PACKET_BUFFER_OCCUPIED_THRESHOLD_HIGH 33 // must be min 13 slots to ensure no issue - 50% = 7 packets
#define ALGO_MAX_THROUGHPUT (1000 / (SDU_INTERVAL_US / 1000)) * DATA_SIZE_BYTE * 8 / 1000
#define ALGO_HARD_LIMIT 16 // >= 16kbps are needed for LC3
#define ALGO_SOFT_LIMIT_1 ALGO_MAX_THROUGHPUT * 0.5
#define ALGO_SOFT_LIMIT_2 ALGO_MAX_THROUGHPUT * 0.8

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static bool LED_ON = true;
static double prr = 0.0;
static int8_t per_adv_rssi = 0;
static uint8_t per_adv_txp_idx = 0;
static uint8_t link_quality_idx_proposal = 0;

/* ------------------------------------------------------ */
/* LEDs */
/* ------------------------------------------------------ */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* ------------------------------------------------------ */
/* Button */
/* ------------------------------------------------------ */
#define SW0_NODE	DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LED_ON = false;
	gpio_pin_set_dt(&led1, 0);
	gpio_pin_set_dt(&led2, 0);
	gpio_pin_set_dt(&led3, 0);
	gpio_pin_set_dt(&led4, 0);
}

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
K_THREAD_STACK_DEFINE(thread_acl_stack_area, STACKSIZE);
static struct k_thread thread_acl_data;
static struct bt_conn *acl_conn;

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
		acl_conn = conn;
		printk("ACL Connected\n");
		k_sem_give(&acl_connected);
	}
}

static void acl_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("ACL Disconnected (reason 0x%02x)\n", reason);
	k_thread_start(&thread_acl_data);
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
	int err = ble_hci_vsc_set_tx_pwr(13);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	k_work_submit(&adv_work);
}

static K_SEM_DEFINE(txp_finish_sem, 0, 1);

static bool iso_just_established = true;
RING_BUF_DECLARE(PacketBuffer, PACKET_BUFFER_SIZE * DATA_SIZE_BYTE);

static uint8_t last_indicated_opcode = 0;
static uint8_t last_indicated_link_quality_idx_proposal = 0;

uint32_t get_current_kbps()
{
	return (prr / 100) * ALGO_MAX_THROUGHPUT;
}

void indicate_work_handler(struct k_work *item)
{
	/* Fetch ACL connection handle */
	uint16_t acl_handle = 0;
	int err = bt_hci_get_conn_handle(acl_conn, &acl_handle);
	if (err) {
		printk("Failed to fetch the ACL connection handle (err %d)\n", err);
		return;
	}

	/* Set TXP */
	err = ble_hci_vsc_set_conn_tx_pwr_index(acl_handle, per_adv_txp_idx);
	if (err) {
		printk("Failed to set tx power (err %d)\n", err);
		return;
	}

	/* Determine Opcode */
	uint8_t new_opc = 0;
	uint32_t free_slots = ring_buf_space_get(&PacketBuffer) / DATA_SIZE_BYTE;
	if (PACKET_BUFFER_SIZE - free_slots > PACKET_BUFFER_OCCUPIED_THRESHOLD_HIGH) {
		new_opc = 10; // ready for downshift
	} else if (PACKET_BUFFER_SIZE - free_slots > PACKET_BUFFER_OCCUPIED_THRESHOLD_LOW) {
		new_opc = 11; // normal buffering speed - ready for next adjustment
	} else {
		new_opc = 12; // ignore
	}

	/* Prepare Indication */
	#define CONST_IND_DATA_SIZE 2
	uint8_t ind_data_size = CONST_IND_DATA_SIZE;
	static uint8_t ind_data[CONST_IND_DATA_SIZE];

	if (last_indicated_opcode == new_opc) { // don't indicate the same opcode twice
		ind_data_size = ind_data_size - 1;
	} else {
		last_indicated_opcode = new_opc;
	}

	/* Adaptation Algorithm */
	uint32_t kbps = get_current_kbps();
	if (per_adv_rssi < -80) {
		link_quality_idx_proposal = 10;
	} else {
		link_quality_idx_proposal = 0;
	}
	// if (kbps < ALGO_MAX_THROUGHPUT * 0.90) {
	// 	// increase
	// 	if ((curr - last_decreased_ts > 1000 && curr - last_decreased_ts > 5000) || kbps < ALGO_HARD_LIMIT) {
	// 		params_idx = params_idx < 2 ? params_idx + 1 : 2;
	// 		last_switch_ts = curr;
	// 	}
	// } else if (kbps < ALGO_MAX_THROUGHPUT * (0.90 + 0.09)) {
	// 	// ignore
	// } else {
	// 	if (curr - last_switch_ts > 5000 && curr - last_decreased_ts > 5000) { // > 10s
	// 		// decrease
	// 		params_idx = params_idx > 0 ? params_idx - 1 : 0;
	// 		last_decreased_ts = curr;
	// 	}
	// }

	if (last_indicated_link_quality_idx_proposal == link_quality_idx_proposal && ind_data_size != CONST_IND_DATA_SIZE) {
		return;
	}
	last_indicated_link_quality_idx_proposal = link_quality_idx_proposal;

	/* Create Indication */
	ind_data[0] = link_quality_idx_proposal;
	ind_data[1] = new_opc;
	ind_params.attr = &hts_svc.attrs[2];
	ind_params.data = &ind_data;
	ind_params.len = sizeof(uint8_t) * ind_data_size;
	(void)bt_gatt_indicate(NULL, &ind_params);
}
K_WORK_DEFINE(indicate_work, indicate_work_handler);

void acl_indicate()
{
	k_work_submit(&indicate_work);
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

#define PA_RETRY_COUNT 10

static bool         per_adv_found;
static bool         per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint16_t     per_interval_ms;

static uint32_t seq_num = 0;
static uint32_t prev_seq_num = 0;

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
	k_sem_give(&sem_per_big_info);
}

static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	per_adv_rssi = info->rssi;
	per_adv_txp_idx = get_hci_vsc_tx_pwr_idx(info->tx_power);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
	.recv = recv_cb,
};

// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da
uint64_t maverage_values[PRR_MAVG_WINDOW_SIZE] = {0}; // all are zero as a start
uint64_t maverage_current_position = 0;
uint64_t maverage_current_sum = 0;
uint64_t maverage_sample_length = sizeof(maverage_values) / sizeof(maverage_values[0]);

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

void buffer_timer_handler(struct k_timer *dummy)
{
	uint32_t free_slots = ring_buf_space_get(&PacketBuffer);

	// initial buffer fill as stream establishment
	if (PACKET_BUFFER_SIZE - (free_slots / DATA_SIZE_BYTE) > PACKET_BUFFER_OCCUPIED_THRESHOLD_HIGH && iso_just_established) {
		iso_just_established = false;
	} 
	
	if (iso_just_established) {
		return;
	}
	
	if (!ring_buf_is_empty(&PacketBuffer)) {
		uint8_t data[DATA_SIZE_BYTE];
		ring_buf_get(&PacketBuffer, (uint8_t*)&data, DATA_SIZE_BYTE);

		uint8_t count_arr[4];
		for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
			if(i < 4) {
				count_arr[i] = data[i];
			}
		}
		seq_num = sys_get_le32(count_arr);

		gpio_pin_toggle_dt(&led2);

		if (seq_num - 1 != prev_seq_num && prev_seq_num < seq_num) {
			for (uint8_t i = 0; i < seq_num - prev_seq_num; i++) {
				prr = RollingmAvg(0);
			}
		} else {
			prr = RollingmAvg(1);
		}

		acl_indicate(prr);

		if (seq_num - 1 != prev_seq_num) {
			printk("LOST - ");
		}

		prev_seq_num = seq_num;
	} else {
		prr = RollingmAvg(0);
	}

	printk("Buffer occupied: %u out of %u - prr: %.02f%% - seq_num: %u - rssi: %d, tx_power index: %d\n", PACKET_BUFFER_SIZE - free_slots / DATA_SIZE_BYTE, PACKET_BUFFER_SIZE, prr, seq_num, per_adv_rssi, per_adv_txp_idx);
}
K_TIMER_DEFINE(buffer_timer, buffer_timer_handler, NULL);

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	if(info->flags == (BT_ISO_FLAGS_VALID | BT_ISO_FLAGS_TS)) { // valid ISO packet
		uint8_t count_arr[4];

		for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
			if(i < 4) {
				count_arr[i] = buf->data[i];
			}
		}
		seq_num = sys_get_le32(count_arr);

		ring_buf_put(&PacketBuffer, buf->data, DATA_SIZE_BYTE);

		if (buf->len > DATA_SIZE_BYTE) { // DOUBLE BUFFERING ACTIVATED
			for(uint8_t i = 0; i < DATA_SIZE_BYTE; i++) {
				if(i < 4) {
					count_arr[i] = buf->data[DATA_SIZE_BYTE + i];
				}
			}
			seq_num = sys_get_le32(count_arr);

			ring_buf_put(&PacketBuffer, buf->data + DATA_SIZE_BYTE, DATA_SIZE_BYTE);
		}
		
		// if (LED_ON) {
		// 	if (pdr > 20) {
		// 		gpio_pin_set_dt(&led1, 1);
		// 	} else {
		// 		gpio_pin_set_dt(&led1, 0);
		// 	}

		// 	if (pdr > 40) {
		// 		gpio_pin_set_dt(&led2, 1);
		// 	} else {
		// 		gpio_pin_set_dt(&led2, 0);
		// 	}

		// 	if (pdr > 60) {
		// 		gpio_pin_set_dt(&led3, 1);
		// 	} else {
		// 		gpio_pin_set_dt(&led3, 0);
		// 	}

		// 	if (pdr > 80) {
		// 		gpio_pin_set_dt(&led4, 1);
		// 	} else {
		// 		gpio_pin_set_dt(&led4, 0);
		// 	}
		// }
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	gpio_pin_toggle_dt(&led1);

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
	if (!device_is_ready(led1.port) || !device_is_ready(led2.port) ||  \
		!device_is_ready(led3.port) || !device_is_ready(led4.port)) {
 		printk("Error setting LED\n");
 	}

 	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	err |= gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
 	if (err < 0) {
 		printk("Error setting LED (err %d)\n", err);
 	}

	/* Initialize the Button */
	int ret;
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

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

	/* Start buffer timer */
	k_timer_start(&buffer_timer, K_NO_WAIT, K_USEC(SDU_INTERVAL_US));

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
		sync_create_param.timeout = 10000; //(per_interval_ms * PA_RETRY_COUNT) / 10;
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

			printk("Deleting Periodic Advertising Sync...\n");
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

			printk("Deleting Periodic Advertising Sync...\n");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			continue;
		}
		printk("Periodic sync established.\n");

big_sync_create:
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}

		err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
		if (err) {
			printk("BIG Sync failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			printk("done.\n");

			goto per_sync_lost_check;
		}

		err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
		if (err) {
			printk("BIG Sync Lost failed (err %d)\n", err);
			return;
		}

per_sync_lost_check:
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			goto big_sync_create;
		}
	} while (true);
}