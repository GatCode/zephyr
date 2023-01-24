/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

static bool         per_adv_found;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

K_THREAD_STACK_DEFINE(threadA_stack_area, STACKSIZE);
static struct k_thread threadA_data;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define HAS_LED     1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool                  led_is_on;
extern uint8_t sync_chan_idx;
extern uint16_t sync_ival_us;
extern uint16_t sync_event_counter;
extern uint16_t sync_skip_event;

extern uint8_t lll_chan_sel_2_custom(uint16_t counter, uint16_t chan_id);
extern uint8_t ll_test_tx(uint8_t chan, uint8_t len, uint8_t type, uint8_t phy,
		   uint8_t cte_len, uint8_t cte_type, uint8_t switch_pattern_len,
		   const uint8_t *ant_id, int8_t tx_power);

uint8_t sync_chan_idx = 255;
uint16_t sync_ival_us = 0;
uint16_t sync_event_counter = 0;
uint16_t sync_skip_event = 0;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}
#endif

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	// printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i %s "
	//        "C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
	//        "Interval: 0x%04x (%u ms), SID: %u\n",
	//        le_addr, info->adv_type, info->tx_power, info->rssi, name,
	//        (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	//        (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	//        phy2str(info->primary_phy), phy2str(info->secondary_phy),
	//        info->interval, info->interval * 5 / 4, info->sid);

	if (!per_adv_found && info->interval) {
		per_adv_found = true;

		per_sid = info->sid;
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
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	// printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	//        "Interval 0x%04x (%u ms), PHY %s\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr,
	//        info->interval, info->interval * 5 / 4, phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);

	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char data_str[129];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	       "RSSI %i, CTE %u, data length %u, data: %s\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
	       info->rssi, info->cte_type, buf->len, data_str);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb
};

struct k_poll_signal signal;

struct k_poll_event sync_stablished = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY, &signal);

void my_work_handler(struct k_work *work)
{
    printk("Send DTM on channel %u\n", sync_chan_idx);

	// send
	struct bt_hci_cp_le_tx_test *cp;
	struct net_buf *buf;

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_TX_TEST, sizeof(*cp));
	if (!buf) {
		return -ENOBUFS;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->tx_ch = sync_chan_idx;
	cp->test_data_len = 255;
	cp->pkt_payload = BT_HCI_TEST_PKT_PAYLOAD_01010101;
	int r_val = bt_hci_cmd_send(BT_HCI_OP_LE_TX_TEST, buf);
	// printk("%d\n", r_val);

	// ll_test_tx(sync_chan_idx, 255, BT_HCI_TEST_PKT_PAYLOAD_01010101, BT_HCI_LE_TX_PHY_2M, BT_HCI_LE_TEST_CTE_DISABLED, BT_HCI_LE_TEST_CTE_TYPE_ANY, BT_HCI_LE_TEST_SWITCH_PATTERN_LEN_ANY, NULL, BT_HCI_TX_TEST_POWER_MAX);

	// calc the next one
	sync_event_counter++;
	sync_chan_idx = lll_chan_sel_2_custom(sync_event_counter + sync_skip_event, sync_chan_idx);
}

K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&my_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

void threadA(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while(1) {
		if (sync_stablished.signal->signaled) {
			printk("sync_stablished.signal->signaled - ival: %u us\n", sync_ival_us); // TODO: Sync invercal is off - 1.25
			// k_timer_start(&my_timer, K_NO_WAIT, K_USEC(1000));
			while(1) {
				

				struct bt_hci_cp_le_tx_test *cp;
				struct net_buf *buf;

				buf = bt_hci_cmd_create(BT_HCI_OP_LE_TX_TEST, sizeof(*cp));
				if (!buf) {
					return -ENOBUFS;
				}

				cp = net_buf_add(buf, sizeof(*cp));
				cp->tx_ch = sync_chan_idx;
				cp->test_data_len = 255;
				cp->pkt_payload = BT_HCI_TEST_PKT_PAYLOAD_01010101;
				int r_val = bt_hci_cmd_send(BT_HCI_OP_LE_TX_TEST, buf);
				
				sync_event_counter++;
				sync_chan_idx = lll_chan_sel_2_custom(sync_event_counter + sync_skip_event, sync_chan_idx);

				k_sleep(K_MSEC(1000));
			}
		}
		k_sleep(K_USEC(1));
	}
}


void main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	int err;

	printk("Starting Periodic Advertising Synchronization Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	printk("Start scanning...");
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("success.\n");

	k_poll_signal_init(&signal);

	k_thread_create(&threadA_data, threadA_stack_area,
			K_THREAD_STACK_SIZEOF(threadA_stack_area),
			threadA, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&threadA_data, "thread_a");

	k_thread_start(&threadA_data);

	do {
		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("Found periodic advertising.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = 0xaa;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, TIMEOUT_SYNC_CREATE);
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

		printk("Deleting Periodic Advertising Sync...");
		err = bt_le_per_adv_sync_delete(sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}

		while(1) {
			// if(sync_chan_idx <= 40) {
			// 	k_timer_start(&my_timer, K_NO_WAIT, K_MSEC(sync_ival_ms));

			// 	err = bt_le_per_adv_sync_delete(sync);
			// 	if (err) {
			// 		printk("failed (err %d)\n", err);
			// 		return;
			// 	}
			// }
			k_sleep(K_MSEC(1000));
		}

		// printk("Waiting for periodic sync lost...\n");
		// err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		// if (err) {
		// 	printk("failed (err %d)\n", err);
		// 	return;
		// }
		// printk("Periodic sync lost.\n");
	} while (true);
}
