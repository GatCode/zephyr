/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/types.h>

#include <zephyr/console/console.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(iso_connected, LOG_LEVEL_DBG);

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME))

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <nrfx_timer.h>
#include <nrfx_dppi.h>
#include <nrfx_ipc.h>

#include <zephyr/drivers/gpio.h>

#define AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL 1
#define AUDIO_PRESENTATION_TIMER_CURR_TIME_CAPTURE_CHANNEL 2

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

const nrfx_timer_t audio_sync_timer_instance =
	NRFX_TIMER_INSTANCE(1);
const nrfx_timer_t audio_presentation_timer_instance =
	NRFX_TIMER_INSTANCE(2);

static uint8_t dppi_channel_timer_clear;

static nrfx_timer_config_t cfg = {
	.frequency = (uint32_t)1000000U,
	.mode = NRF_TIMER_MODE_TIMER,
	.bit_width = NRF_TIMER_BIT_WIDTH_32,
	.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
	.p_context = NULL
};

static void event_handler(nrf_timer_event_t event_type, void *ctx)
{
	nrfx_timer_clear(&audio_presentation_timer_instance);
	nrfx_timer_disable(&audio_presentation_timer_instance);

	gpio_pin_toggle_dt(&led);
}

enum benchmark_role {
	ROLE_CENTRAL,
	ROLE_PERIPHERAL,
	ROLE_QUIT
};

#define DEFAULT_CIS_RTN         0
#define DEFAULT_CIS_INTERVAL_US 5000
#define DEFAULT_CIS_LATENCY_MS  BT_ISO_LATENCY_MIN // = 5ms
#define DEFAULT_CIS_PHY         BT_GAP_LE_PHY_2M
#define DEFAULT_CIS_SDU_SIZE    CONFIG_BT_ISO_TX_MTU
#define DEFAULT_CIS_PACKING     0
#define DEFAULT_CIS_FRAMING     0
#define DEFAULT_CIS_COUNT       CONFIG_BT_ISO_MAX_CHAN
#define DEFAULT_CIS_SEC_LEVEL   BT_SECURITY_L1

#define BUFFERS_ENQUEUED 2 /* Number of buffers enqueue for each channel */

BUILD_ASSERT(BUFFERS_ENQUEUED * CONFIG_BT_ISO_MAX_CHAN <= CONFIG_BT_ISO_TX_BUF_COUNT,
	     "Not enough buffers to enqueue");

struct iso_recv_stats {
	uint32_t iso_recv_count;
	uint32_t iso_lost_count;
};

struct iso_chan_work {
	struct bt_iso_chan chan;
	struct k_work_delayable send_work;
	struct bt_iso_info info;
	uint16_t seq_num;
} iso_chans[CONFIG_BT_ISO_MAX_CHAN];

static enum benchmark_role role;
static struct bt_conn *default_conn;
static struct bt_iso_chan *cis[CONFIG_BT_ISO_MAX_CHAN];
static bool advertiser_found;
static bt_addr_le_t adv_addr;
static uint32_t last_received_counter;
static struct iso_recv_stats stats_current_conn;
static struct iso_recv_stats stats_overall;
static int64_t iso_conn_start_time;
static size_t total_iso_conn_count;
static uint32_t iso_send_count;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU];

static K_SEM_DEFINE(sem_adv, 0, 1);
static K_SEM_DEFINE(sem_iso_accept, 0, 1);
static K_SEM_DEFINE(sem_iso_connected, 0, CONFIG_BT_ISO_MAX_CHAN);
static K_SEM_DEFINE(sem_iso_disconnected, 0, CONFIG_BT_ISO_MAX_CHAN);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

static int audio_sync_timer_init(void)
{
	nrfx_err_t ret;

	ret = nrfx_timer_init(&audio_sync_timer_instance, &cfg, NULL);
	if (ret - NRFX_ERROR_BASE_NUM) {
		printk("nrfx timer init error - Return value: %d\n", ret);
		return ret;
	}

	ret = nrfx_timer_init(&audio_presentation_timer_instance, &cfg, event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		printk("nrfx timer init error - Return value: %d\n", ret);
		return ret;
	}

	nrfx_timer_enable(&audio_sync_timer_instance);
	// nrfx_timer_enable(&audio_presentation_timer_instance);
	IRQ_DIRECT_CONNECT(TIMER2_IRQn, 0, nrfx_timer_2_irq_handler, 0);
	irq_enable(TIMER2_IRQn);

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		printk("nrfx DPPI channel alloc error (timer clear) - Return value: %d\n", ret);
		return ret;
	}

	nrf_ipc_publish_set(NRF_IPC, NRF_IPC_EVENT_RECEIVE_4, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(audio_sync_timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		printk("nrfx DPPI channel enable error (timer clear) - Return value: %d\n", ret);
		return ret;
	}

	printk("Audio sync timer initialized\n");

	return 0;
}

SYS_INIT(audio_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DEFAULT_CIS_SDU_SIZE, /* bytes */
	.rtn = DEFAULT_CIS_RTN,
	.phy = DEFAULT_CIS_PHY,
};

static struct bt_iso_chan_io_qos iso_rx_qos = {
	.sdu = DEFAULT_CIS_SDU_SIZE, /* bytes */
	.rtn = DEFAULT_CIS_RTN,
	.phy = DEFAULT_CIS_PHY,
};

static struct bt_iso_chan_qos iso_qos = {
	.tx = &iso_tx_qos,
	.rx = &iso_rx_qos,
};

static struct bt_iso_cig_param cig_create_param = {
	.interval = DEFAULT_CIS_INTERVAL_US, /* in microseconds */
	.latency = DEFAULT_CIS_LATENCY_MS, /* milliseconds */
	.sca = BT_GAP_SCA_UNKNOWN,
	.packing = DEFAULT_CIS_PACKING,
	.framing = DEFAULT_CIS_FRAMING,
	.cis_channels = cis,
	.num_cis = DEFAULT_CIS_COUNT
};

static void print_stats(char *name, struct iso_recv_stats *stats)
{
	uint32_t total_packets;

	total_packets = stats->iso_recv_count + stats->iso_lost_count;

	LOG_INF("%s: Received %u/%u (%.2f%%) - Total packets lost %u",
		name, stats->iso_recv_count, total_packets,
		(float)stats->iso_recv_count * 100 / total_packets,
		stats->iso_lost_count);
}

static void iso_send(struct bt_iso_chan *chan)
{
	int ret;
	struct net_buf *buf;
	struct iso_chan_work *chan_work;

	chan_work = CONTAINER_OF(chan, struct iso_chan_work, chan);

	if (!chan_work->info.can_send) {
		return;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	if (buf == NULL) {
		LOG_ERR("Could not allocate buffer");
		k_work_reschedule(&chan_work->send_work, K_USEC(cig_create_param.interval));
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, iso_data, iso_tx_qos.sdu);

	ret = bt_iso_chan_send(chan, buf, chan_work->seq_num++,
			       BT_ISO_TIMESTAMP_NONE);
	if (ret < 0) {
		LOG_ERR("Unable to send data: %d", ret);
		net_buf_unref(buf);
		k_work_reschedule(&chan_work->send_work, K_USEC(cig_create_param.interval));
		return;
	}

	iso_send_count++;

	// if ((iso_send_count % 100) == 0) {
	// 	LOG_INF("Sending value %u", iso_send_count);
	// }
}

static void iso_timer_timeout(struct k_work *work)
{
	iso_send(&iso_chans[0].chan);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	struct iso_chan_work *chan_work;

	chan_work = CONTAINER_OF(chan, struct iso_chan_work, chan);

	k_work_reschedule(&chan_work->send_work, K_MSEC(0));
}

static void iso_recv(struct bt_iso_chan *chan,
		     const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	// LOG_INF("Received %u - %u %u %u %u!", info->seq_num, buf->data[10], buf->data[11], buf->data[12], buf->data[13]);

	uint32_t total_packets;
	static bool stats_latest_arr[200];
	static size_t stats_latest_arr_pos;

	// if (info->flags & BT_ISO_FLAGS_VALID) {
	// 	stats_current_conn.iso_recv_count++;
	// 	stats_overall.iso_recv_count++;
	// 	stats_latest_arr[stats_latest_arr_pos++] = true;
	// } else {
	// 	stats_current_conn.iso_lost_count++;
	// 	stats_overall.iso_lost_count++;
	// 	stats_latest_arr[stats_latest_arr_pos++] = false;
	// }

	if (buf->data[10] == 1) {
		stats_current_conn.iso_recv_count++;
		stats_overall.iso_recv_count++;
		stats_latest_arr[stats_latest_arr_pos++] = true;
	} else {
		stats_current_conn.iso_lost_count++;
		stats_overall.iso_lost_count++;
		stats_latest_arr[stats_latest_arr_pos++] = false;
	}

	if (stats_latest_arr_pos == sizeof(stats_latest_arr)) {
		stats_latest_arr_pos = 0;
	}

	total_packets = stats_overall.iso_recv_count + stats_overall.iso_lost_count;

	if ((info->seq_num % 100) == 0) {
		struct iso_recv_stats stats_latest = { 0 };

		for (int i = 0; i < ARRAY_SIZE(stats_latest_arr); i++) {
			/* If we have not yet received 100 packets, break
			 * early
			 */
			if (i == total_packets) {
				break;
			}

			if (stats_latest_arr[i]) {
				stats_latest.iso_recv_count++;
			} else {
				stats_latest.iso_lost_count++;
			}
		}

		LOG_INF("Data len    : %u byte - buffer full of: %u%u%u%u...", buf->len, buf->data[10], buf->data[11], buf->data[12], buf->data[13]);
		print_stats("Overall     ", &stats_overall);
		print_stats("Current Conn", &stats_current_conn);
		print_stats("Latest 200  ", &stats_latest); // 200 = 1s
		LOG_INF("Throughput  : %u kbps", stats_latest.iso_recv_count * buf->len * 8);

		uint32_t recv_frame_ts = nrfx_timer_capture(&audio_sync_timer_instance,
						    AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
		LOG_INF("Netcore SDU Sync Reference: %u | Delta Sync Ref / APP core: %u", info->ts, recv_frame_ts);
		LOG_INF(""); /* Empty line to separate the stats */

		nrfx_timer_compare(&audio_presentation_timer_instance, NRF_TIMER_CC_CHANNEL1, 1000 - recv_frame_ts, true);
		nrfx_timer_enable(&audio_presentation_timer_instance);
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	struct iso_chan_work *chan_work;
	int err;

	LOG_INF("ISO Channel %p connected", chan);

	chan_work = CONTAINER_OF(chan, struct iso_chan_work, chan);
	err = bt_iso_chan_get_info(chan, &chan_work->info);
	if (err != 0) {
		LOG_ERR("Could get info about chan %p: %d", chan, err);
	}

	/* If multiple CIS was created, this will be the value of the last
	 * created in the CIG
	 */
	iso_conn_start_time = k_uptime_get();

	chan_work = CONTAINER_OF(chan, struct iso_chan_work, chan);
	chan_work->seq_num = 0U;

	k_sem_give(&sem_iso_connected);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	/* Calculate cumulative moving average - Be aware that this may
	 * cause overflow at sufficiently large counts or durations
	 *
	 * This duration is calculated for each CIS disconnected from the time
	 * of the last created CIS.
	 */
	static int64_t average_duration;
	uint64_t iso_conn_duration;
	uint64_t total_duration;

	if (iso_conn_start_time > 0) {
		iso_conn_duration = k_uptime_get() - iso_conn_start_time;
	} else {
		iso_conn_duration = 0;
	}
	total_duration = iso_conn_duration + (total_iso_conn_count - 1) * average_duration;

	average_duration = total_duration / total_iso_conn_count;

	LOG_INF("ISO Channel %p disconnected with reason 0x%02x after "
		"%llu milliseconds (average duration %llu)",
		chan, reason, iso_conn_duration, average_duration);

	k_sem_give(&sem_iso_disconnected);
}

static struct bt_iso_chan_ops iso_ops = {
	.recv          = iso_recv,
	.connected     = iso_connected,
	.disconnected  = iso_disconnected,
	.sent          = iso_sent,
};

static int iso_accept(const struct bt_iso_accept_info *info,
		      struct bt_iso_chan **chan)
{
	LOG_INF("Incoming ISO request from %p", (void *)info->acl);

	for (int i = 0; i < ARRAY_SIZE(iso_chans); i++) {
		if (iso_chans[i].chan.state == BT_ISO_STATE_DISCONNECTED) {
			LOG_INF("Returning instance %d", i);
			*chan = &iso_chans[i].chan;
			cig_create_param.num_cis++;

			k_sem_give(&sem_iso_accept);
			return 0;
		}
	}

	LOG_ERR("Could not accept any more CIS");

	*chan = NULL;

	return -ENOMEM;
}

static struct bt_iso_server iso_server = {
#if defined(CONFIG_BT_SMP)
	.sec_level = DEFAULT_CIS_SEC_LEVEL,
#endif /* CONFIG_BT_SMP */
	.accept = iso_accept,
};

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
		__fallthrough;
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, DEVICE_NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[DEVICE_NAME_LEN];

	if (advertiser_found) {
		return;
	}

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	if (strncmp(DEVICE_NAME, name, strlen(DEVICE_NAME))) {
		return;
	}

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	LOG_INF("Found peripheral with address %s (RSSI %i)",
		le_addr, info->rssi);


	bt_addr_le_copy(&adv_addr, info->addr);
	advertiser_found = true;
	k_sem_give(&sem_adv);
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0 && role == ROLE_CENTRAL) {
		LOG_INF("Failed to connect to %s: %u", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	} else if (role == ROLE_PERIPHERAL) {
		default_conn = bt_conn_ref(conn);
	}

	LOG_INF("Connected: %s", addr);

	k_sem_give(&sem_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	k_sem_give(&sem_disconnected);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void reset_sems(void)
{
	(void)k_sem_reset(&sem_adv);
	(void)k_sem_reset(&sem_iso_accept);
	(void)k_sem_reset(&sem_iso_connected);
	(void)k_sem_reset(&sem_iso_disconnected);
	(void)k_sem_reset(&sem_connected);
	(void)k_sem_reset(&sem_disconnected);
}

static int run_peripheral(void)
{
	int err;
	static bool initialized;

	/* Reset */
	cig_create_param.num_cis = 0;
	iso_conn_start_time = 0;
	last_received_counter = 0;
	memset(&stats_current_conn, 0, sizeof(stats_current_conn));
	reset_sems();

	if (!initialized) {
		LOG_INF("Registering ISO server");
		err = bt_iso_server_register(&iso_server);
		if (err != 0) {
			LOG_ERR("ISO server register failed: %d", err);
			return err;
		}
		initialized = true;
	}

	LOG_INF("Starting advertising");
	err = bt_le_adv_start(
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_CONNECTABLE |
					BT_LE_ADV_OPT_USE_NAME |
					BT_LE_ADV_OPT_FORCE_NAME_IN_AD,
				BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL),
		NULL, 0, NULL, 0);
	if (err != 0) {
		LOG_ERR("Advertising failed to start: %d", err);
		return err;
	}

	LOG_INF("Waiting for ACL connection");
	err = k_sem_take(&sem_connected, K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to take sem_connected: %d", err);
		return err;
	}

	err = bt_le_adv_stop();
	if (err != 0) {
		LOG_ERR("Advertising failed to stop: %d", err);
		return err;
	}

	LOG_INF("Waiting for ISO connection");

	err = k_sem_take(&sem_iso_accept, K_SECONDS(2));
	if (err != 0) {
		return err;
	}

	LOG_INF("ISO accepted");

	for (int i = 0; i < cig_create_param.num_cis; i++) {
		err = k_sem_take(&sem_iso_connected, K_FOREVER);
		if (err != 0) {
			LOG_ERR("failed to take sem_iso_connected: %d", err);
			return err;
		}
	}
	total_iso_conn_count++;

	for (size_t i = 0; i < cig_create_param.num_cis; i++) {
		struct k_work_delayable *work = &iso_chans[i].send_work;
		k_work_init_delayable(work, iso_timer_timeout);
		iso_send(&iso_chans[0].chan);
	}

	/* Wait for disconnect */
	err = k_sem_take(&sem_disconnected, K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to take sem_disconnected: %d", err);
		return err;
	}

	for (int i = 0; i < cig_create_param.num_cis; i++) {
		err = k_sem_take(&sem_iso_disconnected, K_FOREVER);
		if (err != 0) {
			LOG_ERR("failed to take sem_iso_disconnected: %d", err);
			return err;
		}
	}

	LOG_INF("Disconnected - Cleaning up");
	for (size_t i = 0; i < cig_create_param.num_cis; i++) {
		(void)k_work_cancel_delayable(&iso_chans[i].send_work);
	}

	while(1) {
		k_sleep(K_MSEC(100));
	}

	return 0;
}

int main(void)
{
	int err;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return 0;
	}

	LOG_INF("Starting Bluetooth Throughput example");

	err = bt_enable(NULL);
	if (err != 0) {
		LOG_ERR("Bluetooth init failed: %d", err);
		return 0;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_le_scan_cb_register(&scan_callbacks);

	err = console_init();
	if (err != 0) {
		LOG_ERR("Console init failed: %d", err);
		return 0;
	}

	LOG_INF("Bluetooth initialized");

	for (int i = 0; i < ARRAY_SIZE(iso_chans); i++) {
		iso_chans[i].chan.ops = &iso_ops;
		iso_chans[i].chan.qos = &iso_qos;
		cis[i] = &iso_chans[i].chan;
	}

	/* Init data */
	for (int i = 0; i < iso_tx_qos.sdu; i++) {
		if (i < sizeof(iso_send_count)) {
			continue;
		}
		iso_data[i] = (uint8_t)2;
	}

	err = run_peripheral();
	if (err != 0) {
		LOG_ERR("Running the peripheral failed: %d", err);
		return 0;
	}

	while(1) {
		k_sleep(K_MSEC(10));
	}

	LOG_INF("Exiting");
	return 0;
}
