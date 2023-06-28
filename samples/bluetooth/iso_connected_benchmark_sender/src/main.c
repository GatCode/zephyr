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

#include <zephyr/drivers/gpio.h>

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME))

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

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

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
static struct bt_iso_cig *cig;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU];

static K_SEM_DEFINE(sem_adv, 0, 1);
static K_SEM_DEFINE(sem_iso_accept, 0, 1);
static K_SEM_DEFINE(sem_iso_connected, 0, CONFIG_BT_ISO_MAX_CHAN);
static K_SEM_DEFINE(sem_iso_disconnected, 0, CONFIG_BT_ISO_MAX_CHAN);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

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

	if (((chan_work->seq_num + 1) % 100) == 0) {
		gpio_pin_toggle_dt(&led);
	}

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
	struct bt_iso_chan *chan;
	struct iso_chan_work *chan_work;
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);

	chan_work = CONTAINER_OF(delayable, struct iso_chan_work, send_work);
	chan = &chan_work->chan;

	iso_send(chan);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	struct iso_chan_work *chan_work;

	// LOG_INF("Sent value %u", iso_send_count);

	chan_work = CONTAINER_OF(chan, struct iso_chan_work, chan);

	k_work_reschedule(&chan_work->send_work, K_MSEC(0));
}

static void iso_recv(struct bt_iso_chan *chan,
		     const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	uint32_t total_packets;
	static bool stats_latest_arr[200];
	static size_t stats_latest_arr_pos;

	/* NOTE: The packets received may be on different CISes */

	// if (info->flags & BT_ISO_FLAGS_VALID) {
	// 	stats_current_conn.iso_recv_count++;
	// 	stats_overall.iso_recv_count++;
	// 	stats_latest_arr[stats_latest_arr_pos++] = true;
	// } else {
	// 	stats_current_conn.iso_lost_count++;
	// 	stats_overall.iso_lost_count++;
	// 	stats_latest_arr[stats_latest_arr_pos++] = false;
	// }

	if (buf->data[10] == 2) {
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

	// LOG_INF("Received %u - %u %u %u %u!", info->seq_num, buf->data[10], buf->data[11], buf->data[12], buf->data[13]);

	if ((total_packets % 200) == 0) {
		struct iso_recv_stats stats_latest = { 0 };

		for (int i = 0; i < ARRAY_SIZE(stats_latest_arr); i++) {
			/* If we have not yet received 1000 packets, break
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
		LOG_INF(""); /* Empty line to separate the stats */
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

static int start_scan(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err != 0) {
		LOG_ERR("Scan start failed: %d", err);
		return err;
	}

	LOG_INF("Scan started");

	return 0;
}

static int stop_scan(void)
{
	int err;

	err = bt_le_scan_stop();
	if (err != 0) {
		LOG_ERR("Scan stop failed: %d", err);
		return err;
	}

	LOG_INF("Scan stopped");

	return 0;
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

static int central_create_connection(void)
{
	int err;

	advertiser_found = false;

	err = start_scan();
	if (err != 0) {
		LOG_ERR("Could not start scan: %d", err);
		return err;
	}

	LOG_INF("Waiting for advertiser");
	err = k_sem_take(&sem_adv, K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to take sem_adv: %d", err);
		return err;
	}

	LOG_INF("Stopping scan");
	err = stop_scan();
	if (err != 0) {
		LOG_ERR("Could not stop scan: %d", err);
		return err;
	}

	LOG_INF("Connecting");
	err = bt_conn_le_create(&adv_addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err != 0) {
		LOG_ERR("Create connection failed: %d", err);
		return err;
	}

	err = k_sem_take(&sem_connected, K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to take sem_connected: %d", err);
		return err;
	}

	return 0;
}

static int central_create_cig(void)
{
	struct bt_iso_connect_param connect_param[CONFIG_BT_ISO_MAX_CHAN];
	int err;

	iso_conn_start_time = 0;

	LOG_INF("Creating CIG");

	err = bt_iso_cig_create(&cig_create_param, &cig);
	if (err != 0) {
		LOG_ERR("Failed to create CIG: %d", err);
		return err;
	}

	LOG_INF("Connecting ISO channels");

	for (int i = 0; i < cig_create_param.num_cis; i++) {
		connect_param[i].acl = default_conn;
		connect_param[i].iso_chan = &iso_chans[i].chan;
	}

	err = bt_iso_chan_connect(connect_param, cig_create_param.num_cis);
	if (err != 0) {
		LOG_ERR("Failed to connect iso: %d", err);
		return err;
	}
	total_iso_conn_count++;

	for (int i = 0; i < cig_create_param.num_cis; i++) {
		err = k_sem_take(&sem_iso_connected, K_FOREVER);
		if (err != 0) {
			LOG_ERR("failed to take sem_iso_connected: %d", err);
			return err;
		}
	}

	return 0;
}

static void reset_sems(void)
{
	(void)k_sem_reset(&sem_adv);
	(void)k_sem_reset(&sem_iso_accept);
	(void)k_sem_reset(&sem_iso_connected);
	(void)k_sem_reset(&sem_iso_disconnected);
	(void)k_sem_reset(&sem_connected);
	(void)k_sem_reset(&sem_disconnected);
}

static int run_central(void)
{
	int err;

	iso_conn_start_time = 0;
	last_received_counter = 0;
	memset(&stats_current_conn, 0, sizeof(stats_current_conn));
	reset_sems();

	err = central_create_connection();
	if (err != 0) {
		LOG_ERR("Failed to create connection: %d", err);
		return err;
	}

	err = central_create_cig();
	if (err != 0) {
		LOG_ERR("Failed to create CIG or connect CISes: %d", err);
		return err;
	}
	LOG_INF("CIG created.");


	for (size_t i = 0; i < cig_create_param.num_cis; i++) {
		struct k_work_delayable *work = &iso_chans[i].send_work;

		k_work_init_delayable(work, iso_timer_timeout);

		for (int j = 0; j < BUFFERS_ENQUEUED; j++) {
			iso_send(&iso_chans[i].chan);
		}
	}

	err = k_sem_take(&sem_disconnected, K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to take sem_disconnected: %d", err);
		return err;
	}

	LOG_INF("Disconnected - Cleaning up");
	for (size_t i = 0; i < cig_create_param.num_cis; i++) {
		(void)k_work_cancel_delayable(&iso_chans[i].send_work);
	}

	for (int i = 0; i < cig_create_param.num_cis; i++) {
		err = k_sem_take(&sem_iso_disconnected, K_FOREVER);
		if (err != 0) {
			LOG_ERR("failed to take sem_iso_disconnected: %d", err);
			return err;
		}
	}

	err = bt_iso_cig_terminate(cig);
	if (err != 0) {
		LOG_ERR("Could not terminate CIG: %d", err);
		return err;
	}
	cig = NULL;

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
		iso_data[i] = (uint8_t)1;
	}

	err = run_central();
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
