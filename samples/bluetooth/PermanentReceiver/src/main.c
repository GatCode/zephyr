#include <bluetooth/bluetooth.h>
#include <device.h>
#include <drivers/counter.h>
#include <drivers/gpio.h>

#include <drivers/timer/nrf_rtc_timer.h>
#include <hal/nrf_rtc.h>
#include <hal/nrf_timer.h>

#define TIMER DT_LABEL(DT_NODELABEL(timer2))
static const struct device *counter_dev;
static uint64_t rtc_offset = 0;

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

void start_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	counter_start(counter_dev);
	rtc_offset = z_nrf_rtc_timer_read();
}

void status_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	uint32_t ticks = 0;
	counter_get_value(counter_dev, &ticks);
	// printk("Button pressed at %u\n", ticks);
	printk("Button pressed at %llu\n", z_nrf_rtc_timer_read() - rtc_offset);
	printk("Button pressed at tticks %.2f ms\n", k_ticks_to_us_floor64(z_nrf_rtc_timer_read() - rtc_offset) / 1000.0);


	// printk("Freq: %u\n", counter_get_frequency(counter_dev));
}

struct recv_stat {
	uint64_t recv;
	uint64_t lost;
	uint64_t last_packet_id;
	double avg_latency;
	uint64_t n_avg;
};

static struct recv_stat statistic;

static void print_stat(char *message, struct recv_stat *stat)
{
	uint64_t total_packets = stat->recv + stat->lost;

	if(total_packets % 100 == 0) {
		printk("%s: Received %llu/%llu (%.2f%%) - Total packets lost %llu - moving avg latency %.2f ms\n",
			message, stat->recv, total_packets,
			(float)stat->recv * 100 / total_packets, stat->lost, stat->avg_latency);
	}
}

struct adv_payload {
	uint8_t id[8];
	uint8_t timestamp[8];
};

static bool data_cb(struct bt_data *data, void *user_data)
{
	switch (data->type) {
	case BT_DATA_MANUFACTURER_DATA:
		memcpy(user_data, data->data, sizeof(struct adv_payload));
		return false;
	default:
		return true;
	}
}

static uint32_t last_timestamp = 0;
// #define ABS(X) ((X < 0) ? (-X) : (X))

void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	if(adv_type == BT_GAP_ADV_TYPE_EXT_ADV) {

		uint32_t timestamp = 0;
		counter_get_value(counter_dev, &timestamp);
		last_timestamp = timestamp;

		struct adv_payload payload[sizeof(struct adv_payload)];
		bt_data_parse(buf, data_cb, payload);

		uint64_t sender_packet_id =
		(uint64_t)payload->id[0] << 0 * 8 |
		(uint64_t)payload->id[1] << 1 * 8 |
		(uint64_t)payload->id[2] << 2 * 8 |
		(uint64_t)payload->id[3] << 3 * 8 |
		(uint64_t)payload->id[4] << 4 * 8 |
		(uint64_t)payload->id[5] << 5 * 8 |
		(uint64_t)payload->id[6] << 6 * 8 |
		((uint64_t)payload->id[7] << 7 * 8);

		uint64_t sender_timestamp =
		(uint64_t)payload->timestamp[0] << 0 * 8 |
		(uint64_t)payload->timestamp[1] << 1 * 8 |
		(uint64_t)payload->timestamp[2] << 2 * 8 |
		(uint64_t)payload->timestamp[3] << 3 * 8 |
		(uint64_t)payload->timestamp[4] << 4 * 8 |
		(uint64_t)payload->timestamp[5] << 5 * 8 |
		(uint64_t)payload->timestamp[6] << 6 * 8 |
		((uint64_t)payload->timestamp[7] << 7 * 8);


		// printk("Received Packet %u ticks\n", sender_timestamp);

		uint64_t lost_amount = 0;
		if(sender_packet_id != statistic.last_packet_id + 1) {
			lost_amount = sender_packet_id - statistic.last_packet_id;
			statistic.lost += lost_amount;
		} else {
			statistic.recv++;

			// uint32_t timestamp = 0;
			// counter_get_value(counter_dev, &timestamp);
			// last_timestamp = timestamp;

			
		}

		// printk("Received Packet number %llu at TS: %u with TS: %u\n", sender_packet_id, last_timestamp, sender_timestamp);
		// printk("   - diff: %u - internal ID: %llu\n", last_timestamp - sender_timestamp, statistic.recv + statistic.lost);

		uint64_t curr_timestamp = z_nrf_rtc_timer_read() - rtc_offset;
		double latency_ms = k_ticks_to_us_floor64(curr_timestamp - sender_timestamp) / 1000.0;
		// printk("Latency: %.2f ms\n", k_ticks_to_us_floor64(curr_timestamp - sender_timestamp) / 1000.0);

		// uint64_t counter_us = ((uint64_t)(last_timestamp - sender_timestamp) / CYC_PER_TICK * USEC_PER_SEC) / z_impl_counter_get_frequency(counter_dev);
		// printk("Recv Ticks: %u ticks - %llu us - %.2f ms\n", last_timestamp, counter_us, counter_us / 1000000.0);

		// statistic.avg_latency = ABS(sender_timestamp - last_timestamp);
		

		statistic.avg_latency = latency_ms;//(statistic.avg_latency * statistic.n_avg + latency_ms) / (statistic.n_avg + 1);
		// statistic.n_avg++;

		print_stat("Statistic: ", &statistic);
		statistic.last_packet_id = sender_packet_id;
	}
}

void main(void)
{
	statistic.recv = 0;
	statistic.lost = 0;
	statistic.last_packet_id = 0;
	statistic.avg_latency = 0.0;
	statistic.n_avg = 1;

	int err;

	counter_dev = device_get_binding(TIMER);
	if (counter_dev == NULL) {
		printk("Device not found\n");
		return;
	}

	/* Initialize the Buttons */
	if (!device_is_ready(button0.port) || !device_is_ready(button1.port)) {
		printk("Error: button device is not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", err, button1.port->name, button1.pin);
		return;
	}

	gpio_init_callback(&button0_cb_data, start_button_pressed, BIT(button0.pin));
	gpio_init_callback(&button1_cb_data, status_button_pressed, BIT(button1.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	gpio_add_callback(button1.port, &button1_cb_data);
	
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	struct bt_le_scan_param param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M | BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0c40  /* 40 ms / 2s */,
        .window = 0x0c40 /* 40 / 2s */,
		.timeout = 0,
		.interval_coded = 0,
        .window_coded = 0,
    };

	err = bt_le_scan_start(&param, &scan_cb);
	if (err) {
		printk("Bluetooth scan start failed (err %d)\n", err);
		return;
	}
}
