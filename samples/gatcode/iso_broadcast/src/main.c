#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
#include <io_coder.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* D-Cube Defines */
/* ------------------------------------------------------ */
#define REMOTE true
#define SENDER_START_DELAY_MS 25000

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)

/* ------------------------------------------------------ */
/* Defines Sender */
/* ------------------------------------------------------ */
#define SDU_INTERVAL_US 10000 // 5ms min due to ISO_Interval must be multiple of 1.25ms && > NSE * Sub_Interval
#define TRANSPORT_LATENCY_MS 10 // 5ms-4s
#define RETRANSMISSION_NUMBER 0

/* ------------------------------------------------------ */
/* Start */
/* ------------------------------------------------------ */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
			  BT_ISO_SDU_BUF_SIZE(DATA_SIZE_BYTE), 8, NULL);

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

static struct bt_iso_chan bis_iso_chan;

uint32_t iso_send_count = 0;
uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
struct net_buf *buf;
uint32_t last_send_ts;

void send_packet_handler(struct k_timer *dummy)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(iso_send_count++, iso_data);
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	printk("Sending value %u\n", iso_send_count);
	int err = write_8_bit(&io_encoder, iso_send_count % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}
K_TIMER_DEFINE(send_packet, send_packet_handler, NULL);

static void iso_sent(struct bt_iso_chan *chan)
{
	uint32_t curr_send_ts = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
	uint32_t diff = curr_send_ts - last_send_ts;
	uint32_t wait_time = 2 * SDU_INTERVAL_US > diff ? 2 * SDU_INTERVAL_US - diff : 0;
	k_timer_start(&send_packet, K_USEC(wait_time), K_NO_WAIT);
	last_send_ts = curr_send_ts;
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE, /* bytes */
	.rtn = RETRANSMISSION_NUMBER,
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
	.interval = SDU_INTERVAL_US, /* in microseconds */
	.latency = TRANSPORT_LATENCY_MS, /* milliseconds */
	.packing = BT_ISO_PACKING_SEQUENTIAL, /* 0 - sequential, 1 - interleaved */
	.framing = BT_ISO_FRAMING_UNFRAMED, /* 0 - unframed, 1 - framed */
};

void main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	printk("Starting ISO Broadcast Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
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

	if(REMOTE) {
		k_sleep(K_MSEC(SENDER_START_DELAY_MS));
	} else {
		k_sleep(K_MSEC(5000));
	}

	last_send_ts = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
	iso_sent(&bis_iso_chan);
}
