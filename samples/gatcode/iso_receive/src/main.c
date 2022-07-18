#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
#include <io_coder.h>
#include <stdlib.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)

#define PDR_WATCHDOG_FREQ_MS 1000
#define FIFO_SIZE 10

/* ------------------------------------------------------ */
/* Defines Receiver */
/* ------------------------------------------------------ */
#define PRESENTATION_DELAY_US 10000
#define MAXIMUM_SUBEVENTS 31 // MSE | 1-31

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static double pdr = 0.0;
static uint16_t iso_interval = 0;

/* ------------------------------------------------------ */
/* Start */
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

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	// char le_addr[BT_ADDR_LE_STR_LEN];

	// bt_addr_le_to_str(biginfo->addr, le_addr, sizeof(le_addr));

	// printk("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
	//        "num_bis %u, nse %u, interval 0x%04x (%u ms), "
	//        "bn %u, pto %u, irc %u, max_pdu %u, "
	//        "sdu_interval %u us, max_sdu %u, phy %s, "
	//        "%s framing, %sencrypted\n",
	//        bt_le_per_adv_sync_get_index(sync), le_addr, biginfo->sid,
	//        biginfo->num_bis, biginfo->sub_evt_count,
	//        biginfo->iso_interval,
	//        (biginfo->iso_interval * 5 / 4),
	//        biginfo->burst_number, biginfo->offset,
	//        biginfo->rep_count, biginfo->max_pdu, biginfo->sdu_interval,
	//        biginfo->max_sdu, phy2str(biginfo->phy),
	//        biginfo->framing ? "with" : "without",
	//        biginfo->encryption ? "" : "not ");

	iso_interval = biginfo->iso_interval;
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

// moving average algo copied from: https://gist.github.com/mrfaptastic/3fd6394c5d6294c993d8b42b026578da
uint16_t maverage_values[FIFO_SIZE] = {0}; // all are zero as a start
uint16_t maverage_current_position  = 0;
uint64_t maverage_current_sum = 0;
uint16_t maverage_sample_length = sizeof(maverage_values) / sizeof(maverage_values[0]);

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

static uint32_t packets_since_last_reset = 0;

void pdr_watchdog_handler(struct k_timer *dummy)
{
	// uint32_t curr = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
	// printk("curr: %u, last_recv_packet_ts: %u, diff: %u\n", curr, last_recv_packet_ts, curr - last_recv_packet_ts);
	// if (curr - last_recv_packet_ts > 1000000) { // > 1s
	// 	pdr = 0.0;
	// 	printk("PDR: %.2f%%\n", pdr);
	// }

	uint32_t expected = 25;
	pdr = RollingmAvg(((float)packets_since_last_reset / (float)expected) * 100.0);

	printk("PDR: %.2f%%\n", pdr / 100.0);

	packets_since_last_reset=0;
}
K_TIMER_DEFINE(pdr_watchdog, pdr_watchdog_handler, NULL);

void recv_packet_handler(struct k_timer *dummy)
{
	packets_since_last_reset++;
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



		// uint32_t curr = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
		// uint32_t packet_delta = abs(last_recv_packet_ts - curr);
		// uint32_t iso_interval_us = iso_interval * 1.25 * 1000.0;
		// double lost_packets = (double)packet_delta / (double)iso_interval_us;
		// // printk("lost packets: %u, packet_delta: %u, iso_interval_us: %u Packet ID: %u\n", (uint8_t)lost_packets - 1, packet_delta, iso_interval_us, seq_num);
		// if (prev_seq_num + 1 != seq_num) {
		// 		// printk("\n------------------------- LOST PACKET -------------------------\n");
		// 		// pdr = RollingmAvg(0);
		// } else {
		// 	packets_since_last_reset++;
		// 	// pdr = RollingmAvg(1);	
		// }
		// prev_seq_num = seq_num;

		// printk("PDR: %.2f%%\n", pdr);
		// last_recv_packet_ts = curr;

		uint32_t info_ts = info->ts;
		uint32_t curr = k_cyc_to_us_near32(nrf_rtc_counter_get((NRF_RTC_Type*)NRF_RTC0_BASE));
		uint32_t delta = curr - info_ts;
		k_timer_start(&recv_packet, K_USEC(delta), K_NO_WAIT);
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

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	printk("Starting Synchronized Receiver Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Start PDR Watchdog timer */
	k_timer_start(&pdr_watchdog, K_MSEC(PDR_WATCHDOG_FREQ_MS), K_MSEC(PDR_WATCHDOG_FREQ_MS));

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

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
