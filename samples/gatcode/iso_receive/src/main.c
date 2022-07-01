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
#include <hal/nrf_rtc.h>
#include <io_coder.h>
#include <sync_timer.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* Defines */
/* ------------------------------------------------------ */
#define BIS_ISO_CHAN_COUNT 1
#define DATA_SIZE_BYTE 50 // must be >= 23 (MTU minimum) && <= 251 (PDU_LEN_MAX)
#define PRESENTATION_DELAY_US 10000
#define MAXIMUM_SUBEVENTS 31 // MSE | 1-31
#define LED_ON true

/* ------------------------------------------------------ */
/* Importatnt Globals */
/* ------------------------------------------------------ */
static float pdr = 0.0;

/* ------------------------------------------------------ */
/* General */
/* ------------------------------------------------------ */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
/* UUIDs - https://www.guidgenerator.com */
#define BT_UUID_QUALITY_SERVICE BT_UUID_128_ENCODE(0x595d0001, 0xc3b2, 0x40a2, 0xab93, 0x28e30fa26298)
#define BT_UUID_PDR_CHARACTERISTIC BT_UUID_128_ENCODE(0x2fe883b0, 0x4305, 0x42f8, 0x8efe, 0x51bafee55253)

static void pdr_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       uint16_t value)
{
	ARG_UNUSED(attr);
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BAS Notifications %s", notif_enabled ? "enabled" : "disabled\n");
}

BT_GATT_SERVICE_DEFINE(acl_srv,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_QUALITY_SERVICE)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_PDR_CHARACTERISTIC),
			       BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(pdr_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		if (LED_ON) {
			gpio_pin_set_dt(&led, 1);
		}
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (LED_ON) {
		gpio_pin_set_dt(&led, 0);
	}
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void init_acl(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("ACL advertising failed to start (err %d)\n", err);
		return;
	}

	printk("ACL advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_acl = {
	.cancel = auth_cancel,
};

void acl_work_handler(struct k_work *work)
{
	static uint8_t pdr_arr[5];
	uint32_t mantissa;
	uint8_t exponent;

	mantissa = (uint32_t)(pdr * 100);
	exponent = (uint8_t)-2;
	
	pdr_arr[0] = 0;
	sys_put_le24(mantissa, (uint8_t *)&pdr_arr[1]);
	pdr_arr[4] = exponent;

	bt_gatt_notify(NULL, &acl_srv.attrs[1], &pdr_arr, sizeof(pdr_arr));
}
K_WORK_DEFINE(acl_work, acl_work_handler);

void acl_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&acl_work);
}
K_TIMER_DEFINE(acl_timer, acl_timer_handler, NULL);

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
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

static uint32_t seq_num = 0;

void gpio_work_handler(struct k_work *work)
{
    int err = write_8_bit(&io_encoder, seq_num % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}
K_WORK_DEFINE(gpio_work, gpio_work_handler);

void recv_packet_handler(struct k_timer *dummy)
{
	k_work_submit(&gpio_work);
}
K_TIMER_DEFINE(recv_packet, recv_packet_handler, NULL);

static uint32_t prev_seq_num = 0;
static uint32_t packets_recv = 0;
static uint32_t packets_lost = 0;

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
		printk(" | Packet ID: %u ", seq_num);

		// struct bt_iso_info iso_chan_info;
		// bt_iso_chan_get_info(chan, &iso_chan_info);

		packets_recv++;
		if(prev_seq_num + 1 != seq_num) {
			printk("\n------------------------- LOST PACKET -------------------------\n");
			packets_lost++; // Quick and Dirty hack - maybe account for multiple lost packets
		}
		pdr = (float)packets_recv * 100 / (packets_recv + packets_lost);
		printk("PDR: %.2f%%\n", pdr);
		prev_seq_num = seq_num;

		uint32_t info_ts = info->ts;
		uint32_t curr = audio_sync_timer_curr_time_get();
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

	if (!device_is_ready(led.port)) {
		printk("Error setting LED (err %d)\n", err);
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		printk("Error setting LED (err %d)\n", err);
	}

	printk("Starting Synchronized Receiver Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Init ACL connection...");
	init_acl();
	bt_conn_auth_cb_register(&auth_cb_acl);
	k_timer_start(&acl_timer, K_MSEC(100), K_MSEC(100));

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
