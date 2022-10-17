#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

/* ------------------------------------------------------ */
/* ISO Definitions */
/* ------------------------------------------------------ */
#define DATA_SIZE_BYTE 23
#define ISO_CHAN_COUNT 1
#define SDU_INTERVAL_US 10000
#define TRANSPORT_LATENCY_MS 20
#define RETRANSMISSION_NUMBER 2
#define PHY BT_GAP_LE_PHY_2M

/* ------------------------------------------------------ */
/* ISO */
/* ------------------------------------------------------ */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, ISO_CHAN_COUNT,
		BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, ISO_CHAN_COUNT);

struct net_buf *buf;
static uint32_t seq_num;
uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
static struct bt_iso_chan bis_iso_chan;

static void iso_connected(struct bt_iso_chan *chan)
{
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE,
	.rtn = RETRANSMISSION_NUMBER,
	.phy = PHY,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos
};

static struct bt_iso_chan *bis[ISO_CHAN_COUNT] = {
	&bis_iso_chan
};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = SDU_INTERVAL_US,
	.latency = TRANSPORT_LATENCY_MS,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */
void main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

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

	/* Start Broadcasting */
	while (true) {
		k_sleep(K_USEC(big_create_param.interval));

		buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		sys_put_le32(++seq_num, iso_data);
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));

		int ret = bt_iso_chan_send(&bis_iso_chan, buf, seq_num, BT_ISO_TIMESTAMP_NONE);
		if (ret < 0) {
			printk("Unable to broadcast data: %d", ret);
			net_buf_unref(buf);
			return;
		}

		printk("Sent: %u\n", seq_num);
	}
}
