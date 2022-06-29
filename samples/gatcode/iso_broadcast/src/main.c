#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_rtc.h>
#include <io_coder.h>

static struct io_coder io_encoder = {0};

/* ------------------------------------------------------ */
/* D-Cube Defines */
/* ------------------------------------------------------ */
#define REMOTE false
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
#define RETRANSMISSION_NUMBER 2
#define BROADCAST_ENQUEUE_COUNT 2U // Guarantee always data to send

/* ------------------------------------------------------ */
/* Start */
/* ------------------------------------------------------ */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BROADCAST_ENQUEUE_COUNT * BIS_ISO_CHAN_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

static uint32_t seq_num;

static void iso_connected(struct bt_iso_chan *chan)
{
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);
	k_sem_give(&sem_big_term);
}

static struct bt_iso_chan bis_iso_chan;

uint8_t iso_data[DATA_SIZE_BYTE] = { 0 };
struct net_buf *buf;

void gpio_work_handler(struct k_work *work)
{
    printk("Sending value %u\n", seq_num);
	int err = write_8_bit(&io_encoder, seq_num % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}
}
K_WORK_DEFINE(gpio_work, gpio_work_handler);

static void iso_sent(struct bt_iso_chan *chan)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(++seq_num, iso_data);
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf);
	if (ret < 0) {
		printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	k_work_submit(&gpio_work);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE, /* bytes */
	.rtn = RETRANSMISSION_NUMBER,
	.phy = BT_GAP_LE_PHY_2M,
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

#define HCI_OPCODE_VS_SET_OP_FLAGS BT_OP(BT_OGF_VS, 0x3F3)
#define HCI_OPCODE_VS_SET_BD_ADDR BT_OP(BT_OGF_VS, 0x3F0)
#define HCI_OPCODE_VS_SET_ADV_TX_PWR BT_OP(BT_OGF_VS, 0x3F5)
#define HCI_OPCODE_VS_SET_CONN_TX_PWR BT_OP(BT_OGF_VS, 0x3F6)
#define HCI_OPCODE_VS_SET_LED_PIN_MAP BT_OP(BT_OGF_VS, 0x001)
#define HCI_OPCODE_VS_SET_RADIO_FE_CFG BT_OP(BT_OGF_VS, 0x3A3)
#define HCI_OPCODE_VS_SET_PRI_EXT_ADV_MAX_TX_PWR BT_OP(BT_OGF_VS, 0x000)

struct ble_hci_vs_cp_set_radio_fe_cfg {
	int8_t max_tx_power;
	uint8_t ant_id;
} __packed;

struct ble_hci_vs_rp_status {
	int8_t status;
} __packed;

enum ble_hci_vs_max_tx_power {
	BLE_HCI_VSC_MAX_TX_PWR_0dBm = 0,
	BLE_HCI_VSC_MAX_TX_PWR_3dBm = 3,
};

int ble_hci_vsc_set_radio_high_pwr_mode(bool high_power_mode)
{
	int ret;
	struct ble_hci_vs_cp_set_radio_fe_cfg *cp;
	struct ble_hci_vs_rp_status *rp;
	struct net_buf *buf, *rsp = NULL;

	buf = bt_hci_cmd_create(HCI_OPCODE_VS_SET_RADIO_FE_CFG, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return -ENOMEM;
	}
	cp = net_buf_add(buf, sizeof(*cp));
	if (high_power_mode) {
		printk("Enable VREGRADIO.VREQH\n");
		cp->max_tx_power = BLE_HCI_VSC_MAX_TX_PWR_3dBm;
	} else {
		printk("Disable VREGRADIO.VREQH\n");
		cp->max_tx_power = BLE_HCI_VSC_MAX_TX_PWR_0dBm;
	}
	cp->ant_id = 0;

	ret = bt_hci_cmd_send_sync(HCI_OPCODE_VS_SET_RADIO_FE_CFG, buf, &rsp);
	if (ret) {
		printk("Error for HCI VS command HCI_OPCODE_VS_SET_RADIO_FE_CFG\n");
		return ret;
	}

	rp = (void *)rsp->data;
	ret = rp->status;
	net_buf_unref(rsp);
	return ret;
}

enum ble_hci_vs_tx_power {
	BLE_HCI_VSC_TX_PWR_0dBm = 0,
	BLE_HCI_VSC_TX_PWR_Neg1dBm = -1,
	BLE_HCI_VSC_TX_PWR_Neg2dBm = -2,
	BLE_HCI_VSC_TX_PWR_Neg3dBm = -3,
	BLE_HCI_VSC_TX_PWR_Neg4dBm = -4,
	BLE_HCI_VSC_TX_PWR_Neg5dBm = -5,
	BLE_HCI_VSC_TX_PWR_Neg6dBm = -6,
	BLE_HCI_VSC_TX_PWR_Neg7dBm = -7,
	BLE_HCI_VSC_TX_PWR_Neg8dBm = -8,
	BLE_HCI_VSC_TX_PWR_Neg12dBm = -12,
	BLE_HCI_VSC_TX_PWR_Neg16dBm = -16,
	BLE_HCI_VSC_TX_PWR_Neg20dBm = -20,
	BLE_HCI_VSC_TX_PWR_Neg40dBm = -40,
	BLE_HCI_VSC_TX_PWR_INVALID = 99,
	BLE_HCI_VSC_PRI_EXT_ADV_MAX_TX_PWR_DISABLE = 127,
};

struct ble_hci_vs_cp_set_adv_tx_pwr {
	int8_t tx_power;
} __packed;

int ble_hci_vsc_set_adv_tx_pwr(enum ble_hci_vs_tx_power tx_power)
{
	int ret;
	struct ble_hci_vs_cp_set_adv_tx_pwr *cp;
	struct ble_hci_vs_rp_status *rp;
	struct net_buf *buf, *rsp = NULL;

	buf = bt_hci_cmd_create(HCI_OPCODE_VS_SET_ADV_TX_PWR, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return -ENOMEM;
	}
	cp = net_buf_add(buf, sizeof(*cp));
	cp->tx_power = tx_power;

	ret = bt_hci_cmd_send_sync(HCI_OPCODE_VS_SET_ADV_TX_PWR, buf, &rsp);
	if (ret) {
		printk("Error for HCI VS command HCI_OPCODE_VS_SET_ADV_TX_PWR\n");
		return ret;
	}
	printk("ble_hci_vsc_set_adv_tx_pwr finished\n");

	rp = (void *)rsp->data;
	ret = rp->status;
	net_buf_unref(rsp);

	return ret;
}

int ble_hci_vsc_set_pri_ext_adv_max_tx_pwr(enum ble_hci_vs_tx_power tx_power)
{
	int ret;
	struct ble_hci_vs_cp_set_adv_tx_pwr *cp;
	struct ble_hci_vs_rp_status *rp;
	struct net_buf *buf, *rsp = NULL;

	buf = bt_hci_cmd_create(HCI_OPCODE_VS_SET_PRI_EXT_ADV_MAX_TX_PWR, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffern\n");
		return -ENOMEM;
	}
	cp = net_buf_add(buf, sizeof(*cp));
	cp->tx_power = tx_power;

	ret = bt_hci_cmd_send_sync(HCI_OPCODE_VS_SET_PRI_EXT_ADV_MAX_TX_PWR, buf, &rsp);
	if (ret) {
		printk("Error for HCI VS command HCI_OPCODE_VS_SET_PRI_EXT_ADV_MAX_TX_PWR\n");
		return ret;
	}

	rp = (void *)rsp->data;
	ret = rp->status;
	net_buf_unref(rsp);
	return ret;
}

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

	#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_TX_POWER, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CUSTOM, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	#define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			BT_LE_PER_ADV_OPT_USE_TX_POWER)

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_CUSTOM);
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

	ble_hci_vsc_set_radio_high_pwr_mode(false);
	ble_hci_vsc_set_adv_tx_pwr(BLE_HCI_VSC_TX_PWR_0dBm);
	ble_hci_vsc_set_pri_ext_adv_max_tx_pwr(BLE_HCI_VSC_TX_PWR_0dBm);

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

	printk("Initialize sending (fill buffer)\n");
	for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
		iso_sent(&bis_iso_chan);
	}
}
