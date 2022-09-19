#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* ------------------------------------------------------ */
/* Basic Definitions */
/* ------------------------------------------------------ */
#define MAX_RTN 8
#define SDU_INTERVAL_US 20000
#define TRANSPORT_LATENCY_MS 20
#define DATA_SIZE_BYTE 50

/* ------------------------------------------------------ */
/* Global Controller Overwrites */
/* ------------------------------------------------------ */
extern int8_t txp_global_overwrite;
extern uint8_t rtn_global_overwrite;

/* ------------------------------------------------------ */
/* Important Globals */
/* ------------------------------------------------------ */
static uint32_t seq_num;

/* ------------------------------------------------------ */
/* ACL */
/* ------------------------------------------------------ */
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

#define DEVICE_NAME_ACL "nRF52840"
#define DEVICE_NAME_ACL_LEN (sizeof(DEVICE_NAME_ACL) - 1)

#define CONFIG_BLE_ACL_CONN_INTERVAL_MIN 16 // * 1.25 - 20ms
#define CONFIG_BLE_ACL_CONN_INTERVAL_MAX 32 // * 1.25 - 40ms
#define CONFIG_BLE_ACL_SLAVE_LATENCY 0
#define CONFIG_BLE_ACL_SUP_TIMEOUT 400

#define BT_LE_CONN_PARAM_MULTI \
		BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL_MIN, CONFIG_BLE_ACL_CONN_INTERVAL_MAX, \
		CONFIG_BLE_ACL_SLAVE_LATENCY, CONFIG_BLE_ACL_SUP_TIMEOUT)

#define BT_CONN_LE_CREATE_CONN_CUSTOM \
	BT_CONN_LE_CREATE_PARAM(BT_CONN_LE_OPT_NONE, \
				BT_GAP_SCAN_FAST_INTERVAL, \
				BT_GAP_SCAN_FAST_WINDOW)

static K_SEM_DEFINE(acl_connected, 0, 1);

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if ((data_len == DEVICE_NAME_ACL_LEN) &&
	    (strncmp(DEVICE_NAME_ACL, data, DEVICE_NAME_ACL_LEN) == 0)) {
		bt_le_scan_stop();

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN_CUSTOM, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			printk("Could not init ACL connection\n");
			return ret;
		}

		return 0;
	}

	return -ENOENT;
}

static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			printk("AD malformed\n");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* We're only interested in general connectable events */
	if (type == BT_HCI_ADV_IND) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

void work_scan_start(struct k_work *item)
{
	int ret = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (ret) {
		printk("ACL scanning failed to start (ret %d)\n", ret);
		return;
	}

	printk("ACL scanning successfully started\n");
}
K_WORK_DEFINE(start_scan_work, work_scan_start);

static void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		printk("Read RSSI err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	uint16_t handle = 0;
	int err = bt_hci_get_conn_handle(conn, &handle);
	if (err) {
		printk("Failed to read ACL connection handle\n");
	}
	int8_t rssi = 0;
	read_conn_rssi(handle, &rssi);

	uint8_t iso_receiver_rssi = ((uint8_t *)data)[0];
	printk("ACL RSSI: %d | PER RSSI: -%d\n", rssi, iso_receiver_rssi);
	
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HTS)) {
		memcpy(&uuid, BT_UUID_HTS_MEASUREMENT, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HTS_MEASUREMENT)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_INDICATE;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("ACL connected - %s\n", addr);

	/* Start Service Discovery */
	memcpy(&uuid, BT_UUID_HTS, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	int err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("ACL discovery failed(err %d)\n", err);
		return;
	}

	k_sem_give(&acl_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("ACL disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(conn);

	/* Start ACL Scanning */
	k_work_submit(&start_scan_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* ------------------------------------------------------ */
/* ISO Stuff */
/* ------------------------------------------------------ */
#define BUF_ALLOC_TIMEOUT (10) /* milliseconds */
#define BIG_TERMINATE_TIMEOUT_US (60 * USEC_PER_SEC) /* microseconds */
#define BIS_ISO_CHAN_COUNT 1

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);

struct net_buf *buf;
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

static void iso_sent(struct bt_iso_chan *chan)
{
	buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	sys_put_le32(++seq_num, iso_data);
	iso_data[4] = (uint8_t)txp_global_overwrite;
	net_buf_add_mem(buf, iso_data, sizeof(iso_data));

	int ret = bt_iso_chan_send(&bis_iso_chan, buf, seq_num, BT_ISO_TIMESTAMP_NONE);
	if (ret < 0) {
		printk("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	if (seq_num % 1000 > 600) {
		txp_global_overwrite = 8;
	} else if (seq_num % 1000 > 300) {
		txp_global_overwrite = 0;
	} else {
		txp_global_overwrite = -40;
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = DATA_SIZE_BYTE,
	.rtn = MAX_RTN,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos
};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT] = {
	&bis_iso_chan
};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = SDU_INTERVAL_US,
	.latency = TRANSPORT_LATENCY_MS,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

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

	/* Start ACL Scanning */
	k_work_submit(&start_scan_work);

	err = k_sem_take(&acl_connected, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}

	#define BT_LE_EXT_ADV_NCONN_NAME_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MAX_2, \
			NULL)

	/* Create a non-connectable non-scannable advertising set */
	/* Between 100ms and 150ms */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME_CUSTOM, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	#define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(BT_GAP_PER_ADV_SLOW_INT_MIN, \
			BT_GAP_PER_ADV_SLOW_INT_MAX, \
			BT_LE_PER_ADV_OPT_NONE)

	/* Set periodic advertising parameters */
	/* Between 1.0s and 1.2s */
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

	txp_global_overwrite = 8;
	rtn_global_overwrite = 8;

	/* Start ISO Stream */
	iso_sent(&bis_iso_chan);
}
