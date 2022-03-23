#include <bluetooth/bluetooth.h>
#include <io_coder.h>
#include <hw_info.h>

static struct io_coder io_encoder = {0};

struct adv_payload {
	uint8_t id[8];
};

/* ------------------------------------------------------ */
/* Sender Specific */
/* ------------------------------------------------------ */

static uint64_t packet_id = 1;
static struct adv_payload payload = { .id = {0,0,0,0,0,0,0,0} };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &payload, sizeof(struct adv_payload)),
};

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.timeout = 1,
	.num_events = 1,
};

void sent_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{
	// split uint64_t in uint8_t and store in adv_data
	for (int i = 0; i < 8; i++) {
        payload.id[i] = packet_id >> (8 * i);
    }

	int err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set ad (err %d)\n", err);
		return;
	}

	printk("Sending Packet: %lli\n", packet_id);

	err = write_8_bit(&io_encoder, packet_id % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}

	packet_id++;

	k_sleep(K_MSEC(1000));

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return;
	}
}

int setup_and_start_sending()
{
	int err;
	struct bt_le_ext_adv *adv;

	struct bt_le_adv_param param = {
		.options = /*BT_LE_ADV_OPT_CODED |*/ BT_LE_ADV_OPT_EXT_ADV,
		.interval_min = 0x0040 /* 40ms */,
		.interval_max = 0x0040 /* 40ms */,
		.secondary_max_skip = 0,
	};

	struct bt_le_ext_adv_cb adv_callbacks = {
		.sent = sent_cb,
	};

	err = bt_le_ext_adv_create(&param, &adv_callbacks, &adv);
	if (err) {
		return err;
	}

	// split uint64_t in uint8_t and store in adv_data
	for (int i = 0; i < 8; i++) {
        payload.id[i] = packet_id >> (8 * i);
    }
	packet_id++;

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		return err;
	}

	err = write_8_bit(&io_encoder, packet_id % 256);
	if(err) {
		printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
	}

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		return err;
	}

	return 0;
}

/* ------------------------------------------------------ */
/* Receiver Specific */
/* ------------------------------------------------------ */

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

void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	if(adv_type == BT_GAP_ADV_TYPE_EXT_ADV) {
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

		printk("id: %llu\n", sender_packet_id);

		int err = write_8_bit(&io_encoder, sender_packet_id % 256);
		if(err) {
			printk("Error writing 8bit value to P1.01 - P1.08 (err %d)\n", err);
		}
	}
}

int setup_and_start_receiving()
{
	int err;

	struct bt_le_scan_param param = {
        .type = /*BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M |*/ BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0c80  /* 2000 ms */,
        .window = 0x0c80 /* 2000 ms */,
		.timeout = 0,
		.interval_coded = 0,
        .window_coded = 0,
    };

	err = bt_le_scan_start(&param, &scan_cb);
	if (err) {
		return err;
	}

	return 0;
}

/* ------------------------------------------------------ */
/* Main */
/* ------------------------------------------------------ */

void main(void)
{
	int err;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	uint64_t id;
	err = get_device_id(&id);
	if(err) {
		printk("Error getting id (err %d)\n", err);
	}

	if(id == remote_213 /*local_42*/) {
		k_sleep(K_MSEC(3000)); // wait for receiver to be ready
		err = setup_and_start_sending();
		if (err) {
			printk("Failed to setup and start extended advertising (err %d)\n", err);
			return;
		}
	} else if(id == remote_116 /*local_56*/) {
		err = setup_and_start_receiving();
		if (err) {
			printk("Failed to setup and start extended advertising (err %d)\n", err);
			return;
		}
	}
}
