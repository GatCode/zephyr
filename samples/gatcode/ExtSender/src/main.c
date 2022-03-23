#include <bluetooth/bluetooth.h>
#include <io_coder.h>

static struct io_coder io_encoder = {0};

static uint64_t packet_id = 0;

struct adv_payload {
	uint8_t id[8];
};

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

struct bt_le_ext_adv_cb adv_callbacks = {
	.sent = sent_cb,
};

void main(void)
{
	int err;
	struct bt_le_ext_adv *adv;

	err = setup_8_bit_io_consecutive(&io_encoder, 1, 8, true, false);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	struct bt_le_adv_param param = {
		.options = /*BT_LE_ADV_OPT_CODED |*/ BT_LE_ADV_OPT_EXT_ADV,
		.interval_min = 0x0040 /* 40ms */,
		.interval_max = 0x0040 /* 40ms */,
		.secondary_max_skip = 0,
	};

	err = bt_le_ext_adv_create(&param, &adv_callbacks, &adv);
	if (err) {
		printk("Failed to create extended advertisement (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set ad (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return;
	}
}
