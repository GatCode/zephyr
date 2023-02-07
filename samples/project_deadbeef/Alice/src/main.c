#include <zephyr/bluetooth/bluetooth.h>

static uint8_t deadbeef[] = { 0xde, 0xad, 0xbe, 0xef };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, deadbeef, 4),
};

void main(void)
{
	struct bt_le_ext_adv *adv;
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	#define BT_LE_EXT_ADV_NCONN_NAME_CUSTOM BT_LE_ADV_PARAM( \
			BT_LE_ADV_OPT_EXT_ADV | \
			BT_LE_ADV_OPT_USE_NAME, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			BT_GAP_ADV_FAST_INT_MIN_2, \
			NULL)

	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME_CUSTOM, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err) {
		printk("Failed to set periodic advertising parameters"
		       " (err %d)\n", err);
		return;
	}

	err = bt_le_per_adv_start(adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising "
				"(err %d)\n", err);
		return;
	}

	err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
	if (err) {
		printk("Failed (err %d)\n", err);
		return;
	}
}
