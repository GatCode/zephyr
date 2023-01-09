#include <zephyr/bluetooth/bluetooth.h>

static uint8_t mfg_data[3] = { 0 };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
};

void main(void)
{
	struct bt_le_ext_adv *adv;
	int err;

	err = bt_enable(NULL);
	if (err) {
		return;
	}

	/* 7.5ms */

	#define BT_LE_EXT_ADV_NCONN_NAME_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \ 
						 BT_LE_ADV_OPT_USE_NAME, \
						 BT_GAP_ADV_SLOW_INT_MIN, \
						 BT_GAP_ADV_SLOW_INT_MIN, \
						 NULL)

	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME_CUSTOM, NULL, &adv);
	if (err) {
		return;
	}


	/* Set periodic advertising parameters */
	// 7.5ms

	#define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(BT_GAP_PER_ADV_FAST_INT_MIN_1, \
						  BT_GAP_PER_ADV_FAST_INT_MIN_1, \
						  BT_LE_PER_ADV_OPT_NONE)

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

	printk("Set Periodic Advertising Data...");
	err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
	if (err) {
		printk("Failed (err %d)\n", err);
		return;
	}
	printk("done.\n");

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		return;
	}

	while (true) {
		k_sleep(K_SECONDS(10));
	}
}

	/* Set periodic advertising parameters */
	// 30ms

	// #define BT_LE_PER_ADV_CUSTOM BT_LE_PER_ADV_PARAM(BT_GAP_PER_ADV_FAST_INT_MIN_1, \
	// 					  BT_GAP_PER_ADV_FAST_INT_MIN_1, \
	// 					  BT_LE_PER_ADV_OPT_NONE)

	// err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_CUSTOM);
	// if (err) {
	// 	printk("Failed to set periodic advertising parameters"
	// 	       " (err %d)\n", err);
	// 	return;
	// }

	// /* Enable Periodic Advertising */
	// err = bt_le_per_adv_start(adv);
	// if (err) {
	// 	printk("Failed to enable periodic advertising (err %d)\n", err);
	// 	return;
	// }

	// printk("Set Periodic Advertising Data...");
	// err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
	// if (err) {
	// 	printk("Failed (err %d)\n", err);
	// 	return;
	// }
	// printk("done.\n");

	// printk("Start Extended Advertising...");
	// err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	// if (err) {
	// 	printk("Failed to start extended advertising "
	// 		"(err %d)\n", err);
	// 	return;
	// }
	// printk("done.\n");

	// while (true) {
	// 	k_sleep(K_SECONDS(10));

	// 	// mfg_data[2]++;

	// 	// printk("Set Periodic Advertising Data...");
	// 	// err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
	// 	// if (err) {
	// 	// 	printk("Failed (err %d)\n", err);
	// 	// 	return;
	// 	// }
	// 	// printk("done.\n");

	// 	// k_sleep(K_SECONDS(10));
	// }
// }
