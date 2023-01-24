/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>

static uint8_t mfg_data[] = { 0xde, 0xad, 0xbe, 0xef };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 4),
};

void main(void)
{
	struct bt_le_ext_adv *adv;
	int err;

	printk("Starting Periodic Advertising Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	#define BT_LE_EXT_ADV_NCONN_NAME_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
						 BT_LE_ADV_OPT_USE_NAME, \
						 BT_GAP_ADV_SLOW_INT_MIN, \
						 BT_GAP_ADV_SLOW_INT_MAX, \
						 NULL)

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

	printk("Start Extended Advertising...");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising "
				"(err %d)\n", err);
		return;
	}
	printk("done.\n");

	printk("Set Periodic Advertising Data...");
	err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
	if (err) {
		printk("Failed (err %d)\n", err);
		return;
	}
	printk("done.\n");

	while (true) {
		k_sleep(K_SECONDS(10));
	}
}
