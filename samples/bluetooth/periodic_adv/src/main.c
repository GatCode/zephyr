/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
};

#define PDU_CHANNEL_MAP_SIZE 5

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

	static uint8_t chan_map[PDU_CHANNEL_MAP_SIZE];
	uint8_t ch = 0x1F;

	while (true) {
		k_sleep(K_SECONDS(10));
		
		for (uint8_t i = 0; i < PDU_CHANNEL_MAP_SIZE; i++) {
			chan_map[i] = ch++;
		}
		chan_map[PDU_CHANNEL_MAP_SIZE - 1] = 0x1F; // packet validity
				
		struct bt_hci_cp_le_per_adv_chm_update *cp;
		struct net_buf *buf;

		buf = bt_hci_cmd_create(BT_HCI_OP_LE_PER_ADV_CHM_UPDATE, sizeof(*cp));
		if (!buf) {
			return;
		}

		cp = net_buf_add(buf, sizeof(*cp));

		memcpy(&cp->ch_map[0], &chan_map[0], 4);
		cp->ch_map[4] = chan_map[4] & BIT_MASK(5);

		bt_hci_cmd_send_sync(BT_HCI_OP_LE_PER_ADV_CHM_UPDATE, buf, NULL);

		printk("UPDATED PER ADV CHM\n");

		while (true) {
			// stay here
		}
	}
}
