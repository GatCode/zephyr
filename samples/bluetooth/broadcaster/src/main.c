#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

// #include "../../../subsys/bluetooth/controller/ll_sw/ll_test.h"

void main(void)
{
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	do {
		// struct bt_hci_cp_le_tx_test_v4 *cp;
		// struct bt_hci_cp_le_tx_test_v4_tx_power *pw;
		// struct net_buf *buf;

		// buf = bt_hci_cmd_create(BT_HCI_OP_LE_TX_TEST_V4, sizeof(*cp) + sizeof(*pw));
		// if (!buf) {
		// 	return;
		// }

		// cp = net_buf_add(buf, sizeof(*cp));
		// cp->tx_ch = 35;
		// cp->test_data_len = 11;
		// cp->pkt_payload = BT_HCI_TEST_PKT_PAYLOAD_01010101;
		// cp->phy = BT_HCI_LE_TX_PHY_2M;
		// cp->cte_len = BT_HCI_LE_TEST_CTE_DISABLED;
		// cp->cte_type = BT_HCI_LE_TEST_CTE_TYPE_ANY;
		// cp->switch_pattern_len = BT_HCI_LE_TEST_SWITCH_PATTERN_LEN_ANY;

		// pw = net_buf_add(buf, sizeof(*pw));
		// pw->tx_power = BT_HCI_TX_TEST_POWER_MAX;

		// err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_TX_TEST_V4, buf, NULL);
		// if (err) {
		// 	return;
		// }

		struct bt_hci_cp_le_tx_test *cp;
		struct net_buf *buf;

		buf = bt_hci_cmd_create(BT_HCI_OP_LE_TX_TEST, sizeof(*cp));
		if (!buf) {
			return -ENOBUFS;
		}

		cp = net_buf_add(buf, sizeof(*cp));
		cp->tx_ch = 5;
		cp->test_data_len = 255;
		cp->pkt_payload = BT_HCI_TEST_PKT_PAYLOAD_01010101;
		int r_val = bt_hci_cmd_send(BT_HCI_OP_LE_TX_TEST, buf);

		printk("Go\n");

		k_sleep(K_MSEC(100));
	} while (1);
}
