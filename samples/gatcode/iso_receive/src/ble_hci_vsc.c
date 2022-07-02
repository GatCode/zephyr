#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include "ble_hci_vsc.h"
#include "bluetooth/hci.h"

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
		printk("Unable to allocate command buffer\n");
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

int ble_hci_vsc_set_tx_pwr(struct ble_hci_vs_tx_pwr_setting tx_power_setting)
{
	int ret;

	ret = ble_hci_vsc_set_radio_high_pwr_mode(tx_power_setting.add_3dBm);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_radio_high_pwr_mode\n");
		return ret;
	}

	ret = ble_hci_vsc_set_adv_tx_pwr(tx_power_setting.tx_power);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_adv_tx_pwr\n");
		return ret;
	}

	ret = ble_hci_vsc_set_pri_ext_adv_max_tx_pwr(tx_power_setting.tx_power);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_pri_ext_adv_max_tx_pwr\n");
		return ret;
	}

	return ret;
}