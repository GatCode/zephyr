#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include "ble_hci_vsc.h"
#include "bluetooth/hci.h"

static struct ble_hci_vs_tx_pwr_setting available_vs_tx_pwr_settings[14];

enum ble_hci_vs_max_tx_power {
	BLE_HCI_VSC_MAX_TX_PWR_0dBm = 0,
	BLE_HCI_VSC_MAX_TX_PWR_3dBm = 3,
};

void init_ble_hci_vsc_tx_pwr()
{
	available_vs_tx_pwr_settings[0].tx_power = BLE_HCI_VSC_TX_PWR_Neg40dBm;
	available_vs_tx_pwr_settings[0].add_3dBm = false;
	available_vs_tx_pwr_settings[1].tx_power = BLE_HCI_VSC_TX_PWR_Neg20dBm;
	available_vs_tx_pwr_settings[1].add_3dBm = false;
	available_vs_tx_pwr_settings[2].tx_power = BLE_HCI_VSC_TX_PWR_Neg16dBm;
	available_vs_tx_pwr_settings[2].add_3dBm = false;
	available_vs_tx_pwr_settings[3].tx_power = BLE_HCI_VSC_TX_PWR_Neg12dBm;
	available_vs_tx_pwr_settings[3].add_3dBm = false;
	available_vs_tx_pwr_settings[4].tx_power = BLE_HCI_VSC_TX_PWR_Neg8dBm;
	available_vs_tx_pwr_settings[4].add_3dBm = false;
	available_vs_tx_pwr_settings[5].tx_power = BLE_HCI_VSC_TX_PWR_Neg7dBm;
	available_vs_tx_pwr_settings[5].add_3dBm = false;
	available_vs_tx_pwr_settings[6].tx_power = BLE_HCI_VSC_TX_PWR_Neg6dBm;
	available_vs_tx_pwr_settings[6].add_3dBm = false;
	available_vs_tx_pwr_settings[7].tx_power = BLE_HCI_VSC_TX_PWR_Neg5dBm;
	available_vs_tx_pwr_settings[7].add_3dBm = false;
	available_vs_tx_pwr_settings[8].tx_power = BLE_HCI_VSC_TX_PWR_Neg4dBm;
	available_vs_tx_pwr_settings[8].add_3dBm = false;
	available_vs_tx_pwr_settings[9].tx_power = BLE_HCI_VSC_TX_PWR_Neg3dBm;
	available_vs_tx_pwr_settings[9].add_3dBm = false;
	available_vs_tx_pwr_settings[10].tx_power = BLE_HCI_VSC_TX_PWR_Neg2dBm;
	available_vs_tx_pwr_settings[10].add_3dBm = false;
	available_vs_tx_pwr_settings[11].tx_power = BLE_HCI_VSC_TX_PWR_Neg1dBm;
	available_vs_tx_pwr_settings[11].add_3dBm = false;
	available_vs_tx_pwr_settings[12].tx_power = BLE_HCI_VSC_TX_PWR_0dBm;
	available_vs_tx_pwr_settings[12].add_3dBm = false;
	available_vs_tx_pwr_settings[13].tx_power = BLE_HCI_VSC_TX_PWR_0dBm;
	available_vs_tx_pwr_settings[13].add_3dBm = true;
}

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
		// printk("Enable VREGRADIO.VREQH\n");
		cp->max_tx_power = BLE_HCI_VSC_MAX_TX_PWR_3dBm;
	} else {
		// printk("Disable VREGRADIO.VREQH\n");
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

int ble_hci_vsc_set_tx_pwr(uint8_t pwr_setting_index)
{
	int ret;

	struct ble_hci_vs_tx_pwr_setting setting = available_vs_tx_pwr_settings[pwr_setting_index];

	ret = ble_hci_vsc_set_radio_high_pwr_mode(setting.add_3dBm);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_radio_high_pwr_mode\n");
		return ret;
	}

	ret = ble_hci_vsc_set_adv_tx_pwr(setting.tx_power);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_adv_tx_pwr\n");
		return ret;
	}

	ret = ble_hci_vsc_set_pri_ext_adv_max_tx_pwr(setting.tx_power);
	if (ret) {
		printk("Error for HCI VS command ble_hci_vsc_set_pri_ext_adv_max_tx_pwr\n");
		return ret;
	}

	return ret;
}