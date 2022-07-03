/*
 * NOTE: parts of the code below and the corresponding source file
 * are copied from the Nordic nrf5340_audio sample (sdk-nrf on GitHub)
 */

#ifndef _BLE_HCI_VSC_H_
#define _BLE_HCI_VSC_H_

#include <zephyr/kernel.h>

#define HCI_OPCODE_VS_SET_ADV_TX_PWR BT_OP(BT_OGF_VS, 0x3F5)
#define HCI_OPCODE_VS_SET_RADIO_FE_CFG BT_OP(BT_OGF_VS, 0x3A3)
#define HCI_OPCODE_VS_SET_PRI_EXT_ADV_MAX_TX_PWR BT_OP(BT_OGF_VS, 0x000)

struct ble_hci_vs_rp_status {
	int8_t status;
} __packed;

struct ble_hci_vs_cp_set_adv_tx_pwr {
	int8_t tx_power;
} __packed;

struct ble_hci_vs_cp_set_conn_tx_pwr {
	uint16_t handle;
	int8_t tx_power;
} __packed;

struct ble_hci_vs_cp_set_radio_fe_cfg {
	int8_t max_tx_power;
	uint8_t ant_id;
} __packed;

struct ble_hci_vs_tx_pwr_setting {
	int8_t tx_power;
	bool add_3dBm;
} __packed;

enum ble_hci_vs_tx_power {
	BLE_HCI_VSC_TX_PWR_0dBm = 0,
	BLE_HCI_VSC_TX_PWR_Neg1dBm = -1,
	BLE_HCI_VSC_TX_PWR_Neg2dBm = -2,
	BLE_HCI_VSC_TX_PWR_Neg3dBm = -3,
	BLE_HCI_VSC_TX_PWR_Neg4dBm = -4,
	BLE_HCI_VSC_TX_PWR_Neg5dBm = -5,
	BLE_HCI_VSC_TX_PWR_Neg6dBm = -6,
	BLE_HCI_VSC_TX_PWR_Neg7dBm = -7,
	BLE_HCI_VSC_TX_PWR_Neg8dBm = -8,
	BLE_HCI_VSC_TX_PWR_Neg12dBm = -12,
	BLE_HCI_VSC_TX_PWR_Neg16dBm = -16,
	BLE_HCI_VSC_TX_PWR_Neg20dBm = -20,
	BLE_HCI_VSC_TX_PWR_Neg40dBm = -40,
	BLE_HCI_VSC_TX_PWR_INVALID = 99,
	BLE_HCI_VSC_PRI_EXT_ADV_MAX_TX_PWR_DISABLE = 127,
};

enum ble_hci_vs_led_function_id {
	PAL_LED_ID_CPU_ACTIVE = 0x10,
	PAL_LED_ID_ERROR = 0x11,
	PAL_LED_ID_BLE_TX = 0x12,
	PAL_LED_ID_BLE_RX = 0x13,
};

enum ble_hci_vs_led_function_mode {
	PAL_LED_MODE_ACTIVE_LOW = 0x00,
	PAL_LED_MODE_ACTIVE_HIGH = 0x01,
	PAL_LED_MODE_DISABLE_TOGGLE = 0xFF,
};

/**
 * @brief Initializes the available_vs_tx_pwr_settings options.
 */
void init_ble_hci_vsc_tx_pwr();

/**
 * @brief Enable VREGRADIO.VREQH in NET core for getting +3dBm TX power
 *        Note, this will add +3 dBm for the primary advertisement channels
 *        as well even ble_hci_vsc_set_pri_ext_adv_max_tx_pwr() has been used
 * @param high_power_mode	Enable VREGRADIO.VREQH or not
 *
 * @return 0 for success, error otherwise.
 */
int ble_hci_vsc_set_radio_high_pwr_mode(bool high_power_mode);

/**
 * @brief Set advertising TX power
 * @param tx_power TX power setting for the advertising.
 *                 Please check ble_hci_vs_tx_power for possible settings
 *
 * @return 0 for success, error otherwise.
 */
int ble_hci_vsc_set_adv_tx_pwr(enum ble_hci_vs_tx_power tx_power);

/**
 * @brief Set the maximum transmit power on primary advertising channels
 * @param tx_power TX power setting for the primary advertising channels
 *                 in advertising extension, which are BLE channel 37, 38 and 39
 *                 Please check ble_hci_vs_tx_power for possible settings
 *                 Set to BLE_HCI_VSC_PRI_EXT_ADV_MAX_TX_PWR_DISABLE (-127) for
 *                 disabling this feature
 *
 * @return 0 for success, error otherwise.
 */
int ble_hci_vsc_set_pri_ext_adv_max_tx_pwr(enum ble_hci_vs_tx_power tx_power);

/**
 * @brief Set the maximum transmit power
 * @param pwr_setting_index 	TX power setting index - see available_vs_tx_pwr_settings.
 *
 * @return 0 for success, error otherwise.
 */
int ble_hci_vsc_set_tx_pwr(uint8_t pwr_setting_index);

#endif /* _BLE_HCI_VSC_H_ */
