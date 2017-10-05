/*
 * ble_common.h
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */



#ifndef BLUETOOTH_BLE_COMMON_H_
#define BLUETOOTH_BLE_COMMON_H_

#include "app_timer.h"
#include <stdint-gcc.h>

#define DEVICE_NAME                     "SecuCar"

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)


#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define APP_BLE_CONN_CFG_TAG            1                                 /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           1                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO           1

#define SEC_PARAMS_BOND                 1                                               /**< Perform bonding. */
#if LESC_MITM_NC
#define SEC_PARAMS_MITM                 1                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_YESNO                   /**< Display Yes/No to force Numeric Comparison. */
#else
#define SEC_PARAMS_MITM                 0                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                            /**< No I/O caps. */
#endif
#define SEC_PARAMS_LESC                 1                                               /**< LE Secure Connections pairing required. */
#define SEC_PARAMS_KEYPRESS             0                                               /**< Keypress notifications not required. */
#define SEC_PARAMS_OOB                  0                                               /**< Out Of Band data not available. */
#define SEC_PARAMS_MIN_KEY_SIZE         7                                               /**< Minimum encryption key size in octets. */
#define SEC_PARAMS_MAX_KEY_SIZE         16                                              /**< Maximum encryption key size in octets. */

#define BLE_GAP_LESC_P256_SK_LEN        32
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

#define BLE_UUID_UART                   0x1000

extern uint16_t m_conn_handle_peripheral;
extern uint16_t m_conn_handle_central;

/**@brief Variable length data encapsulation in terms of length and pointer to data.
 */
typedef struct
{
    uint8_t  * p_data;    /**< Pointer to data. */
    uint16_t   data_len;  /**< Length of data. */
} data_t;

/**@brief GAP LE Secure Connections P-256 Private Key. */
typedef struct
{
  uint8_t   sk[BLE_GAP_LESC_P256_SK_LEN];   /**< LE Secure Connections Elliptic Curve Diffie-Hellman P-256 Private Key in little-endian. */
} ble_gap_lesc_p256_sk_t;

void BleStackInit(void);
void GapParamsInit(void);
void GattInit(void);
void ConnParamsInit(void);
void ServicesInit(void);
#endif /* BLUETOOTH_BLE_COMMON_H_ */
