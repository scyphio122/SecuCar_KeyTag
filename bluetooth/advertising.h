/*
 * advertising.h
 *
 *  Created on: Sep 5, 2017
 *      Author: root
 */

#ifndef BLUETOOTH_ADVERTISING_H_
#define BLUETOOTH_ADVERTISING_H_

#define ADVERTISING_DEVICE_NAME			"SecuCar"

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define BLE_PERIPHERAL_MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define BLE_PERIPHERAL_MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define BLE_PERIPHERAL_SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define BLE_PERIPHERAL_SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */


#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif


void BleStackInit(void);
void AdvertisingInit(void);
void AdvertisingStart(void);
#endif /* BLUETOOTH_ADVERTISING_H_ */
