/*
 * ble_common.h
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */

#ifndef BLUETOOTH_BLE_COMMON_H_
#define BLUETOOTH_BLE_COMMON_H_

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define APP_BLE_CONN_CFG_TAG            1                                 /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           1                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */


void BleStackInit(void);

#endif /* BLUETOOTH_BLE_COMMON_H_ */
