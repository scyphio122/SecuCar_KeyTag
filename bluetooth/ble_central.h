/*
 * ble_central.h
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */

#ifndef BLUETOOTH_BLE_CENTRAL_H_
#define BLUETOOTH_BLE_CENTRAL_H_

#include "ble_common.h"
#include "ble.h"

void BleCentralStartScaning(void);


void on_adv_report(const ble_evt_t * const p_ble_evt);

#endif /* BLUETOOTH_BLE_CENTRAL_H_ */
