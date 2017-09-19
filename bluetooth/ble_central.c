/*
 * ble_central.c
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */
#include "ble_central.h"
#include "ble_common.h"
#include "ble.h"
#include "app_error.h"
#include "app_timer.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <string.h>

#define BLE_CENTRAL_SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define BLE_CENTRAL_SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define BLE_CENTRAL_SCAN_TIMEOUT                    0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */


#define BLE_CENTRAL_MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define BLE_CENTRAL_MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define BLE_CENTRAL_SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define BLE_CENTRAL_SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                     2                                   /**< Size of a UUID, in bytes. */


/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;


static char const m_target_periph_name[] = "Nordic_Blinky";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)BLE_CENTRAL_MIN_CONNECTION_INTERVAL,
    (uint16_t)BLE_CENTRAL_MAX_CONNECTION_INTERVAL,
    (uint16_t)BLE_CENTRAL_SLAVE_LATENCY,
    (uint16_t)BLE_CENTRAL_SUPERVISION_TIMEOUT
};


/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = BLE_CENTRAL_SCAN_INTERVAL,
    .window   = BLE_CENTRAL_SCAN_WINDOW,
    .timeout  = BLE_CENTRAL_SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t _bleCentralParseAdvData(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
void BleCentralStartScaning(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;
    bool       do_connect = false;

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.data_len = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    bool name_found = false;
    err_code = _bleCentralParseAdvData(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete.
        err_code = _bleCentralParseAdvData(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit
            return;
        }
        else
        {
            name_found = true;
        }
    }
    else
    {
        name_found = true;
    }

    if (name_found)
    {
        if (strlen(m_target_periph_name) != 0)
        {
            if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
    }

    if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}

