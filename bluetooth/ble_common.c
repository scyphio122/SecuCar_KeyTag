/*
 * BleCommon.c
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */

#include "ble_common.h"
#include <stdint-gcc.h>
#include "app_error.h"
#include "ble.h"

#include "ble_hci.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "advertising.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_gatt.h"
#include "ble_uart_service.h"


NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */

 uint16_t           m_conn_handle_peripheral = BLE_CONN_HANDLE_INVALID;    /**< Connection handle for peripheral */

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle_peripheral, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling BLE Stack events common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    char passkey[BLE_GAP_PASSKEY_LEN + 1];
    uint16_t role = ble_conn_state_role(conn_handle);

    BleUartOnBleEvt(&m_ble_uart, p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            break;

//        case BLE_GAP_EVT_PASSKEY_DISPLAY:
//            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
//            passkey[BLE_GAP_PASSKEY_LEN] = 0x00;
//            NRF_LOG_INFO("%s: BLE_GAP_EVT_PASSKEY_DISPLAY: passkey=%s match_req=%d",
//                         nrf_log_push(roles_str[role]),
//                         nrf_log_push(passkey),
//                         p_ble_evt->evt.gap_evt.params.passkey_display.match_request);
//
//            if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request)
//            {
//                NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
//                m_num_comp_conn_handle = conn_handle;
//                m_numneric_match_requested = true;
//            }
//            break;
//
//        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
//            NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_KEY_REQUEST", nrf_log_push(roles_str[role]));
//            break;
//
//        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
//            NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST", nrf_log_push(roles_str[role]));
//
//            static nrf_value_length_t peer_public_key_raw = {0};
//
//            peer_public_key_raw.p_value = &p_ble_evt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk[0];
//            peer_public_key_raw.length = BLE_GAP_LESC_P256_PK_LEN;
//
//            err_code = nrf_crypto_ecc_public_key_from_raw(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
//                                                          &peer_public_key_raw,
//                                                          &m_peer_public_key);
//            APP_ERROR_CHECK(err_code);
//
//            err_code = nrf_crypto_ecdh_shared_secret_compute(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
//                                                             &m_private_key,
//                                                             &m_peer_public_key,
//                                                             &m_dh_key);
//            APP_ERROR_CHECK(err_code);
//
//            err_code = sd_ble_gap_lesc_dhkey_reply(conn_handle, &m_lesc_dh_key);
//            APP_ERROR_CHECK(err_code);
//            break;
//
//         case BLE_GAP_EVT_AUTH_STATUS:
//             NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
//                          nrf_log_push(roles_str[role]),
//                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
//                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
//                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
//                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
//                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
//            break;

        default:
            // No implementation needed.
            break;
    }
}

int cnt = 0;
ble_uart_evt_type_t evtHistory[16] = {  0xFF, 0xFF, 0xFF, 0xFF,
                                        0xFF, 0xFF, 0xFF, 0xFF,
                                        0xFF, 0xFF, 0xFF, 0xFF,
                                        0xFF, 0xFF, 0xFF, 0xFF};

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    evtHistory[cnt++] = p_ble_evt->header.evt_id;

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);
    }

    on_ble_evt(conn_handle, p_ble_evt);

}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void GapParamsInit(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
void GattInit(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module. */
void ConnParamsInit(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Peer Manager. */
void PeerManagerInit(void)
{
//    ble_gap_sec_params_t sec_params;
//    ret_code_t err_code;
//
//    err_code = pm_init();
//    APP_ERROR_CHECK(err_code);
//
//    memset(&sec_params, 0, sizeof(ble_gap_sec_params_t));
//
//    // Security parameters to be used for all security procedures.
//    sec_params.bond           = SEC_PARAMS_BOND;
//    sec_params.mitm           = SEC_PARAMS_MITM;
//    sec_params.lesc           = SEC_PARAMS_LESC;
//    sec_params.keypress       = SEC_PARAMS_KEYPRESS;
//    sec_params.io_caps        = SEC_PARAMS_IO_CAPABILITIES;
//    sec_params.oob            = SEC_PARAMS_OOB;
//    sec_params.min_key_size   = SEC_PARAMS_MIN_KEY_SIZE;
//    sec_params.max_key_size   = SEC_PARAMS_MAX_KEY_SIZE;
//    sec_params.kdist_own.enc  = 1;
//    sec_params.kdist_own.id   = 1;
//    sec_params.kdist_peer.enc = 1;
//    sec_params.kdist_peer.id  = 1;
//
//    err_code = pm_sec_params_set(&sec_params);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = pm_register(pm_evt_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = fds_register(fds_evt_handler);
//    APP_ERROR_CHECK(err_code);
//
//    // Private public keypair must be generated at least once for each device. It can be stored
//    // beyond this point. Here it is generated at bootup.
//    err_code = lesc_generate_key_pair();
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void BleStackInit(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for initializing services that will be used by the application.
 */
void ServicesInit()
{
    uint32_t                        err_code;
    ble_uart_init_t                 uart_init;

    //handler assignement
    uart_init.evt_handler = BleUartHandler;
    err_code = BleUartServiceInit(&m_ble_uart, &uart_init);
    APP_ERROR_CHECK(err_code);

}
