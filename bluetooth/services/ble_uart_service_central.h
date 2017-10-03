/*
 * ble_uart_service_central.h
 *
 *  Created on: Sep 30, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#ifndef BLUETOOTH_SERVICES_BLE_UART_SERVICE_CENTRAL_H_
#define BLUETOOTH_SERVICES_BLE_UART_SERVICE_CENTRAL_H_

/**
 * @defgroup ble_rscs_c Running Speed and Cadence Service Client
 * @{
 * @ingroup ble_sdk_srv
 *
 * @details  This module contains the APIs and types exposed by the Running Speed and Cadence
 *           Service Client module. These APIs and types can be used by the application to perform
 *           discovery of Running Speed and Cadence Service at the peer and interact with it.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_rscs_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_RSCS_C_BLE_OBSERVER_PRIO,
 *                                   ble_rscs_c_on_ble_evt, &instance);
 *          @endcode
 */

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BLE_UART_C_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_rscs_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_UART_C_DEF(_name)                                                                       \
ble_uart_c_t _name;                                                                                 \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_UART_C_BLE_OBSERVER_PRIO,                                                  \
                     ble_uart_c_on_ble_evt, &_name)



/**@brief   Structure containing the handles related to the Running Speed and Cadence Service found on the peer. */
typedef struct
{
    uint16_t uart_cccd_handle;                /**< Handle of the CCCD of the UART characteristic. */
    uint16_t uart_handle;                     /**< Handle of the UART characteristic as provided by the SoftDevice. */
} ble_uart_c_db_t;

/**@brief   Ble UART Client event type. */
typedef enum
{
    BLE_UART_EVT_DISCOVERY_COMPLETE = 1,        /**< Event indicating that the Heart Rate Service has been discovered at the peer. */
    BLE_UART_EVT_NOTIFICATION_RECEIVED,         /**< Event indicating that a notification of the Heart Rate Measurement characteristic has been received from the peer. */
    BLE_UART_EVT_INDICATION_RECEIVED,
    BLE_UART_EVT_WRITE_RECEIVED,
} ble_uart_c_evt_type_t;

/**@brief   Structure containing the Uart packet. */
typedef struct
{
    uint8_t  command;
    uint8_t  packet_size;
    uint8_t  data[16];
    uint16_t crc16;
} ble_uart_packet_t;

/**@brief   Running Speed and Cadence Event structure. */
typedef struct
{
    ble_uart_c_evt_type_t evt_type;  /**< Type of the event. */
    uint16_t  conn_handle;           /**< Connection handle on which the rscs_c event  occured.*/
    union
    {
        ble_uart_c_db_t         uart_db;           /**< Running Speed and Cadence Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_uart_packet_t       uart_packet;       /**< Running Speed and Cadence measurement received. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_RSC_NOTIFICATION. */
    } rx_params;

    union
    {
        ble_uart_c_db_t         uart_db;           /**< Running Speed and Cadence Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_uart_packet_t       uart_packet;       /**< Running Speed and Cadence measurement received. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_RSC_NOTIFICATION. */
    } tx_params;

    union
    {
        ble_uart_c_db_t         uart_db;           /**< Running Speed and Cadence Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_uart_packet_t       uart_packet;       /**< Running Speed and Cadence measurement received. This will be filled if the evt_type is @ref BLE_RSCS_C_EVT_RSC_NOTIFICATION. */
    } notify_params;
} ble_uart_c_evt_t;

// Forward declaration of the ble_rscs_c_t type.
typedef struct ble_uart_c_s ble_uart_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_uart_c_evt_handler_t) (ble_uart_c_t * p_ble_uart_c, ble_uart_c_evt_t * p_evt);

/**@brief   Running Speed and Cadence client structure. */
struct ble_uart_c_s
{
    uint16_t                 conn_handle;      /**< Connection handle as provided by the SoftDevice. */
    ble_uart_c_db_t          rx_peer_db;          /**< Handles related to RSCS on the peer*/
    ble_uart_c_db_t          tx_peer_db;
    ble_uart_c_db_t          notify_peer_db;
    ble_uart_c_evt_handler_t evt_handler;      /**< Application event handler to be called when there is an event related to the Running Speed and Cadence service. */
};

/**@brief   Running Speed and Cadence client initialization structure. */
typedef struct
{
    ble_uart_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Running Speed and Cadence Client module whenever there is an event related to the Running Speed and Cadence Service. */
} ble_uart_c_init_t;


/**@brief      Function for initializing the Running Speed and Cadence Service Client module.
 *
 * @details    This function will initialize the module and set up Database Discovery to discover
 *             the Running Speed and Cadence Service. After calling this function, call @ref ble_db_discovery_start
 *             to start discovery once a link with a peer has been established.
 *
 * @param[out] p_ble_rscs_c      Pointer to the RSC Service client structure.
 * @param[in]  p_ble_rscs_c_init Pointer to the RSC Service initialization structure containing
 *                               the initialization information.
 *
 * @retval     NRF_SUCCESS      Operation success.
 * @retval     NRF_ERROR_NULL   A parameter is NULL.
 *                              Otherwise, an error code returned by @ref ble_db_discovery_evt_register.
 */
uint32_t BleUartServiceCentralInit(ble_uart_c_t * p_ble_uart_c, ble_uart_c_init_t * p_ble_uart_c_init);


/**@brief   Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Running Speed and Cadence
 *          Service client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Running Speed and Cadence Service client structure.
 */
void ble_uart_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


uint32_t ble_uart_c_indicate_enable(ble_uart_c_t * p_ble_uart_c);


/**@brief   Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery modue.
 *          This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of Running Speed and Cadence service at the peer.
 *          If so, it will call the application's event handler indicating that the RSC service has
 *          been discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param     p_ble_rscs_c Pointer to the Runnind Speed and Cadence Service client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_uart_on_db_disc_evt(ble_uart_c_t * p_ble_uart_c, ble_db_discovery_evt_t const * p_evt);


/**@brief   Function for assigning handles to a this instance of rscs_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate this link to this instance of the module. This makes it
 *          possible to handle several link and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles will be
 *          provided from the discovery event @ref BLE_RSCS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in]   p_ble_rscs_c    Pointer to the RSC client structure instance to associate.
 * @param[in]   conn_handle     Connection handle to associated with the given RSCS Client Instance.
 * @param[in]   p_peer_handles  Attribute handles on the RSCS server that you want this RSCS client
 *                              to interact with.
 */
uint32_t ble_uart_c_handles_assign(ble_uart_c_t *    p_ble_uart_c,
                                   uint16_t          conn_handle,
                                   ble_uart_c_db_t * p_rx_peer_handles,
                                   ble_uart_c_db_t * p_tx_peer_handles,
                                   ble_uart_c_db_t * p_notify_peer_handles);

void BleUartCentralHandler(ble_uart_c_t * p_uart_c, ble_uart_c_evt_t * p_uart_c_evt);

uint32_t BleUartCentralSendCommand(uint8_t command, uint8_t* data_to_send, uint8_t data_size);

BLE_UART_C_DEF(m_uart_c);
#ifdef __cplusplus
}
#endif


/** @} */ // End tag for the file.
#endif /* BLUETOOTH_SERVICES_BLE_UART_SERVICE_CENTRAL_H_ */
