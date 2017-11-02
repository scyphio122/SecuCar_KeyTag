/*
 * nfc.c
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */
#include "nrf.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_gpio.h"
#include "nrf_nvic.h"
#include "nfc.h"
#include "fifo.h"
#include <string.h>
#include <stdint-gcc.h>

static uint8_t      _nfcTxBuffer[NFC_BUFFER_SIZE];
static uint8_t      _nfcRxBuffer[NFC_BUFFER_SIZE];

volatile nfc_initialization_state_e nfcInitState = E_NOT_INITIALIZED;

void NFCT_IRQHandler()
{
    if (NRF_NFCT->EVENTS_FIELDDETECTED)
    {
        NRF_NFCT->EVENTS_FIELDDETECTED = 0;
        NfcActivate();
        nrf_gpio_pin_clear(17);
    }


    if (NRF_NFCT->EVENTS_READY)
    {
        NRF_NFCT->EVENTS_READY = 0;
        if (nfcInitState == E_DEV_NAME_PREPARED)
        {
            NfcTriggerTx();
        }
        else
        {
            NfcActivate();
        }
    }

    if (NRF_NFCT->EVENTS_ENDTX)
    {
        NRF_NFCT->EVENTS_ENDTX = 0;
        if (nfcInitState == E_DEV_NAME_PREPARED)
        {
            nfcInitState = E_SENT_DEVICE_NAME;
        }
        else
        if (nfcInitState == E_RECEIVED_MAIN_KEY)
        {
            nfcInitState = E_SENT_DEACTIVATING_COMMAND;
        }
        else    // FORBIDDEN STATE
        {
            sd_nvic_SystemReset();
        }

        NfcEnableRx();
    }

    if (NRF_NFCT->EVENTS_ENDRX)
    {
        NRF_NFCT->EVENTS_ENDRX = 0;

        if (nfcInitState == E_SENT_DEVICE_NAME)
        {
            nfcInitState = E_RECEIVED_MAIN_KEY;
        }
        else
        if (nfcInitState == E_SENT_DEACTIVATING_COMMAND)
        {
            nfcInitState = E_RECEIVED_OK_RESPONSE;
        }
        else    // FORBIDDEN STATE
        {
            sd_nvic_SystemReset();
        }

        NfcActivate();
    }

    if (NRF_NFCT->EVENTS_FIELDLOST)
    {
        NRF_NFCT->EVENTS_FIELDLOST = 0;
        NfcStartSensingField();
        nrf_gpio_pin_set(17);
    }
}

void NfcInit()
{
    uint32_t err = sd_nvic_SetPriority(NFCT_IRQn, 3);
    err = sd_nvic_EnableIRQ(NFCT_IRQn);

    NRF_NFCT->INTENSET = (NFCT_INTENSET_FIELDDETECTED_Set << NFCT_INTENSET_FIELDDETECTED_Pos) |
                         (NFCT_INTENSET_FIELDLOST_Set << NFCT_INTENSET_FIELDLOST_Pos) |
                         (NFCT_INTENSET_READY_Set << NFCT_INTENSET_READY_Pos) |
                         (NFCT_INTENSET_ENDTX_Enabled << NFCT_INTENSET_ENDTX_Pos) |
                         (NFCT_INTENSET_ENDRX_Enabled << NFCT_INTENSET_ENDRX_Pos);

    NRF_NFCT->TXD.FRAMECONFIG = NFCT_TXD_FRAMECONFIG_CRCMODETX_CRC16TX << NFCT_TXD_FRAMECONFIG_CRCMODETX_Pos;
    NRF_NFCT->RXD.FRAMECONFIG = NFCT_RXD_FRAMECONFIG_CRCMODERX_CRC16RX << NFCT_RXD_FRAMECONFIG_CRCMODERX_Pos;

    NRF_NFCT->PACKETPTR = (uint32_t)NULL;
    NRF_NFCT->MAXLEN = NFC_BUFFER_SIZE;

    NRF_NFCT->SHORTS = NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Enabled << NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Pos;

}

void NfcStartSensingField()
{
    NRF_NFCT->TASKS_SENSE = 1;
}

void NfcActivate()
{
    NRF_NFCT->TASKS_ACTIVATE = 1;
}

int NfcPrepareTxData(uint8_t* data, uint16_t dataSize)
{
    if (dataSize > NFC_BUFFER_SIZE)
    {
        return -1;
    }

    memcpy(_nfcTxBuffer, data, dataSize);
}

void NfcTriggerTx()
{
    NRF_NFCT->PACKETPTR = (uint32_t)_nfcTxBuffer;
    NRF_NFCT->TASKS_STARTTX = 1;
}

void NfcEnableRx()
{
    NRF_NFCT->PACKETPTR = (uint32_t)_nfcRxBuffer;
    NRF_NFCT->TASKS_ENABLERXDATA = 1;
}

int NfcGetAvailableRxDataCount()
{
    return NRF_NFCT->RXD.AMOUNT;
}

void NfcGetRxData(uint8_t* data, uint16_t startIndex, uint16_t dataSize)
{
    memcpy(data, &_nfcRxBuffer[startIndex], dataSize);
}

void NfcDisable()
{
    NRF_NFCT->TASKS_DISABLE = 1;
}


