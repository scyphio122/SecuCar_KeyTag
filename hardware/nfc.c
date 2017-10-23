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

uint8_t nfc_tx_buf[] = "Hello";


void NFCT_IRQHandler()
{
    if (NRF_NFCT->EVENTS_FIELDDETECTED)
    {
        NRF_NFCT->EVENTS_FIELDDETECTED = 0;
        nrf_gpio_pin_clear(14);
    }
    else if (NRF_NFCT->EVENTS_FIELDLOST)
    {
        NRF_NFCT->EVENTS_FIELDLOST = 0;
        nrf_gpio_pin_set(14);
    }
}

void NfcInit()
{
    sd_nvic_SetPriority(NFCT_IRQn, 3);
    sd_nvic_EnableIRQ(NFCT_IRQn);

    NRF_NFCT->INTENSET = (NFCT_INTENSET_FIELDDETECTED_Set << NFCT_INTENSET_FIELDDETECTED_Pos) |
                         (NFCT_INTENSET_FIELDLOST_Set << NFCT_INTENSET_FIELDLOST_Pos);

    NRF_NFCT->TXD.AMOUNT = sizeof(nfc_tx_buf);

    NRF_NFCT->PACKETPTR = (uint32_t)nfc_tx_buf;

    NRF_NFCT->TASKS_SENSE = 1;
}
