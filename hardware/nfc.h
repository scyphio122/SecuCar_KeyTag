/*
 * nfc.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#ifndef INC_NFC_H_
#define INC_NFC_H_

typedef enum
{
    E_NOT_INITIALIZED,
    E_DEV_NAME_PREPARED,
    E_SENT_DEVICE_NAME,
    E_RECEIVED_MAIN_KEY,
    E_SENT_DEACTIVATING_COMMAND,
    E_RECEIVED_OK_RESPONSE,
    E_INITIALIZED
}nfc_initialization_state_e;

#define NFC_BUFFER_SIZE    64

extern volatile nfc_initialization_state_e nfcInitState;

void NfcInit();

void NfcActivate();

void NfcStartSensingField();

void NfcEnableRx();

int NfcGetAvailableRxDataCount();

void NfcGetRxData(uint8_t* data, uint16_t startIndex, uint16_t dataSize);

int NfcPrepareTxData(uint8_t* data, uint16_t dataSize);

void NfcTriggerTx();

void NfcDisable();

#endif /* INC_NFC_H_ */
