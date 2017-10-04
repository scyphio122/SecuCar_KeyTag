/*
 * crypto.c
 *
 *  Created on: Oct 4, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#include "crypto.h"
#include "Systick.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <limits.h>
#include "nrf_sdh_soc.h"
#include "internal_memory_organization.h"
#include "internal_flash.h"
#include <string.h>

static uint8_t _lastKey[CRYPTO_KEY_SIZE];

uint32_t CryptoGenerateKey(uint8_t* generatedKey, uint8_t* generatedKeySize)
{
    uint8_t availableBytes = 0;
    while (availableBytes < CRYPTO_KEY_SIZE)
    {
        sd_rand_application_bytes_available_get(&availableBytes);
        SystickDelayMs(1);
    }

    *generatedKeySize = CRYPTO_KEY_SIZE;
    uint32_t retval = sd_rand_application_vector_get(generatedKey, CRYPTO_KEY_SIZE);

    if (retval == NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES)
        return retval;

    return NRF_SUCCESS;
}

uint32_t CryptoCheckMainKey()
{
    uint8_t generatedKey[CRYPTO_KEY_SIZE];

    memcpy(generatedKey, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, CRYPTO_KEY_SIZE);

    for (uint8_t i=0; i<CRYPTO_KEY_SIZE; ++i)
    {
        if (generatedKey[i] != CHAR_MAX)
            return true;
    }

    return false;
}

uint32_t CryptoGenerateAndStoreMainKey()
{
    uint32_t retval = NRF_SUCCESS;
    uint8_t generatedKey[CRYPTO_KEY_SIZE];
    uint8_t generatedKeySize = 0;

    retval = CryptoGenerateKey(generatedKey, &generatedKeySize);

    if (retval == NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES)
        return retval;

    retval = IntFlashUpdatePage(generatedKey, CRYPTO_KEY_SIZE, (uint32_t*)CRYPTO_MAIN_KEY_ADDRESS);

    if (retval != FLASH_OP_SUCCESS)
        return retval;

    return NRF_SUCCESS;
}

uint32_t CryptoEncryptData(uint8_t* dataToEncrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* encryptedData)
{
    // Assert the data size. It should be 4 bytes aligned
    if ((dataSize % sizeof(uint32_t)) != 0 || dataSize > 16)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    nrf_ecb_hal_data_t ecb;
    memset(&ecb, 0, sizeof(nrf_ecb_hal_data_t));
    memcpy(ecb.cleartext, dataToEncrypt, dataSize);
    memcpy(ecb.key, key, keySize);

    sd_ecb_block_encrypt(&ecb);

    memcpy(_lastKey, key, keySize);
    memcpy(encryptedData, ecb.ciphertext, dataSize);

    return NRF_SUCCESS;
}

uint32_t CryptoDecryptData(uint8_t* dataToDecrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* decryptedData)
{
    // Assert the data size. It should be 4 bytes aligned
    if ((dataSize % sizeof(uint32_t)) != 0 || dataSize > 16)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    nrf_ecb_hal_data_t ecb;
    memset(&ecb, 0, sizeof(nrf_ecb_hal_data_t));
    memcpy(ecb.cleartext, dataToDecrypt, dataSize);
    memcpy(ecb.key, key, keySize);

    sd_ecb_block_encrypt(&ecb);

    memcpy(decryptedData, ecb.ciphertext, dataSize);

    return NRF_SUCCESS;
}
