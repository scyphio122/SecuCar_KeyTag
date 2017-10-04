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
#include "aes.h"

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
    // Assert the data size.
    if (dataSize != CRYPTO_KEY_SIZE)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    AES_ECB_encrypt(dataToEncrypt, key, encryptedData, dataSize);

    return NRF_SUCCESS;
}

uint32_t CryptoDecryptData(uint8_t* dataToDecrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* decryptedData)
{
    // Assert the data size.
    if (dataSize != CRYPTO_KEY_SIZE)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    AES_ECB_decrypt(dataToDecrypt, key, decryptedData, dataSize);

    return NRF_SUCCESS;
}
