/*
 * crypto.h
 *
 *  Created on: Oct 4, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#ifndef INC_CRYPTO_H_
#define INC_CRYPTO_H_

#include "nrf_sdh_soc.h"

#define CRYPTO_KEY_SIZE         16

/**
 * @brief This function checks if the main key is stored in the internal memory at @ref CRYPTO_MAIN_KEY_ADDRESS
 * @return true if key already exists, false otherwise
 */
uint32_t CryptoCheckMainKey();

/**
 * @brief This function generates key of size @ref CRYPTO_KEY_SIZE.
 * @param generatedKey[in] - buffer for key
 * @param generatedKeySize[in] - generated key size
 * @return  NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES - key generation error
 *          NRF_SUCCESS
 */
uint32_t CryptoGenerateKey(uint8_t* generatedKey, uint8_t* generatedKeySize);

/**
 * @brief This function generates the main key and stores it in internal memory at @ref CRYPTO_MAIN_KEY_ADDRESS
 * @return  NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES - error during key generation
 *          NRF_ERROR_DATA_SIZE - key is stored between two pages
 *          FLASH_OP_WRITE_ERROR - key storage error
 *          NRF_SUCCESS - key stored successfully
 */
uint32_t CryptoGenerateAndStoreMainKey();

/**
 * @brief This function encrypts up to 16 bytes of data
 * @param dataToEncrypt[in] - pointer to the buffer with data which is to be encrypted
 * @param dataSize[in] - size of the data to be encrypted
 * @param key[in] - pointer to the buffer with the key
 * @param keySize[in] - key size
 * @param encryptedData[out] - pointer to the buffer with encrypted data. Encrypted data size is the same as data to encrypt buffer size
 * @return NRF_SUCCESS
 */
uint32_t CryptoEncryptData(uint8_t* dataToEncrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* encryptedData);

/**
 * @brief This function decrypts up to 16 bytes of data
 * @param dataToDecrypt[in] - pointer to the buffer with data to be decrypted
 * @param dataSize[in] - size of data to be decrypted
 * @param key[in] - pointer to the buffer holding key
 * @param keySize[in] - size of the key in bytes
 * @param decryptedData[out] - pointer to the buffer holding decrypted data
 * @return NRF_SUCCESS
 */
uint32_t CryptoDecryptData(uint8_t* dataToDecrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* decryptedData);

#endif /* INC_CRYPTO_H_ */
