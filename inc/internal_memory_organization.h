/*
 * internal_memory_organization.h
 *
 *  Created on: Oct 4, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#ifndef INC_INTERNAL_MEMORY_ORGANIZATION_H_
#define INC_INTERNAL_MEMORY_ORGANIZATION_H_

#include "crypto.h"

#define INTERNAL_FLASH_PAGE_SIZE                    (uint16_t)0x1000
#define FLASH_MEMORY_END_ADDRESS                    (uint32_t*)0x80000

#define PERSISTENT_CONFIG_PAGE_ADDRESS              (uint32_t*)0x78000

#define INTERNAL_FLASH_SWAP_PAGE_ADDRESS            (uint32_t*)0x79000

#define CRYPTO_MAIN_KEY_ADDRESS                     (PERSISTENT_CONFIG_PAGE_ADDRESS)
#define KEY_TAG_ALARM_DISARMING_COMMAND_ADDRESS     (uint32_t*)((uint32_t)CRYPTO_MAIN_KEY_ADDRESS + CRYPTO_KEY_SIZE)
#endif /* INC_INTERNAL_MEMORY_ORGANIZATION_H_ */
