/*
 * system_utils.h
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#ifndef INC_SYSTEM_UTILS_H_
#define INC_SYSTEM_UTILS_H_

#include <stdint.h>

#define STM32_SYSTEM_BOOTLOADER_ADDRESS 0x1FF09800
#define FLASH_START_ADDRESS             0x08000000

/**
 * @brief Performs the necessary de-initialization and jumps to the system bootloader.
 */
void system_utils_jump_to_bootloader(void);

/**
 * @brief Calculates the CRC-32 hash of the application flash area.
 * @return The calculated 32-bit CRC value.
 */
uint32_t system_utils_calculate_flash_hash(void);

/**
 * @brief Gets the pre-calculated flash hash.
 * @return The 32-bit CRC value calculated at boot.
 */
uint32_t system_utils_get_flash_hash(void);

#endif /* INC_SYSTEM_UTILS_H_ */
