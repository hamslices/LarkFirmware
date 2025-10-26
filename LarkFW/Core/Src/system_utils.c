/*
 * system_utils.c
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 *
 * @attention
 *
 * Copyright (C) 2025 HamSlices
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "usbd_cdc_if.h"
#include "system_utils.h"

/* --- Linker Script Symbols --- */
// These symbols are defined automatically by the linker script. We declare them
// here to get their memory addresses.
extern uint32_t _sidata;                // Start address of the .data initial values section in FLASH
extern uint32_t _sdata;                 // Start address of the .data section in RAM
extern uint32_t _edata;                 // End address of the .data section in RAM

// All Live Peripherals
extern SPI_HandleTypeDef  hspi1;
extern ADC_HandleTypeDef  hadc3;
extern CRC_HandleTypeDef  hcrc;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern TIM_HandleTypeDef  htim2, htim3, htim5, htim6, htim8;

volatile static uint32_t g_flash_hash = 0;

static uint32_t get_firmware_size_in_bytes(void);

// --- Public Functions ---

void system_utils_jump_to_bootloader(void)
{
    void (*SysMemBootJump)(void);

    // --- Step 1: Gracefully shut down all active peripherals ---
    // This is the new, critical safety step. We de-initialize every peripheral
    // to ensure no DMA transfers or other operations are active when we
    // reset the clocks and attempt the jump.
    USBD_DeInit(&hUsbDeviceFS);
    HAL_SPI_DeInit(&hspi1);
    HAL_TIM_Base_DeInit(&htim2);
    HAL_TIM_Base_DeInit(&htim3);
    HAL_TIM_Base_DeInit(&htim5);
    HAL_TIM_Base_DeInit(&htim6);
    HAL_TIM_Base_DeInit(&htim8);
    HAL_ADC_DeInit(&hadc3);
    // The HAL DeInit functions typically
    // handle disabling the associated DMA channels.

    // --- Step 2: Disable all interrupts globally ---
    __disable_irq();

    // --- Step 3: Disable the SysTick timer ---
    SysTick->CTRL = 0;

    // --- Step 4: De-initialize the RCC (clock system) ---
    // This is now safe to do because all peripherals have been stopped.
    HAL_RCC_DeInit();

    // --- Step 5: Forcefully clear all NVIC interrupts ---
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // --- Step 6: Disable and Clean Caches ---
    SCB_DisableICache();
    SCB_CleanInvalidateDCache();
    SCB_DisableDCache();

    // --- Step 7: Disable the MPU ---
    HAL_MPU_Disable();

    // --- Step 8: Re-enable all interrupts globally ---
    // The ST bootloader requires the global interrupt flag (PRIMASK) to be clear.
    __enable_irq();

    // --- Step 9: Set the Main Stack Pointer (MSP) ---
    __set_MSP(*(uint32_t *)STM32_SYSTEM_BOOTLOADER_ADDRESS);

    // --- Step 10: Jump to the bootloader's entry point ---
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) (STM32_SYSTEM_BOOTLOADER_ADDRESS + 4)));
    SysMemBootJump();

    // --- The code should never reach here ---
    while (1);
}

/**
 * @brief Calculates the CRC-32 hash of the application firmware in flash.
 * @note  This function is made robust against compiler optimizations by
 *        checking hardware state and using a volatile global variable.
 * @retval The calculated 32-bit hash of the firmware.
 */
uint32_t system_utils_calculate_flash_hash(void)
{
    uint32_t firmware_size_bytes = get_firmware_size_in_bytes();
    uint32_t* ptr = (uint32_t*)FLASH_START_ADDRESS;

    // 2. Reset the CRC hardware peripheral to its default state. This is good practice
    //    to ensure the calculation starts from a clean slate.
    __HAL_CRC_DR_RESET(&hcrc);

    // 3. Calculate the hash. The HAL function returns the result directly.
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, ptr, firmware_size_bytes);

    // 4. Perform the final XOR. This is a standard part of the CRC-32 algorithm.
    //    Store the result in the volatile global variable.
    g_flash_hash = crc_result ^ 0xFFFFFFFF;

    return g_flash_hash;
}

uint32_t system_utils_get_flash_hash(void)
{
    return g_flash_hash;
}

// --- Private Functions ---

static uint32_t get_firmware_size_in_bytes(void)
{
    // The start of the firmware is always the beginning of flash.
    uint32_t start_address = FLASH_START_ADDRESS;

    // The size of the .data section's image in flash is the same as its size in RAM.
    uint32_t data_size_bytes = (uint32_t)&_edata - (uint32_t)&_sdata;

    // The end address of the entire firmware image in flash is the start of the
    // .data image (_sidata) plus its size.
    uint32_t end_address = (uint32_t)&_sidata + data_size_bytes;

    // The total size is simply the difference.
    return end_address - start_address;
}

