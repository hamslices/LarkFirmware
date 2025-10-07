/*
 * error_reporter.h
 *
 *  Created on: Sep 8, 2025
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

#ifndef INC_ERROR_REPORTER_H_
#define INC_ERROR_REPORTER_H_

#include <stdint.h>

/**
 * @brief A struct to hold the CPU state at the time of a hard fault.
 */
typedef struct {
    uint32_t program_counter;         // The address of the faulting instruction
    uint32_t link_register;           // The return address
    uint32_t program_status_register; // CPU flags
} hard_fault_info_t;

/**
 * @brief  Handles generic fatal errors (e.g., from HAL).
 * @note   This function will not return. It takes control of the system and
 *         begins signaling the error state visually.
 */
void error_reporter_indicate_fatal_error(void);

/**
 * @brief  Handles a specific CPU Hard Fault.
 * @note   This function is called by the low-level HardFault_Handler. It will
 *         optionally print crash details and will not return.
 * @param  fault_info A pointer to the struct containing crash details.
 */
void error_reporter_handle_hard_fault(hard_fault_info_t* fault_info);

#endif /* INC_ERROR_REPORTER_H_ */
