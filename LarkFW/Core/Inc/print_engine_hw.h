/*
 * print_engine_hw.h
 *
 *  Created on: Sep 4, 2025
 *      Author: HamSlices
 *
 * @attention
 *
 * Copyright (C) 2025 HamSlices
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef INC_PRINT_ENGINE_HW_H_
#define INC_PRINT_ENGINE_HW_H_

#include <stdatomic.h>
#include "stm32h7xx_hal.h"
#include "config_manager.h"
#include "print_engine_definitions.h"

// --- Public Function Prototypes ---

/**
 * @brief Initializes the hardware abstraction layer.
 * @details This function must be called once at startup. It establishes the communication
 *          links between this hardware layer and the main print engine state machine by
 *          storing pointers to shared atomic state variables.
 * @param status_ptr Pointer to the main engine's atomic status flags variable.
 * @param state_ptr Pointer to the main engine's atomic state machine variable.
 * @param step_counter_ptr Pointer to the atomic counter for steps within a print line.
 * @param move_counter_ptr Pointer to the atomic counter for steps in a manual move.
 */
void print_engine_hw_init(volatile atomic_uint_least32_t* status_ptr,
                          volatile atomic_print_engine_state_t* state_ptr,
                          volatile atomic_uint_least16_t* step_counter_ptr,
                          volatile atomic_int_least32_t* move_counter_ptr);

/**
 * @brief Applies a new set of job settings to the hardware.
 * @details Takes a high-level settings structure and configures all relevant hardware,
 *          including motor direction, current, microstepping resolution, and the
 *          calculated timer periods for motor speed and strobe pulse width.
 * @param settings A pointer to the settings structure for the current job.
 */
void print_engine_hw_apply_settings(const print_job_settings_t* settings);

/**
 * @brief Starts a print job.
 * @details This function orchestrates the beginning of a print job. It generates the
 *          S-curve acceleration profile, pre-loads the motor timer with the first
 *          step's period to ensure a correct first pulse, and enables the motor and timers.
 */
void print_engine_hw_start_printing(void);

/**
 * @brief Starts a continuous, non-printing forward feed.
 * @details Used for user-initiated actions like loading paper. The motor runs at a
 *          pre-defined speed until `print_engine_hw_stop_all()` is called.
 * @param settings Job settings configured for the manual feed (e.g., speed, current).
 */
void print_engine_hw_start_manual_feed(const print_job_settings_t* settings);

/**
 * @brief Starts a finite, non-printing motor move.
 * @details Moves the motor a specific number of microsteps, as defined by the
 *          `g_move_counter_ptr` which should be set before calling this function.
 * @param settings Job settings configured for the move (e.g., speed, direction).
 */
void print_engine_hw_start_move(const print_job_settings_t* settings);

/**
 * @brief Immediately stops all hardware activity.
 * @details This is the main hardware shutdown function. It disables all motor and
 *          strobe timers and their interrupts, and puts the motor driver into an
 *          idle, low-power state.
 */
void print_engine_hw_stop_all(void);

/**
 * @brief Gracefully requests the hardware layer to stop motor activity.
 * @details This function is non-blocking. It sets a flag that the motor step ISR
 *          will check at a safe time (the start of the next step) to stop motion
 *          without corrupting the final pulse.
 */
void print_engine_hw_request_stop(void);

/**
 * @brief Transmits one line of print data to the printhead via SPI DMA.
 * @details Initiates a non-blocking SPI transfer. It also handles flushing the
 *          D-cache to ensure the DMA controller sees the latest data from the CPU.
 * @param buffer Pointer to the buffer containing the line data.
 * @return HAL_StatusTypeDef result from the HAL_SPI_Transmit_DMA call.
 */
HAL_StatusTypeDef print_engine_hw_transmit_line(uint8_t* buffer);

// --- ISR Prototypes ---

// These functions are called directly from the interrupt vector table (e.g., in stm32h7xx_it.c)

/**
 * @brief Interrupt Service Routine for the main motor step timer (TIM3).
 * @details This is the core real-time ISR. It acts as a dispatcher, calling the
 *          appropriate handler based on the current print engine state.
 */
void print_engine_hw_step_isr(void);

/**
 * @brief Interrupt Service Routine for the watchdog timer (TIM6).
 * @details This ISR fires only if the main step ISR fails to execute in time. It
 *          indicates a system lock-up and triggers a fatal error state.
 */
void print_engine_hw_watchdog_isr(void);

/**
 * @brief Interrupt Service Routine for the strobe-off timer (TIM5).
 * @details This ISR is triggered at the end of a strobe pulse. Its only job is
 *          to turn off the strobe signals to the printhead.
 */
void print_engine_hw_strobe_off_isr(void);

/**
 * @brief Callback function executed upon SPI DMA transfer completion.
 * @details Called from the HAL SPI ISR. It signals to the main engine that the
 *          print data for the *next* line is now loaded into the printhead's
 *          shift registers and is ready to be latched.
 */
void print_engine_hw_spi_tx_complete_callback(void);

#endif /* INC_PRINT_ENGINE_HW_H_ */
