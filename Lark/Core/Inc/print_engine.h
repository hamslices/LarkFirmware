/*
 * print_engine.h
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

#ifndef INC_PRINT_ENGINE_H_
#define INC_PRINT_ENGINE_H_

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer_spsc.h"

// This struct provides a clear, boolean view of the printer's state,
// abstracting away the underlying bit flags.
typedef struct {
    uint32_t raw_flags;
    bool is_busy;
    bool is_printing;
    bool is_paused;
    bool is_awaiting_data;
    bool has_thermal_fault;
    bool has_operational_fault;
    bool has_fatal_error;
    bool has_command_fail;
    bool has_job_aborted;
} print_engine_status_t;

// --- Public Function Prototypes ---

/**
 * @brief Initializes the print engine module.
 * @param image_buffer Pointer to the main ring buffer for incoming image data.
 */
void print_engine_init(ring_buffer_spsc_t* image_buffer);

/**
 * @brief Main state machine manager for the print engine.
 * @details This function should be called repeatedly from the main application loop.
 * It checks the current state and performs the necessary actions, such as
 * starting a job, feeding the data pipeline, or handling state transitions.
 */
void print_engine_manage_state(void);

/**
 * @brief Immediately cancels the current job and cleans up.
 */
void print_engine_purge_job(void);

/**
 * @brief Decodes the raw status flags into a user-friendly struct.
 * @param status Pointer to a struct that will be populated with the decoded status.
 */
void print_engine_get_decoded_status(print_engine_status_t* status);

/**
 * @brief Gets the complete system status, including latched command failures.
 * @return A 32-bit integer with all current and latched status flags.
 */
uint32_t print_engine_get_status(void);

/**
 * @brief Gets the real-time status flags based on the current state machine.
 * @details This function is the single source of truth for translating the internal
 * `PE_STATE_*` enum into the public `STATUS_*` bit flags.
 * @return A 32-bit integer containing the combined status flags.
 */
uint32_t print_engine_get_realtime_status(void);

/**
 * @brief Updates the status variable with real-time I/O states.
 * @param io_status A bitmask of the current hardware I/O statuses.
 */
void print_engine_update_io_status(uint32_t io_status);

/**
 * @brief Triggers a fatal error state.
 */
void print_engine_set_fatal_error(void);

/**
 * @brief Clears specified latched fault flags.
 * @param flags_to_clear A bitmask of the flags to clear.
 */
void print_engine_clear_faults(uint32_t flags_to_clear);

/**
 * @brief Checks if the engine is in a state that allows a new job to start.
 * @return 1 if a new job can start, 0 otherwise.
 */
uint8_t print_engine_can_start_new_job(void);

/**
 * @brief Checks if the engine can accept new print data into its buffer.
 * @return 1 if data can be queued, 0 otherwise.
 */
uint8_t print_engine_can_queue_data(void);

/**
 * @brief Starts a self-test print job using a built-in image.
 */
void print_engine_start_self_test_job(void);

/**
 * @brief Starts a continuous, non-printing stock advance.
 */
void print_engine_advance_stock(void);

/**
 * @brief Stops a continuous stock advance.
 */
void print_engine_stop_stock_advance(void);

/**
 * @brief Starts a new user print job.
 */
void print_engine_start_user_job(void);

/**
 * @brief Checks if the print engine is currently busy with any operation.
 * @return true if the state is not IDLE, false otherwise.
 */
bool print_engine_is_busy(void);

/**
 * @brief Moves the motor a specified number of "lines".
 * @details This function initiates a non-printing motor move. It uses the user's
 * currently configured settings for speed, current, and resolution.
 * @param lines The number of lines to move. A positive value moves forward,
 *              a negative value moves in reverse.
 * @return true if the move was started successfully, false if the engine was busy.
 */
bool print_engine_move_motor(int32_t steps);

/**
 * @brief Clears all user-resettable fault and error status flags.
 * @details This function is designed to be the single point of action for a
 *          'clear faults' command, whether from a host command or a physical
 *          button press. It resets all latched fault flags and any live
 *          status bits that represent an error condition.
 */
void print_engine_clear_all_faults(void);

#endif /* INC_PRINT_ENGINE_H_ */
