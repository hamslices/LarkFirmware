/*
 * print_engine.c
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

#include <stdbool.h>
#include <string.h>
#include <stdatomic.h>
#include "main.h"
#include "print_engine.h"
#include "assets.h"
#include "command_status.h"
#include "fault_logger.h"

// --- Module Variables ---

// --- Core State and Status Variables ---
// These atomic variables are shared with the hardware layer (`print_engine_hw.c`)
// to ensure safe communication between the main loop and ISR contexts.
static volatile atomic_uint_least32_t status_var              = ATOMIC_VAR_INIT(STATUS_PIPELINE_SLOT_FREE);
static volatile atomic_print_engine_state_t g_current_state   = ATOMIC_VAR_INIT(PE_STATE_IDLE);
static volatile atomic_uint_least16_t print_line_step_counter = ATOMIC_VAR_INIT(0);
static volatile atomic_int_least32_t  g_move_steps_remaining  = ATOMIC_VAR_INIT(0);

// --- Latched Fault and Command Status ---
// These flags are "sticky". Once set, they remain set until explicitly cleared.
// This ensures that transient errors are not missed by the host system.
static uint32_t g_latched_fault_flags       = 0; // Caches hardware faults.

// --- Job Management Variables ---
static uint32_t delay_entry_timestamp       = 0; // Timestamp for the start of the BUFFERING state.
static uint16_t self_test_line_counter      = 0; // Tracks progress through the embedded self-test image.

// --- Buffers and Pointers ---
static __attribute__((section(".cacheless_buffers"))) __attribute__((aligned(32)))
    uint8_t spi_transmit_buffer[SPI_DATA_SIZE_PER_LINE];    // uncached buffer
static ring_buffer_spsc_t* image_line_buffer_ptr = NULL;    // Pointer to the main image data ring buffer.

// --- Temporary Job Settings ---
// Used for operations like self-tests or manual feeds that need to override
// the user's settings without permanently changing them.
static bool g_is_temporary_job = false;
static print_job_settings_t g_active_job_settings; // The settings for the currently running job.
static print_job_settings_t g_saved_user_settings; // A backup of the user's settings.

// A short delay after a print job starts to allow the data buffer to fill
// slightly, preventing data underruns on the first line.
#define PRE_PRINT_BUFFER_DELAY_MS  500

// --- Private Function Prototypes ---
static void _begin_temporary_job(void);
static bool _start_job_if_ready(void);
static uint32_t _get_status_snapshot(void);

static void stop_and_cleanup_print_job(void);
static void pipeline_next_line(void);
static void manage_self_test_data_load(void);
static uint16_t calculate_sequence_length(uint32_t resolution);

static uint8_t has_thermal_fault(uint32_t status);
static uint8_t has_operational_fault(uint32_t status);
static uint8_t has_fatal_error(uint32_t status);

// --- Public Function Implementations ---

void print_engine_init(ring_buffer_spsc_t* image_buffer)
{
    image_line_buffer_ptr = image_buffer;

    fault_logger_init();

    // Pass pointers to our shared atomic variables to the hardware layer.
    print_engine_hw_init(&status_var, &g_current_state,
    		&print_line_step_counter, &g_move_steps_remaining);

    // Load the user's default settings from flash into the active configuration.
    config_manager_get_current_settings(&g_active_job_settings);
    print_engine_hw_apply_settings(&g_active_job_settings);

    // Perform an initial cleanup to ensure the engine starts in a known, clean state.
    stop_and_cleanup_print_job();
}

void print_engine_clear_all_faults(void)
{
    // Define a bitmask of all flags that represent a "fault" condition
    // that a user can and should be able to clear. This includes the main
    // latched flags as well as the real-time hardware flags that cause them.
    const uint32_t all_clearable_flags =
        STATUS_HEAD_OVER_TEMP   | // Real-time thermal fault
        STATUS_MOTOR_OVER_TEMP  | // Real-time thermal fault
        STATUS_NO_STOCK         | // Real-time operational fault
        STATUS_HEAD_UP          | // Real-time operational fault
        STATUS_FATAL_ERROR      | // Latched fatal fault
        STATUS_DATA_UNDERRUN    | // Latched informational job failure reason
        STATUS_COMMAND_FAIL     | // Latched command processing fault
        STATUS_JOB_ABORTED;       // Latched operational fault

    // Call the underlying function that handles the atomic clearing of the
    // bits from both the live status variable and the sticky fault register.
    print_engine_clear_faults(all_clearable_flags);
}

void print_engine_manage_state(void)
{
    print_engine_state_t state = atomic_load(&g_current_state);
    print_engine_status_t current_status;
    print_engine_get_decoded_status(&current_status); // Get decoded status once for efficiency.

    // --- UNIFIED FAULT HANDLING ---
    if ((state == PE_STATE_RAMP_UP   ||
    	 state == PE_STATE_PRINTING  ||
		 state == PE_STATE_FINISHING)) {

    	// 1. Critical (System-Locking) Faults
    	// Check IF a thermal fault is active AND we have NOT already latched a fatal error.
    	if (current_status.has_thermal_fault && !(g_latched_fault_flags & STATUS_FATAL_ERROR)) {
    	    // This is the first time we've seen this fault.
    	    g_latched_fault_flags |= STATUS_FATAL_ERROR;
    	    fault_logger_log_event(atomic_load(&status_var), INTERNAL_CMD_RESULT_NONE);
    	    atomic_store(&g_current_state, PE_STATE_FINISHING);
    	    return; // Exit to allow the state change to take effect
    	}

    	// 2. Operational (Job-Aborting) Faults
    	// Check IF an operational fault is active AND we have NOT already latched a job abort.
    	if (current_status.has_operational_fault && !(g_latched_fault_flags & STATUS_JOB_ABORTED)) {
    	    // This is the first time we've seen this fault.
    	    g_latched_fault_flags |= STATUS_JOB_ABORTED;
    	    fault_logger_log_event(atomic_load(&status_var), INTERNAL_CMD_RESULT_NONE);
    	    atomic_store(&g_current_state, PE_STATE_FINISHING);
    	    return; // Exit to allow the state change to take effect
    	}
    }

    // --- SELF-TEST DATA LOADING ---
    // If a self-test is active, this continuously feeds lines from the
    // embedded test image into the print buffer whenever there is space.
    if (atomic_load(&status_var) & STATUS_SELF_TEST_ACTIVE) {
        manage_self_test_data_load();
    }

    // --- CORE STATE MACHINE ---
    switch (state) {
        case PE_STATE_IDLE:
            break;

        case PE_STATE_BUFFERING:
            // This state provides a brief delay before starting the motor. This allows
            // the host to send the first few lines of image data, preventing an
            // immediate data underrun when the printer starts moving.
            if (delay_entry_timestamp == 0) { delay_entry_timestamp = HAL_GetTick(); }
            if (HAL_GetTick() - delay_entry_timestamp > PRE_PRINT_BUFFER_DELAY_MS ) {
                // If this is a normal user job (not a self-test), load the latest user settings.
                if (!(atomic_load(&status_var) & STATUS_SELF_TEST_ACTIVE)) {
                    config_manager_get_current_settings(&g_active_job_settings);
                    g_active_job_settings.sequence_length = calculate_sequence_length(g_active_job_settings.step_resolution);
                }
                // Apply settings, start the hardware, and transition to the next state.
                print_engine_hw_apply_settings(&g_active_job_settings);
                print_engine_hw_start_printing();
                atomic_store(&g_current_state, PE_STATE_AWAITING_RUN_UP_START);
            }
            break;

        case PE_STATE_AWAITING_RUN_UP_START:
            break;

        case PE_STATE_RAMP_UP:
        case PE_STATE_PRINTING:
            // In the main printing states, check if the print job is complete.
            // A job is finished when the data pipeline is free (last line sent) AND
            // the image data buffer is empty (no more lines to send).
            if ((atomic_load(&status_var) & STATUS_PIPELINE_SLOT_FREE) &&
                (ring_buffer_spsc_used_space(image_line_buffer_ptr) == 0)) {
                atomic_store(&g_current_state, PE_STATE_FINISHING);
            } else {
                // Otherwise, keep feeding the next line of data to the hardware.
                pipeline_next_line();
            }
            break;

        case PE_STATE_FINISHING:
        case PE_STATE_RAMP_DOWN:
            break;

        case PE_STATE_JOB_DONE:
            // The ISRs have signaled that the job is fully complete.
            // Call the cleanup function to reset everything for the next job.
            stop_and_cleanup_print_job();
            break;

        case PE_STATE_MANUAL_FEED:
        case PE_STATE_MANUAL_MOVE:
            // For these states, all work is done by the ISRs. The main loop
            // simply waits until the ISR transitions the state back to IDLE or JOB_DONE.
            break;

        case PE_STATE_FAULT:
            // A fatal, unrecoverable error has occurred. Stop everything.
        	stop_and_cleanup_print_job();
            break;
    }
}

bool print_engine_move_motor(int32_t lines)
{
    if (!print_engine_can_start_new_job()) { return false; }

    // --- TEMPORARY JOB PATTERN: START ---
    _begin_temporary_job();

    // 2. The active settings for this job will be the user's settings (speed, resolution, etc.).
    g_active_job_settings = g_saved_user_settings;

    // 3. CRITICAL: Override ONLY the parameters that are essential for a non-printing move.
    //    We must not energize the printhead, so darkness is set to zero.
    g_active_job_settings.darkness = 0.0f;

    // 4. Calculate dependent values based on the USER'S loaded resolution setting.
    g_active_job_settings.sequence_length = calculate_sequence_length(g_active_job_settings.step_resolution);

    // 5. Calculate the total number of microsteps required for the move.
    int32_t steps_to_move = lines * g_active_job_settings.sequence_length;

    // Set the direction and load the atomic step counter for the ISR.
    if (steps_to_move > 0) {
        g_active_job_settings.direction = DIRECTION_FORWARD;
        atomic_store(&g_move_steps_remaining, steps_to_move);
    } else {
        g_active_job_settings.direction = DIRECTION_REVERSE;
        atomic_store(&g_move_steps_remaining, -steps_to_move);
    }

    // 6. Apply the settings and start the move.
    atomic_fetch_or(&status_var, STATUS_BUSY);
    atomic_store(&g_current_state, PE_STATE_MANUAL_MOVE);
    print_engine_hw_start_move(&g_active_job_settings);
    // --- TEMPORARY JOB PATTERN: END ---

    return true;
}

void print_engine_get_decoded_status(print_engine_status_t* status)
{
    if (!status) { return; }

    // Get the complete, raw status value first.
    status->raw_flags = _get_status_snapshot();

    // Decode the raw flags into easy-to-use booleans.
    status->is_busy             = (status->raw_flags & STATUS_BUSY)                != 0;
    status->is_printing         = (status->raw_flags & STATUS_PRINTING)            != 0;
    status->is_awaiting_data    = (status->raw_flags & STATUS_AWAITING_FIRST_LINE) != 0;
    status->has_command_fail    = (status->raw_flags & STATUS_COMMAND_FAIL)        != 0;
    status->has_job_aborted     = (status->raw_flags & STATUS_JOB_ABORTED)         != 0;

    // Use the static helpers for fault conditions.
    status->has_thermal_fault     = has_thermal_fault(status->raw_flags);
    status->has_operational_fault = has_operational_fault(status->raw_flags);
    status->has_fatal_error       = has_fatal_error(status->raw_flags);
}

void print_engine_start_user_job(void)
{
    // --- NEW GUARD CLAUSE ---
    // This function is called for every image data packet. We only want to
    // attempt to start the job on the VERY FIRST packet, when the engine
    // is still idle. If it's already busy, we must do nothing and exit.
    if (atomic_load(&g_current_state) != PE_STATE_IDLE) {
        return;
    }

    // If we are here, the state is IDLE, so we can now safely proceed with
    // the logic that checks for pre-existing faults and starts the job.
    _start_job_if_ready();
}

void print_engine_purge_job(void)
{
    // Don't stop immediately. Instead, request a graceful shutdown by
    // transitioning to the FINISHING state. The main state machine and ISRs
    // will handle the ramp-down and final cleanup automatically.
    print_engine_state_t current_state = atomic_load(&g_current_state);

    // Only trigger the shutdown if a job is actually running.
    if (current_state == PE_STATE_RAMP_UP || current_state == PE_STATE_PRINTING) {
        atomic_store(&g_current_state, PE_STATE_FINISHING);
    } else {
        stop_and_cleanup_print_job();
    }
}

bool print_engine_is_busy(void)
{
	return (atomic_load(&g_current_state) != PE_STATE_IDLE);
}

uint32_t print_engine_get_realtime_status(void)
{
    // Start with the base flags that are set by ISRs (e.g., hardware faults).
	uint32_t status_flags = atomic_load(&status_var);

    // Add flags based on the current high-level state.
    switch (atomic_load(&g_current_state)) {
        case PE_STATE_IDLE:
        case PE_STATE_JOB_DONE:
            break; // No extra flags needed.
        case PE_STATE_BUFFERING:
        case PE_STATE_AWAITING_RUN_UP_START:
            status_flags |= STATUS_BUSY | STATUS_AWAITING_FIRST_LINE;
            break;
        case PE_STATE_RAMP_UP:
        case PE_STATE_PRINTING:
        case PE_STATE_FINISHING:
        case PE_STATE_RAMP_DOWN:
            status_flags |= STATUS_BUSY | STATUS_PRINTING;
            break;
        case PE_STATE_MANUAL_MOVE:
        	status_flags |= STATUS_BUSY | STATUS_MANUAL_MOVE_ACTIVE;
        	break;
        case PE_STATE_MANUAL_FEED:
            status_flags |= STATUS_BUSY | STATUS_MANUAL_FEED_ACTIVE;
            break;
        case PE_STATE_FAULT:
            status_flags |= STATUS_BUSY;
            break;
    }
    return status_flags;
}

uint32_t print_engine_get_status(void)
{
    // 1. Get the hardware and state machine status as usual.
    uint32_t realtime_status = print_engine_get_realtime_status();
    uint32_t final_status = realtime_status | g_latched_fault_flags;

    // 2. Atomically read and clear the result of the last command.
    internal_command_result_t cmd_result = command_status_get_result();

    // 3. If the last command failed, add the transient flag to THIS response only.
    if (cmd_result != INTERNAL_CMD_RESULT_NONE && cmd_result != INTERNAL_CMD_RESULT_SUCCESS) {
        // a) Add the TRANSIENT status bit to the response for the host.
        final_status |= STATUS_COMMAND_FAIL;

        // b) Log the event for diagnostic purposes.
        //    We log `final_status` because it represents the complete picture of the
        //    device state at the moment the host is being notified of the failure.
        fault_logger_log_event(final_status, cmd_result);
    }

    return final_status;
}

void print_engine_update_io_status(uint32_t io_status)
{
    // This mask defines which flags are managed by this polling function.
    const uint32_t polling_mask = STATUS_HEAD_UP         |
    							  STATUS_NO_STOCK        |
								  STATUS_MOTOR_OVER_TEMP |
								  STATUS_HEAD_OVER_TEMP  |
								  STATUS_MARK;

    // Atomically clear the old polling flags and then set the new ones.
    atomic_fetch_and(&status_var, ~polling_mask);
    atomic_fetch_or(&status_var, io_status);
}

void print_engine_set_fatal_error(void)
{
    // 1. Latch the fatal error flag. This is the "memory" of what happened.
    atomic_fetch_or(&status_var, STATUS_FATAL_ERROR);
    g_latched_fault_flags |= STATUS_FATAL_ERROR;
    atomic_store(&g_current_state, PE_STATE_FINISHING);
}

void print_engine_clear_faults(uint32_t flags_to_clear)
{
    // --- Step 1: Clear the specified bits from the single sticky fault register ---
    g_latched_fault_flags &= ~flags_to_clear;

    // --- Step 2: Atomically clear the same flags from the public status variable ---
    // This is still needed to clear the bits from the live status value.
    atomic_fetch_and(&status_var, ~flags_to_clear);

    // --- Step 3: Handle state machine recovery if a system-locking fault was cleared ---
    if ((flags_to_clear & STATUS_FATAL_ERROR) &&
    	(atomic_load(&g_current_state) == PE_STATE_FAULT))
    {
        // Unlock the state machine and return to a usable IDLE state.
        atomic_store(&g_current_state, PE_STATE_IDLE);
    }
}

uint8_t print_engine_can_start_new_job(void)
{
    print_engine_status_t current_status;
    print_engine_get_decoded_status(&current_status);

    return !current_status.is_busy               &&
           !current_status.has_fatal_error       &&
           !current_status.has_thermal_fault     &&
           !current_status.has_operational_fault &&
		   !current_status.has_job_aborted;
}

uint8_t print_engine_can_queue_data(void)
{
    print_engine_status_t current_status;
    print_engine_get_decoded_status(&current_status);
    // Generally, data can be queued as long as there isn't an error.
    return !current_status.has_fatal_error       &&
    	   !current_status.has_operational_fault &&
		   !current_status.has_job_aborted;
}

void print_engine_start_self_test_job(void)
{
    // Use the unified starter function. It performs the check AND starts the state machine.
    if (_start_job_if_ready()) {
        // If the job successfully started, now we configure it for a self-test.
        _begin_temporary_job();

        // Load the hardcoded settings for the self-test print.
        g_active_job_settings.lines_per_second   = 203.2f;
        g_active_job_settings.darkness           = 80.0f;
        g_active_job_settings.step_resolution    = RESOLUTION_QUARTER_STEP;
        g_active_job_settings.direction          = DIRECTION_FORWARD;
        g_active_job_settings.motor_current_ma   = 800;
        g_active_job_settings.lut_pointer        = NULL;
        g_active_job_settings.sequence_length    = calculate_sequence_length(g_active_job_settings.step_resolution);

        print_engine_hw_apply_settings(&g_active_job_settings);
        self_test_line_counter = 0;
        atomic_fetch_or(&status_var, STATUS_SELF_TEST_ACTIVE);
    }
}

void print_engine_advance_stock(void)
{
	if (!print_engine_can_start_new_job()) { return; }

    // This is a temporary job. Save user settings.
    _begin_temporary_job();

    // Load hardcoded settings for a fast, non-printing feed.
    g_active_job_settings.lines_per_second  = 100.0f;
    g_active_job_settings.darkness          = 0.0f;
    g_active_job_settings.step_resolution   = RESOLUTION_QUARTER_STEP;
    g_active_job_settings.direction         = DIRECTION_FORWARD;
    g_active_job_settings.motor_current_ma  = 800;
    g_active_job_settings.lut_pointer       = NULL;
    g_active_job_settings.sequence_length   = calculate_sequence_length(g_active_job_settings.step_resolution);

    atomic_fetch_or(&status_var, STATUS_BUSY);
    atomic_store(&g_current_state, PE_STATE_MANUAL_FEED);
    print_engine_hw_start_manual_feed(&g_active_job_settings);
}

void print_engine_stop_stock_advance(void)
{
    print_engine_state_t current_state = atomic_load(&g_current_state);

    // Handle both manual move and manual feed
    // This now gracefully stops either type of manual motor activity.
    if (current_state == PE_STATE_MANUAL_FEED || current_state == PE_STATE_MANUAL_MOVE) {
        // Instead of forcing an immediate stop, we now request a graceful shutdown.
        // The ISR will handle the rest at the next safe interval.
        print_engine_hw_request_stop();
    }
}

// --- Private (Static) Functions ---

/**
 * @brief Gets the system status without clearing the read and consume command status
 */
static uint32_t _get_status_snapshot(void)
{
    // This function builds the full status word WITHOUT clearing any flags.
    uint32_t base_status = print_engine_get_realtime_status() | g_latched_fault_flags;
    internal_command_result_t cmd_result = command_status_peek(); // Use the new peek function

    if (cmd_result != INTERNAL_CMD_RESULT_NONE && cmd_result != INTERNAL_CMD_RESULT_SUCCESS) {
        base_status |= STATUS_COMMAND_FAIL;
    }

    return base_status;
}

/**
 * @brief Prepares the engine for a temporary job that overrides user settings.
 * @details This function saves the current active settings and sets a flag
 *          indicating that they should be restored upon job completion.
 */
static void _begin_temporary_job(void)
{
    config_manager_get_current_settings(&g_saved_user_settings);
    g_is_temporary_job = true;
}

/**
 * @brief Central gatekeeper to start any new job.
 * @details Checks if the engine is in a valid state. If yes, it atomically
 *          transitions the state machine to BUFFERING. If no, it latches the
 *          STATUS_JOB_ABORTED flag to ensure the failure is reported.
 * @return true if the job was successfully started, false otherwise.
 */
static bool _start_job_if_ready(void)
{
    if (print_engine_can_start_new_job()) {
        // State is good, atomically start the job.
        print_engine_state_t expected_state = PE_STATE_IDLE;
        if (atomic_compare_exchange_strong(&g_current_state, &expected_state, PE_STATE_BUFFERING)) {
            // This is the true, unambiguous start of a new job.
            // Increment the counter exactly once.
            config_manager_increment_print_jobs_started();
            return true;
        }
        return false;
    }
    else {
        // State is not good. Latch the abort flag to report the failure.
        g_latched_fault_flags |= STATUS_JOB_ABORTED;
        atomic_fetch_or(&status_var, STATUS_JOB_ABORTED);
        return false;
    }
}

/**
 * @brief Central cleanup function to stop all activity and reset state variables.
 */
static void stop_and_cleanup_print_job(void)
{
    print_engine_hw_stop_all();
    if (image_line_buffer_ptr) {
        ring_buffer_spsc_purge(image_line_buffer_ptr);
    }

    // Define a mask of all flags that should be cleared when any job ends.
    // This prevents stale state from ISRs that might complete after the stop command.
    const uint32_t job_flags_to_clear = STATUS_SELF_TEST_ACTIVE     |
                                        STATUS_NEXT_LINE_DATA_READY |
                                        STATUS_BUSY                 |
                                        STATUS_PRINTING             |
                                        STATUS_AWAITING_FIRST_LINE  |
                                        STATUS_DATA_UNDERRUN;

    // Atomically clear all job-related flags and set clean state flags.
    atomic_fetch_and(&status_var, ~job_flags_to_clear);
    atomic_fetch_or(&status_var, STATUS_PIPELINE_SLOT_FREE);

    // --- NEW, INTELLIGENT STATE TRANSITION ---
    if (g_latched_fault_flags & STATUS_FATAL_ERROR) {
        atomic_store(&g_current_state, PE_STATE_FAULT);
    } else {
        atomic_store(&g_current_state, PE_STATE_IDLE);
    }

    // Reset all job-specific counters and timers.
    delay_entry_timestamp    = 0;
    self_test_line_counter   = 0;

    atomic_store(&print_line_step_counter, 0);
    atomic_store(&g_move_steps_remaining, 0);

    // If the completed job was temporary, restore the original user settings.
    // This completes the "temporary job" pattern started by _begin_temporary_job().
    if (g_is_temporary_job) {
        g_active_job_settings = g_saved_user_settings;
        print_engine_hw_apply_settings(&g_active_job_settings);
        g_is_temporary_job = false;
    }

    // --- SAVE THE NVM DATA HERE ---
    // This is the ideal location. This function is the final step for every job,
    // so calling save here commits all the counter increments (lines printed,
    // motor steps) that occurred during that job.
    config_manager_save_counters_to_flash();
}

/**
 * @brief Feeds the next line of data to the hardware if conditions are met.
 */
static void pipeline_next_line(void)
{
    // Check if the hardware's data pipeline has a free slot AND
    // if there is at least one full line of data in our software buffer.
    if ((atomic_load(&status_var) & STATUS_PIPELINE_SLOT_FREE) &&
        (ring_buffer_spsc_used_space(image_line_buffer_ptr) >= SPI_DATA_SIZE_PER_LINE))
    {
        // Mark the pipeline as busy.
        atomic_fetch_and(&status_var, ~STATUS_PIPELINE_SLOT_FREE);

        // Read the data and initiate the SPI DMA transfer.
        memset(spi_transmit_buffer, 0x00, sizeof(spi_transmit_buffer));
        ring_buffer_spsc_read_block(image_line_buffer_ptr,
        		spi_transmit_buffer, sizeof(spi_transmit_buffer));

        if (print_engine_hw_transmit_line(spi_transmit_buffer) != HAL_OK) {
            // If DMA fails to start, it's a critical error.
            stop_and_cleanup_print_job();
        }
    }
}

/**
 * @brief Manages loading data for a self-test print.
 */
static void manage_self_test_data_load(void)
{
    // If we haven't reached the end of the image AND there's space in the buffer...
    if ((self_test_line_counter < SELF_TEST_IMAGE_HEIGHT_PX) &&
        (ring_buffer_spsc_free_space(image_line_buffer_ptr) >= SPI_DATA_SIZE_PER_LINE))
    {
        // ...copy the next line from the embedded binary asset into the buffer.
        const uint8_t* data_ptr = _binary_self_test_image_bin_start + (self_test_line_counter * SPI_DATA_SIZE_PER_LINE);
        ring_buffer_spsc_write_block(image_line_buffer_ptr, data_ptr, SPI_DATA_SIZE_PER_LINE);
        self_test_line_counter++;
    }
}

/**
 * @brief Helper to check for thermal fault flags.
 */
static uint8_t has_thermal_fault(uint32_t status)
{
    return (status & (STATUS_MOTOR_OVER_TEMP | STATUS_HEAD_OVER_TEMP)) != 0;
}

/**
 * @brief Helper to check for operational fault flags.
 */
static uint8_t has_operational_fault(uint32_t status)
{
    return (status & (STATUS_HEAD_UP | STATUS_NO_STOCK)) != 0;
}

/**
 * @brief Helper to check for the fatal error flag.
 */
static uint8_t has_fatal_error(uint32_t status)
{
    return (status & STATUS_FATAL_ERROR) != 0;
}

/**
 * @brief Calculates the number of microsteps in a full electrical cycle (one "line")
 *        for a given resolution.
 */
static uint16_t calculate_sequence_length(uint32_t resolution)
{
    switch(resolution) {
        case RESOLUTION_FULL_STEP:    return 8;
        case RESOLUTION_HALF_STEP:    return 16;
        case RESOLUTION_QUARTER_STEP: return 32;
        case RESOLUTION_EIGHTH_STEP:  return 64;
    }
    return 32; // Safe default
}
