/*
 * config_manager.h
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

#ifndef INC_CONFIG_MANAGER_H_
#define INC_CONFIG_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "step_strobe_luts.h"

// Publicly exposed configuration parameter indices
typedef enum {
    CONFIG_LINES_PER_SECOND,
    CONFIG_MOTOR_CURRENT,
    CONFIG_STEP_RESOLUTION,
    CONFIG_DIRECTION,
    CONFIG_DARKNESS,
    CONFIG_DEBUGING,
    CONFIG_PARAM_COUNT
} config_param_t;

// Publicly exposed values for specific configurations
typedef enum {
    RESOLUTION_FULL_STEP,
    RESOLUTION_HALF_STEP,
    RESOLUTION_QUARTER_STEP,
    RESOLUTION_EIGHTH_STEP
} resolution_t;

typedef enum {
    DIRECTION_FORWARD,
    DIRECTION_REVERSE
} direction_t;

// Define a structure specifically for the counters
typedef struct __attribute__((packed)) {
    uint64_t lines_printed;         // Total print head lines energized (64-bit to prevent rollover)
    uint64_t motor_steps_moved;     // Total motor steps commanded (64-bit)
    uint32_t power_on_cycles;       // How many times the device has been booted
    uint32_t print_jobs_started;    // How many times a print command was received
    uint32_t user_settings_saves;   // How many times user settings were saved
    uint32_t counter_snapshots;     // How many times these counters were saved
} SystemCounters_t;

// This struct holds all the settings needed to run a single print job.
typedef struct {
    float lines_per_second;
    float darkness;
    uint32_t step_resolution;
    uint32_t direction;
    uint16_t sequence_length;
    uint32_t motor_current_ma;
    IO_State const* lut_pointer;
} print_job_settings_t;

// The structure of a single user settings record stored in flash.
typedef struct __attribute__((packed)) {
    uint32_t magic_number;
    uint32_t sequence_number;
    uint32_t config_table[CONFIG_PARAM_COUNT];
    uint32_t crc32;
} __attribute__((aligned(32))) persistent_settings_t;

// The structure of a single system counters record stored in flash.
typedef struct __attribute__((packed)) {
    uint32_t magic_number;
    uint32_t sequence_number;
    SystemCounters_t system_counters;
    uint32_t crc32;
} __attribute__((aligned(32))) persistent_counters_t;


/* --- Public Function Prototypes --- */

/**
 * @brief Initializes the configuration manager. MUST be called once at startup.
 */
void config_manager_init(void);

/**
 * @brief Loads the most recent user settings from NVM into RAM.
 * @return `true` if a valid configuration was found and loaded, `false` otherwise.
 */
bool config_manager_load_from_flash(void);

/**
 * @brief Saves the current in-RAM user settings to the next available slot in NVM.
 * @return `true` if the save operation was successful, `false` on failure.
 */
bool config_manager_save_to_flash(void);

/**
 * @brief Saves the current in-RAM system counters to the next available slot in NVM.
 * @return `true` if the save operation was successful, `false` on failure.
 */
bool config_manager_save_counters_to_flash(void);

/**
 * @brief Erases the user settings and system counters NVM sectors and reloads defaults.
 * @return `true` if the erase was successful, `false` otherwise.
 */
bool config_manager_erase_from_flash(void);

/**
 * @brief Sets a configuration parameter in the live RAM table, with value clamping.
 * @param index The `config_param_t` enum of the parameter to set.
 * @param value The new 32-bit value for the parameter.
 */
void config_manager_set(config_param_t index, uint32_t value);

/**
 * @brief Gets a configuration parameter from the live RAM table.
 * @param index The `config_param_t` enum of the parameter to get.
 * @return The current 32-bit value of the parameter.
 */
uint32_t config_manager_get(config_param_t index);

/**
 * @brief Populates a settings struct with the current live configuration values.
 * @param settings Pointer to a `print_job_settings_t` struct to be filled.
 */
void config_manager_get_current_settings(print_job_settings_t* settings);


/* --- System Counter Management --- */

/**
 * @brief Increments the total lines printed counter in RAM.
 * @param amount The number of lines to add to the counter.
 */
void config_manager_increment_lines_printed(uint32_t amount);

/**
 * @brief Accumulates microsteps to track the equivalent number of full motor steps.
 * @param microsteps_per_full_step The number of microsteps that constitute one full motor step.
 */
void config_manager_increment_motor_steps(uint32_t microsteps_per_full_step);

/**
 * @brief Increments the print jobs started counter in RAM.
 */
void config_manager_increment_print_jobs_started(void);

/**
 * @brief Gets a constant pointer to the live system counters struct.
 * @return A const pointer to the `SystemCounters_t` struct in RAM.
 */
const SystemCounters_t* config_manager_get_counters(void);

#endif /* INC_CONFIG_MANAGER_H_ */
