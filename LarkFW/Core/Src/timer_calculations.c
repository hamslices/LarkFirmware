/*
 * timer_calculations.c
 *
 *  Created on: Sep 10, 2025
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

#include "timer_calculations.h"
#include "hardware_definitions.h"

// --- Private Helper Function Prototypes ---

static uint32_t calculate_motor_period_ticks(float lines_per_second, uint16_t sequence_length);
static float convert_motor_ticks_to_us(uint32_t motor_ticks);
static uint32_t calculate_strobe_pulse_ticks(const print_job_settings_t* settings, float actual_step_period_us);

// --- Public Function ---

/**
 * @brief This function orchestrates calls to smaller helpers.
 */
timer_params_t calculate_all_timer_params(const print_job_settings_t* settings)
{
    timer_params_t params = {0, 0};

    // 1. Calculate the base motor timer period.
    params.motor_period_ticks = calculate_motor_period_ticks(settings->lines_per_second,
                                                             settings->sequence_length);
    if (params.motor_period_ticks == 0) {
        // If speed is zero, all other params are zero.
        return params;
    }

    // 2. Convert the integer ticks back to the actual, precise step time in microseconds.
    //    This is crucial for the strobe calculation to be accurate.
    float actual_step_period_us = convert_motor_ticks_to_us(params.motor_period_ticks);

    // 3. Calculate the strobe pulse duration based on the actual motor timing.
    params.strobe_pulse_ticks = calculate_strobe_pulse_ticks(settings, actual_step_period_us);

    return params;
}


// --- Private Helper Function Implementations ---

/**
 * @brief Calculates the motor timer's auto-reload value from print settings.
 * @param lines_per_second The target print speed.
 * @param sequence_length The number of microsteps per line for the current resolution.
 * @return The calculated timer ticks, or 0 if speed is invalid.
 */
static uint32_t calculate_motor_period_ticks(float lines_per_second, uint16_t sequence_length)
{
    if (lines_per_second <= 0.0f || sequence_length == 0) {
        return 0;
    }

    float ideal_step_period_us = (1000000.0f / lines_per_second) / (float)sequence_length;
    uint32_t motor_ticks = (uint32_t)((ideal_step_period_us * MOTOR_TIMER_CLOCK_MHZ) - 1.0f + 0.5f);

    return (motor_ticks > 0) ? motor_ticks : 1; // Prevent a period of zero
}

/**
 * @brief Converts a timer period in ticks back to a time in microseconds.
 * @param motor_ticks The value of the timer's auto-reload register.
 * @return The actual period of one timer cycle in microseconds.
 */
static float convert_motor_ticks_to_us(uint32_t motor_ticks)
{
    return ((float)(motor_ticks + 1) / MOTOR_TIMER_CLOCK_MHZ);
}

/**
 * @brief Calculates the strobe pulse duration in timer ticks.
 * @param settings The current job settings (contains darkness).
 * @param actual_step_period_us The precise, quantized time for each motor step.
 * @return The calculated number of ticks for the strobe timer's compare register.
 */
static uint32_t calculate_strobe_pulse_ticks(const print_job_settings_t* settings, float actual_step_period_us)
{
    if (settings->darkness <= 0.0f) {
        return 0;
    }

    // Normalize darkness from [0, 100] to [0.0, 1.0].
    float darkness_norm = settings->darkness / 100.0f;
    if (darkness_norm > 1.0f) darkness_norm = 1.0f;

    // Calculate the total time window available for a strobe group.
    uint16_t steps_per_group = settings->sequence_length / STROBE_GROUPS_PER_CYCLE;
    float total_group_time_us = actual_step_period_us * (float)steps_per_group;

    // Convert timings to strobe timer ticks.
    const float ticks_per_us = (float)STROBE_TIMER_HZ / 1000000.0f;
    const uint32_t total_available_ticks = (uint32_t)(total_group_time_us           * ticks_per_us);
    const uint32_t min_pulse_ticks       = (uint32_t)(MINIMUM_PULSE_WIDTH_US        * ticks_per_us);
    const uint32_t margin_ticks          = (uint32_t)(STROBE_PULSE_SAFETY_MARGIN_US * ticks_per_us);

    uint32_t max_allowed_pulse_ticks = (total_available_ticks > margin_ticks) ? (total_available_ticks - margin_ticks) : 0;

    if (max_allowed_pulse_ticks == 0) {
        return 0;
    }

    uint32_t final_pulse_ticks = (uint32_t)(darkness_norm * max_allowed_pulse_ticks);

    // Clamp the final value to the allowed range.
    if (final_pulse_ticks < min_pulse_ticks)         final_pulse_ticks = min_pulse_ticks;
    if (final_pulse_ticks > max_allowed_pulse_ticks) final_pulse_ticks = max_allowed_pulse_ticks;

    return final_pulse_ticks;
}
