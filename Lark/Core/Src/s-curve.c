/*
 * s-curve.c
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

#include <math.h>
#include "s-curve.h"
#include "hardware_definitions.h"

// --- Module Variables ---
// This is the actual definition and memory allocation for the LUT.
uint32_t g_accel_profile_ticks[MAX_ACCEL_STEPS];

uint16_t generate_scurve_lut(const print_job_settings_t* settings)
{
    // --- MODIFIED --- Use a local variable for the step count.
    uint16_t accel_steps = 0;

    // 1. Calculate the number of microsteps required for the ramp distance.
	accel_steps = (uint16_t)(settings->sequence_length * ACCEL_RAMP_DISTANCE_LINES);
    if (accel_steps > MAX_ACCEL_STEPS) {
    	accel_steps = MAX_ACCEL_STEPS; // Clamp to the buffer size
    }
    if (accel_steps < 2) {
        // Ramp is too short to be meaningful, return 0.
        return 0;
    }

    // 2. Determine the speed range for the S-curve.
    float max_speed_pps = settings->lines_per_second * (float)settings->sequence_length;
    float start_speed_pps = MINIMUM_PRINT_SPEED_PPS;

    // If the target speed is at or below the minimum, no ramp is needed.
    if (max_speed_pps <= start_speed_pps) {
        return 0;
    }

    float speed_range = max_speed_pps - start_speed_pps;

    // 3. Populate the LUT with calculated timer ticks for each step.
    for (uint16_t i = 0; i < accel_steps; i++) {
        // Cosine easing function to create the S-curve shape
        float phase = (float)i * M_PI / (float)(accel_steps - 1);
        float speed_pps = start_speed_pps + speed_range * (1.0f - cosf(phase)) * 0.5f;

        // Convert speed (pulses/sec) to timer period (ticks)
        if (speed_pps > 0) {
            uint32_t ticks = (uint32_t)(((float)MOTOR_TIMER_CLOCK_HZ / speed_pps) + 0.5f);
            g_accel_profile_ticks[i] = (ticks > 0) ? ticks : 1;
        } else {
            g_accel_profile_ticks[i] = (uint32_t)-1;
        }
    }

    return accel_steps;
}
