/*
 * s-curve.h
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

#ifndef INC_S_CURVE_H_
#define INC_S_CURVE_H_

#include <stdint.h>
#include "config_manager.h" // For print_job_settings_t

// --- Motion Profile Configuration ---

// The physical distance over which the acceleration/deceleration ramp occurs,
// measured in equivalent print lines.
#define ACCEL_RAMP_DISTANCE_LINES   2.0f // 6 MAX

// The absolute maximum number of steps we can allocate for a ramp.
// Based on the highest resolution (1/8 step = 64 steps/line).
// NOTE: (64 steps/line * 2.0 lines = 128). 384 provides a safe margin.
#define MAX_ACCEL_STEPS             384 // 64 x 6

// The minimum speed (in pulses per second) from which the ramp will start.
#define MINIMUM_PRINT_SPEED_PPS     1000.0f

// --- Public Variable Declarations ---

// Declare the LUT as 'extern'. This makes it accessible to other files
// that include this header, like print_engine_hw.c. The actual memory for
// it is defined in s-curve.c.
extern uint32_t g_accel_profile_ticks[MAX_ACCEL_STEPS];

// --- Public Function Prototypes ---

/**
 * @brief   On-the-fly S-curve LUT generator.
 * @details This version now returns the number of calculated steps directly,
 *          avoiding the need for an output pointer.
 * @param settings The settings for the current print job.
 * @return The number of steps in the generated ramp.
 */
uint16_t generate_scurve_lut(const print_job_settings_t* settings);

#endif /* INC_S_CURVE_H_ */
