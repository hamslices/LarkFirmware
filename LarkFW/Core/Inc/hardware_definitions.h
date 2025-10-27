/*
 * hardware_definitions.h
 *
 *  Created on: Sep 5, 2025
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

#ifndef INC_HARDWARE_DEFINITIONS_H_
#define INC_HARDWARE_DEFINITIONS_H_

// --- Core Timing Constants ---
#define MOTOR_TIMER_CLOCK_HZ       2000000.0f  // The clock frequency for TIM3 in Hz
#define MOTOR_TIMER_CLOCK_MHZ      2.0f        // The clock frequency for TIM3 in MHz
#define STROBE_TIMER_HZ            10000000.0f // The clock frequency for TIM5 in Hz

// --- Physical Mechanism Constants ---
#define PITCH_MM                   0.125f            // Physical pitch of the mechanism in mm
#define DPMM                       (1.0f / PITCH_MM) // Dots Per Millimeter
#define DPI                        (DPMM * 25.4f)    // Dots Per Inch

// --- Motor Driver DAC Constants ---
#define V_REF_V                    3.3f       // Reference voltage for the DAC
#define V_TO_I_GAIN                0.3125f    // V/A gain of the motor driver's current sense
#define DAC_MAX_VALUE_12_BIT       4095.0f    // Max raw value for the 12-bit DAC

// The number of strobe groups fired per full electrical cycle of the motor.
#define STROBE_GROUPS_PER_CYCLE 4

// The time in nano seconds to pulse the latch
#define LATCH_TIME_NS 600

#endif /* INC_HARDWARE_DEFINITIONS_H_ */
