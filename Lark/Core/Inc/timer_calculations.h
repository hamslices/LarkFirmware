/*
 * timer_calculations.h
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

#ifndef INC_TIMER_CALCULATIONS_H_
#define INC_TIMER_CALCULATIONS_H_

#include <stdint.h>
#include "config_manager.h"

// A safety buffer (in microseconds) to prevent the strobe pulse from running
// too close to the end of its allotted time window. This accounts for ISR latency.
#define STROBE_PULSE_SAFETY_MARGIN_US  4

// The absolute minimum duration for a strobe pulse to ensure it registers correctly.
#define MINIMUM_PULSE_WIDTH_US         1

typedef struct {
    uint32_t motor_period_ticks;
    uint32_t strobe_pulse_ticks;
} timer_params_t;

/**
 * @brief  Calculates all timer parameters based on print job settings.
 *
 * Uses the provided print job configuration to derive timer values such as
 * prescalers, periods, and duty cycles required for proper operation.
 *
 * @param  settings Pointer to a structure containing the print job settings.
 * @retval timer_params_t Structure with the computed timer parameters.
 */
timer_params_t calculate_all_timer_params(const print_job_settings_t* settings);

#endif /* INC_TIMER_CALCULATIONS_H_ */
