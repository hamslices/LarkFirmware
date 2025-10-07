/*
 * timer_calculations.h
 *
 *  Created on: Sep 10, 2025
 *      Author: HamSlices
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
