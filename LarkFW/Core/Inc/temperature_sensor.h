/*
 * temperature_sensor.h
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

#ifndef INC_TEMPERATURE_SENSOR_H_
#define INC_TEMPERATURE_SENSOR_H_

#include "stm32h7xx_hal.h"

// --- Public Function Prototypes ---

/**
 * @brief  Initializes the temperature sensor module and calibrates the ADC.
 * @note   This function should be called once at startup. It prepares the ADC
 *         but does not start any conversions.
 * @param  hadc_temp: Pointer to the ADC_HandleTypeDef structure that is configured
 *                    for the temperature sensor and Vrefint channels.
 * @retval HAL_StatusTypeDef: HAL_OK if initialization is successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef temp_sensor_init(ADC_HandleTypeDef *hadc_temp);

/**
 * @brief  Gets the last successfully calculated CPU temperature.
 * @note   This function is non-blocking and returns the stored value. It does not
 *         trigger a new reading.
 * @retval float: The temperature in degrees Celsius.
 */
float temp_sensor_get_temp_c(void);

/**
 * @brief  Performs periodic processing for the temperature sensor module.
 * @note   This function should be called regularly (e.g., from a main loop or
 *         timer callback). It collects ADC data and updates the stored temperature.
 */
void temp_sensor_process(void);

#endif /* INC_TEMPERATURE_SENSOR_H_ */
