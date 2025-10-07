/*
 * temperature_sensor.h
 *
 *  Created on: Sep 10, 2025
 *      Author: HamSlices
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
