/*
 * led_manager.h
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#ifndef INC_LED_MANAGER_H_
#define INC_LED_MANAGER_H_

#include <stdbool.h>

/**
 * @brief Initializes the LED manager and its underlying hardware (PWM timers).
 * @return 1 on success, 0 failure.
 */
HAL_StatusTypeDef led_manager_init(void);

/**
 * @brief Updates the LED color based on the global system status.
 * @note  This function should be called repeatedly from the main loop.
 */
void led_manager_update(void);

/**
 * @brief Performs a visual "color wheel" test sequence.
 */
void led_manager_run_color_wheel(void);

#endif /* INC_LED_MANAGER_H_ */
