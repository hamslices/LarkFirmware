/*
 * led_manager.h
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
