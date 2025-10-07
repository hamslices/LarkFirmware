/*
 * button_handler.h
 *
 *  Created on: Sep 2, 2025
 *      Author: HamSlaices
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

#ifndef INC_BUTTON_HANDLER_H_
#define INC_BUTTON_HANDLER_H_

#define SELF_TEST_HOLD_DURATION_MS 5000 // 5 seconds

/**
 * @brief Defines the function signature for button event callbacks.
 * @details All button callbacks are of this type: they take no arguments
 *          and return no value.
 */
typedef void (*button_callback_t)(void);

/**
 * @brief A structure to hold pointers to all possible button event callbacks.
 */
typedef struct {
    button_callback_t on_self_test_triggered;
    button_callback_t on_stock_advance_start;
    button_callback_t on_stock_advance_stop;
    button_callback_t on_clear_faults_triggered;
} button_handler_callbacks_t;

/**
 * @brief Initializes the button handler module.
 * @param callbacks A struct containing the function pointers to call on button events.
 */
void button_handler_init(const button_handler_callbacks_t* callbacks);

/**
 * @brief Polls the hardware buttons and processes their states.
 */
void button_handler_poll(void);

#endif /* INC_BUTTON_HANDLER_H_ */
