/*
 * button_handler.h
 *
 *  Created on: Sep 2, 2025
 *      Author: HamSlaices
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
