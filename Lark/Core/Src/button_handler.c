/*
 * button_handler.c
 *
 *  Created on: Sep 2, 2025
 *      Author: HamSlices
 */

#include <stddef.h>
#include "stm32h7xx_hal.h"
#include "button_handler.h"
#include "hardware.h"

typedef enum {
    SELF_TEST_STATE_IDLE,       // Button is up, waiting for a press
    SELF_TEST_STATE_PRESSED,    // Button is down, timing for hold duration
    SELF_TEST_STATE_TRIGGERED   // Hold duration met, action triggered, waiting for release
} self_test_button_state_t;

// --- Private Module Variables ---
static self_test_button_state_t g_self_test_state = SELF_TEST_STATE_IDLE;
static uint32_t self_test_press_timestamp = 0;
static uint8_t stock_advance_button_last_state = 0;

// A static copy of the callbacks provided during initialization.
static button_handler_callbacks_t g_callbacks;

void button_handler_init(const button_handler_callbacks_t* callbacks)
{
    g_self_test_state = SELF_TEST_STATE_IDLE;
    self_test_press_timestamp = 0;
    stock_advance_button_last_state = 0;

    // Safely copy the provided callbacks into our static struct.
    if (callbacks != NULL) {
        g_callbacks = *callbacks;
    } else {
        // If a NULL pointer is passed, zero out the struct to ensure we don't
        // attempt to call a null function pointer, which would cause a hard fault.
        g_callbacks = (button_handler_callbacks_t){
            .on_self_test_triggered    = NULL,
            .on_stock_advance_start    = NULL,
            .on_stock_advance_stop     = NULL,
			.on_clear_faults_triggered = NULL
        };
    }
}

void button_handler_poll(void)
{
    // --- 1. Handle the Self-Test / Clear Faults Button (Dual-Action State Machine) ---
    uint8_t is_btn_pressed = hardware_read_button_one();

    switch (g_self_test_state)
    {
        case SELF_TEST_STATE_IDLE:
            if (is_btn_pressed) {
                // Button was just pressed. Start the timer and change state.
                g_self_test_state = SELF_TEST_STATE_PRESSED;
                self_test_press_timestamp = HAL_GetTick();
            }
            break;

        case SELF_TEST_STATE_PRESSED:
            if (!is_btn_pressed) {
                // --- ACTION 1: SHORT PRESS (Clear Faults) ---
                // The button was released BEFORE the hold time expired.
                // This is our "clear faults" action.
                if (g_callbacks.on_clear_faults_triggered != NULL) {
                    g_callbacks.on_clear_faults_triggered();
                }
                // Reset the state machine.
                g_self_test_state = SELF_TEST_STATE_IDLE;
            }
            else if ((uint32_t)(HAL_GetTick() - self_test_press_timestamp) > SELF_TEST_HOLD_DURATION_MS) {
                // --- ACTION 2: LONG PRESS (Self-Test) ---
                // The hold time has expired and the button is still held down.
                // Trigger the self-test action.
                if (g_callbacks.on_self_test_triggered != NULL) {
                    g_callbacks.on_self_test_triggered();
                }
                // Move to the next state to wait for the button to be released.
                g_self_test_state = SELF_TEST_STATE_TRIGGERED;
            }
            break;

        case SELF_TEST_STATE_TRIGGERED:
            if (!is_btn_pressed) {
                // The self-test action was already triggered, and now the button
                // has been released. Reset to allow for another cycle.
                g_self_test_state = SELF_TEST_STATE_IDLE;
            }
            break;
    }

    // --- 2. Handle the Stock Advance Button ---
    uint8_t current_stock_button_state = hardware_read_button_two();

    if (current_stock_button_state && !stock_advance_button_last_state) {
        if (g_callbacks.on_stock_advance_start != NULL) {
            g_callbacks.on_stock_advance_start();
        }
    } else if (!current_stock_button_state && stock_advance_button_last_state) {
        if (g_callbacks.on_stock_advance_stop != NULL) {
            g_callbacks.on_stock_advance_stop();
        }
    }
    stock_advance_button_last_state = current_stock_button_state;
}
