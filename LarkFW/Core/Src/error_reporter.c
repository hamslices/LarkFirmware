/*
 * error_reporter.c
 *
 *  Created on: Sep 8, 2025
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

#include "main.h"
#include "error_reporter.h"

// STEP 1: DEFINE THE DESIRED TIME UNIT (This is the only line you need to tune)
// This is the base "time unit" for the Morse code blinker in milliseconds.
#define DESIRED_TIME_UNIT_MS          150

// STEP 2: DEFINE THE CALIBRATION CONSTANTS (Do not change these)
#define MEASURED_DELAY_ITERATIONS     1000000
#define MEASURED_DELAY_MS             10

// STEP 3: CALCULATE THE FINAL LOOP COUNT (Automatic)
// This macro calculates the necessary loop iterations to achieve the desired delay.
// It uses 64-bit intermediates (LL) to prevent overflow during calculation.
#define ERROR_DELAY_ITERATIONS ( (MEASURED_DELAY_ITERATIONS * 1LL * DESIRED_TIME_UNIT_MS) / MEASURED_DELAY_MS )

// Define the LED pin mappings based on the hardware target.

#define LD1_GPIO_Port PWM3_GPIO_Port  // red
#define LD1_Pin       PWM3_Pin
#define LD2_GPIO_Port PWM2_GPIO_Port  // green
#define LD2_Pin       PWM2_Pin
#define LD3_GPIO_Port PWM1_GPIO_Port  // blue
#define LD3_Pin       PWM1_Pin

/* --- Private Function Implementations --- */

/**
 * @brief  A simple, blocking delay that does NOT depend on SysTick.
 * @note   This is the lowest-level helper and forms the base "time unit" for patterns.
 */
static void error_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < ERROR_DELAY_ITERATIONS; ++i) {
        __NOP();
    }
}

/**
 * @brief  Ensures the clock is enabled for a given GPIO port.
 * @param  port A pointer to the GPIO port (e.g., GPIOA, GPIOB, etc.).
 */
static void enable_gpio_clock(GPIO_TypeDef* port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
}

/**
 * @brief  Blinks a single Morse code dot or dash.
 * @param  duration_units The relative duration (e.g., 1 for dot, 3 for dash).
 */
static void blink_morse_element(int duration_units)
{
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); // Red ON
    for(int t = 0; t < duration_units; ++t) error_delay();
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);   // OFF
}

/**
 * @brief  Inserts a pause between Morse code letters.
 */
static void pause_inter_letter(void)
{
    const int inter_letter_pause = 3;
    for(int t = 0; t < inter_letter_pause; ++t) error_delay();
}

/**
 * @brief  Inserts a longer pause between Morse code words (or patterns).
 */
static void pause_inter_word(void)
{
    const int inter_word_pause = 7;
    for(int t = 0; t < inter_word_pause; ++t) error_delay();
}

/**
 * @brief  Blinks the Morse code pattern for the letter 'S' (...).
 */
static void blink_morse_s(void)
{
    const int dot_time = 1;
    const int inter_element_pause = 1;

    for (int i = 0; i < 3; i++) {
        blink_morse_element(dot_time);
        for(int t = 0; t < inter_element_pause; ++t) error_delay();
    }
}

/**
 * @brief  Blinks the Morse code pattern for the letter 'O' (---).
 */
static void blink_morse_o(void)
{
    const int dash_time = 3;
    const int inter_element_pause = 1;

    for (int i = 0; i < 3; i++) {
        blink_morse_element(dash_time);
        for(int t = 0; t < inter_element_pause; ++t) error_delay();
    }
}

/**
 * @brief  Manually initializes all RGB LED pins as GPIO outputs.
 * @note   This function is robust and handles LEDs being on different GPIO ports.
 */
static void error_init_led_pins(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    HAL_GPIO_DeInit(LD1_GPIO_Port, LD1_Pin);
    HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_DeInit(LD3_GPIO_Port, LD3_Pin);

    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    enable_gpio_clock(LD1_GPIO_Port);
    gpio_init.Pin = LD1_Pin;
    HAL_GPIO_Init(LD1_GPIO_Port, &gpio_init);

    enable_gpio_clock(LD2_GPIO_Port);
    gpio_init.Pin = LD2_Pin;
    HAL_GPIO_Init(LD2_GPIO_Port, &gpio_init);

    enable_gpio_clock(LD3_GPIO_Port);
    gpio_init.Pin = LD3_Pin;
    HAL_GPIO_Init(LD3_GPIO_Port, &gpio_init);

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //all off
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}

/**
 * @brief  The core error state function. Contains the infinite SOS blink loop.
 * @note   This function will never return.
 */
static void enter_error_loop(void)
{
    error_init_led_pins();

    while (1)
    {
        blink_morse_s();  pause_inter_letter();
        blink_morse_o();  pause_inter_letter();
        blink_morse_s();  pause_inter_word();
    }
}


/* --- Public Function Implementations --- */

void error_reporter_indicate_fatal_error(void)
{
    __disable_irq();
    enter_error_loop();
}

void error_reporter_handle_hard_fault(hard_fault_info_t* fault_info)
{
    UNUSED(fault_info);

    __disable_irq();
    enter_error_loop();
}
