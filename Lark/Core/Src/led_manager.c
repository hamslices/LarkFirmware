/*
 * led_manager.c
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#include <stdbool.h>
#include "main.h"
#include "led_manager.h"
#include "gamma_lut.h"

#define HSV_HUE_MAX             255
#define HSV_SATURATION_MAX      255
#define HSV_VALUE_MAX           255
#define HSV_HUE_REGION_COUNT    6
#define HSV_HUE_REGION_WIDTH    (HSV_HUE_MAX / HSV_HUE_REGION_COUNT) // Approximately 42.5

// --- Private Typedefs and Variables ---
typedef enum {
    LED_STATE_UNKNOWN,
    LED_STATE_BLACK,
    LED_STATE_SILVER,
    LED_STATE_GRAY,
    LED_STATE_WHITE,
    LED_STATE_MAROON,
    LED_STATE_RED,
    LED_STATE_PURPLE,
    LED_STATE_FUCHSIA,   // Magenta
    LED_STATE_GREEN,
    LED_STATE_LIME,      // Bright Green
    LED_STATE_OLIVE,
    LED_STATE_YELLOW,
    LED_STATE_NAVY,
    LED_STATE_BLUE,
    LED_STATE_TEAL,
    LED_STATE_AQUA,      // Cyan
    LED_STATE_ORANGE,
    LED_STATE_ORCHID,
    LED_STATE_VIOLET,
    LED_STATE_TOMATO,
    LED_STATE_GREENYELLOW,
    LED_STATE_ROYALBLUE
} led_state_t;

// A container to describe the desired state AND behavior of the LED.
typedef struct {
    led_state_t state;
    bool is_blinking;
} led_behavior_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

// --- Color Lookup Table (LUT) ---
// This array is indexed by the led_state_t enum.
static const rgb_color_t led_color_lut[] = {
    [LED_STATE_UNKNOWN]     = {0x00, 0x80, 0x00}, // Default to Green
    [LED_STATE_BLACK]       = {0x00, 0x00, 0x00},
    [LED_STATE_SILVER]      = {0xC0, 0xC0, 0xC0},
    [LED_STATE_GRAY]        = {0x80, 0x80, 0x80},
    [LED_STATE_WHITE]       = {0xFF, 0xFF, 0xFF},
    [LED_STATE_MAROON]      = {0x80, 0x00, 0x00},
    [LED_STATE_RED]         = {0xFF, 0x00, 0x00},
    [LED_STATE_PURPLE]      = {0x80, 0x00, 0x80},
    [LED_STATE_FUCHSIA]     = {0xFF, 0x00, 0xFF},
    [LED_STATE_GREEN]       = {0x00, 0x80, 0x00},
    [LED_STATE_LIME]        = {0x00, 0xFF, 0x00},
    [LED_STATE_OLIVE]       = {0x80, 0x80, 0x00},
    [LED_STATE_YELLOW]      = {0xFF, 0xFF, 0x00},
    [LED_STATE_NAVY]        = {0x00, 0x00, 0x80},
    [LED_STATE_BLUE]        = {0x00, 0x00, 0xFF},
    [LED_STATE_TEAL]        = {0x00, 0x80, 0x80},
    [LED_STATE_AQUA]        = {0x00, 0xFF, 0xFF},
    [LED_STATE_ORANGE]      = {0xFF, 0xA5, 0x00},
    [LED_STATE_ORCHID]      = {0xDA, 0x70, 0xD6},
    [LED_STATE_VIOLET]      = {0xEE, 0x82, 0xEE},
    [LED_STATE_TOMATO]      = {0xFF, 0x63, 0x47},
    [LED_STATE_GREENYELLOW] = {0xAD, 0xFF, 0x2F},
    [LED_STATE_ROYALBLUE]   = {0x41, 0x69, 0xE1}
};

// The current state is now described by the behavior struct.
static led_behavior_t current_behavior = {LED_STATE_UNKNOWN, false};

// Timer
extern TIM_HandleTypeDef htim1;

// --- Variables for managing blinking ---
#define BLINK_INTERVAL_MS 250 // Blink on for 250ms, off for 250ms (2Hz)
static uint32_t last_blink_toggle_tick = 0;
static bool is_led_on_in_blink_cycle   = false;

// --- Private (Static) Helper Functions ---
static void set_led_rgb(uint8_t r, uint8_t g, uint8_t b);
static void set_pwm_gamma_corrected(uint16_t r, uint16_t g, uint16_t b);
static void set_led_hsv(uint8_t h, uint8_t s, uint8_t v);
static rgb_color_t hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v);
static bool manage_led_blinking(led_behavior_t desired_behavior);
static void apply_led_color_from_state(led_state_t led_state);
static led_behavior_t get_desired_led_behavior(uint32_t system_status);

// --- Public Function Implementations ---

HAL_StatusTypeDef led_manager_init(void)
{
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) { return HAL_ERROR; }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) { return HAL_ERROR; }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) { return HAL_ERROR; }
    set_led_rgb(0, 0, 0); // Start with LED off
    return HAL_OK;
}

void led_manager_update(void)
{
    // 1. Get the desired behavior based on the current system status.
    led_behavior_t desired_behavior = get_desired_led_behavior(print_engine_get_realtime_status());

    // 2. Manage blink timing and determine if the hardware needs an update.
    bool needs_hw_update = manage_led_blinking(desired_behavior);

    // 3. If an update is needed, apply it to the hardware.
    if (needs_hw_update) {
        if (current_behavior.is_blinking && !is_led_on_in_blink_cycle) {
            set_led_rgb(led_color_lut[LED_STATE_BLACK].r,
            		    led_color_lut[LED_STATE_BLACK].g,
					    led_color_lut[LED_STATE_BLACK].b);
        } else {
            apply_led_color_from_state(current_behavior.state);
        }
    }
}

void led_manager_run_color_wheel(void)
{
    for (int i = 0; i < HSV_HUE_MAX; i++) {
        set_led_hsv(i, HSV_SATURATION_MAX, HSV_VALUE_MAX);
        HAL_Delay(10);
    }
}

// --- Private Function Implementations ---

static bool manage_led_blinking(led_behavior_t desired_behavior)
{
    // Structs must be compared member by member.
    bool behavior_has_changed = (desired_behavior.state != current_behavior.state) ||
                                (desired_behavior.is_blinking != current_behavior.is_blinking);

    bool needs_update = behavior_has_changed;

    if (behavior_has_changed) {
        current_behavior = desired_behavior;
        // When behavior changes, always start with the LED on.
        is_led_on_in_blink_cycle = true;
        last_blink_toggle_tick = HAL_GetTick();
    }

    if (current_behavior.is_blinking) {
        uint32_t current_tick = HAL_GetTick();
        if (current_tick - last_blink_toggle_tick >= BLINK_INTERVAL_MS) {
            last_blink_toggle_tick = current_tick;
            is_led_on_in_blink_cycle = !is_led_on_in_blink_cycle; // Toggle
            needs_update = true; // A toggle requires a hardware update
        }
    }

    return needs_update;
}

static led_behavior_t get_desired_led_behavior(uint32_t system_status)
{
    // Define the mask for all LIVE physical hardware warnings.
    const uint32_t physical_warning_mask = STATUS_HEAD_UP |
                                           STATUS_NO_STOCK |
                                           STATUS_MOTOR_OVER_TEMP |
                                           STATUS_HEAD_OVER_TEMP;

    // --- STATE HIERARCHY ---
    // The order of these checks is critical. The most severe states are checked first.

    // 1. FATAL ERRORS (Highest Priority - System is Locked)
    if (system_status & STATUS_FATAL_ERROR) {
        return (led_behavior_t){LED_STATE_RED, true}; // Blinking Red
    }

    // 2. LATCHED JOB ABORT (High Priority - Needs Clearing)
    // This indicates a previous job failed and the fault must be cleared by the user.
    if (system_status & STATUS_JOB_ABORTED) {
        return (led_behavior_t){LED_STATE_ORANGE, true}; // Blinking Orange
    }

    // 3. LIVE PHYSICAL WARNINGS (Immediate User Action Needed)
    if (system_status & physical_warning_mask) {
        return (led_behavior_t){LED_STATE_YELLOW, false}; // Solid Yellow
    }

    // 4. INFORMATIONAL JOB FAILURES (Non-Latching)
    if (system_status & STATUS_DATA_UNDERRUN) {
        // Using a different color (Tomato) to distinguish from the latched abort state.
        return (led_behavior_t){LED_STATE_TOMATO, false}; // Solid Tomato-Red
    }

    // 5. ACTIVE / BUSY STATES (Normal Operation)
    if (system_status & STATUS_PRINTING) {
        return (led_behavior_t){LED_STATE_PURPLE, false}; // Solid Purple
    }
    if (system_status & STATUS_AWAITING_FIRST_LINE) {
        return (led_behavior_t){LED_STATE_TEAL, false}; // Solid Teal
    }
    if ((system_status & STATUS_MANUAL_FEED_ACTIVE) ||
        (system_status & STATUS_MANUAL_MOVE_ACTIVE)) {
        return (led_behavior_t){LED_STATE_AQUA, false}; // Solid Cyan (Aqua)
    }

    // 6. IDLE STATE (Lowest Priority - Ready)
    // If no other flags are set, the printer is idle and ready.
    return (led_behavior_t){LED_STATE_GREEN, false}; // Solid Green
}

static void apply_led_color_from_state(led_state_t led_state)
{
    // Bounds check to prevent reading past the end of the LUT.
    // If the state is invalid, default to a known safe color (Green).
    if (led_state >= (sizeof(led_color_lut) / sizeof(rgb_color_t))) {
        led_state = LED_STATE_GREEN;
    }

    // Retrieve the color directly from the lookup table.
    const rgb_color_t color = led_color_lut[led_state];

    // Set the hardware PWM to the desired color.
    set_led_rgb(color.r, color.g, color.b);
}

static void set_pwm_gamma_corrected(uint16_t r, uint16_t g, uint16_t b)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, gamma_lut_12bit[r]);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, gamma_lut_12bit[g]);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, gamma_lut_12bit[b]);
}

static void set_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t r_12bit = (r << 4) | (r >> 4);
    uint16_t g_12bit = (g << 4) | (g >> 4);
    uint16_t b_12bit = (b << 4) | (b >> 4);
    set_pwm_gamma_corrected(r_12bit, g_12bit, b_12bit);
}

static void set_led_hsv(uint8_t h, uint8_t s, uint8_t v)
{
    rgb_color_t color = hsv_to_rgb(h, s, v);
    set_led_rgb(color.r, color.g, color.b);
}

static rgb_color_t hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v)
{
    rgb_color_t rgb;

    if (s == 0) {
        rgb.r = rgb.g = rgb.b = v;
        return rgb;
    }

    uint8_t region = h / HSV_HUE_REGION_WIDTH;
    uint8_t remainder = (h - (region * HSV_HUE_REGION_WIDTH)) * HSV_HUE_REGION_COUNT;

    uint8_t p = (v * (255 - s)) >> 8;
    uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:  rgb.r = v; rgb.g = t; rgb.b = p; break;
        case 1:  rgb.r = q; rgb.g = v; rgb.b = p; break;
        case 2:  rgb.r = p; rgb.g = v; rgb.b = t; break;
        case 3:  rgb.r = p; rgb.g = q; rgb.b = v; break;
        case 4:  rgb.r = t; rgb.g = p; rgb.b = v; break;
        default: rgb.r = v; rgb.g = p; rgb.b = q; break;
    }

    return rgb;
}
