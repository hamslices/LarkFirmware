/*
 * hardware.c
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

#include "main.h"
#include "hardware.h"
#include "hardware_definitions.h"

#define CPU_HZ                     480000000
#define NANO_SEC_CONVERSION_FACTOR 1000000000.0f
#define UNIT_CONVERSION_FACTOR     1000.0f

// DAC Hardware Peripheral
extern DAC_HandleTypeDef hdac1;

#if HARDWARE_TARGET == TARGET_DEV_BOARD
//=============================================================================
//           <<<<< DEVELOPMENT BOARD IMPLEMENTATION >>>>>
// The compiler will only include this entire block if HARDWARE_TARGET is
// set to TARGET_DEV_BOARD.
//=============================================================================

void hardware_init(void)
{
    // Latch DWT init
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Set initial motor state
    hardware_motor_idle();
}

uint32_t hardware_poll_status_pins(void)
{
    uint32_t current_io_status = 0;

    // --- Atomic Snapshot ---
    // Read the entire state of the GPIOE input data register only ONCE.
    // This creates a consistent "snapshot" of all pin states at a single moment in time.
    const uint32_t gpiod_idr_snapshot = GPIOD->IDR;

    // --- Process the Snapshot ---
    // All subsequent checks are performed on the local variable, which is faster
    // and prevents any "tearing" issues if the pin states change during this function's execution.
    if ((gpiod_idr_snapshot & HEAD_UP_Pin))         { current_io_status |= STATUS_HEAD_UP; }
    if ((gpiod_idr_snapshot & MOTOR_OVER_TEMP_Pin)) { current_io_status |= STATUS_MOTOR_OVER_TEMP; }
    if ((gpiod_idr_snapshot & HEAD_OVER_TEMP_Pin))  { current_io_status |= STATUS_HEAD_OVER_TEMP; }
    if ((gpiod_idr_snapshot & NO_STOCK_Pin))        { current_io_status |= STATUS_NO_STOCK; }
    if ((gpiod_idr_snapshot & MARK_Pin))            { current_io_status |= STATUS_MARK; }

    return current_io_status;
}

void hardware_apply_motor_step(IO_State state)
{
    MSTEP_GPIO_Port->BSRR = (state.microstep_value) ? MSTEP_Pin : (MSTEP_Pin << 16);
}

void hardware_apply_strobe_state(IO_State state)
{
    const uint32_t GPIOB_STROBES = STRB1_Pin | STRB6_Pin;
    const uint32_t GPIOE_STROBES = STRB2_Pin | STRB3_Pin | STRB4_Pin | STRB5_Pin;
    uint32_t gpiob_set = 0, gpioe_set = 0;
    if (state.strobe_mask & (1 << 0)) gpiob_set |= STRB1_Pin;
    if (state.strobe_mask & (1 << 1)) gpioe_set |= STRB2_Pin;
    if (state.strobe_mask & (1 << 2)) gpioe_set |= STRB3_Pin;
    if (state.strobe_mask & (1 << 3)) gpioe_set |= STRB4_Pin;
    if (state.strobe_mask & (1 << 4)) gpioe_set |= STRB5_Pin;
    if (state.strobe_mask & (1 << 5)) gpiob_set |= STRB6_Pin;
    GPIOB->BSRR = gpiob_set | ((GPIOB_STROBES & ~gpiob_set) << 16);
    GPIOE->BSRR = gpioe_set | ((GPIOE_STROBES & ~gpioe_set) << 16);
}

void hardware_all_strobes_off(void)
{
    GPIOB->BSRR = (STRB1_Pin | STRB6_Pin) << 16;
    GPIOE->BSRR = (STRB2_Pin | STRB3_Pin | STRB4_Pin | STRB5_Pin) << 16;
}

void hardware_latch_pulse(void)
{
    const uint32_t ns = LATCH_TIME_NS;
    // Perform floating-point math once to avoid doing it inside the critical section.
    const uint32_t cycles = (uint32_t)((ns * (CPU_HZ / NANO_SEC_CONVERSION_FACTOR)) + 0.5f);

    // --- Interrupt-Safe Critical Section ---

    // 1. Save the current global interrupt state (PRIMASK register) into a variable.
    uint32_t primask_status = __get_PRIMASK();

    // 2. Disable interrupts for the precise timing pulse.
    __disable_irq();

    // 3. Perform the timing-critical operation.
    uint32_t start = DWT->CYCCNT;
    LATCH_GPIO_Port->BSRR = (LATCH_Pin << 16); // Latch LOW
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
    LATCH_GPIO_Port->BSRR = LATCH_Pin;        // Latch HIGH

    // 4. Restore the interrupt state to whatever it was before this function was called.
    __set_PRIMASK(primask_status);
}

void hardware_step_set_resolution(uint8_t resolution)
{
    UNUSED(resolution);
}

uint8_t hardware_read_button_one(void)
{
    return (SELF_TEST_BTN_GPIO_Port->IDR & SELF_TEST_BTN_Pin) ? 1 : 0;
}

uint8_t hardware_read_button_two(void)
{
    return (STOCK_ADV_BTN_GPIO_Port->IDR & STOCK_ADV_BTN_Pin) ? 1 : 0;
}

void hardware_motor_set_direction(uint8_t forward)
{
    UNUSED(forward);
}

void hardware_motor_idle(void)
{
    MSTEP_GPIO_Port->BSRR = (MSTEP_Pin << 16); // reset pin state
}

void hardware_motor_active(void)
{
    // No implementation for dev board
}

void hardware_motor_set_current(uint32_t current_ma)
{
    UNUSED(current_ma);
}

void hardware_strobes_enable(void)
{
    // No implementation for dev board
}

void hardware_strobes_disable(void)
{
    // No implementation for dev board
}

#elif HARDWARE_TARGET == TARGET_PRODUCTION
//=============================================================================
//           <<<<< PRODUCTION BOARD IMPLEMENTATION >>>>>
// The compiler will only include this entire block if HARDWARE_TARGET is
// set to TARGET_PRODUCTION.
//=============================================================================

void hardware_init(void)
{
    // Latch DWT init
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Motor controller init
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);   // turn sleep off
	HAL_GPIO_WritePin(nRESET_GPIO_Port, nRESET_Pin, GPIO_PIN_RESET); // turn reset on
	HAL_Delay(100);
	HAL_GPIO_WritePin(nRESET_GPIO_Port, nRESET_Pin, GPIO_PIN_SET);   // turn reset off

    // Set initial motor state
	hardware_motor_idle();
}

uint32_t hardware_poll_status_pins(void)
{
    uint32_t current_io_status = 0;

    // --- Atomic Snapshot ---
    // Based on main.h for the production target, we know that four status
    // pins are on GPIOE and one is on GPIOB. We read each port's Input
    // Data Register only ONCE to get a consistent snapshot of the hardware state.
    const uint32_t gpiob_idr_snapshot = GPIOB->IDR;
    const uint32_t gpioe_idr_snapshot = GPIOE->IDR;

    // --- Process the Snapshot ---
    // All subsequent checks are performed on the local variables. This is faster
    // than multiple hardware reads and prevents "input tearing," ensuring the
    // returned status represents a true state at a single moment in time.

    // Check the pin on GPIOB
    if ((gpiob_idr_snapshot & HUP_Pin))   { current_io_status |= STATUS_HEAD_UP; }

    // Check the pins on GPIOE
    if ((gpioe_idr_snapshot & MTO_Pin))   { current_io_status |= STATUS_MOTOR_OVER_TEMP; }
    if ((gpioe_idr_snapshot & HTO_Pin))   { current_io_status |= STATUS_HEAD_OVER_TEMP; }
    if ((gpioe_idr_snapshot & PAPER_Pin)) { current_io_status |= STATUS_NO_STOCK; }
    if ((gpioe_idr_snapshot & MARK_Pin))  { current_io_status |= STATUS_MARK; }

    return current_io_status;
}

void hardware_apply_motor_step(IO_State state)
{
    STEP_GPIO_Port->BSRR = (state.microstep_value) ? STEP_Pin : (STEP_Pin << 16);
}

void hardware_apply_strobe_state(IO_State state)
{
    const uint32_t GPIOD_STROBES = nSTB1_Pin | nSTB2_Pin | nSTB3_Pin |
    							   nSTB4_Pin | nSTB5_Pin | nSTB6_Pin;
    uint32_t gpiod_set = 0;

    if (state.strobe_mask & (1 << 0)) gpiod_set |= nSTB1_Pin;
    if (state.strobe_mask & (1 << 1)) gpiod_set |= nSTB2_Pin;
    if (state.strobe_mask & (1 << 2)) gpiod_set |= nSTB3_Pin;
    if (state.strobe_mask & (1 << 3)) gpiod_set |= nSTB4_Pin;
    if (state.strobe_mask & (1 << 4)) gpiod_set |= nSTB5_Pin;
    if (state.strobe_mask & (1 << 5)) gpiod_set |= nSTB6_Pin;

    GPIOD->BSRR = gpiod_set | ((GPIOD_STROBES & ~gpiod_set) << 16);
}

void hardware_all_strobes_off(void)
{
    GPIOD->BSRR = (nSTB1_Pin | nSTB2_Pin | nSTB3_Pin | nSTB4_Pin | nSTB5_Pin | nSTB6_Pin);
}

void hardware_latch_pulse(void)
{
    const uint32_t ns = LATCH_TIME_NS;
    // Perform floating-point math once to avoid doing it inside the critical section.
    const uint32_t cycles = (uint32_t)((ns * (CPU_HZ / NANO_SEC_CONVERSION_FACTOR)) + 0.5f);

    // --- Interrupt-Safe Critical Section ---

    // 1. Save the current global interrupt state (PRIMASK register) into a variable.
    uint32_t primask_status = __get_PRIMASK();

    // 2. Disable interrupts for the precise timing pulse.
    __disable_irq();

    // 3. Perform the timing-critical operation.
    uint32_t start = DWT->CYCCNT;
    nLAT_GPIO_Port->BSRR = (nLAT_Pin << 16); // Latch LOW
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
    nLAT_GPIO_Port->BSRR = nLAT_Pin;        // Latch HIGH

    // 4. Restore the interrupt state to whatever it was before this function was called.
    __set_PRIMASK(primask_status);
}

void hardware_step_set_resolution(uint8_t resolution)
{
    switch (resolution) {
        case RESOLUTION_FULL_STEP:
            HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
            break;

        case RESOLUTION_HALF_STEP:
            HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
            break;

        case RESOLUTION_QUARTER_STEP:
            HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_SET);
            break;

        case RESOLUTION_EIGHTH_STEP:
            HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_SET);
            break;

        default:
            HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_SET);
            break;
    }
}

uint8_t hardware_read_button_one(void)
{
    return (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET);
}

uint8_t hardware_read_button_two(void)
{
    return (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET);
}

void hardware_motor_set_direction(uint8_t forward)
{
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (forward ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void hardware_motor_idle(void)
{
    STEP_GPIO_Port->BSRR = (STEP_Pin << 16); // reset motor state to low
    nEN_GPIO_Port->BSRR = nEN_Pin;           // disable to motor driver
}

void hardware_motor_active(void) {
	nEN_GPIO_Port->BSRR = (nEN_Pin << 16); // enable the motor driver
}

void hardware_motor_set_current(uint32_t current_ma)
{
    const uint32_t MAX_CURRENT_MA = (uint32_t)((V_TO_I_GAIN * V_REF_V) * UNIT_CONVERSION_FACTOR);
    if (current_ma > MAX_CURRENT_MA) {
    	current_ma = MAX_CURRENT_MA;
    }
    float current_in_amps = current_ma / UNIT_CONVERSION_FACTOR;
    float desired_voltage = current_in_amps / V_TO_I_GAIN;
    uint32_t dac_value = (uint32_t)((desired_voltage * DAC_MAX_VALUE_12_BIT) / V_REF_V);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}

void hardware_strobes_enable(void)
{
	nOE_GPIO_Port->BSRR = (nOE_Pin << 16); // enable buffer output
}

void hardware_strobes_disable(void)
{
	nOE_GPIO_Port->BSRR = nOE_Pin; // disable buffer output
}

#else
    #error "No valid HARDWARE_TARGET specified in hardware.c! Please define either TARGET_DEV_BOARD or TARGET_PRODUCTION."
#endif // HARDWARE_TARGET
