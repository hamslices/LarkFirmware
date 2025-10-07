/*
 * hardware.h
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#ifndef INC_HARDWARE_H_
#define INC_HARDWARE_H_

#include <stdint.h>
#include "step_strobe_luts.h"

/**
 * @brief init's the DWT, resets the motor driver IC,
 * 		  step pin state and disables driver.
 */
void hardware_init(void);

/**
 * @brief Polls all status input pins (head up, no stock, etc.).
 * @return A bitmask of the currently active IO statuses.
 */
uint32_t hardware_poll_status_pins(void);

/**
 * @brief Sets the motor step pin high or low.
 * @param state The IO_State for the current step.
 */
void hardware_apply_motor_step(IO_State state);

/**
 * @brief Atomically sets the specified strobe pins ON and all others OFF.
 * @param state The IO_State for the current step, containing the strobe mask.
 */
void hardware_apply_strobe_state(IO_State state);

/**
 * @brief Turns all strobe output pins off.
 */
void hardware_all_strobes_off(void);

/**
 * @brief Pulses the latch pin to load data into the print head drivers.
 */
void hardware_latch_pulse(void);

/**
 * @brief Sets the motor driver's step resolution (microstepping) pins.
 * @param resolution The desired resolution from the config_param_t enum.
 */
void hardware_step_set_resolution(uint8_t resolution);

/**
 * @brief Reads the state of the self-test / clear fault button.
 * @note Assumes active-low logic (returns 1 when pin is LOW).
 * @return 1 if pressed, 0 if not pressed.
 */
uint8_t hardware_read_button_one(void);

/**
 * @brief Reads the state of the stock advance button.
 * @note Assumes active-low logic (returns 1 when pin is LOW).
 * @return 1 if pressed, 0 if not pressed.
 */
uint8_t hardware_read_button_two(void);

/**
 * @brief Sets the motor direction pin.
 * @param forward 1 for forward, 0 for reverse.
 */
void hardware_motor_set_direction(uint8_t forward);

/**
 * @brief Puts the A4985 motor driver into a safe, quiescent idle state.
 */
void hardware_motor_idle(void);

/**
 * @brief Puts the A4985 motor driver into an active state
 */
void hardware_motor_active(void);

/**
 * @brief Sets DAC's Vref based on current
 * @param current_ma the current value
 */
void hardware_motor_set_current(uint32_t current_ma);

/**
 * @brief enables the output of the buffer
 */
void hardware_strobes_enable(void);

/**
 * @brief disables the outputs of the buffer
 */
void hardware_strobes_disable(void);

#endif /* INC_HARDWARE_H_ */
