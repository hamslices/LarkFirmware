/*
 * luts.h
 *
 *  Created on: Aug 30, 2025
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

#ifndef INC_STEP_STROBE_LUTS_H_
#define INC_STEP_STROBE_LUTS_H_

// 1. Define the structure that the tables use.
//    (Assuming the types for demonstration)
typedef struct {
    unsigned int strobe_mask;
    int microstep_value;
} IO_State;

// 2. Declare the lookup tables as 'extern const'.
//    This makes them globally accessible in a read-only manner.
extern const IO_State io_state_lut_full[];
extern const IO_State io_state_lut_half[];
extern const IO_State io_state_lut_quarter[];
extern const IO_State io_state_lut_eighth[];

#endif /* INC_STEP_STROBE_LUTS_H_ */
