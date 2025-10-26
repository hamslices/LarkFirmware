/*
 * luts.c
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

#include "step_strobe_luts.h"

// LUT for FULL step resolution (8 steps per line)
// Strobe states are inverted (active low)
const IO_State io_state_lut_full[8] =
{
    { .strobe_mask = ~( 1 << 0),                   .microstep_value = 1 }, // Index 0
    { .strobe_mask = ~( 1 << 0),                   .microstep_value = 0 }, // Index 1
    { .strobe_mask = ~((1 << 1) | (1 << 2)),       .microstep_value = 1 }, // Index 2
    { .strobe_mask = ~((1 << 1) | (1 << 2)),       .microstep_value = 0 }, // Index 3
    { .strobe_mask = ~((1 << 3) | (1 << 4)),       .microstep_value = 1 }, // Index 4
    { .strobe_mask = ~((1 << 3) | (1 << 4)),       .microstep_value = 0 }, // Index 5
    { .strobe_mask = ~( 1 << 5),                   .microstep_value = 1 }, // Index 6
    { .strobe_mask = ~( 1 << 5),                   .microstep_value = 0 }  // Index 7
};

// LUT for HALF step resolution (16 steps per line)
// Strobe states are inverted (active low)
const IO_State io_state_lut_half[16] =
{
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 }
};

// LUT for QUARTER step resolution (32 steps per line)
// Strobe states are inverted (active low)
const IO_State io_state_lut_quarter[32] =
{
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 }
};

// LUT for EIGHTH step resolution (64 steps per line)
// Strobe states are inverted (active low)
const IO_State io_state_lut_eighth[64] =
{
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 0),             .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 1) | (1 << 2)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 1 },
	{ .strobe_mask = ~((1 << 3) | (1 << 4)), .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
    { .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 1 },
	{ .strobe_mask = ~( 1 << 5),             .microstep_value = 0 }
};
