/*
 * luts.c
 *
 *  Created on: Aug 30, 2025
 *      Author: HamSlices
 */

#include "step_strobe_luts.h"

// LUT for FULL step resolution (8 steps per line)
const IO_State io_state_lut_full[8] =
{
    { .strobe_mask = (1 << 0),                  .microstep_value = 1 }, // Index 0
    { .strobe_mask = (1 << 0),                  .microstep_value = 0 }, // Index 1
    { .strobe_mask = (1 << 1) | (1 << 2),       .microstep_value = 1 }, // Index 2
    { .strobe_mask = (1 << 1) | (1 << 2),       .microstep_value = 0 }, // Index 3
    { .strobe_mask = (1 << 3) | (1 << 4),       .microstep_value = 1 }, // Index 4
    { .strobe_mask = (1 << 3) | (1 << 4),       .microstep_value = 0 }, // Index 5
    { .strobe_mask = (1 << 5),                  .microstep_value = 1 }, // Index 6
    { .strobe_mask = (1 << 5),                  .microstep_value = 0 }  // Index 7
};

// LUT for HALF step resolution (16 steps per line)
const IO_State io_state_lut_half[16] =
{
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 }
};

// LUT for QUARTER step resolution (32 steps per line)
const IO_State io_state_lut_quarter[32] =
{
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 }
};

// LUT for EIGHTH step resolution (64 steps per line)
const IO_State io_state_lut_eighth[64] =
{
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
	{ .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
	{ .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
	{ .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
	{ .strobe_mask = (1 << 0), .microstep_value = 1 },
	{ .strobe_mask = (1 << 0), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 1 },
	{ .strobe_mask = (1 << 1) | (1 << 2), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 1 },
	{ .strobe_mask = (1 << 3) | (1 << 4), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
	{ .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
	{ .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
	{ .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
    { .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 },
	{ .strobe_mask = (1 << 5), .microstep_value = 1 },
	{ .strobe_mask = (1 << 5), .microstep_value = 0 }
};
