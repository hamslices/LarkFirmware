/*
 * print_engine_definitions.h
 *
 *  Created on: Sep 4, 2025
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

#ifndef INC_PRINT_ENGINE_DEFINITIONS_H_
#define INC_PRINT_ENGINE_DEFINITIONS_H_

#include <stdatomic.h>

// Publicly exposed status bits
typedef enum {
    STATUS_NONE                 = 0,
    STATUS_PRINTING             = (1 << 0),
    STATUS_PIPELINE_SLOT_FREE   = (1 << 1),
    STATUS_NEXT_LINE_DATA_READY = (1 << 2),
    STATUS_HEAD_OVER_TEMP       = (1 << 3),
    STATUS_MOTOR_OVER_TEMP      = (1 << 4),
    STATUS_NO_STOCK             = (1 << 5),
    STATUS_HEAD_UP              = (1 << 6),
    STATUS_FATAL_ERROR          = (1 << 7),
    STATUS_BUSY                 = (1 << 8),
    STATUS_AWAITING_FIRST_LINE  = (1 << 9),
    STATUS_MARK                 = (1 << 10),
    STATUS_SELF_TEST_ACTIVE     = (1 << 11),
    STATUS_MANUAL_FEED_ACTIVE   = (1 << 12),
    STATUS_MANUAL_MOVE_ACTIVE   = (1 << 13),
    STATUS_DATA_UNDERRUN        = (1 << 14),
    STATUS_COMMAND_FAIL         = (1 << 15),
    STATUS_JOB_ABORTED          = (1 << 16)
} print_status_t;

typedef enum {
    PE_STATE_IDLE,
    PE_STATE_BUFFERING,
    PE_STATE_AWAITING_RUN_UP_START,
    PE_STATE_RAMP_UP,
    PE_STATE_PRINTING,
    PE_STATE_FINISHING,
    PE_STATE_RAMP_DOWN,
    PE_STATE_JOB_DONE,
    PE_STATE_MANUAL_MOVE,
    PE_STATE_MANUAL_FEED,
    PE_STATE_FAULT
} print_engine_state_t;


typedef atomic_uint_least8_t atomic_print_engine_state_t;

#endif /* INC_PRINT_ENGINE_DEFINITIONS_H_ */
