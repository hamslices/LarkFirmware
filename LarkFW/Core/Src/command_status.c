/*
 * command_status.c
 *
 *  Created on: Sep 7, 2025
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

#include <stdatomic.h>
#include "command_status.h"

// This static variable is the mailbox.
static volatile atomic_int g_last_command_result = ATOMIC_VAR_INIT(INTERNAL_CMD_RESULT_NONE);

void command_status_set_failure(internal_command_result_t reason)
{
    // We ONLY store the value if it's a failure code.
    if (reason != INTERNAL_CMD_RESULT_SUCCESS && reason != INTERNAL_CMD_RESULT_NONE) {
        atomic_store(&g_last_command_result, reason);
    }
}

internal_command_result_t command_status_get_result(void)
{
    // Atomically read the current value and replace it with NONE.
    // This ensures the status is only reported once.
    return (internal_command_result_t)atomic_exchange(&g_last_command_result, INTERNAL_CMD_RESULT_NONE);
}

internal_command_result_t command_status_peek(void)
{
    return (internal_command_result_t)atomic_load(&g_last_command_result);
}
