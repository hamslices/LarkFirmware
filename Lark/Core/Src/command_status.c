/*
 * command_status.c
 *
 *  Created on: Sep 7, 2025
 *      Author: HamSlices
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
