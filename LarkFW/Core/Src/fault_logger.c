/*
 * fault_logger.c
 *
 *  Created on: Sep 13, 2025
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

#include <string.h>
#include "stm32h7xx_hal.h"
#include "fault_logger.h"

#define MAX_FAULT_LOG_ENTRIES 25

// --- Private Module Variables ---

// The actual ring buffer for our raw log data.
static fault_log_entry_t g_fault_log[MAX_FAULT_LOG_ENTRIES];

// Points to the *next* slot to be written, effectively the "head" of the buffer.
static uint8_t g_log_head_index = 0;

// Tracks how many slots in the buffer contain valid data. This is crucial for the
// host to know how many entries to request.
static uint8_t g_log_valid_entry_count = 0;

// --- Public Function Implementations ---

void fault_logger_init(void) {
    // Zero out the entire log buffer and reset the pointers/counters.
    memset(g_fault_log, 0, sizeof(g_fault_log));
    g_log_head_index = 0;
    g_log_valid_entry_count = 0;
}

void fault_logger_log_event(uint32_t flags, internal_command_result_t cmd_res) {
    // Get a pointer to the next available slot in the ring buffer.
    fault_log_entry_t* entry = &g_fault_log[g_log_head_index];

    // Populate the entry with the provided fault data.
    entry->uptime_ms = HAL_GetTick();
    entry->status_flags = flags;
    entry->command_result = cmd_res;
    entry->is_valid = true;

    // Advance the head index for the next write, wrapping around if necessary.
    g_log_head_index = (g_log_head_index + 1) % MAX_FAULT_LOG_ENTRIES;

    // The number of valid entries increases until the buffer is full.
    if (g_log_valid_entry_count < MAX_FAULT_LOG_ENTRIES) {
        g_log_valid_entry_count++;
    }
}

uint8_t fault_logger_get_entry_count(void) {
    return g_log_valid_entry_count;
}

const fault_log_entry_t* fault_logger_get_entry(uint8_t index) {
    // Ensure the requested index is valid.
    if (index >= g_log_valid_entry_count) {
        return NULL;
    }

    // This math correctly calculates the physical index of the Nth most recent entry.
    // The newest entry is the one right before the current head pointer.
    // Adding MAX_FAULT_LOG_ENTRIES before the modulo prevents issues with negative numbers.
    int read_index = (g_log_head_index + MAX_FAULT_LOG_ENTRIES - 1 - index) % MAX_FAULT_LOG_ENTRIES;

    return &g_fault_log[read_index];
}
