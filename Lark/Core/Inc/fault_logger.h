/*
 * fault_logger.h
 *
 *  Created on: Sep 13, 2025
 *      Author: HamSlices
 */

#ifndef INC_FAULT_LOGGER_H_
#define INC_FAULT_LOGGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "command_status.h" // For internal_command_result_t

/**
 * @brief The structure for a single, raw log entry.
 */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint32_t status_flags;
    uint8_t  command_result;
    uint8_t  is_valid;
} fault_log_entry_t;

/**
 * @brief Initializes the fault logger module. Should be called once at startup.
 */
void fault_logger_init(void);

/**
 * @brief Records a new fault event in the log's circular buffer.
 * @param flags The complete 32-bit status register at the time of the fault.
 * @param cmd_res The result code if the fault was caused by a command failure.
 */
void fault_logger_log_event(uint32_t flags, internal_command_result_t cmd_res);

/**
 * @brief Gets the number of valid entries currently in the log.
 * @return The number of entries, from 0 to MAX_FAULT_LOG_ENTRIES.
 */
uint8_t fault_logger_get_entry_count(void);

/**
 * @brief Retrieves a read-only pointer to a raw log entry.
 * @param index The index of the entry to get (0 is the newest, 1 is the second newest, etc.).
 * @return A constant pointer to the log entry, or NULL if the index is invalid.
 */
const fault_log_entry_t* fault_logger_get_entry(uint8_t index);

#endif /* INC_FAULT_LOGGER_H_ */
