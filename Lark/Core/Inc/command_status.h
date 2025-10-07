/*
 * command_status.h
 *
 *  Created on: Sep 7, 2025
 *      Author: HamSlices
 */

#ifndef INC_COMMAND_STATUS_H_
#define INC_COMMAND_STATUS_H_

/**
 * @brief The detailed, internal result of a command execution.
 * This is the "message" that the protocol_handler leaves in the mailbox.
 */
typedef enum {
    INTERNAL_CMD_RESULT_NONE,
    INTERNAL_CMD_RESULT_SUCCESS,
    INTERNAL_CMD_RESULT_FAIL_BUSY,
    INTERNAL_CMD_RESULT_FAIL_CRC,
    INTERNAL_CMD_RESULT_FAIL_HW,
    INTERNAL_CMD_RESULT_FAIL_REJECTED,
    INTERNAL_CMD_RESULT_FAIL_BAD_LENGTH,
    INTERNAL_CMD_RESULT_FAIL_INVALID_PARAM,
    INTERNAL_CMD_RESULT_FAIL_UNKNOWN_COMMAND,
	INTERNAL_CMD_RESULT_FAIL_TRANSMIT,
	INTERNAL_CMD_RESULT_FAIL_PARSER_ERROR
} internal_command_result_t;

/**
 * @brief Sets the internal result of the last command. Called by the protocol handler.
 * @param result The detailed outcome from the internal_command_result_t enum.
 */
void command_status_set_failure(internal_command_result_t reason);

/**
 * @brief Gets the internal result of the last command. Called by the print engine.
 * @note This is a "consume-on-read" function. The status is reset to NONE after it's read.
 * @return The detailed outcome from the internal_command_result_t enum.
 */
internal_command_result_t command_status_get_result(void);

/**
 * @brief Gets the internal result of the last command.
 * @return The detailed outcome from the internal_command_result_t enum.
 */
internal_command_result_t command_status_peek(void);

#endif /* INC_COMMAND_STATUS_H_ */
