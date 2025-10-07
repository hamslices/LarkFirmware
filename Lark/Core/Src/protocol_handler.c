/*
 * protocol_handler.c
 *
 *  Created on: Sep 1, 2025
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

#include <stdbool.h>
#include <stdatomic.h>
#include "main.h"
#include "protocol_handler.h"
#include "usbd_cdc_if.h"
#include "command_status.h"
#include "cdc_parser.h"
#include "version.h"
#include "fault_logger.h"

// External handle for the USB device, defined in the ST HAL.
extern USBD_HandleTypeDef hUsbDeviceFS;

// --- Private Function Prototypes (Command Handlers) ---
static void handle_reset(const cdc_packet_t *packet);
static void handle_bootloader(const cdc_packet_t *packet);
static void handle_purge(const cdc_packet_t *packet);
static void handle_clear_faults(const cdc_packet_t *packet);
static void handle_set_config(const cdc_packet_t *packet);
static void handle_get_config(const cdc_packet_t *packet);
static void handle_get_system(const cdc_packet_t *packet);
static void handle_print_self_test(const cdc_packet_t *packet);
static void handle_load_flash(const cdc_packet_t *packet);
static void handle_save_flash(const cdc_packet_t *packet);
static void handle_erase_flash(const cdc_packet_t *packet);
static void handle_move_motor(const cdc_packet_t *packet);
static void handle_get_fault_log_info(const cdc_packet_t *packet);
static void handle_get_fault_log_entry(const cdc_packet_t *packet);
static void handle_get_system_counters(const cdc_packet_t *packet);
static void handle_get_UID(const cdc_packet_t *packet);

// A special value for the command table to indicate that a command's payload
// can be of any length (e.g., image data).
#define ANY_LENGTH 0xFFFF

// Packet, Payload and Frame size.
#define HEADER_SIZE         5
#define MAX_CDC_PACKET_SIZE 64
#define MAX_TX_PAYLOAD_SIZE 100 // increase, if sending larger data packets.
#define MAX_FRAME_SIZE      (HEADER_SIZE + MAX_TX_PAYLOAD_SIZE)

/**
 * @struct command_entry_t
 * @brief Defines the structure for a single entry in the command dispatch table.
 */
typedef struct {
    uint8_t command_id;                               /**< The unique command byte. */
    uint16_t expected_length;                         /**< The required payload length. Use ANY_LENGTH for variable size. */
    void (*handler_func)(const cdc_packet_t* packet); /**< Pointer to the function that handles this command. */
} command_entry_t;

// --- Private Module Variables ---
static cdc_parser_t parser;                              // The state machine for parsing the incoming byte stream.
static ring_buffer_spsc_t* rx_buffer_ptr         = NULL; // Pointer to the shared USB RX ring buffer.
static ring_buffer_spsc_t* image_line_buffer_ptr = NULL; // Pointer to the shared image data ring buffer.

// An atomic flag to manage the USB reception state. The USB CDC endpoint must
// be manually re-armed after each reception. This flag prevents re-arming
// while the RX buffer is full.
static volatile atomic_uint_least8_t g_is_usb_rx_paused = ATOMIC_VAR_INIT(1);

// --- Private Function Prototypes (Helpers) ---
static void protocol_handler(const cdc_packet_t *packet);
static bool send_frame(uint8_t command, uint8_t* payload, uint16_t payload_size);
static void handle_image_data_command(const cdc_packet_t *packet);
static uint32_t read_u32_from_payload(const uint8_t* buffer);
static int32_t read_s32_from_payload(const uint8_t* buffer);

/**
 * @brief Command dispatch table.
 * @details This table-driven approach is highly extensible. To add a new command,
 * simply define a new handler function and add a single entry to this array.
 * This avoids a large, cumbersome switch-case statement.
 */
static const command_entry_t command_table[] = {
    { CMD_RESET,                  0, handle_reset },
    { CMD_BOOTLOADER,             0, handle_bootloader },
    { CMD_PURGE,                  0, handle_purge },
    { CMD_CLEAR_FAULTS,           4, handle_clear_faults },
    { CMD_SET_CONFIG,             5, handle_set_config },
    { CMD_GET_CONFIG,             1, handle_get_config },
    { CMD_GET_SYSTEM,             1, handle_get_system },
    { CMD_SELF_TEST,              0, handle_print_self_test },
    { CMD_LOAD_FLASH,             0, handle_load_flash },
    { CMD_SAVE_FLASH,             0, handle_save_flash },
    { CMD_ERASE_FLASH,            0, handle_erase_flash },
    { CMD_MOVE_MOTOR,             4, handle_move_motor },
    { CMD_GET_FAULT_LOG_INFO,     0, handle_get_fault_log_info },
    { CMD_GET_FAULT_LOG_ENTRY,    1, handle_get_fault_log_entry },
    { CMD_GET_SYSTEM_COUNTERS,    0, handle_get_system_counters },
    { CMD_GET_UID,                0, handle_get_UID }
};

// --- Public Function Implementations ---

void protocol_init(ring_buffer_spsc_t* usb_rx_buffer, ring_buffer_spsc_t* image_buffer)
{
    rx_buffer_ptr = usb_rx_buffer;
    image_line_buffer_ptr = image_buffer;
    // Initialize the parser with a callback to our main protocol handler.
    cdc_parser_init(&parser, protocol_handler);

    // Manually arm the USB endpoint for the very first time.
    atomic_store(&g_is_usb_rx_paused, 0);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

void protocol_handle_usb_data_flow(void)
{
    // --- PART 1: PROCESS DATA (with back-pressure) ---
    // This block's only job is to move data from the USB buffer to the
    // image buffer, but only if the image buffer has enough free space.
    // This is the "traffic light" that stops the flow when things are full.
    if (ring_buffer_spsc_free_space(image_line_buffer_ptr) > MAX_RX_PACKET_PAYLOAD) {
        parser_result_t result = parser_process_buffer(&parser, rx_buffer_ptr);
        if (result == PARSER_ERROR_CORRUPT_PACKET) {
            command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_PARSER_ERROR);
        }
    }

    // --- PART 2: MANAGE THE USB ENDPOINT (always be ready) ---
    // This block runs COMPLETELY INDEPENDENTLY from Part 1. Its only job
    // is to re-arm the USB hardware as soon as the immediate USB buffer
    // (`rx_buffer_ptr`) has any space. This breaks the deadlock.
    if (atomic_load(&g_is_usb_rx_paused)) {
        if (ring_buffer_spsc_free_space(rx_buffer_ptr) > MAX_CDC_PACKET_SIZE) {
            atomic_store(&g_is_usb_rx_paused, 0);
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        }
    }
}

void protocol_reset_parser(void)
{
    cdc_parser_init(&parser, protocol_handler);
    // Ensure that if we were paused, we immediately restart reception.
    if (atomic_exchange(&g_is_usb_rx_paused, 0)) {
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
}

void protocol_cdc_receive_callback(uint8_t* Buf, uint32_t Len)
{
    // The USB hardware automatically stops listening after a reception.
    // We must set our flag to reflect this paused state. `protocol_handle_usb_data_flow`
    // will be responsible for re-arming it later.
    atomic_store(&g_is_usb_rx_paused, 1);

    // Quickly copy the data from the USB driver's buffer into our application's
    // ring buffer and return, keeping ISR time to a minimum.
    ring_buffer_spsc_write_block(rx_buffer_ptr, Buf, (uint16_t)Len);
}


// --- Private (Static) Functions ---

/**
 * @brief The main callback function that is executed by the parser for each valid packet.
 * @details This function acts as the central command dispatcher. It validates the
 * command against the current system state and then uses the command table to
 * execute the appropriate handler.
 * @param packet A pointer to the validated packet received from the parser.
 */
static void protocol_handler(const cdc_packet_t *packet)
{
    // --- GATEKEEPER LOGIC ---

    // Special case: Image data is high-frequency and bypasses the main gatekeeper.
    if (packet->command == CMD_IMAGE_DATA) {
        handle_image_data_command(packet);
        return;
    }

    // If the print engine is busy, block any commands that could disrupt its
    // current operation.
    if (print_engine_is_busy()) {
        switch (packet->command) {
            // Allow these non-disruptive commands to pass through even when busy.
            case CMD_GET_CONFIG:
            case CMD_GET_SYSTEM:
            case CMD_SET_CONFIG:
            case CMD_GET_FAULT_LOG_INFO:
            case CMD_GET_FAULT_LOG_ENTRY:
                break;
            default:
                // Block all other commands (e.g., PURGE, MOVE_MOTOR, etc.).
                command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_REJECTED);
                return;
        }
    }

    // --- COMMAND DISPATCH ---
    // Iterate through the command table to find a matching command ID.
    for (size_t i = 0; i < (sizeof(command_table) / sizeof(command_table[0])); ++i) {
        if (command_table[i].command_id == packet->command) {
            // Found a match. First, validate the payload length.
            if (command_table[i].expected_length != ANY_LENGTH &&
                command_table[i].expected_length != packet->length) {
                command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_BAD_LENGTH);
                return;
            }

            // Length is valid. Execute the associated handler function.
            command_table[i].handler_func(packet);
            return; // Command handled.
        }
    }

    // If the loop completes without finding a match, the command is unknown.
    command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_UNKNOWN_COMMAND);
}

/** @brief Handler for the CMD_RESET command. */
static void handle_reset(const cdc_packet_t *packet)
{
    UNUSED(packet);
    HAL_Delay(100); //keep USB alive for 100ms before resetting.
    NVIC_SystemReset();
}

/** @brief Handler for the CMD_BOOTLOADER command. */
static void handle_bootloader(const cdc_packet_t *packet)
{
    UNUSED(packet);
    HAL_Delay(100); //keep USB alive for 100ms before going into DFU Mode.
    system_utils_jump_to_bootloader();
}

/** @brief Handler for the CMD_PURGE command. */
static void handle_purge(const cdc_packet_t *packet)
{
    UNUSED(packet);
    print_engine_purge_job();
    ring_buffer_spsc_purge(rx_buffer_ptr);
    protocol_reset_parser();
}

/** @brief Handler for the CMD_CLEAR_FAULTS command. */
static void handle_clear_faults(const cdc_packet_t *packet)
{
    uint32_t flags = read_u32_from_payload(packet->payload);
    print_engine_clear_faults(flags);
}

/** @brief Handler for the CMD_SET_CONFIG command. */
static void handle_set_config(const cdc_packet_t *packet)
{
    config_param_t set_index = (config_param_t)packet->payload[0];
    if (set_index >= CONFIG_PARAM_COUNT) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_INVALID_PARAM);
        return;
    }
    uint32_t value = read_u32_from_payload(&packet->payload[1]);
    config_manager_set(set_index, value);
}

/** @brief Handler for the CMD_GET_CONFIG command. */
static void handle_get_config(const cdc_packet_t *packet)
{
    config_param_t get_index = (config_param_t)packet->payload[0];
    if (get_index >= CONFIG_PARAM_COUNT) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_INVALID_PARAM);
        // Send back a zero value on error.
        uint32_t zero_val = 0;
        send_frame(CMD_CONFIG_RESPONSE, (uint8_t*)&zero_val, sizeof(uint32_t));
        return;
    }

    uint32_t read_value = config_manager_get(get_index);
    if (!send_frame(CMD_CONFIG_RESPONSE, (uint8_t*)&read_value, sizeof(uint32_t))) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_TRANSMIT);
    }
}

/** @brief Handler for the CMD_GET_SYSTEM command. */
static void handle_get_system(const cdc_packet_t *packet) {
    uint32_t sys_val;
    uint8_t sub_command = packet->payload[0];

    switch(sub_command)
    {
        case VERSION:
            memcpy(&sys_val, FIRMWARE_VERSION_STRING, FIRMWARE_VERSION_LENGTH);
            break;

        case VERIFY_FLASH:
            sys_val = system_utils_get_flash_hash();
            break;

        case REALTIME_STATUS:
        	sys_val = print_engine_get_realtime_status();
            break;

        case STATUS:
            sys_val = print_engine_get_status();
            break;

        case TEMPERATURE: {
            float tmp = temp_sensor_get_temp_c();
            memcpy(&sys_val, &tmp, sizeof(uint32_t));
            break;
        }

        default:
            command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_INVALID_PARAM);
            return;
    }

    if (!send_frame(CMD_SYSTEM_RESPONSE, (uint8_t*)&sys_val, sizeof(uint32_t))) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_TRANSMIT);
    }
}

/** @brief Handler for the CMD_PRINT_SELF_TEST command. */
static void handle_print_self_test(const cdc_packet_t *packet)
{
    UNUSED(packet);
    // Purge any existing data/jobs before starting the test.
    ring_buffer_spsc_purge(rx_buffer_ptr);
    protocol_reset_parser();
    print_engine_start_self_test_job();
}

/** @brief Handler for the CMD_LOAD_FLASH command. */
static void handle_load_flash(const cdc_packet_t *packet)
{
    UNUSED(packet);
    if (config_manager_load_from_flash() == false) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_CRC);
    }
}

/** @brief Handler for the CMD_SAVE_FLASH command. */
static void handle_save_flash(const cdc_packet_t *packet)
{
    UNUSED(packet);
    if (config_manager_save_to_flash() == false) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_HW);
    } else {
    	//write user flash saves to flash
    	config_manager_save_counters_to_flash();
	}
}

/** @brief Handler for the CMD_ERASE_FLASH command. */
static void handle_erase_flash(const cdc_packet_t *packet)
{
    UNUSED(packet);
    if (config_manager_erase_from_flash() == false) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_HW);
    }
}

/** @brief Handler for the CMD_MOVE_MOTOR command. */
static void handle_move_motor(const cdc_packet_t *packet)
{
    // Unpack the signed 32-bit number of lines from the payload.
    int32_t lines = read_s32_from_payload(packet->payload);

    // Pass the command to the print engine.
    if (print_engine_move_motor(lines) == false) {
        // This can fail if the engine is busy (a rare race condition) or if lines is zero.
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_REJECTED);
    }
}

/** @brief Handler for the CMD_GET_FAULT_LOG_INFO command. */
static void handle_get_fault_log_info(const cdc_packet_t *packet)
{
    UNUSED(packet);
    uint8_t count = fault_logger_get_entry_count();
    send_frame(CMD_FAULT_LOG_INFO_RESPONSE, &count, sizeof(count));
}

/** @brief Handler for the CMD_GET_FAULT_LOG_ENTRY command. */
static void handle_get_fault_log_entry(const cdc_packet_t *packet)
{
    uint8_t index = packet->payload[0];
    const fault_log_entry_t* entry = fault_logger_get_entry(index);

    if (entry != NULL) {
        // --- SINGLE MEMCPY to a buffer ---
        uint8_t payload_buffer[sizeof(fault_log_entry_t)];
        // Copy the entire struct into the buffer in one operation.
        memcpy(payload_buffer, entry, sizeof(fault_log_entry_t));
        send_frame(CMD_FAULT_LOG_ENTRY_RESPONSE, payload_buffer, sizeof(payload_buffer));
    } else {
        // If the index is invalid, send back a zero-length payload.
        send_frame(CMD_FAULT_LOG_ENTRY_RESPONSE, NULL, 0);
    }
}

/**
 * @brief Handler for the CMD_GET_SYSTEM_COUNTERS command.
 * @details Retrieves the live system diagnostic counters from the config manager,
 *          packages the entire SystemCounters_t struct into a payload, and sends
 *          it back to the host. This follows the same pattern as the fault log retrieval.
 */
static void handle_get_system_counters(const cdc_packet_t *packet)
{
    UNUSED(packet); // This command does not use a payload from the host.

    // Retrieve a constant pointer to the live system counters.
    const SystemCounters_t* counters = config_manager_get_counters();

    if (counters != NULL) {
        // Use a local buffer to ensure proper memory handling and alignment.
        uint8_t payload_buffer[sizeof(SystemCounters_t)];

        // Copy the entire struct into the buffer in a single, safe operation.
        memcpy(payload_buffer, counters, sizeof(SystemCounters_t));

        // Send the buffer as the payload of the response frame.
        if (!send_frame(CMD_SYSTEM_COUNTERS_RESPONSE, payload_buffer, sizeof(payload_buffer))) {
            command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_TRANSMIT);
        }
    } else {
        // This case should not happen, but as a safeguard, send a zero-length
        // payload to indicate an internal error or that data is unavailable.
        send_frame(CMD_SYSTEM_COUNTERS_RESPONSE, NULL, 0);
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_HW);
    }
}

static void handle_get_UID(const cdc_packet_t *packet)
{
    UNUSED(packet);
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0(); uid[1] = HAL_GetUIDw1(); uid[2] = HAL_GetUIDw2();

    if (!send_frame(CMD_UID_RESPONSE, (uint8_t*)&uid, sizeof(uid))) {
        command_status_set_failure(INTERNAL_CMD_RESULT_FAIL_TRANSMIT);
    }
}

// --- Helper and Special-Case Handler Functions ---

/**
 * @brief Constructs a complete frame and sends it over USB.
 * @param command The command byte for the frame.
 * @param payload Pointer to the payload data.
 * @param payload_size The size of the payload in bytes.
 * @return true if the transmission was successfully started, false otherwise.
 */
static bool send_frame(uint8_t command, uint8_t* payload, uint16_t payload_size)
{
    if (payload_size > MAX_TX_PAYLOAD_SIZE) {
        return false;
    }

    // Use a local buffer to construct the full frame with headers.
    uint8_t frame_buffer[MAX_FRAME_SIZE];
    frame_buffer[0] = SOP1_BYTE;
    frame_buffer[1] = SOP2_BYTE;
    frame_buffer[2] = command;
    frame_buffer[3] = (uint8_t)(payload_size & 0xFF);
    frame_buffer[4] = (uint8_t)((payload_size >> 8) & 0xFF);

    if (payload_size > 0 && payload != NULL) {
        memcpy(&frame_buffer[HEADER_SIZE], payload, payload_size);
    }

    // Transmit the fully constructed frame.
    if (CDC_Transmit_FS(frame_buffer, HEADER_SIZE + payload_size) == USBD_OK) {
        return true;
    }

    return false;
}

/**
 * @brief Special-case handler for high-frequency image data commands.
 * @param packet The validated image data packet.
 */
static void handle_image_data_command(const cdc_packet_t *packet)
{
    // Step 1: Attempt to start the job.
    // This function is idempotent (safe to call multiple times) and now contains
    // the critical logic to latch STATUS_JOB_ABORTED if it fails to start.
    print_engine_start_user_job();

    // Step 2: NOW check if we are in a state where data can be accepted.
    // This check will correctly fail if the job was just aborted by the call
    // above, or if the job is running but the buffer is full.
    if (!print_engine_can_queue_data()) {
        return; // Drop the packet if we can't queue it.
    }

    // Step 3: If we are here, it is safe to write the data to the buffer.
    if (packet->length <= SPI_DATA_SIZE_PER_LINE) {
        ring_buffer_spsc_write_block(image_line_buffer_ptr, packet->payload, packet->length);
    }
}

/**
 * @brief Safely reads an unsigned 32-bit integer from a byte buffer.
 * @param buffer Pointer to the start of the 4-byte data.
 * @return The reinterpreted uint32_t value.
 */
static uint32_t read_u32_from_payload(const uint8_t* buffer)
{
    uint32_t value;
    // memcpy is used to avoid strict-aliasing issues and ensure portability.
    memcpy(&value, buffer, sizeof(uint32_t));
    return value;
}

/**
 * @brief Safely reads a signed 32-bit integer from a byte buffer.
 * @param buffer Pointer to the start of the 4-byte data.
 * @return The reinterpreted int32_t value.
 */
static int32_t read_s32_from_payload(const uint8_t* buffer)
{
    int32_t value;
    // memcpy is used to avoid strict-aliasing issues and ensure portability.
    memcpy(&value, buffer, sizeof(int32_t));
    return value;
}
