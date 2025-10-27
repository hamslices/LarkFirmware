/*
 * protocol_definitions.h
 *
 *  Created on: Sep 3, 2025
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

#ifndef INC_PROTOCOL_DEFINITIONS_H_
#define INC_PROTOCOL_DEFINITIONS_H_

// --- Protocol Framing Bytes ---
// These bytes mark the beginning of every valid packet.
#define SOP1_BYTE 0xAA
#define SOP2_BYTE 0x55

// --- Maximum Sizes ---
#define MAX_RX_PACKET_PAYLOAD  300

// --- Command Definitions ---
typedef enum {
    //======================================================================
    //    COMMANDS (Host -> Device, MSB is 0)
    //======================================================================

    //--- General System Commands ---
    CMD_GET_SYSTEM          = 0x01, /** @brief Command to request system-level information (version, status, etc.). */
    CMD_GET_SYSTEM_COUNTERS = 0x02, /** @brief Command to request lifetime system counters. */
    CMD_GET_UID             = 0x03, /** @brief Command to request the device's unique 96-bit serial number. */
    CMD_RESET               = 0x04, /** @brief Command to trigger a software reset of the device. */
    CMD_BOOTLOADER          = 0x05, /** @brief Command to make the device enter its bootloader. */

    //--- Configuration Commands ---
    CMD_GET_CONFIG          = 0x10, /** @brief Command to request a configuration value from the device. */
    CMD_SET_CONFIG          = 0x11, /** @brief Command to set a configuration value on the device. */

    //--- Non-Volatile Memory (Flash) Commands ---
    CMD_SAVE_FLASH          = 0x18, /** @brief Command to save current parameters to non-volatile storage. */
    CMD_LOAD_FLASH          = 0x19, /** @brief Command to load parameters from non-volatile storage. */
    CMD_ERASE_FLASH         = 0x1A, /** @brief Command to erase the user settings area in non-volatile storage. */

    //--- Printing & Motion Commands ---
    CMD_IMAGE_DATA          = 0x20, /** @brief Command to send a single line of image data to the device. */
    CMD_MOVE_MOTOR          = 0x21, /** @brief Command to move the motor by a specified number of lines. */
    CMD_PURGE               = 0x22, /** @brief Command to instruct the device to purge all internal data buffers. */

    //--- Diagnostics & Maintenance Commands ---
    CMD_SELF_TEST           = 0x30, /** @brief Command to trigger a hardware self-test on the device. */
    CMD_CLEAR_FAULTS        = 0x31, /** @brief Command to clear specific latched fault flags. */
    CMD_GET_FAULT_LOG_INFO  = 0x32, /** @brief Command to request metadata (e.g., entry count) about the fault log. */
    CMD_GET_FAULT_LOG_ENTRY = 0x33, /** @brief Command to request a specific entry from the fault log by its index. */


    //======================================================================
    //    RESPONSES (Device -> Host, MSB is 1)
    //    Convention: Response = Command + 0x80
    //======================================================================
    CMD_SYSTEM_RESPONSE           = 0x81, /** @brief (CMD_GET_SYSTEM + 0x80) Response containing requested system information. */
    CMD_SYSTEM_COUNTERS_RESPONSE  = 0x82, /** @brief (CMD_GET_SYSTEM_COUNTERS + 0x80) Response containing system counters. */
    CMD_UID_RESPONSE              = 0x83, /** @brief (CMD_GET_UID + 0x80) Response containing the device's serial number. */

    CMD_CONFIG_RESPONSE           = 0x90, /** @brief (CMD_GET_CONFIG + 0x80) Response containing requested configuration data. */

    CMD_FAULT_LOG_INFO_RESPONSE   = 0xB2, /** @brief (CMD_GET_FAULT_LOG_INFO + 0x80) Response containing metadata about the fault log. */
    CMD_FAULT_LOG_ENTRY_RESPONSE  = 0xB3, /** @brief (CMD_GET_FAULT_LOG_ENTRY + 0x80) Response containing a specific fault log entry. */

} protocol_command_t;


// --- Sub-Commands for CMD_GET_SYSTEM ---
typedef enum {
    STATUS           = 0,
    VERSION          = 1,
    VERIFY_FLASH     = 2,
    TEMPERATURE      = 3,
    REALTIME_STATUS  = 4
} system_sub_command_t;


#endif /* INC_PROTOCOL_DEFINITIONS_H_ */
