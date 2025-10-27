/*
 * cdc_parser.c
 *
 *  Created on: Jul 26, 2025
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

#include "stm32h7xx_hal.h"
#include "cdc_parser.h"

#define PARSER_TIMEOUT_MS 500 // 0.5 seconds

// Internal helper to reset the state machine
static void reset_parser(cdc_parser_t *parser)
{
    parser->state = STATE_WAIT_FOR_SOP1;
}

void cdc_parser_init(cdc_parser_t *parser, packet_handler_t handler)
{
    reset_parser(parser);
    parser->handler = handler;
}

parser_result_t parser_process_buffer(cdc_parser_t *parser, ring_buffer_spsc_t *rb)
{
    uint8_t byte;

    // This loop allows the parser to process as much data as is available in the
    // ring buffer in a single call, which is very efficient.
    while (ring_buffer_spsc_used_space(rb) > 0)
    {
        switch (parser->state)
        {
            case STATE_WAIT_FOR_SOP1:
                ring_buffer_spsc_read_block(rb, &byte, 1);
                if (byte == SOP1_BYTE) {
                    parser->state = STATE_WAIT_FOR_SOP2;
                }
                // If not SOP1, we just discard the byte and loop to the next one.
                break;

            case STATE_WAIT_FOR_SOP2:
                if (ring_buffer_spsc_used_space(rb) < 1) return PARSER_NEEDS_MORE_DATA;
                ring_buffer_spsc_read_block(rb, &byte, 1);
                if (byte == SOP2_BYTE) {
                    parser->state = STATE_READ_COMMAND;
                } else {
                    reset_parser(parser);
                    // No error return, just reset and look for the next valid packet.
                }
                break;

            case STATE_READ_COMMAND:
                if (ring_buffer_spsc_used_space(rb) < 1) return PARSER_NEEDS_MORE_DATA;
                ring_buffer_spsc_read_block(rb, &parser->packet.command, 1);
                parser->state = STATE_READ_LENGTH;
                break;

            case STATE_READ_LENGTH:
                if (ring_buffer_spsc_used_space(rb) < 2) return PARSER_NEEDS_MORE_DATA;

                uint8_t length_bytes[2];
                ring_buffer_spsc_read_block(rb, length_bytes, 2);
                parser->packet.length = (uint16_t)(length_bytes[0] | (length_bytes[1] << 8));

                if (parser->packet.length > MAX_RX_PACKET_PAYLOAD) {
                    reset_parser(parser);
                    return PARSER_ERROR_CORRUPT_PACKET; // This is a hard error.
                }

                parser->payload_counter = 0;
                parser->state = STATE_READ_PAYLOAD;
                parser->payload_start_ms = HAL_GetTick();

                // Handle zero-length packets here and return immediately.
                if (parser->packet.length == 0) {
                    if (parser->handler) {
                        parser->handler(&parser->packet);
                    }
                    reset_parser(parser);
                    return PARSER_SUCCESS; // Return to allow back-pressure check.
                }
                break;

            case STATE_READ_PAYLOAD:
            {
                // *** CHANGE 1: Check for timeout FIRST! ***
                // Before reading any new bytes, check if the time window for this packet has expired.
                if ((HAL_GetTick() - parser->payload_start_ms) >= PARSER_TIMEOUT_MS) {
                    reset_parser(parser);
                    // We return NEEDS_MORE_DATA instead of PARSER_TIMEOUT here.
                    // This is because we have reset the state and now need to re-process
                    // whatever is in the buffer from the 'STATE_WAIT_FOR_SOP1' state.
                    // The while() loop will immediately cycle and do the right thing.
                    break; // Break will go to the top of the while() loop.
                }

                // Determine how much we can read in this pass.
                size_t bytes_still_needed = parser->packet.length - parser->payload_counter;
                size_t bytes_available = ring_buffer_spsc_used_space(rb);
                size_t bytes_to_read = (bytes_available < bytes_still_needed) ? bytes_available : bytes_still_needed;

                // Read a chunk if available.
                if (bytes_to_read > 0) {
                    ring_buffer_spsc_read_block(rb, &parser->packet.payload[parser->payload_counter], bytes_to_read);
                    parser->payload_counter += bytes_to_read;
                }

                // Check if the payload is now complete.
                if (parser->payload_counter >= parser->packet.length) {
                    // PAYLOAD COMPLETE: Deliver the packet.
                    if (parser->handler) {
                        parser->handler(&parser->packet);
                    }
                    reset_parser(parser);
                    return PARSER_SUCCESS;
                } else {
                    // *** CHANGE 2: The old timeout check is removed from here. ***
                    // PAYLOAD INCOMPLETE: Not an error, we just need to wait for more data.
                    return PARSER_NEEDS_MORE_DATA;
                }
                break; // Unreachable but kept for style.
            }

            default:
                reset_parser(parser);
                return PARSER_ERROR_CORRUPT_PACKET; // Should not happen.
        }
    }

    // If the while loop finishes, it's because the ring buffer is empty.
    return PARSER_NEEDS_MORE_DATA;
}
