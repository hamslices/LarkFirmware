/*
 * cdc_parser.h
 *
 *  Created on: Jul 26, 2025
 *      Author: HamSlices
 */

#ifndef CDC_PARSER_H
#define CDC_PARSER_H

#include <stdint.h>
#include "protocol_definitions.h"
#include "ring_buffer_spsc.h"

// --- Protocol Definitions ---
typedef enum {
    STATE_WAIT_FOR_SOP1,
    STATE_WAIT_FOR_SOP2,
    STATE_READ_COMMAND,
    STATE_READ_LENGTH,
    STATE_READ_PAYLOAD
} cdc_parser_state_t;

typedef enum {
    PARSER_NEEDS_MORE_DATA,         // The parser stopped because it needs more bytes to continue.
    PARSER_SUCCESS,                 // A complete, valid packet was found and handled.
    PARSER_ERROR_CORRUPT_PACKET     // Invalid data was found (bad SOP, bad length), parser has reset.
} parser_result_t;

// Structure to hold a fully parsed packet's data
typedef struct {
    uint8_t command;
    uint16_t length;  // Length of the payload
    uint8_t payload[MAX_RX_PACKET_PAYLOAD];
} cdc_packet_t;

// --- Callback and Parser Object ---
typedef void (*packet_handler_t)(const cdc_packet_t *packet);

typedef struct {
    cdc_parser_state_t state;
    cdc_packet_t packet;
    uint16_t payload_counter;
    packet_handler_t handler;
    uint32_t payload_start_ms;
} cdc_parser_t;

// --- Public Functions ---

/**
 * @brief Initialize a CDC parser instance.
 *
 * Sets up the parser object and registers the supplied packet handler callback.
 *
 * @param parser Pointer to the parser instance to initialize.
 * @param handler Callback function to handle parsed packets.
 */
void cdc_parser_init(cdc_parser_t *parser, packet_handler_t handler);

/**
 * @brief Process data from the ring buffer through the CDC parser.
 *
 * Reads bytes from the single-producer/single-consumer ring buffer, attempts to
 * parse complete packets, and dispatches them to the registered handler.
 *
 * @param parser Pointer to the active parser instance.
 * @param rb Pointer to the SPSC ring buffer containing input data.
 * @return Result code indicating parser status.
 */
parser_result_t parser_process_buffer(cdc_parser_t *parser, ring_buffer_spsc_t *rb);

#endif /* CDC_PARSER_H */

