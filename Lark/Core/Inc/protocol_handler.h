/*
 * protocol_handler.h
 *
 *  Created on: Sep 1, 2025
 *      Author: HamSlices
 */

#ifndef INC_PROTOCOL_HANDLER_H_
#define INC_PROTOCOL_HANDLER_H_

#include "ring_buffer_spsc.h"

/**
 * @brief Initializes the protocol handler and its underlying parser.
 * @param usb_rx_buffer Pointer to the ring buffer for raw USB RX data.
 * @param image_buffer Pointer to the ring buffer for image line data.
 */
void protocol_init(ring_buffer_spsc_t* usb_rx_buffer, ring_buffer_spsc_t* image_buffer);

/**
 * @brief Processes incoming raw data from the USB receive buffer.
 * @note  This function should be called repeatedly in the main loop.
 */
void protocol_handle_usb_data_flow(void);

/**
 * @brief Resets the state of the command parser.
 * @note  Called when a job is purged or an error occurs.
 */
void protocol_reset_parser(void);

/**
 * @brief This is the callback function that the CDC layer will use.
 * @note  It should be registered with the CDC interface.
 */
void protocol_cdc_receive_callback(uint8_t* Buf, uint32_t Len);

#endif /* INC_PROTOCOL_HANDLER_H_ */
