/*
 * protocol_handler.h
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
