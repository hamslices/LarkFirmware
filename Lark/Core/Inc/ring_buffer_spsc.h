/*
 * ring_buffer.h
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

#ifndef RING_BUFFER_SPSC_H
#define RING_BUFFER_SPSC_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief A lock-free, single-producer, single-consumer (SPSC) ring buffer structure.
 *
 * This buffer is safe for use between a single ISR (producer) and the main
 * loop (consumer) without needing to disable interrupts or use locks.
 * It achieves this by ensuring only the producer touches the 'head' and only
 * the consumer touches the 'tail'. One slot is kept empty to distinguish
 * between a full and empty buffer.
 */
typedef struct {
    uint8_t * const buffer;         // The actual buffer memory, fixed at init
    const size_t max_size;          // The size of the buffer, fixed at init
    volatile size_t head;           // Written by producer (ISR) only
    volatile size_t tail;           // Written by consumer (main loop) only
} ring_buffer_spsc_t;


// --- Public Functions ---

void ring_buffer_spsc_init(ring_buffer_spsc_t *rb, uint8_t *buffer, const size_t size);


// --- Producer Functions (For ISR / CDC Callback) ---

/**
 * @brief Writes a block of data to the buffer using memcpy.
 * @param src Pointer to the source data to write.
 * @param size The number of bytes to write.
 * @return The number of bytes actually written (can be less than requested if buffer is full).
 */
size_t ring_buffer_spsc_write_block(ring_buffer_spsc_t *rb, const uint8_t *src, size_t size);


// --- Consumer Functions (For Main Loop / Parser) ---

/**
 * @brief Reads a block of data from the buffer using memcpy.
 * @return The number of bytes actually read.
 */
size_t ring_buffer_spsc_read_block(ring_buffer_spsc_t *rb, uint8_t *dest, size_t size);


// --- Utility Functions ---

size_t ring_buffer_spsc_used_space(const ring_buffer_spsc_t *rb);
size_t ring_buffer_spsc_free_space(const ring_buffer_spsc_t *rb);
size_t ring_buffer_spsc_capacity(const ring_buffer_spsc_t *rb);

/**
 * @brief  Purges the ring buffer, effectively clearing its contents.
 * @param  rb: Pointer to the ring_buffer_spsc_t structure.
 */
void ring_buffer_spsc_purge(ring_buffer_spsc_t *rb);

#endif /* RING_BUFFER_SPSC_H */

