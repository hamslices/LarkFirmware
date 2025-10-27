/*
 * ring_buffer.c
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

#include <string.h>
#include "ring_buffer_spsc.h"

void ring_buffer_spsc_init(ring_buffer_spsc_t *rb, uint8_t *buffer, const size_t size)
{
    // Cast away const for initialization only.
    *(uint8_t**)&rb->buffer = buffer;
    *(size_t*)&rb->max_size = size;
    rb->head = 0;
    rb->tail = 0;
}

// --- Producer Functions ---

size_t ring_buffer_spsc_write_block(ring_buffer_spsc_t *rb, const uint8_t *src, size_t size)
{
    size_t free_space = ring_buffer_spsc_free_space(rb);

    if (size > free_space) {
        size = free_space;
    }

    if (size == 0 || src == NULL) {
        return 0;
    }

    // Check for wrap-around case (write will be in two parts)
    if (rb->head + size > rb->max_size) {
        size_t part1_size = rb->max_size - rb->head;
        size_t part2_size = size - part1_size;
        memcpy(rb->buffer + rb->head, src, part1_size);
        memcpy(rb->buffer, src + part1_size, part2_size);
    } else {
        // Data can be written in a single contiguous block
        memcpy(rb->buffer + rb->head, src, size);
    }

    // Atomically update the head pointer
    rb->head = (rb->head + size) % rb->max_size;
    return size;
}

// --- Consumer Functions ---

size_t ring_buffer_spsc_read_block(ring_buffer_spsc_t *rb, uint8_t *dest, size_t size)
{
    size_t used_space = ring_buffer_spsc_used_space(rb);

    if (size > used_space) {
        size = used_space;
    }

    if (size == 0 || dest == NULL) {
        return 0;
    }

    // Check for wrap-around case (data is in two parts)
    if (rb->tail + size > rb->max_size) {
        size_t part1_size = rb->max_size - rb->tail;
        size_t part2_size = size - part1_size;
        memcpy(dest, rb->buffer + rb->tail, part1_size);
        memcpy(dest + part1_size, rb->buffer, part2_size);
    } else {
        // Data is in a single contiguous block
        memcpy(dest, rb->buffer + rb->tail, size);
    }

    // Atomically update the tail pointer
    rb->tail = (rb->tail + size) % rb->max_size;
    return size;
}

// --- Utility Functions ---

size_t ring_buffer_spsc_used_space(const ring_buffer_spsc_t *rb)
{
    return (rb->head - rb->tail + rb->max_size) % rb->max_size;
}

size_t ring_buffer_spsc_free_space(const ring_buffer_spsc_t *rb)
{
    return rb->max_size - ring_buffer_spsc_used_space(rb) - 1;
}

size_t ring_buffer_spsc_capacity(const ring_buffer_spsc_t *rb)
{
    // The capacity is the max size minus the one slot that is always kept empty.
    return rb->max_size - 1;
}

void ring_buffer_spsc_purge(ring_buffer_spsc_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

