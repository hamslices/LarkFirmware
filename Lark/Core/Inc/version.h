/*
 * version.h
 *
 *  Created on: Sep 2, 2025
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

#ifndef INC_VERSION_H_
#define INC_VERSION_H_

/**
 * @brief Firmware version string.
 * @note This string MUST be exactly 4 characters long to match the protocol.
 */
#define FIRMWARE_VERSION_STRING "0105" //version 01.04

/**
 * @brief The length of the firmware version string, in bytes.
 * @note This is defined separately to avoid using strlen() at runtime.
 */
#define FIRMWARE_VERSION_LENGTH 4

#endif /* INC_VERSION_H_ */
