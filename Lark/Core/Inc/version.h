/*
 * version.h
 *
 *  Created on: Sep 2, 2025
 *      Author: HamSlices
 */

#ifndef INC_VERSION_H_
#define INC_VERSION_H_

/**
 * @brief Firmware version string.
 * @note This string MUST be exactly 4 characters long to match the protocol.
 */
#define FIRMWARE_VERSION_STRING "0104" //version 01.04

/**
 * @brief The length of the firmware version string, in bytes.
 * @note This is defined separately to avoid using strlen() at runtime.
 */
#define FIRMWARE_VERSION_LENGTH 4

#endif /* INC_VERSION_H_ */
