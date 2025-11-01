/*
 * assets.h
 *
 *  Created on: Sep 5, 2025
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

#ifndef INC_ASSETS_H_
#define INC_ASSETS_H_

#include <stdint.h>

/* ========================================================================== */
/* ==                        SELF-TEST IMAGE ASSET                         == */
/* ========================================================================== */

/**
 * @brief Extern declarations for the self-test image asset.
 * @note These symbols are not defined in any C file. They are created by the
 *       'objcopy' tool and their addresses are resolved by the linker based on the
 *       linker script's placement rules.
 */
extern const uint8_t _binary_self_test_image_bin_start[];
extern const uint8_t _binary_self_test_image_bin_end[];
extern const uint8_t _binary_self_test_image_bin_size;

/**
 * @brief Metadata for the self-test image.
 */
#define SELF_TEST_IMAGE_WIDTH_PX   1728
#define SELF_TEST_IMAGE_HEIGHT_PX  846


/* ========================================================================== */
/* ==                         ( Future Assets Go Here )                    == */
/* ========================================================================== */

// When you add a new asset like 'logo.bin', its extern symbols
// and defines would go here.
//
// extern const uint8_t _binary_logo_bin_start[];
// #define LOGO_WIDTH_PX   128
// #define LOGO_HEIGHT_PX  64

#endif /* INC_ASSETS_H_ */
