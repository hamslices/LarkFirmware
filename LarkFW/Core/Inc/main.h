/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ring_buffer_spsc.h"
#include "button_handler.h"
#include "print_engine_hw.h"
#include "error_reporter.h"
#include "temperature_sensor.h"
#include "print_engine.h"
#include "config_manager.h"
#include "protocol_handler.h"
#include "led_manager.h"
#include "hardware.h"
#include "system_utils.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SPI_DATA_SIZE_PER_LINE 216

#define IMAGE_LINE_BUFFER_SIZE (216 * 200)
#define RX_BUFFER_SIZE         (216 * 50)
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VREF_Pin             GPIO_PIN_4
#define VREF_GPIO_Port       GPIOA

#define SPI_CLK_Pin          GPIO_PIN_5
#define SPI_CLK_GPIO_Port    GPIOA
#define SPI_DATA_Pin         GPIO_PIN_7
#define SPI_DATA_GPIO_Port   GPIOA

#define n_LAT_Pin            GPIO_PIN_4
#define n_LAT_GPIO_Port      GPIOC

#define HUP_Pin              GPIO_PIN_5
#define HUP_GPIO_Port        GPIOC

#define PAPER_Pin            GPIO_PIN_0
#define PAPER_GPIO_Port      GPIOB

#define n_OE_Pin             GPIO_PIN_1
#define n_OE_GPIO_Port       GPIOB

#define MARK_Pin             GPIO_PIN_11
#define MARK_GPIO_Port       GPIOE

#define H_TMP_Pin            GPIO_PIN_12
#define H_TMP_GPIO_Port      GPIOE

#define DIR_Pin              GPIO_PIN_14
#define DIR_GPIO_Port        GPIOE

#define STEP_Pin             GPIO_PIN_15
#define STEP_GPIO_Port       GPIOE

#define n_SLEEP_Pin          GPIO_PIN_10
#define n_SLEEP_GPIO_Port    GPIOB

#define BTN2_Pin             GPIO_PIN_14
#define BTN2_GPIO_Port       GPIOB
#define BTN1_Pin             GPIO_PIN_15
#define BTN1_GPIO_Port       GPIOB

#define n_STB6_Pin           GPIO_PIN_8
#define n_STB6_GPIO_Port     GPIOD
#define n_STB4_Pin           GPIO_PIN_9
#define n_STB4_GPIO_Port     GPIOD
#define n_STB5_Pin           GPIO_PIN_10
#define n_STB5_GPIO_Port     GPIOD
#define n_STB3_Pin           GPIO_PIN_11
#define n_STB3_GPIO_Port     GPIOD
#define n_STB2_Pin           GPIO_PIN_12
#define n_STB2_GPIO_Port     GPIOD
#define n_STB1_Pin           GPIO_PIN_13
#define n_STB1_GPIO_Port     GPIOD

#define PWM1_Pin             GPIO_PIN_6
#define PWM1_GPIO_Port       GPIOC
#define PWM2_Pin             GPIO_PIN_7
#define PWM2_GPIO_Port       GPIOC
#define PWM3_Pin             GPIO_PIN_8
#define PWM3_GPIO_Port       GPIOC

#define EXT_GPIO_A_Pin       GPIO_PIN_9
#define EXT_GPIO_A_GPIO_Port GPIOC
#define EXT_GPIO_B_Pin       GPIO_PIN_8
#define EXT_GPIO_B_GPIO_Port GPIOA

#define n_RST_Pin            GPIO_PIN_10
#define n_RST_GPIO_Port      GPIOA

#define SWDIO_Pin            GPIO_PIN_13
#define SWDIO_GPIO_Port      GPIOA
#define SWCLK_Pin            GPIO_PIN_14
#define SWCLK_GPIO_Port      GPIOA

#define n_EN_Pin             GPIO_PIN_1
#define n_EN_GPIO_Port       GPIOD

#define M_TMP_Pin            GPIO_PIN_3
#define M_TMP_GPIO_Port      GPIOD

#define MS1_Pin              GPIO_PIN_5
#define MS1_GPIO_Port        GPIOD
#define MS2_Pin              GPIO_PIN_7
#define MS2_GPIO_Port        GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
