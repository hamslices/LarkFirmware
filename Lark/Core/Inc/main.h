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
#define TARGET_DEV_BOARD  1
#define TARGET_PRODUCTION 2

// Set this in your build configuration
#define HARDWARE_TARGET TARGET_PRODUCTION
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MARK_Pin GPIO_PIN_2
#define MARK_GPIO_Port GPIOE
#define PAPER_Pin GPIO_PIN_3
#define PAPER_GPIO_Port GPIOE
#define MTO_Pin GPIO_PIN_4
#define MTO_GPIO_Port GPIOE
#define HTO_Pin GPIO_PIN_5
#define HTO_GPIO_Port GPIOE
#define VREF_Pin GPIO_PIN_4
#define VREF_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_5
#define CLK_GPIO_Port GPIOA
#define DIN_Pin GPIO_PIN_7
#define DIN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_9
#define LED_BLUE_GPIO_Port GPIOE
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_11
#define LED_GREEN_GPIO_Port GPIOE
#define BTN2_Pin GPIO_PIN_12
#define BTN2_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOE
#define DIR_Pin GPIO_PIN_14
#define DIR_GPIO_Port GPIOE
#define STEP_Pin GPIO_PIN_15
#define STEP_GPIO_Port GPIOE
#define nEN_Pin GPIO_PIN_10
#define nEN_GPIO_Port GPIOB
#define HUP_Pin GPIO_PIN_12
#define HUP_GPIO_Port GPIOB
#define nLAT_Pin GPIO_PIN_13
#define nLAT_GPIO_Port GPIOB
#define nOE_Pin GPIO_PIN_8
#define nOE_GPIO_Port GPIOD
#define nSLEEP_Pin GPIO_PIN_12
#define nSLEEP_GPIO_Port GPIOD
#define nRESET_Pin GPIO_PIN_13
#define nRESET_GPIO_Port GPIOD
#define MS2_Pin GPIO_PIN_14
#define MS2_GPIO_Port GPIOD
#define MS1_Pin GPIO_PIN_15
#define MS1_GPIO_Port GPIOD
#define nSTB3_Pin GPIO_PIN_0
#define nSTB3_GPIO_Port GPIOD
#define nSTB2_Pin GPIO_PIN_1
#define nSTB2_GPIO_Port GPIOD
#define nSTB1_Pin GPIO_PIN_2
#define nSTB1_GPIO_Port GPIOD
#define nSTB4_Pin GPIO_PIN_3
#define nSTB4_GPIO_Port GPIOD
#define nSTB5_Pin GPIO_PIN_4
#define nSTB5_GPIO_Port GPIOD
#define nSTB6_Pin GPIO_PIN_5
#define nSTB6_GPIO_Port GPIOD
#define GPIOA_Pin GPIO_PIN_8
#define GPIOA_GPIO_Port GPIOB
#define GPIOB_Pin GPIO_PIN_9
#define GPIOB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
