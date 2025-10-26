/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/**
 ******************************************************************************

  ██╗  ██╗ █████╗ ███╗   ███╗███████╗██╗     ██╗ ██████╗███████╗███████╗
  ██║  ██║██╔══██╗████╗ ████║██╔════╝██║     ██║██╔════╝██╔════╝██╔════╝
  ███████║███████║██╔████╔██║███████╗██║     ██║██║     █████╗  ███████╗
  ██╔══██║██╔══██║██║╚██╔╝██║╚════██║██║     ██║██║     ██╔══╝  ╚════██║
  ██║  ██║██║  ██║██║ ╚═╝ ██║███████║███████╗██║╚██████╗███████╗███████║
  ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝╚══════╝╚═╝ ╚═════╝╚══════╝╚══════╝
														  HamSlices 2025
	Lark Print Engine Firmware
	Revision 01.07

	For Hardware Revision 01.00.01 & 01.00.02

		A HW_bug in REV 01.00.01 prevents the raw head
		temperature value from being read by the ADC.

	---------------------------------------------------------------------------

	@attention

	Copyright (C) 2025 HamSlices

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.

	---------------------------------------------------------------------------
	Change Log:

		(08/16/2025) Created the Project, will continue development once
						the hardware arrives and is assembled.
		(08/25/2025) Added some typedef's and timer call backs.
		(08/31/2025) Integrating test code into project.
		(08/31/2025) Fixed timer 1 configuration, added set led functions
		(08/31/2025) Added start up color wheel routine
		(08/32/2025) Nuked existing code.
		(08/32/2025) Added new codebase.
		(09/06/2025) Added new new codebase.
		(09/07/2025) Fixed lost packet issue, fixed motor and strobe timing.
		(09/07/2025) Enabled Icache & Dcache.
		(09/07/2025) Fixed lost first line.
		(09/08/2025) Re-factored protocol handler (now a table)
		(09/08/2025) Added error reporting for hard faults
		(09/08/2025) Added flash ware leveling
		(09/08/2025) Swapped led channels
		(09/08/2025) Fixed linker script
		(09/09/2025) Fixed zero darkness bug, now prints at zero darkness.
		(09/09/2025) Added real motor movement command VS sending blank lines.
		(09/09/2025) Updated Comments.
		(09/11/2025) Added non cached memory of 1K to fix DMA issue
						(I changed the linker script again)
						Now using the memory protection unit for cache exclusion.
		(09/11/2025) Added CPU Temperature Readout, now using ADC3 and Timer 2.
		(09/11/2025) Added S-Curve to motor function ramp-up/ramp-down.
		(09/11/2025) Moved timer calculations to separate files.
		(09/11/2025) Now using ADC3 and Timer 2.
		(09/11/2025) post-build script is now associated with 'Flash' build.
						This preserves both 'Release' and 'Debug' builds.
		(09/12/2025) Fixed Bootloader jump function.
		(09/12/2025) Modified config_mamager.c, uses a pointer lut to clamp values
		(09/12/2025) Removed Pause state.
		(09/12/2025) Changed LED state colors.
		(09/12/2025) Other minor changes + formatting.
		(09/12/2025) Button now clears all faults.
		(09/12/2025) Added timeout logic to parser.
		(09/12/2025) Added job aborted state.
		(09/12/2025) Fixed Shutdown cases to be graceful.
		(09/13/2025) Fixed lost packet bug in the CDC parser
		(09/13/2025) Added fault logging.
		(09/20/2025) Added printer stats that save to non volatile memory,
						and refactored config_mananger.
		(09/20/2025) Separated Stats from configuration, fixed flash writes and reads.
		(10/05/2025) Fixed job start counter, increased image buffer size,
		                fixed user configuration counter.
		(10/23/2025) Inverted Strobe and PWM logic.
		-----------------------------------------------------------------------

		(10/25/2025) Moved code from old project.
		                Flip-ed PWM polarity in MXcube, reverted PWM inversion code.
		                Rev-ed Firmware Version.
		                Tested compilation.

        =======================================================================

        TODO: Check Motor 'direction' and 'ratio' pin values for proper polarity.
		TODO: Build Hardware.

		=======================================================================


     Application Build Result (Release -O3) ...................... [10/25/2025]
	 --------------------------------------------------------------------------
 	 Application ...................................... [Bank 1] Size 71.19  KB
 	 Self Test Image .......................[Sector 4-5][Bank 2] Size 163.48 KB
 	 Statistics Counter ....................[Sector   6][Bank 2] Size 128    KB
 	 User Configuration ....................[Sector   7][Bank 2] Size 128    KB
     --------------------------------------------------------------------------
 	 RAM Cache-less ................................... [D1]     Size 384    KB
 	 RAM .............................................. [D1]     Size 65.22  KB
 	 --------------------------------------------------------------------------

 ******************************************************************************
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    void (*task_func)(void);    // Pointer to the task function
    uint32_t period_ms;         // How often the task should run (in milliseconds)
    uint32_t last_run_ms;       // The tick time when the task last ran
} scheduler_task_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer_memory[RX_BUFFER_SIZE];
ring_buffer_spsc_t my_rx_buffer;
uint8_t image_line_buffer_memory[IMAGE_LINE_BUFFER_SIZE];
ring_buffer_spsc_t image_line_buffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void initialize_system_modules(void);
static void hardware_status_poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define the schedule for our non-critical tasks.
scheduler_task_t housekeeping_tasks[] = {
    // Task Function,                Period (ms), Last Run
    { button_handler_poll,            50,          0 }, // Poll buttons every 50ms
    { hardware_status_poll,           10,          0 }, // Poll HW status every 10ms
    { led_manager_update,             20,          0 }, // Update LED every 20ms
    { temp_sensor_process,            100,         0 }  // Poll Temperature conversion
};

const size_t housekeeping_task_count = sizeof(housekeeping_tasks) / sizeof(housekeeping_tasks[0]);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DAC1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  initialize_system_modules();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- 1. HIGH-PRIORITY "FAST LOOP" ---
      // These tasks run on every single iteration of the while loop to minimize latency.
      // They are critical for keeping the data pipeline full.
      protocol_handle_usb_data_flow();
      print_engine_manage_state();

      // --- 2. LOW-PRIORITY "SCHEDULED LOOP" ---
      // These housekeeping tasks do not need to run as frequently.
      // We use the scheduler to run them at a relaxed, predictable rate.
      uint32_t current_tick = HAL_GetTick();

      for (size_t i = 0; i < housekeeping_task_count; ++i) {
          if (current_tick - housekeeping_tasks[i].last_run_ms >= housekeeping_tasks[i].period_ms) {
              housekeeping_tasks[i].task_func(); // Execute the housekeeping task
              housekeeping_tasks[i].last_run_ms = current_tick; // Record the time it ran
          }
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void initialize_system_modules(void) {
    // Initialize low-level utilities first
	system_utils_calculate_flash_hash(); //hashes the system flash

	// Initialize ADC & PWM & TIMER
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) { Error_Handler(); }
    if (temp_sensor_init(&hadc3)   != HAL_OK) { Error_Handler(); }
    if (led_manager_init()         != HAL_OK) { Error_Handler(); }

    // Initialize buffers
    ring_buffer_spsc_init(&my_rx_buffer, rx_buffer_memory, RX_BUFFER_SIZE);
    ring_buffer_spsc_init(&image_line_buffer, image_line_buffer_memory, IMAGE_LINE_BUFFER_SIZE);

    // Initialize main application modules
    config_manager_init();
    protocol_init(&my_rx_buffer, &image_line_buffer);
    print_engine_init(&image_line_buffer);
    hardware_init();

    // Create an instance of the callback structure...
    button_handler_callbacks_t button_callbacks = {
        .on_self_test_triggered = print_engine_start_self_test_job,
        .on_stock_advance_start = print_engine_advance_stock,
        .on_stock_advance_stop  = print_engine_stop_stock_advance
    };
    button_handler_init(&button_callbacks);

    // Final readiness check and visual feedback
    if (!(print_engine_get_realtime_status() & STATUS_FATAL_ERROR)) {
        led_manager_run_color_wheel();
    }
}

static void hardware_status_poll(void)
{
    uint32_t current_io_status = hardware_poll_status_pins();
    print_engine_update_io_status(current_io_status);
}

// --- HAL CALLBACKS / ISRs ---

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
    	print_engine_hw_spi_tx_complete_callback();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        print_engine_hw_step_isr();
    }
    else if (htim->Instance == TIM6) {
        print_engine_hw_watchdog_isr();
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
        print_engine_hw_strobe_off_isr();
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // A HAL or other application-level fatal error has occurred.
  // Call the central error reporter to indicate the fault.
  // This function will never return.
  error_reporter_indicate_fatal_error();
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
