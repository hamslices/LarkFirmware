/*
 * temperature_sensor.c
 *
 *  Created on: Sep 10, 2025
 *      Author: HamSlices
 */

#include "temperature_sensor.h"

// --- Private Defines ---
#define ADC_CHANNEL_COUNT      2
#define TEMP_READ_INTERVAL_MS  1000 // How often we want a new reading (1 sec)

// --- Private Variables ---
static ADC_HandleTypeDef *hadc_internal;

// uncached buffer
volatile static __attribute__((section(".cacheless_buffers"))) __attribute__((aligned(32)))
    uint16_t adc_dma_buffer[ADC_CHANNEL_COUNT];

static float cpu_temperature_c = 0.0f;

// --- Private Functions ---

// --- Public Function Implementations ---

HAL_StatusTypeDef temp_sensor_init(ADC_HandleTypeDef *hadc_temp)
{
    if (hadc_temp == NULL) { return HAL_ERROR; }
    hadc_internal = hadc_temp;

    // Calibrate the ADC once at startup
    if (HAL_ADCEx_Calibration_Start(hadc_internal, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    // Start the ADC in DMA mode. The hardware timer will now trigger
    // conversions automatically in the background, forever.
    return HAL_ADC_Start_DMA(hadc_internal, (uint32_t*)adc_dma_buffer, 2);
}

void temp_sensor_process(void)
{
    uint16_t vrefint_raw     = adc_dma_buffer[0];
    uint16_t temp_sensor_raw = adc_dma_buffer[1];

    if (vrefint_raw == 0) { return; }

    // Perform all calculations that read from the special memory region here.
    float vdda_millivolts = (float)VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR) / vrefint_raw;
    float corrected_temp_val = (float)temp_sensor_raw * (float)TEMPSENSOR_CAL_VREFANALOG / vdda_millivolts;

    if (*TEMPSENSOR_CAL2_ADDR == *TEMPSENSOR_CAL1_ADDR) {
        cpu_temperature_c = -273.15f;
    } else {
        cpu_temperature_c = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) *
                            (corrected_temp_val - *TEMPSENSOR_CAL1_ADDR) /
                            (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR) +
                            TEMPSENSOR_CAL1_TEMP;
    }
}

float temp_sensor_get_temp_c(void)
{
    return cpu_temperature_c;
}

