#include "battery.h"
#include "app_freertos.h"
#include "logging.h"
#include "main.h"
#include "stm32h5xx_hal_adc.h"
#include <inttypes.h>

const static float ADC_conversion = 3.2f;

volatile float battvolts = 0.0f;

volatile uint16_t adc_value = 0;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim15;

void battery_exec(void) {
	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_Base_Start(&htim15);
	for (;;) {
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
			battvolts = adc_value * 3.3f / 4095.0f * ADC_conversion;
			// Logging_Print("%f V %" PRIu16 "\r\n", battvolts, adc_value);
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		adc_value = HAL_ADC_GetValue(hadc);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(batteryTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
