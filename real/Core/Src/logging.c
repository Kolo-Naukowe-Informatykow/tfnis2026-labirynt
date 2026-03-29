#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_freertos.h"
#include "logging.h"

extern UART_HandleTypeDef huart1;

void logging_exec() {
	LogMsg_t currentLog;
	for (;;) {
		if (osMessageQueueGet(logQueueHandle, &currentLog, NULL, osWaitForever) == osOK) {

			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)currentLog.msg, strlen(currentLog.msg));

			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(loggerTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void Logging_Print(const char *fmt, ...) {
	LogMsg_t log;
	va_list args;
	va_start(args, fmt);
	vsnprintf(log.msg, MAX_LOG_LEN, fmt, args);
	va_end(args);

	osMessageQueuePut(logQueueHandle, &log, 0, 0);
}
