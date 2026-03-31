/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logging.h"
#include "motors.h"
#include "battery.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for masterTask */
osThreadId_t masterTaskHandle;
const osThreadAttr_t masterTask_attributes = {
  .name = "masterTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for motorsTask */
osThreadId_t motorsTaskHandle;
const osThreadAttr_t motorsTask_attributes = {
  .name = "motorsTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for distanceSensorsTask */
osThreadId_t distanceSensorsTaskHandle;
const osThreadAttr_t distanceSensorsTask_attributes = {
  .name = "distanceSensorsTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for loggerTask */
osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "loggerTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 1024 * 4
};
/* Definitions for logQueue */
osMessageQueueId_t logQueueHandle;
uint8_t logQueueBuffer[ 16 * sizeof( LogMsg_t ) ];
osStaticMessageQDef_t logQueueControlBlock;
const osMessageQueueAttr_t logQueue_attributes = {
  .name = "logQueue",
  .cb_mem = &logQueueControlBlock,
  .cb_size = sizeof(logQueueControlBlock),
  .mq_mem = &logQueueBuffer,
  .mq_size = sizeof(logQueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of logQueue */
  logQueueHandle = osMessageQueueNew (16, sizeof(LogMsg_t), &logQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of masterTask */
  masterTaskHandle = osThreadNew(startMasterTask, NULL, &masterTask_attributes);

  /* creation of motorsTask */
  motorsTaskHandle = osThreadNew(startMotorsTask, NULL, &motorsTask_attributes);

  /* creation of distanceSensorsTask */
  distanceSensorsTaskHandle = osThreadNew(startDistanceSensorsTask, NULL, &distanceSensorsTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(startImuTask, NULL, &imuTask_attributes);

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(startBatteryTask, NULL, &batteryTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(startBuzzerTask, NULL, &buzzerTask_attributes);

  /* creation of loggerTask */
  loggerTaskHandle = osThreadNew(startLoggerTask, NULL, &loggerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_startMasterTask */
/**
* @brief Function implementing the masterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMasterTask */
void startMasterTask(void *argument)
{
  /* USER CODE BEGIN masterTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END masterTask */
}

/* USER CODE BEGIN Header_startMotorsTask */
/**
* @brief Function implementing the motorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorsTask */
void startMotorsTask(void *argument)
{
  /* USER CODE BEGIN motorsTask */
  motors_exec();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motorsTask */
}

/* USER CODE BEGIN Header_startDistanceSensorsTask */
/**
* @brief Function implementing the distanceSensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDistanceSensorsTask */
void startDistanceSensorsTask(void *argument)
{
  /* USER CODE BEGIN distanceSensorsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END distanceSensorsTask */
}

/* USER CODE BEGIN Header_startImuTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startImuTask */
void startImuTask(void *argument)
{
  /* USER CODE BEGIN imuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imuTask */
}

/* USER CODE BEGIN Header_startBatteryTask */
/**
* @brief Function implementing the batteryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBatteryTask */
void startBatteryTask(void *argument)
{
  /* USER CODE BEGIN batteryTask */
  battery_exec();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END batteryTask */
}

/* USER CODE BEGIN Header_startBuzzerTask */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBuzzerTask */
void startBuzzerTask(void *argument)
{
  /* USER CODE BEGIN buzzerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END buzzerTask */
}

/* USER CODE BEGIN Header_startLoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startLoggerTask */
void startLoggerTask(void *argument)
{
  /* USER CODE BEGIN loggerTask */
  logging_exec();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END loggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

