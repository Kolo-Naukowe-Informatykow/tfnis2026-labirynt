/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIST6_GPIO1_Pin GPIO_PIN_13
#define DIST6_GPIO1_GPIO_Port GPIOC
#define DIST6_GPIO1_EXTI_IRQn EXTI13_IRQn
#define DIST2_GPIO1_Pin GPIO_PIN_14
#define DIST2_GPIO1_GPIO_Port GPIOC
#define DIST2_GPIO1_EXTI_IRQn EXTI14_IRQn
#define DIST2_XSHUT_Pin GPIO_PIN_15
#define DIST2_XSHUT_GPIO_Port GPIOC
#define LED_INFO_0_Pin GPIO_PIN_0
#define LED_INFO_0_GPIO_Port GPIOH
#define LED_INFO_1_Pin GPIO_PIN_1
#define LED_INFO_1_GPIO_Port GPIOH
#define DIST6_XSHUT_Pin GPIO_PIN_0
#define DIST6_XSHUT_GPIO_Port GPIOC
#define BATT_MEAS_Pin GPIO_PIN_1
#define BATT_MEAS_GPIO_Port GPIOC
#define M_NFAULT_Pin GPIO_PIN_2
#define M_NFAULT_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_3
#define IMU_INT1_GPIO_Port GPIOC
#define ML1_Pin GPIO_PIN_0
#define ML1_GPIO_Port GPIOA
#define ML2_Pin GPIO_PIN_1
#define ML2_GPIO_Port GPIOA
#define MR1_Pin GPIO_PIN_2
#define MR1_GPIO_Port GPIOA
#define MR2_Pin GPIO_PIN_3
#define MR2_GPIO_Port GPIOA
#define BT_CS_Pin GPIO_PIN_4
#define BT_CS_GPIO_Port GPIOA
#define BT_CLK_Pin GPIO_PIN_5
#define BT_CLK_GPIO_Port GPIOA
#define BT_MISO_Pin GPIO_PIN_6
#define BT_MISO_GPIO_Port GPIOA
#define BT_MOSI_Pin GPIO_PIN_7
#define BT_MOSI_GPIO_Port GPIOA
#define BT_RST_Pin GPIO_PIN_4
#define BT_RST_GPIO_Port GPIOC
#define BT_IRQ_Pin GPIO_PIN_5
#define BT_IRQ_GPIO_Port GPIOC
#define DIST1_GPIO1_Pin GPIO_PIN_0
#define DIST1_GPIO1_GPIO_Port GPIOB
#define DIST1_GPIO1_EXTI_IRQn EXTI0_IRQn
#define DIST1_XSHUT_Pin GPIO_PIN_1
#define DIST1_XSHUT_GPIO_Port GPIOB
#define IMU_INT2_Pin GPIO_PIN_2
#define IMU_INT2_GPIO_Port GPIOB
#define DIST_SCL_Pin GPIO_PIN_10
#define DIST_SCL_GPIO_Port GPIOB
#define DIST_SDA_Pin GPIO_PIN_12
#define DIST_SDA_GPIO_Port GPIOB
#define M_NSLEEP_Pin GPIO_PIN_13
#define M_NSLEEP_GPIO_Port GPIOB
#define BUZZER_P_Pin GPIO_PIN_14
#define BUZZER_P_GPIO_Port GPIOB
#define BUZZER_N_Pin GPIO_PIN_15
#define BUZZER_N_GPIO_Port GPIOB
#define ENC_L_1_Pin GPIO_PIN_6
#define ENC_L_1_GPIO_Port GPIOC
#define ENC_L_2_Pin GPIO_PIN_7
#define ENC_L_2_GPIO_Port GPIOC
#define DIST4_GPIO1_Pin GPIO_PIN_8
#define DIST4_GPIO1_GPIO_Port GPIOC
#define DIST4_GPIO1_EXTI_IRQn EXTI8_IRQn
#define DIST4_XSHUT_Pin GPIO_PIN_9
#define DIST4_XSHUT_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_8
#define LED_STATUS_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define DIST3_GPIO1_Pin GPIO_PIN_11
#define DIST3_GPIO1_GPIO_Port GPIOA
#define DIST3_GPIO1_EXTI_IRQn EXTI11_IRQn
#define DIST3_XSHUT_Pin GPIO_PIN_12
#define DIST3_XSHUT_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_15
#define IMU_CS_GPIO_Port GPIOA
#define IMU_CLK_Pin GPIO_PIN_10
#define IMU_CLK_GPIO_Port GPIOC
#define IMU_MISO_Pin GPIO_PIN_11
#define IMU_MISO_GPIO_Port GPIOC
#define IMU_MOSI_Pin GPIO_PIN_12
#define IMU_MOSI_GPIO_Port GPIOC
#define SW_DIP_1_Pin GPIO_PIN_2
#define SW_DIP_1_GPIO_Port GPIOD
#define DEBUG_SWO_Pin GPIO_PIN_3
#define DEBUG_SWO_GPIO_Port GPIOB
#define DIST5_GPIO1_Pin GPIO_PIN_4
#define DIST5_GPIO1_GPIO_Port GPIOB
#define DIST5_GPIO1_EXTI_IRQn EXTI4_IRQn
#define DIST5_XSHUT_Pin GPIO_PIN_5
#define DIST5_XSHUT_GPIO_Port GPIOB
#define ENC_R_2_Pin GPIO_PIN_6
#define ENC_R_2_GPIO_Port GPIOB
#define ENC_R_1_Pin GPIO_PIN_7
#define ENC_R_1_GPIO_Port GPIOB
#define SW_DIP_2_Pin GPIO_PIN_8
#define SW_DIP_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
