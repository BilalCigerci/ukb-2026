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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define FT232_TX_Pin GPIO_PIN_2
#define FT232_TX_GPIO_Port GPIOA
#define FT232_RX_Pin GPIO_PIN_3
#define FT232_RX_GPIO_Port GPIOA
#define RECOVERY_STAT_1_Pin GPIO_PIN_4
#define RECOVERY_STAT_1_GPIO_Port GPIOA
#define RECOVERY_1_Pin GPIO_PIN_5
#define RECOVERY_1_GPIO_Port GPIOA
#define RECOVERY_ADC_1_Pin GPIO_PIN_4
#define RECOVERY_ADC_1_GPIO_Port GPIOC
#define RECOVERY_ADC_2_Pin GPIO_PIN_5
#define RECOVERY_ADC_2_GPIO_Port GPIOC
#define RECOVERY_STAT_2_Pin GPIO_PIN_2
#define RECOVERY_STAT_2_GPIO_Port GPIOB
#define RECOVERY_2_Pin GPIO_PIN_10
#define RECOVERY_2_GPIO_Port GPIOB
#define GYRO_CS_Pin GPIO_PIN_12
#define GYRO_CS_GPIO_Port GPIOB
#define SENSOR_SCK_Pin GPIO_PIN_13
#define SENSOR_SCK_GPIO_Port GPIOB
#define SENSOR_MISO_Pin GPIO_PIN_14
#define SENSOR_MISO_GPIO_Port GPIOB
#define SENSOR_MOSI_Pin GPIO_PIN_15
#define SENSOR_MOSI_GPIO_Port GPIOB
#define ACC_INT_Pin GPIO_PIN_6
#define ACC_INT_GPIO_Port GPIOC
#define ACC_INT_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_EN_Pin GPIO_PIN_9
#define SENSOR_EN_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_8
#define ACC_CS_GPIO_Port GPIOA
#define GYRO_INT_Pin GPIO_PIN_9
#define GYRO_INT_GPIO_Port GPIOA
#define GYRO_INT_EXTI_IRQn EXTI9_5_IRQn
#define BARO_INT_Pin GPIO_PIN_11
#define BARO_INT_GPIO_Port GPIOA
#define BARO_INT_EXTI_IRQn EXTI15_10_IRQn
#define BARO_CS_Pin GPIO_PIN_12
#define BARO_CS_GPIO_Port GPIOA
#define MAX3232_TX_Pin GPIO_PIN_10
#define MAX3232_TX_GPIO_Port GPIOC
#define MAX3232_RX_Pin GPIO_PIN_11
#define MAX3232_RX_GPIO_Port GPIOC
#define W25Q_CS_Pin GPIO_PIN_6
#define W25Q_CS_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_8
#define USER_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
