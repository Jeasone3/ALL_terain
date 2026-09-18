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
#include "stm32f1xx_hal.h"

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
#define IN4_Pin GPIO_PIN_0
#define IN4_GPIO_Port GPIOC
#define IN3_Pin GPIO_PIN_1
#define IN3_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_3
#define IN1_GPIO_Port GPIOC
#define EC_Pin GPIO_PIN_4
#define EC_GPIO_Port GPIOA
#define EC_EXTI_IRQn EXTI4_IRQn
#define TR_Pin GPIO_PIN_5
#define TR_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_10
#define BEEP_GPIO_Port GPIOB
#define IN8_Pin GPIO_PIN_12
#define IN8_GPIO_Port GPIOB
#define IN7_Pin GPIO_PIN_13
#define IN7_GPIO_Port GPIOB
#define IN6_Pin GPIO_PIN_14
#define IN6_GPIO_Port GPIOB
#define IN5_Pin GPIO_PIN_15
#define IN5_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_8
#define UP_GPIO_Port GPIOA
#define DOWN_Pin GPIO_PIN_9
#define DOWN_GPIO_Port GPIOA
#define RIGHT_Pin GPIO_PIN_10
#define RIGHT_GPIO_Port GPIOA
#define OK_Pin GPIO_PIN_11
#define OK_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_12
#define LEFT_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_11
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_12
#define AIN2_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_2
#define BIN2_GPIO_Port GPIOD
#define MPU_SDA_Pin GPIO_PIN_3
#define MPU_SDA_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_4
#define MPU_SCL_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_7
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
