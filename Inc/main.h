/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define MX_LED_BLUE_Pin GPIO_PIN_4
#define MX_LED_BLUE_GPIO_Port GPIOA
#define MX_MOTOR_BIN2_Pin GPIO_PIN_12
#define MX_MOTOR_BIN2_GPIO_Port GPIOB
#define MX_MOTOR_BIN1_Pin GPIO_PIN_13
#define MX_MOTOR_BIN1_GPIO_Port GPIOB
#define MX_MOTOR_AIN1_Pin GPIO_PIN_14
#define MX_MOTOR_AIN1_GPIO_Port GPIOB
#define MX_MOTOR_AIN2_Pin GPIO_PIN_15
#define MX_MOTOR_AIN2_GPIO_Port GPIOB
#define MX_IIC_CLK_Pin GPIO_PIN_8
#define MX_IIC_CLK_GPIO_Port GPIOB
#define MX_IIC_DATA_Pin GPIO_PIN_9
#define MX_IIC_DATA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
