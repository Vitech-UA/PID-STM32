/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	DISPLAY_INT, DISPLAY_FLOAT
} display_type_t;
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
void sensor_error_handler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZERO_DETECT_Pin GPIO_PIN_5
#define ZERO_DETECT_GPIO_Port GPIOA
#define ZERO_DETECT_EXTI_IRQn EXTI9_5_IRQn
#define DS18B20_DQ_Pin GPIO_PIN_12
#define DS18B20_DQ_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_10
#define BTN2_GPIO_Port GPIOA
#define BTN2_EXTI_IRQn EXTI15_10_IRQn
#define BTN1_Pin GPIO_PIN_11
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOA
#define MAX7219_NCS_Pin GPIO_PIN_4
#define MAX7219_NCS_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
