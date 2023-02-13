/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define S7_Pin GPIO_PIN_14
#define S7_GPIO_Port GPIOC
#define X2_Pin GPIO_PIN_0
#define X2_GPIO_Port GPIOA
#define Y2_Pin GPIO_PIN_1
#define Y2_GPIO_Port GPIOA
#define S9_Pin GPIO_PIN_2
#define S9_GPIO_Port GPIOA
#define S9_EXTI_IRQn EXTI2_IRQn
#define S10_Pin GPIO_PIN_3
#define S10_GPIO_Port GPIOA
#define S10_EXTI_IRQn EXTI3_IRQn
#define SBAT_Pin GPIO_PIN_4
#define SBAT_GPIO_Port GPIOA
#define S6_Pin GPIO_PIN_5
#define S6_GPIO_Port GPIOA
#define S6_EXTI_IRQn EXTI9_5_IRQn
#define ENCODE2_Pin GPIO_PIN_6
#define ENCODE2_GPIO_Port GPIOA
#define X1_Pin GPIO_PIN_7
#define X1_GPIO_Port GPIOA
#define Y1_Pin GPIO_PIN_0
#define Y1_GPIO_Port GPIOB
#define ENCODE1_Pin GPIO_PIN_1
#define ENCODE1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOB
#define S2_EXTI_IRQn EXTI15_10_IRQn
#define S11_Pin GPIO_PIN_11
#define S11_GPIO_Port GPIOB
#define S11_EXTI_IRQn EXTI15_10_IRQn
#define S8_Pin GPIO_PIN_12
#define S8_GPIO_Port GPIOB
#define S8_EXTI_IRQn EXTI15_10_IRQn
#define S5_Pin GPIO_PIN_13
#define S5_GPIO_Port GPIOB
#define S5_EXTI_IRQn EXTI15_10_IRQn
#define S3_Pin GPIO_PIN_14
#define S3_GPIO_Port GPIOB
#define S3_EXTI_IRQn EXTI15_10_IRQn
#define S1_Pin GPIO_PIN_8
#define S1_GPIO_Port GPIOB
#define S1_EXTI_IRQn EXTI9_5_IRQn
#define S4_Pin GPIO_PIN_9
#define S4_GPIO_Port GPIOB
#define S4_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
