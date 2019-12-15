/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define STATUS_LED_Pin GPIO_PIN_0
#define STATUS_LED_GPIO_Port GPIOC
#define GPS_FIX_Pin GPIO_PIN_1
#define GPS_FIX_GPIO_Port GPIOC
#define GPS_FIX_EXTI_IRQn EXTI1_IRQn
#define PPS_Pin GPIO_PIN_2
#define PPS_GPIO_Port GPIOC
#define PPS_EXTI_IRQn EXTI2_IRQn
#define CARD_DETECT_Pin GPIO_PIN_6
#define CARD_DETECT_GPIO_Port GPIOC
#define CARD_SELECT_Pin GPIO_PIN_7
#define CARD_SELECT_GPIO_Port GPIOC
#define GC_INT_Pin GPIO_PIN_8
#define GC_INT_GPIO_Port GPIOA
#define GC_INT_EXTI_IRQn EXTI9_5_IRQn
#define SET_BTN_Pin GPIO_PIN_10
#define SET_BTN_GPIO_Port GPIOA
#define SET_BTN_EXTI_IRQn EXTI15_10_IRQn
#define MODE_BTN_Pin GPIO_PIN_11
#define MODE_BTN_GPIO_Port GPIOA
#define MODE_BTN_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
