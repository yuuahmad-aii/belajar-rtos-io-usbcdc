/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
void USB_CDC_RxHandler(uint8_t *, uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define INPUT_PIN_0_Pin GPIO_PIN_4
#define INPUT_PIN_0_GPIO_Port GPIOA
#define HC595_LOAD_Pin GPIO_PIN_6
#define HC595_LOAD_GPIO_Port GPIOA
#define INPUT_PIN_1_Pin GPIO_PIN_0
#define INPUT_PIN_1_GPIO_Port GPIOB
#define INPUT_PIN_2_Pin GPIO_PIN_1
#define INPUT_PIN_2_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_2
#define USER_LED_GPIO_Port GPIOB
#define INPUT_PIN_3_Pin GPIO_PIN_10
#define INPUT_PIN_3_GPIO_Port GPIOB
#define INPUT_PIN_4_Pin GPIO_PIN_11
#define INPUT_PIN_4_GPIO_Port GPIOB
#define INPUT_PIN_5_Pin GPIO_PIN_12
#define INPUT_PIN_5_GPIO_Port GPIOB
#define INPUT_PIN_6_Pin GPIO_PIN_13
#define INPUT_PIN_6_GPIO_Port GPIOB
#define INPUT_PIN_7_Pin GPIO_PIN_14
#define INPUT_PIN_7_GPIO_Port GPIOB
#define INPUT_PIN_8_Pin GPIO_PIN_15
#define INPUT_PIN_8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
