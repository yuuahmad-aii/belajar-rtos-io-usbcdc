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
#define PROXY_UMB_A_Pin GPIO_PIN_4
#define PROXY_UMB_A_GPIO_Port GPIOA
#define HC595_LOAD_Pin GPIO_PIN_6
#define HC595_LOAD_GPIO_Port GPIOA
#define ORIENT_OK_Pin GPIO_PIN_0
#define ORIENT_OK_GPIO_Port GPIOB
#define PROXY_TOOLS_Pin GPIO_PIN_1
#define PROXY_TOOLS_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_2
#define USER_LED_GPIO_Port GPIOB
#define PROXY_UMB_B_Pin GPIO_PIN_10
#define PROXY_UMB_B_GPIO_Port GPIOB
#define PROXY_CLAMP_A_Pin GPIO_PIN_11
#define PROXY_CLAMP_A_GPIO_Port GPIOB
#define PROXY_CLAMP_B_Pin GPIO_PIN_12
#define PROXY_CLAMP_B_GPIO_Port GPIOB
#define SENSOR_OLI_Pin GPIO_PIN_13
#define SENSOR_OLI_GPIO_Port GPIOB
#define INPUT_CLAMP_Pin GPIO_PIN_14
#define INPUT_CLAMP_GPIO_Port GPIOB
#define INPUT_UNCLAMP_Pin GPIO_PIN_15
#define INPUT_UNCLAMP_GPIO_Port GPIOB
#define MODBUS_SEL_Pin GPIO_PIN_8
#define MODBUS_SEL_GPIO_Port GPIOA
#define MODBUS_TX_Pin GPIO_PIN_9
#define MODBUS_TX_GPIO_Port GPIOA
#define MODBUS_RX_Pin GPIO_PIN_10
#define MODBUS_RX_GPIO_Port GPIOA
#define INPUT_PWM_Pin GPIO_PIN_15
#define INPUT_PWM_GPIO_Port GPIOA
#define INPUT_CCW_Pin GPIO_PIN_3
#define INPUT_CCW_GPIO_Port GPIOB
#define INPUT_CCW_EXTI_IRQn EXTI3_IRQn
#define INPUT_CW_Pin GPIO_PIN_4
#define INPUT_CW_GPIO_Port GPIOB
#define INPUT_CW_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
