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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define BTN_A1_Pin GPIO_PIN_1
#define BTN_A1_GPIO_Port GPIOA
#define BTN_A1_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BTN_A2_Pin GPIO_PIN_4
#define BTN_A2_GPIO_Port GPIOA
#define BTN_A2_EXTI_IRQn EXTI4_IRQn
#define LED_4_Pin GPIO_PIN_5
#define LED_4_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOA
#define BTN_A3_Pin GPIO_PIN_0
#define BTN_A3_GPIO_Port GPIOB
#define BTN_A3_EXTI_IRQn EXTI0_IRQn
#define DSP_SHIFT_Pin GPIO_PIN_8
#define DSP_SHIFT_GPIO_Port GPIOA
#define DSP_DATA_Pin GPIO_PIN_9
#define DSP_DATA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DSP_LATCH_Pin GPIO_PIN_5
#define DSP_LATCH_GPIO_Port GPIOB
#define LED_4B6_Pin GPIO_PIN_6
#define LED_4B6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */