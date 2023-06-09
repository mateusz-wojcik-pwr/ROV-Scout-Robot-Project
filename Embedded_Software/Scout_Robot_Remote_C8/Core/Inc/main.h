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
#define JS_B_SW_Pin GPIO_PIN_13
#define JS_B_SW_GPIO_Port GPIOC
#define JS_A_SW_Pin GPIO_PIN_14
#define JS_A_SW_GPIO_Port GPIOC
#define JS_AX_ADC_Pin GPIO_PIN_0
#define JS_AX_ADC_GPIO_Port GPIOA
#define JS_AY_ADC_Pin GPIO_PIN_1
#define JS_AY_ADC_GPIO_Port GPIOA
#define JS_BX_ADC_Pin GPIO_PIN_2
#define JS_BX_ADC_GPIO_Port GPIOA
#define JS_BX_ADCA3_Pin GPIO_PIN_3
#define JS_BX_ADCA3_GPIO_Port GPIOA
#define VBAT_ADC_Pin GPIO_PIN_4
#define VBAT_ADC_GPIO_Port GPIOA
#define POT_ADC_Pin GPIO_PIN_5
#define POT_ADC_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define BT3_Pin GPIO_PIN_1
#define BT3_GPIO_Port GPIOB
#define BT2_Pin GPIO_PIN_10
#define BT2_GPIO_Port GPIOB
#define BT1_Pin GPIO_PIN_11
#define BT1_GPIO_Port GPIOB
#define LCD_LED_Pin GPIO_PIN_12
#define LCD_LED_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_12
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_6
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_7
#define NRF_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
