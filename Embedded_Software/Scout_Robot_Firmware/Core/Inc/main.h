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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MOT_A_CURR_ADC_Pin GPIO_PIN_0
#define MOT_A_CURR_ADC_GPIO_Port GPIOC
#define MOT_B_CURR_ADC_Pin GPIO_PIN_1
#define MOT_B_CURR_ADC_GPIO_Port GPIOC
#define MOT_C_CURR_ADC_Pin GPIO_PIN_2
#define MOT_C_CURR_ADC_GPIO_Port GPIOC
#define MOT_D_CURR_ADC_Pin GPIO_PIN_3
#define MOT_D_CURR_ADC_GPIO_Port GPIOC
#define BAT_CURR_CHRG_ADC_Pin GPIO_PIN_0
#define BAT_CURR_CHRG_ADC_GPIO_Port GPIOA
#define US_SENS_B_TRIG_Pin GPIO_PIN_1
#define US_SENS_B_TRIG_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define US_SENS_C_TRIG_Pin GPIO_PIN_4
#define US_SENS_C_TRIG_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BAT_CURR_DISCH_ADC_Pin GPIO_PIN_6
#define BAT_CURR_DISCH_ADC_GPIO_Port GPIOA
#define V_BAT_ADC_Pin GPIO_PIN_7
#define V_BAT_ADC_GPIO_Port GPIOA
#define LIGHT_SENS_ADC_Pin GPIO_PIN_4
#define LIGHT_SENS_ADC_GPIO_Port GPIOC
#define V_CHRG_ADC_Pin GPIO_PIN_5
#define V_CHRG_ADC_GPIO_Port GPIOC
#define NRF_CSN_Pin GPIO_PIN_0
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_1
#define NRF_CE_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_2
#define NRF_IRQ_GPIO_Port GPIOB
#define US_SENS_A_TRIG_Pin GPIO_PIN_12
#define US_SENS_A_TRIG_GPIO_Port GPIOB
#define MOT_A_PWM_Pin GPIO_PIN_6
#define MOT_A_PWM_GPIO_Port GPIOC
#define MOT_B_PWM_Pin GPIO_PIN_7
#define MOT_B_PWM_GPIO_Port GPIOC
#define MOT_C_PWM_Pin GPIO_PIN_8
#define MOT_C_PWM_GPIO_Port GPIOC
#define MOT_D_PWM_Pin GPIO_PIN_9
#define MOT_D_PWM_GPIO_Port GPIOC
#define US_SENS_A_ECHO_Pin GPIO_PIN_8
#define US_SENS_A_ECHO_GPIO_Port GPIOA
#define US_SENS_B_ECHO_Pin GPIO_PIN_9
#define US_SENS_B_ECHO_GPIO_Port GPIOA
#define US_SENS_C_ECHO_Pin GPIO_PIN_10
#define US_SENS_C_ECHO_GPIO_Port GPIOA
#define CHRG_EN_Pin GPIO_PIN_11
#define CHRG_EN_GPIO_Port GPIOA
#define PWR_LEDS_EN_Pin GPIO_PIN_12
#define PWR_LEDS_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOT_D_DIR_Pin GPIO_PIN_15
#define MOT_D_DIR_GPIO_Port GPIOA
#define MOT_C_DIR_Pin GPIO_PIN_10
#define MOT_C_DIR_GPIO_Port GPIOC
#define MOT_B_DIR_Pin GPIO_PIN_11
#define MOT_B_DIR_GPIO_Port GPIOC
#define MOT_A_DIR_Pin GPIO_PIN_12
#define MOT_A_DIR_GPIO_Port GPIOC
#define MOT_EN_Pin GPIO_PIN_2
#define MOT_EN_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define WSLED_DIN_Pin GPIO_PIN_5
#define WSLED_DIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
