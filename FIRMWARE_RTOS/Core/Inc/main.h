/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include "SX1278.h"
#include "st7735newLib.h"
#include "keyboard.h"
#include "beeper.h"

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
#define F_Pin GPIO_PIN_14
#define F_GPIO_Port GPIOC
#define D_Pin GPIO_PIN_15
#define D_GPIO_Port GPIOC
#define E_Pin GPIO_PIN_0
#define E_GPIO_Port GPIOC
#define C_Pin GPIO_PIN_2
#define C_GPIO_Port GPIOC
#define A_Pin GPIO_PIN_3
#define A_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_0
#define Buzzer_GPIO_Port GPIOA
#define PWR_B_Pin GPIO_PIN_1
#define PWR_B_GPIO_Port GPIOA
#define PWR_Pin GPIO_PIN_2
#define PWR_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_3
#define GPIO1_GPIO_Port GPIOA
#define TFT_SCL_Pin GPIO_PIN_5
#define TFT_SCL_GPIO_Port GPIOA
#define TFT_SDA_Pin GPIO_PIN_7
#define TFT_SDA_GPIO_Port GPIOA
#define TFT_RES_Pin GPIO_PIN_4
#define TFT_RES_GPIO_Port GPIOC
#define TFT_DS_Pin GPIO_PIN_5
#define TFT_DS_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_0
#define TFT_CS_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_12
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_14
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_15
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_6
#define LORA_RST_GPIO_Port GPIOC
#define LORA_DIO0_Pin GPIO_PIN_7
#define LORA_DIO0_GPIO_Port GPIOC
#define K_1_Pin GPIO_PIN_8
#define K_1_GPIO_Port GPIOC
#define K_1_EXTI_IRQn EXTI9_5_IRQn
#define K_2_Pin GPIO_PIN_9
#define K_2_GPIO_Port GPIOC
#define K_2_EXTI_IRQn EXTI9_5_IRQn
#define Relay_Pin GPIO_PIN_8
#define Relay_GPIO_Port GPIOA
#define K_5_Pin GPIO_PIN_15
#define K_5_GPIO_Port GPIOA
#define K_5_EXTI_IRQn EXTI15_10_IRQn
#define K_4_Pin GPIO_PIN_10
#define K_4_GPIO_Port GPIOC
#define K_4_EXTI_IRQn EXTI15_10_IRQn
#define K_3_Pin GPIO_PIN_11
#define K_3_GPIO_Port GPIOC
#define K_3_EXTI_IRQn EXTI15_10_IRQn
#define B_Pin GPIO_PIN_12
#define B_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
