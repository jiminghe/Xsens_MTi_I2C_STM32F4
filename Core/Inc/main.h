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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define I2C_ADD0_PIN_Pin GPIO_PIN_5
#define I2C_ADD0_PIN_GPIO_Port GPIOA
#define I2C_ADD1_PIN_Pin GPIO_PIN_6
#define I2C_ADD1_PIN_GPIO_Port GPIOA
#define I2C_ADD2_PIN_Pin GPIO_PIN_7
#define I2C_ADD2_PIN_GPIO_Port GPIOA
#define PSEL0_PIN_Pin GPIO_PIN_7
#define PSEL0_PIN_GPIO_Port GPIOC
#define PSEL1_PIN_Pin GPIO_PIN_9
#define PSEL1_PIN_GPIO_Port GPIOA
#define DATA_READY_PIN_Pin GPIO_PIN_3
#define DATA_READY_PIN_GPIO_Port GPIOB
#define DATA_READY_PIN_EXTI_IRQn EXTI3_IRQn
#define RESET_PIN_Pin GPIO_PIN_5
#define RESET_PIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
