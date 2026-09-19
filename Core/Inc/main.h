/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define DAC_DIN_Pin GPIO_PIN_0
#define DAC_DIN_GPIO_Port GPIOA
#define REC_LED_Pin GPIO_PIN_1
#define REC_LED_GPIO_Port GPIOA
#define DAC_SYNC_Pin GPIO_PIN_2
#define DAC_SYNC_GPIO_Port GPIOA
#define REC_SW_Pin GPIO_PIN_4
#define REC_SW_GPIO_Port GPIOA
#define DAC_LDAC_Pin GPIO_PIN_5
#define DAC_LDAC_GPIO_Port GPIOA
#define DAC_CLR_Pin GPIO_PIN_6
#define DAC_CLR_GPIO_Port GPIOA
#define DAC_SCLK_Pin GPIO_PIN_8
#define DAC_SCLK_GPIO_Port GPIOA
#define AD1_BUSY_Pin GPIO_PIN_0
#define AD1_BUSY_GPIO_Port GPIOB
#define AD1_BUSY_EXTI_IRQn EXTI0_IRQn
#define AD1_CS_Pin GPIO_PIN_8
#define AD1_CS_GPIO_Port GPIOB
#define AD1_DOUT_Pin GPIO_PIN_9
#define AD1_DOUT_GPIO_Port GPIOB
#define AD1_RST_Pin GPIO_PIN_0
#define AD1_RST_GPIO_Port GPIOC
#define AD1_SCK_Pin GPIO_PIN_1
#define AD1_SCK_GPIO_Port GPIOC
#define AD1_CVA_Pin GPIO_PIN_2
#define AD1_CVA_GPIO_Port GPIOC
#define AD1_CVB_Pin GPIO_PIN_3
#define AD1_CVB_GPIO_Port GPIOC
#define AD1_OS2_Pin GPIO_PIN_4
#define AD1_OS2_GPIO_Port GPIOC
#define AD1_RANGE_Pin GPIO_PIN_5
#define AD1_RANGE_GPIO_Port GPIOC
#define AD1_OS0_Pin GPIO_PIN_6
#define AD1_OS0_GPIO_Port GPIOC
#define AD1_OS1_Pin GPIO_PIN_7
#define AD1_OS1_GPIO_Port GPIOC
#define AD2_DOUT_Pin GPIO_PIN_0
#define AD2_DOUT_GPIO_Port GPIOD
#define AD2_OS0_Pin GPIO_PIN_1
#define AD2_OS0_GPIO_Port GPIOD
#define AD2_BUSY_Pin GPIO_PIN_7
#define AD2_BUSY_GPIO_Port GPIOD
#define AD2_BUSY_EXTI_IRQn EXTI9_5_IRQn
#define AD2_CS_Pin GPIO_PIN_8
#define AD2_CS_GPIO_Port GPIOD
#define AD2_SCK_Pin GPIO_PIN_9
#define AD2_SCK_GPIO_Port GPIOD
#define AD2_RST_Pin GPIO_PIN_10
#define AD2_RST_GPIO_Port GPIOD
#define AD2_CVA_Pin GPIO_PIN_11
#define AD2_CVA_GPIO_Port GPIOD
#define AD2_CVB_Pin GPIO_PIN_12
#define AD2_CVB_GPIO_Port GPIOD
#define AD2_RANGE_Pin GPIO_PIN_13
#define AD2_RANGE_GPIO_Port GPIOD
#define AD2_OS2_Pin GPIO_PIN_14
#define AD2_OS2_GPIO_Port GPIOD
#define AD2_OS1_Pin GPIO_PIN_15
#define AD2_OS1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
