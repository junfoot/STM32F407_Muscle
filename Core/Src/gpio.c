/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin|DAC_CLR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DAC_DIN_Pin|DAC_LDAC_Pin|DAC_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AD1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AD1_CVA_Pin|AD1_CVB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, AD1_RST_Pin|AD1_SCK_Pin|AD1_OS2_Pin|AD1_RANGE_Pin
                          |AD1_OS0_Pin|AD1_OS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AD2_CS_Pin|AD2_CVA_Pin|AD2_CVB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, AD2_OS0_Pin|AD2_SCK_Pin|AD2_RST_Pin|AD2_RANGE_Pin
                          |AD2_OS2_Pin|AD2_OS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DAC_SYNC_Pin DAC_DIN_Pin DAC_LDAC_Pin DAC_CLR_Pin DAC_SCLK_Pin */
  GPIO_InitStruct.Pin = DAC_SYNC_Pin|DAC_DIN_Pin|DAC_LDAC_Pin|DAC_CLR_Pin|DAC_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AD1_BUSY_Pin */
  GPIO_InitStruct.Pin = AD1_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AD1_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD1_CS_Pin */
  GPIO_InitStruct.Pin = AD1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD1_DOUT_Pin */
  GPIO_InitStruct.Pin = AD1_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AD1_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD1_RST_Pin AD1_CVA_Pin AD1_CVB_Pin AD1_OS2_Pin
                           AD1_RANGE_Pin AD1_OS0_Pin AD1_OS1_Pin */
  GPIO_InitStruct.Pin = AD1_RST_Pin|AD1_CVA_Pin|AD1_CVB_Pin|AD1_OS2_Pin
                          |AD1_RANGE_Pin|AD1_OS0_Pin|AD1_OS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : AD1_SCK_Pin */
  GPIO_InitStruct.Pin = AD1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD2_DOUT_Pin */
  GPIO_InitStruct.Pin = AD2_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AD2_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD2_BUSY_Pin */
  GPIO_InitStruct.Pin = AD2_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AD2_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD2_OS0_Pin AD2_CS_Pin AD2_RST_Pin AD2_CVA_Pin
                           AD2_CVB_Pin AD2_RANGE_Pin AD2_OS2_Pin AD2_OS1_Pin */
  GPIO_InitStruct.Pin = AD2_OS0_Pin|AD2_CS_Pin|AD2_RST_Pin|AD2_CVA_Pin
                          |AD2_CVB_Pin|AD2_RANGE_Pin|AD2_OS2_Pin|AD2_OS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : AD2_SCK_Pin */
  GPIO_InitStruct.Pin = AD2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD2_SCK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
