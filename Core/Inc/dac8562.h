/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac8562.h
  * @brief   Driver for the DAC8562 dual 16-bit DAC (bit-banged SPI).
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DAC8562_H__
#define __DAC8562_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DAC8562_CH_A     0u
#define DAC8562_CH_B     1u
#define DAC8562_CH_BOTH  2u

void DAC8562_Init(void);
void DAC8562_SetOutput(uint8_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __DAC8562_H__ */
