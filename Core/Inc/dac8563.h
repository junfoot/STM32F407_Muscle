/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac8563.h
  * @brief   Driver for the Armfly (安富莱) DAC8563 dual 16-bit DAC module
  *          (bit-banged SPI). The module analog stage maps the DAC code to
  *          a +/-10 V output swing: 0x0000 -> -10 V, 0x8000 -> 0 V,
  *          0xFFFF -> +10 V.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DAC8563_H__
#define __DAC8563_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DAC8563_CH_A     0u
#define DAC8563_CH_B     1u
#define DAC8563_CH_BOTH  2u

#define DAC8563_VMIN     (-10.0f)
#define DAC8563_VMAX     (10.0f)

void DAC8563_Init(void);
void DAC8563_SetOutput(uint8_t channel, uint16_t value);
void DAC8563_SetVoltage(uint8_t channel, float volts);

/**
  * @brief  Last commanded output voltage of a channel (for telemetry).
  * @param  channel  DAC8563_CH_A or DAC8563_CH_B
  */
float DAC8563_GetVoltage(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __DAC8563_H__ */
