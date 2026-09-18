/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ad7606.h
  * @brief   Driver for two AD7606 ADCs sampling synchronously (bit-banged
  *          serial interface, +/-5 V range, no oversampling).
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __AD7606_H__
#define __AD7606_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define AD7606_CH_PER_CHIP   8u
#define AD7606_TOTAL_CH      16u

void AD7606_Init(void);
void AD7606_StartConversion(void);
void AD7606_ReadAll(int16_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __AD7606_H__ */
