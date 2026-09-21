/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    emg_filter.h
  * @brief   Real-time 16-channel sEMG filter.
  *
  * The filter is intended for the 2 kHz AD7606 stream.  It is a causal
  * band-pass chain (20 Hz high-pass, 50 Hz mains notch, 450 Hz low-pass).
  * The input is the raw AD7606 code and the output is in volts, matching the
  * units used by the USART1 JustFloat stream.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __EMG_FILTER_H__
#define __EMG_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define EMG_FILTER_CHANNELS 16u

void EMG_Filter_Init(void);
void EMG_Filter_Process(const int16_t *raw_adc, volatile float *filtered_volts);

#ifdef __cplusplus
}
#endif

#endif /* __EMG_FILTER_H__ */
