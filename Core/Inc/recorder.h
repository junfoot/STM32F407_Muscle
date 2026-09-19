/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    recorder.h
  * @brief   SD card recorder for EMG (AD7606) and IMU raw data.
  *
  *          Producers (interrupt context) push fixed-size records into a
  *          large RAM ring; the main loop drains the ring into the current
  *          log file. Ring capacity covers >600 ms of full-rate data, so SD
  *          card housekeeping stalls do not lose samples; any overflow is
  *          counted and visible via Recorder_GetDropped() / the REC command.
  *
  *          Log file: LOG0000.BIN, LOG0001.BIN, ... (first free number).
  *          Binary record stream, little-endian:
  *            header (once per file, 32 B):
  *              "EMGL" u16 fmt_ver=1 | u16 adc_ch | u32 adc_rate_hz |
  *              u8 imu_count | reserved...
  *            ADC record (37 B): 0xA1 | u32 sample_seq | 16 x int16 raw
  *            IMU record (32 B): 0xB1 | u32 sample_seq | u8 device_id |
  *              26 B raw 0x61 payload (13 x int16: acc gyro mag angle bat)
  *          sample_seq is the 2000 Hz ADC tick, giving both record types a
  *          common time base and making any gap detectable.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __RECORDER_H__
#define __RECORDER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void     Recorder_Process(void);

/**
  * @brief  Probe the card without starting a recording. Silent (no printf),
  *         so the caller can print a boot status line first.
  * @retval 1 = SD card detected, 0 = no card detected
  */
uint8_t  Recorder_Init(void);

/* Producers, callable from ISR context */
void     Recorder_PushAdc(uint32_t seq, const int16_t *ch16);
void     Recorder_PushImu(uint32_t seq, uint8_t device_id, const uint8_t *payload26);

void     Recorder_Start(void);
void     Recorder_Stop(void);
uint8_t  Recorder_IsActive(void);
uint8_t  Recorder_IsRequested(void);
uint8_t  Recorder_IsCardPresent(void);
uint32_t Recorder_GetDropped(void);
uint32_t Recorder_GetBytesWritten(void);

#ifdef __cplusplus
}
#endif

#endif /* __RECORDER_H__ */
