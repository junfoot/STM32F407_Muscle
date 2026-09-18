/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    serial.h
  * @brief   Non-blocking USART1 transmit path (ring buffer + DMA) shared by
  *          the 2000 Hz data frames and printf() log output.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
  * @brief  Enqueue bytes for transmission. Never blocks; when the buffer is
  *         full the data is dropped and an internal drop counter increases.
  * @param  data  Bytes to send
  * @param  len   Number of bytes
  * @retval Number of bytes actually queued (0 .. len)
  */
uint16_t Serial_Write(const uint8_t *data, uint16_t len);

/**
  * @brief  Advance the queue after a DMA transfer finished.
  *         Call from HAL_UART_TxCpltCallback() for USART1.
  */
void Serial_TxCpltHandler(void);

/**
  * @brief  Number of bytes dropped so far because the TX buffer was full.
  */
uint32_t Serial_GetDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H__ */
