/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Line-based string command interface on USART1.
  *
  *          Commands (case-insensitive, terminated by CR or LF):
  *            DAC A 3.3      -> DAC channel A outputs +3.3 V
  *            DAC B 2.5      -> DAC channel B outputs +2.5 V
  *            DAC AB 0       -> both channels output 0 V
  *            DACR A 49152   -> raw 16-bit code (0x0000=0V .. 0xFFFF=10V)
  *            IMU            -> print online state and latest data of IMU 0-3
  *            REC            -> SD recorder status
  *            REC START      -> start logging to a new LOGxxxx.BIN
  *            REC STOP       -> flush and close the current log file
  *            HELP           -> print this list
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __CMD_H__
#define __CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/** Start the USART1 Receive-to-IDLE circular DMA command receiver. */
void Cmd_Init(void);

/** Forward a HAL Receive-to-IDLE event to the command receiver. */
void Cmd_RxEvent(uint16_t dma_position);

/** Recover the command receiver after a UART/DMA error. */
void Cmd_RxError(void);

/**
  * @brief  Execute a pending command line, if any. Call from the main loop.
  */
void Cmd_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H__ */
