/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Line-based string command interface on USART1.
  *
  *          Commands (case-insensitive, terminated by CR or LF):
  *            DAC A 3.3      -> DAC channel A outputs +3.3 V
  *            DAC B -2.5     -> DAC channel B outputs -2.5 V
  *            DAC AB 0       -> both channels output 0 V
  *            DACR A 49152   -> raw 16-bit code (0x0000=-10V .. 0xFFFF=+10V)
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

/**
  * @brief  Feed one received byte (call from the UART RX callback).
  *         Completes a command line on CR/LF, which is then handled in
  *         Cmd_Process().
  */
void Cmd_RxByte(uint8_t byte);

/**
  * @brief  Execute a pending command line, if any. Call from the main loop.
  */
void Cmd_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H__ */
