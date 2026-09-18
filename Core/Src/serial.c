/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    serial.c
  * @brief   Non-blocking USART1 transmit path (ring buffer + DMA) shared by
  *          the 2000 Hz data frames and printf() log output.
  *
  *          All enqueue/kick decisions run with interrupts masked, so the
  *          producer (main loop) and the DMA completion ISR cannot corrupt
  *          the queue. printf() is retargeted here and never blocks: a full
  *          buffer simply drops output instead of stalling the 2000 Hz loop.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "serial.h"
#include "usart.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define SERIAL_TX_BUF_SIZE   4096u

/* Private variables ---------------------------------------------------------*/
static uint8_t  tx_buf[SERIAL_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0u;    /* next byte to transmit            */
static volatile uint16_t tx_tail = 0u;    /* next free write position         */
static volatile uint16_t tx_dma_len = 0u; /* length of the ongoing DMA chunk  */
static volatile uint32_t tx_dropped = 0u;

/* Private function prototypes -----------------------------------------------*/
static void serial_kick(void);

/**
  * @brief  Start a DMA transfer for the oldest contiguous chunk, if idle.
  *         Must be called with interrupts masked.
  */
static void serial_kick(void)
{
  uint16_t head;
  uint16_t tail;

  if (tx_dma_len != 0u)
  {
    return;
  }

  head = tx_head;
  tail = tx_tail;
  if (head == tail)
  {
    return;
  }

  tx_dma_len = (tail > head) ? (uint16_t)(tail - head) : (uint16_t)(SERIAL_TX_BUF_SIZE - head);
  if (HAL_UART_Transmit_DMA(&huart1, &tx_buf[head], tx_dma_len) != HAL_OK)
  {
    tx_dma_len = 0u;
  }
}

uint16_t Serial_Write(const uint8_t *data, uint16_t len)
{
  uint16_t head;
  uint16_t tail;
  uint16_t space;
  uint16_t first;

  __disable_irq();

  head = tx_head;
  tail = tx_tail;
  space = (tail >= head) ? (uint16_t)(SERIAL_TX_BUF_SIZE - (tail - head) - 1u)
                         : (uint16_t)(head - tail - 1u);
  if (space < len)
  {
    tx_dropped += (uint32_t)len;
    __enable_irq();
    return 0u;
  }

  first = (uint16_t)(SERIAL_TX_BUF_SIZE - tail);
  if (first > len)
  {
    first = len;
  }
  for (uint16_t i = 0u; i < first; i++)
  {
    tx_buf[tail + i] = data[i];
  }
  for (uint16_t i = first; i < len; i++)
  {
    tx_buf[i - first] = data[i];
  }
  tx_tail = (uint16_t)((tail + len) % SERIAL_TX_BUF_SIZE);

  serial_kick();

  __enable_irq();
  return len;
}

void Serial_TxCpltHandler(void)
{
  __disable_irq();
  tx_head = (uint16_t)((tx_head + tx_dma_len) % SERIAL_TX_BUF_SIZE);
  tx_dma_len = 0u;
  serial_kick();
  __enable_irq();
}

uint32_t Serial_GetDropCount(void)
{
  return tx_dropped;
}

/* printf retarget (ARMCC, MicroLIB disabled) --------------------------------*/
#pragma import(__use_no_semihosting)

struct __FILE
{
  int handle;
};
FILE __stdout;

int fputc(int ch, FILE *f)
{
  (void)f;
  (void)Serial_Write((const uint8_t *)&ch, 1u);
  return ch;
}

void _sys_exit(int x)
{
  (void)x;
  for (;;)
  {
  }
}

void _ttywrch(int ch)
{
  (void)ch;
}
