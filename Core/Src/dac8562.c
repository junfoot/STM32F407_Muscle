/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac8562.c
  * @brief   Driver for the DAC8562 dual 16-bit DAC (bit-banged SPI).
  *
  *          24-bit frame, MSB first: [7:3] command, [2:0]... first byte is
  *          0b00 + CMD[2:0] + ADDR[2:0], then 16 data bits. DIN is clocked
  *          in on the falling edge of SCLK. Internal 2.5 V reference is
  *          enabled (gain x2 -> 0..5 V output span); LDAC is held low so
  *          outputs update immediately.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dac8562.h"

/* Private define ------------------------------------------------------------*/
#define DAC8562_CMD_WRITE_UPDATE   0x03u   /* write input register and update DAC */
#define DAC8562_CMD_REFERENCE      0x07u   /* internal reference setup            */
#define DAC8562_SPI_DLY            8u      /* SCLK half period, ~50 ns            */

/* Private function prototypes -----------------------------------------------*/
static void dac8562_write24(uint8_t cmd_addr, uint16_t data);

/**
  * @brief  Send one 24-bit word. cmd_addr = (CMD << 3) | ADDR.
  */
static void dac8562_write24(uint8_t cmd_addr, uint16_t data)
{
  uint32_t word = ((uint32_t)cmd_addr << 16) | (uint32_t)data;

  GPIOA->BSRR = (uint32_t)DAC_SYNC_Pin << 16u;   /* SYNC low: enable shift register */

  for (uint32_t i = 24u; i > 0u; i--)
  {
    if ((word & (1ul << (i - 1u))) != 0u)
    {
      GPIOA->BSRR = (uint32_t)DAC_DIN_Pin;
    }
    else
    {
      GPIOA->BSRR = (uint32_t)DAC_DIN_Pin << 16u;
    }
    for (volatile uint32_t d = 0u; d < DAC8562_SPI_DLY; d++) { __NOP(); }
    GPIOA->BSRR = (uint32_t)DAC_SCLK_Pin;        /* raise SCLK              */
    for (volatile uint32_t d = 0u; d < DAC8562_SPI_DLY; d++) { __NOP(); }
    GPIOA->BSRR = (uint32_t)DAC_SCLK_Pin << 16u; /* falling edge latches bit */
  }

  for (volatile uint32_t d = 0u; d < DAC8562_SPI_DLY; d++) { __NOP(); }
  GPIOA->BSRR = (uint32_t)DAC_SYNC_Pin;          /* SYNC high: load registers */
}

/**
  * @brief  Release CLR, hold LDAC low, enable the internal reference and
  *         drive both outputs to 0 V.
  */
void DAC8562_Init(void)
{
  /* CLR pulse (active low) then keep it high */
  HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_SET);
  /* LDAC tied low: outputs follow the input registers immediately */
  HAL_GPIO_WritePin(DAC_LDAC_GPIO_Port, DAC_LDAC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  /* Enable internal reference (power-on default is off) */
  dac8562_write24((uint8_t)(DAC8562_CMD_REFERENCE << 3), 0x0001u);

  DAC8562_SetOutput(DAC8562_CH_A, 0u);
  DAC8562_SetOutput(DAC8562_CH_B, 0u);
}

/**
  * @brief  Set the DAC output code (0..65535 -> 0..5 V).
  * @param  channel  DAC8562_CH_A, DAC8562_CH_B or DAC8562_CH_BOTH
  */
void DAC8562_SetOutput(uint8_t channel, uint16_t value)
{
  if ((channel == DAC8562_CH_A) || (channel == DAC8562_CH_BOTH))
  {
    dac8562_write24((uint8_t)((DAC8562_CMD_WRITE_UPDATE << 3) | DAC8562_CH_A), value);
  }
  if ((channel == DAC8562_CH_B) || (channel == DAC8562_CH_BOTH))
  {
    dac8562_write24((uint8_t)((DAC8562_CMD_WRITE_UPDATE << 3) | DAC8562_CH_B), value);
  }
}
