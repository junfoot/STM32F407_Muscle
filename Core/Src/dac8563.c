/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac8563.c
  * @brief   Driver for the Armfly (安富莱) DAC8563 dual 16-bit DAC module
  *          (bit-banged SPI).
  *
  *          24-bit frame, MSB first: first byte is 0b00 + CMD[2:0] + ADDR[2:0],
  *          then 16 data bits. DIN is clocked in on the falling edge of SCLK.
  *          The internal 2.5 V reference is enabled; the module analog stage
  *          maps the code to a 0..10 V output swing when both output-range
  *          jumpers J1/J2 are shorted at pins 1-2
  *          (0x0000 -> 0 V, 0x8000 -> about 5 V, 0xFFFF -> 10 V).
  *          LDAC is held low so outputs update immediately.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dac8563.h"

/* Private define ------------------------------------------------------------*/
#define DAC8563_CMD_WRITE_UPDATE   0x03u   /* write input register and update DAC */
#define DAC8563_CMD_REFERENCE      0x07u   /* internal reference setup            */
#define DAC8563_SPI_DLY            8u      /* SCLK half period, ~50 ns            */

#define DAC8563_ZERO_CODE          0x0000u /* 0 V on the 0..10 V module          */

/* Private function prototypes -----------------------------------------------*/
static void dac8563_write24(uint8_t cmd_addr, uint16_t data);

/* Private variables ---------------------------------------------------------*/
static uint16_t dac_last_code[2] = {DAC8563_ZERO_CODE, DAC8563_ZERO_CODE};

/**
  * @brief  Send one 24-bit word. cmd_addr = (CMD << 3) | ADDR.
  */
static void dac8563_write24(uint8_t cmd_addr, uint16_t data)
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
    for (volatile uint32_t d = 0u; d < DAC8563_SPI_DLY; d++) { __NOP(); }
    GPIOA->BSRR = (uint32_t)DAC_SCLK_Pin;        /* raise SCLK              */
    for (volatile uint32_t d = 0u; d < DAC8563_SPI_DLY; d++) { __NOP(); }
    GPIOA->BSRR = (uint32_t)DAC_SCLK_Pin << 16u; /* falling edge latches bit */
  }

  for (volatile uint32_t d = 0u; d < DAC8563_SPI_DLY; d++) { __NOP(); }
  GPIOA->BSRR = (uint32_t)DAC_SYNC_Pin;          /* SYNC high: load registers */
}

/**
  * @brief  Release CLR, hold LDAC low, enable the internal reference and
  *         drive both outputs to 0 V.
  */
void DAC8563_Init(void)
{
  /* CLR pulse (active low) then keep it high */
  HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_SET);
  /* LDAC tied low: outputs follow the input registers immediately */
  HAL_GPIO_WritePin(DAC_LDAC_GPIO_Port, DAC_LDAC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  /* Enable internal reference (power-on default is off) */
  dac8563_write24((uint8_t)(DAC8563_CMD_REFERENCE << 3), 0x0001u);

  DAC8563_SetOutput(DAC8563_CH_A, DAC8563_ZERO_CODE);
  DAC8563_SetOutput(DAC8563_CH_B, DAC8563_ZERO_CODE);
}

/**
  * @brief  Set the DAC output code (0x0000 -> 0 V .. 0xFFFF -> 10 V).
  * @param  channel  DAC8563_CH_A, DAC8563_CH_B or DAC8563_CH_BOTH
  */
void DAC8563_SetOutput(uint8_t channel, uint16_t value)
{
  if ((channel == DAC8563_CH_A) || (channel == DAC8563_CH_BOTH))
  {
    dac8563_write24((uint8_t)((DAC8563_CMD_WRITE_UPDATE << 3) | DAC8563_CH_A), value);
    dac_last_code[DAC8563_CH_A] = value;
  }
  if ((channel == DAC8563_CH_B) || (channel == DAC8563_CH_BOTH))
  {
    dac8563_write24((uint8_t)((DAC8563_CMD_WRITE_UPDATE << 3) | DAC8563_CH_B), value);
    dac_last_code[DAC8563_CH_B] = value;
  }
}

/**
  * @brief  Last commanded output voltage of a channel (for telemetry).
  */
float DAC8563_GetVoltage(uint8_t channel)
{
  if (channel > DAC8563_CH_B)
  {
    return 0.0f;
  }
  return (float)dac_last_code[channel] * (10.0f / 65535.0f);
}

/**
  * @brief  Set the DAC output voltage, clamped to 0..10 V.
  * @param  channel  DAC8563_CH_A, DAC8563_CH_B or DAC8563_CH_BOTH
  * @param  volts    Target voltage in V, 0.0 .. 10.0
  */
void DAC8563_SetVoltage(uint8_t channel, float volts)
{
  float code;

  if (volts > DAC8563_VMAX)
  {
    volts = DAC8563_VMAX;
  }
  else if (volts < DAC8563_VMIN)
  {
    volts = DAC8563_VMIN;
  }

  code = (volts - DAC8563_VMIN) * (65535.0f / (DAC8563_VMAX - DAC8563_VMIN)) + 0.5f;
  DAC8563_SetOutput(channel, (uint16_t)code);
}
