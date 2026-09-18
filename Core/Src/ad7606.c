/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ad7606.c
  * @brief   Driver for two AD7606 ADCs sampling synchronously.
  *
  *          Both chips share the CONVST pulse so all 16 channels are
  *          converted at the same instant. Data is shifted out through the
  *          single DOUTA (DB7) line of each chip: 8 channels x 16 bits,
  *          MSB first, 128 SCLK cycles per chip. RANGE = 0 (+/-5 V),
  *          OS[2:0] = 000 (no oversampling, conversion time < 5 us).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ad7606.h"

/* Private define ------------------------------------------------------------*/
#define AD_CONV_PULSE_DLY   20u   /* CONVST low pulse, ~120 ns */
#define AD_SPI_DLY          8u    /* SCLK half period, ~50 ns  */

/* Private function prototypes -----------------------------------------------*/
static void ad7606_delay(uint32_t cycles);
static void ad7606_read_chip(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                             GPIO_TypeDef *sck_port, uint16_t sck_pin,
                             GPIO_TypeDef *dout_port, uint16_t dout_pin,
                             int16_t *out);

/**
  * @brief  Busy-wait of roughly cycles CPU cycles.
  */
static void ad7606_delay(uint32_t cycles)
{
  while (cycles-- > 0u)
  {
    __NOP();
  }
}

/**
  * @brief  Release RESET on both chips (RANGE/OS levels are already applied
  *         by MX_GPIO_Init before this runs).
  */
void AD7606_Init(void)
{
  HAL_GPIO_WritePin(AD1_RST_GPIO_Port, AD1_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AD2_RST_GPIO_Port, AD2_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(AD1_RST_GPIO_Port, AD1_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AD2_RST_GPIO_Port, AD2_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}

/**
  * @brief  Pulse CONVST A/B of both chips together (rising edge starts the
  *         conversion). Called from the TIM3 update interrupt.
  */
void AD7606_StartConversion(void)
{
  GPIOC->BSRR = (uint32_t)(AD1_CVA_Pin | AD1_CVB_Pin) << 16u;
  GPIOD->BSRR = (uint32_t)(AD2_CVA_Pin | AD2_CVB_Pin) << 16u;
  ad7606_delay(AD_CONV_PULSE_DLY);
  GPIOC->BSRR = (uint32_t)(AD1_CVA_Pin | AD1_CVB_Pin);
  GPIOD->BSRR = (uint32_t)(AD2_CVA_Pin | AD2_CVB_Pin);
}

/**
  * @brief  Shift out 8 channels x 16 bits from one chip, MSB first.
  *         Data changes on the SCLK rising edge and is sampled while SCLK
  *         is high. Must be called after BUSY has fallen.
  */
static void ad7606_read_chip(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                             GPIO_TypeDef *sck_port, uint16_t sck_pin,
                             GPIO_TypeDef *dout_port, uint16_t dout_pin,
                             int16_t *out)
{
  uint32_t sck_set = (uint32_t)sck_pin;
  uint32_t sck_rst = (uint32_t)sck_pin << 16u;

  cs_port->BSRR = (uint32_t)cs_pin << 16u;          /* CS low: frames the readout */
  ad7606_delay(AD_SPI_DLY);

  for (uint32_t ch = 0u; ch < AD7606_CH_PER_CHIP; ch++)
  {
    uint16_t value = 0u;
    for (uint32_t bit = 0u; bit < 16u; bit++)
    {
      sck_port->BSRR = sck_set;                     /* rising edge: next bit out */
      ad7606_delay(AD_SPI_DLY);
      value = (uint16_t)((value << 1) | ((dout_port->IDR & dout_pin) != 0u ? 1u : 0u));
      sck_port->BSRR = sck_rst;
      ad7606_delay(AD_SPI_DLY);
    }
    out[ch] = (int16_t)value;
  }

  cs_port->BSRR = (uint32_t)cs_pin;                 /* CS high */
}

/**
  * @brief  Read both chips. data[0..7]  = AD7606 #1 channels 1..8,
  *         data[8..15] = AD7606 #2 channels 1..8.
  *         Signed code, full scale +/-32767 maps to +/-5 V.
  */
void AD7606_ReadAll(int16_t *data)
{
  ad7606_read_chip(AD1_CS_GPIO_Port, AD1_CS_Pin,
                   AD1_SCK_GPIO_Port, AD1_SCK_Pin,
                   AD1_DOUT_GPIO_Port, AD1_DOUT_Pin,
                   &data[0]);
  ad7606_read_chip(AD2_CS_GPIO_Port, AD2_CS_Pin,
                   AD2_SCK_GPIO_Port, AD2_SCK_Pin,
                   AD2_DOUT_GPIO_Port, AD2_DOUT_Pin,
                   &data[AD7606_CH_PER_CHIP]);
}
