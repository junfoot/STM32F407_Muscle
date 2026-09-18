/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad7606.h"
#include "dac8562.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TX_FRAME_LEN   38u   /* 2 header + 32 data + 2 seq + 2 CRC16 */
#define DAC_CMD_LEN        7u    /* 55 AA 01 ch vH vL sum */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t g_conv_done = 0;          /* set by BUSY falling edge EXTI */
static int16_t  g_adc_values[AD7606_TOTAL_CH];
static uint8_t  g_tx_frame[ADC_TX_FRAME_LEN];
static uint16_t g_tx_seq = 0;
static uint8_t  g_rx_byte;                        /* UART RX, one byte per IT  */
static uint8_t  g_rx_buf[DAC_CMD_LEN];
static uint8_t  g_rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
static void protocol_rx_byte(uint8_t byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  AD7606_Init();
  DAC8562_Init();

  HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1u);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (g_conv_done != 0u)
    {
      g_conv_done = 0u;

      /* ~120 us blocking read of both chips; everything else stays
         interrupt/DMA driven so nothing else stalls */
      AD7606_ReadAll(g_adc_values);

      if (huart1.gState == HAL_UART_STATE_READY)
      {
        uint16_t crc;

        g_tx_frame[0] = 0xAAu;
        g_tx_frame[1] = 0x55u;
        for (uint32_t i = 0u; i < AD7606_TOTAL_CH; i++)
        {
          g_tx_frame[2u + 2u * i] = (uint8_t)((uint16_t)g_adc_values[i] & 0xFFu);
          g_tx_frame[3u + 2u * i] = (uint8_t)((uint16_t)g_adc_values[i] >> 8);
        }
        g_tx_frame[34] = (uint8_t)(g_tx_seq & 0xFFu);
        g_tx_frame[35] = (uint8_t)(g_tx_seq >> 8);
        g_tx_seq++;

        crc = crc16_ccitt(g_tx_frame, 36u);
        g_tx_frame[36] = (uint8_t)(crc & 0xFFu);
        g_tx_frame[37] = (uint8_t)(crc >> 8);

        HAL_UART_Transmit_DMA(&huart1, g_tx_frame, ADC_TX_FRAME_LEN);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  CRC16-CCITT (poly 0x1021, init 0xFFFF), little-endian on the wire.
  */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFu;

  for (uint32_t i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint32_t b = 0u; b < 8u; b++)
    {
      crc = (uint16_t)((crc & 0x8000u) != 0u ? (crc << 1) ^ 0x1021u : crc << 1);
    }
  }
  return crc;
}

/**
  * @brief  Host -> MCU frame: 55 AA 01 <ch> <valueH> <valueL> <sum of the
  *         previous 6 bytes>. ch: 0 = DAC-A, 1 = DAC-B, 2 = both.
  */
static void protocol_rx_byte(uint8_t byte)
{
  switch (g_rx_idx)
  {
    case 0u:
      if (byte == 0x55u)
      {
        g_rx_buf[0] = byte;
        g_rx_idx = 1u;
      }
      break;

    case 1u:
      if (byte == 0xAAu)
      {
        g_rx_buf[1] = byte;
        g_rx_idx = 2u;
      }
      else
      {
        g_rx_idx = (byte == 0x55u) ? 1u : 0u;
      }
      break;

    case 2u:
    case 3u:
    case 4u:
    case 5u:
      g_rx_buf[g_rx_idx++] = byte;
      break;

    case 6u:
    {
      uint8_t sum = 0u;
      for (uint32_t i = 0u; i < 6u; i++)
      {
        sum = (uint8_t)(sum + g_rx_buf[i]);
      }
      if ((sum == byte) && (g_rx_buf[2] == 0x01u) && (g_rx_buf[3] <= DAC8562_CH_BOTH))
      {
        DAC8562_SetOutput(g_rx_buf[3], (uint16_t)((uint16_t)g_rx_buf[4] << 8 | g_rx_buf[5]));
      }
      g_rx_idx = 0u;
      break;
    }

    default:
      g_rx_idx = 0u;
      break;
  }
}

/**
  * @brief  TIM3 update (2000 Hz): start a conversion on both AD7606 chips.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    AD7606_StartConversion();
  }
}

/**
  * @brief  AD7606 BUSY falling edge: conversion finished, data ready.
  *         Both chips convert on the same CONVST pulse, so the chip #1 BUSY
  *         edge covers both (EXTI7/NVIC for chip #2 is intentionally unused).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == AD1_BUSY_Pin)
  {
    g_conv_done = 1u;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    protocol_rx_byte(g_rx_byte);
    HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1u);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    g_rx_idx = 0u;
    HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1u);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
