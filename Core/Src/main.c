/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *  - TIM3 @ 2000 Hz starts a synchronous conversion on both AD7606 chips;
  *    the BUSY falling edge (EXTI0) reads all 16 channels right in the ISR
  *    and pushes the raw frame into the SD recorder ring, so samples are
  *    captured completely even while a FatFs write blocks the main loop.
  *  - 200 Hz of the stream goes to USART1 as a VOFA+ JustFloat frame:
  *    40 little-endian float32 (16 x ADC volts + 4 IMU x [roll,pitch,yaw,
  *    ax,ay,az]) followed by the tail 00 00 80 7F. IMU values are held
  *    between IMU updates (the wireless IMUs upload much slower).
  *  - Every ADC sample and every raw IMU frame is also logged to the SD
  *    card (recorder.c, LOGxxxx.BIN) with a common 2000 Hz sequence number.
  *  - IMU data comes from the WT9011DCL-RF receiver attached to USB OTG FS
  *    (USB Host CDC, CH340 @ 460800), parsed by imu_parser.
  *  - USART1 RX takes line-based string commands (see cmd.h), e.g.
  *    "DAC A 3.3" sets DAC channel A to +3.3 V, "REC STOP" stops logging.
  *  - On-board button K3 (PA15, to GND) toggles SD recording start/stop;
  *    the PA1 LED (active low) is on while recording. See 原理图V2.8.
  *  - printf() is retargeted to USART1 through a non-blocking DMA ring
  *    buffer (serial.c), so logging never stalls the 2000 Hz loop.
  *
  *  Interrupt priorities: TIM3 (conversion start) = 0, EXTI0 (BUSY) = 1,
  *  USART1 = 2, DMA2 Stream7 (USART1 TX) = 3, OTG_FS (USB host) = 5.
  *  SDIO runs in polling mode (no IRQ).
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
#include "sdio.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad7606.h"
#include "dac8563.h"
#include "serial.h"
#include "cmd.h"
#include "imu_parser.h"
#include "recorder.h"
#include "usbh_cdc.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  CDC_STATE_IDLE = 0,
  CDC_STATE_SET_LINE_CODING,
  CDC_STATE_SET_CONTROL_LINE,
  CDC_STATE_START_RECEPTION,
  CDC_STATE_RUNNING
} CDC_AppStateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_COUNT                    4u      /* slaves with device_id 0..3        */
#define IMU_FLOATS_PER_UNIT          6u      /* roll,pitch,yaw + ax,ay,az         */
#define TX_CH_COUNT                  (AD7606_TOTAL_CH + IMU_COUNT * IMU_FLOATS_PER_UNIT)
#define TX_FRAME_LEN                 (TX_CH_COUNT * 4u + 4u)   /* floats + tail  */
#define TX_DECIMATION                10u     /* 2000 Hz / 10 = 200 Hz on UART  */
#define ADC_LSB_VOLTS                (5.0f / 32768.0f)         /* +/-5 V range   */

#define CDC_RX_BUF_SIZE              512U
#define CDC_RX_TIMEOUT_MS            2000U
#define CDC_REQ_SET_CONTROL_LINE_STATE 0x22U
#define CDC_CTRL_LINE_DTR_RTS        0x0003U
#define CDC_LINE_CODING_TIMEOUT_MS   2000U
#define CDC_BAUDRATE                 460800U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;

volatile uint32_t g_sample_seq = 0u;        /* 2000 Hz tick, shared with recorder */
static volatile uint8_t g_new_sample = 0u;  /* set by EXTI ISR after ADC read   */
static int16_t  g_adc_values[AD7606_TOTAL_CH];
static float    g_tx_fdata[TX_CH_COUNT];
static uint8_t  g_tx_frame[TX_FRAME_LEN];
static const uint8_t g_tx_tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};  /* JustFloat tail */
static uint32_t g_next_uart_seq = 0u;

static uint8_t  g_rx_byte;                        /* UART RX, one byte per IT  */

static uint8_t cdc_rx_buf[CDC_RX_BUF_SIZE];
static CDC_LineCodingTypeDef cdc_linecoding;
static volatile CDC_AppStateTypeDef cdc_state = CDC_STATE_IDLE;
static volatile uint8_t  cdc_line_coding_done = 0;
static volatile uint8_t  cdc_rx_ready = 0;
static volatile uint32_t cdc_last_rx_tick = 0;
static uint32_t cdc_line_coding_start_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CDC_Process(void);
static USBH_StatusTypeDef CDC_SetControlLineState(USBH_HandleTypeDef *phost, uint16_t state);
static void build_sample_frame(void);
static void rec_switch_poll(void);
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
  MX_SDIO_SD_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  AD7606_Init();
  DAC8563_Init();
  Recorder_Init();

  printf("\r\nSTM32F407_Muscle ready. USART1 @ 921600 8N1, JustFloat %u ch @ 200 Hz\r\n",
         (unsigned int)TX_CH_COUNT);
  printf("Type HELP for commands.\r\n");

  HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1u);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MX_USB_HOST_Process();
    CDC_Process();
    Recorder_Process();

    if ((g_new_sample != 0u) && ((int32_t)(g_sample_seq - g_next_uart_seq) >= 0))
    {
      g_new_sample = 0u;
      g_next_uart_seq = g_sample_seq + TX_DECIMATION;

      build_sample_frame();
      (void)Serial_Write(g_tx_frame, TX_FRAME_LEN);
    }

    Cmd_Process();

    rec_switch_poll();
    HAL_GPIO_WritePin(REC_LED_GPIO_Port, REC_LED_Pin,
                      (Recorder_IsActive() != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Fill g_tx_frame with one JustFloat frame: 16 ADC channels in
  *         volts, then for each IMU (id 0..3) roll/pitch/yaw in degrees and
  *         ax/ay/az in g. IMU fields hold their last value between updates.
  */
static void build_sample_frame(void)
{
  uint32_t idx = 0u;

  /* latest ADC sample is written by the EXTI ISR; copy it atomically */
  __disable_irq();
  for (uint32_t i = 0u; i < AD7606_TOTAL_CH; i++)
  {
    g_tx_fdata[idx++] = (float)g_adc_values[i] * ADC_LSB_VOLTS;
  }
  __enable_irq();

  for (uint32_t id = 0u; id < IMU_COUNT; id++)
  {
    IMU_Data_t *d = IMU_GetSlaveData((uint8_t)id);

    /* guard against the parser updating the record from the OTG_FS ISR */
    __disable_irq();
    g_tx_fdata[idx++] = d->angle_deg[0];
    g_tx_fdata[idx++] = d->angle_deg[1];
    g_tx_fdata[idx++] = d->angle_deg[2];
    g_tx_fdata[idx++] = d->accel_g[0];
    g_tx_fdata[idx++] = d->accel_g[1];
    g_tx_fdata[idx++] = d->accel_g[2];
    __enable_irq();
  }

  memcpy(g_tx_frame, g_tx_fdata, TX_CH_COUNT * 4u);
  memcpy(&g_tx_frame[TX_CH_COUNT * 4u], g_tx_tail, 4u);
}

/**
  * @brief  Advance the USB CDC state machine (CH340 of the 9011RF receiver).
  */
static void CDC_Process(void)
{
  if (Appli_state == APPLICATION_READY)
  {
    switch (cdc_state)
    {
      case CDC_STATE_IDLE:
        cdc_linecoding.b.dwDTERate = CDC_BAUDRATE;
        cdc_linecoding.b.bCharFormat = 0U;
        cdc_linecoding.b.bParityType = 0U;
        cdc_linecoding.b.bDataBits = 8U;
        cdc_line_coding_done = 0U;
        cdc_line_coding_start_tick = HAL_GetTick();
        USBH_CDC_SetLineCoding(&hUsbHostFS, &cdc_linecoding);
        cdc_state = CDC_STATE_SET_LINE_CODING;
        break;

      case CDC_STATE_SET_LINE_CODING:
        if (cdc_line_coding_done != 0U)
        {
          cdc_state = CDC_STATE_SET_CONTROL_LINE;
        }
        else if ((HAL_GetTick() - cdc_line_coding_start_tick) > CDC_LINE_CODING_TIMEOUT_MS)
        {
          printf("[WARN] CDC SetLineCoding timeout, proceeding\r\n");
          cdc_state = CDC_STATE_SET_CONTROL_LINE;
        }
        break;

      case CDC_STATE_SET_CONTROL_LINE:
      {
        USBH_StatusTypeDef st = CDC_SetControlLineState(&hUsbHostFS, CDC_CTRL_LINE_DTR_RTS);
        if (st != USBH_BUSY)
        {
          cdc_state = CDC_STATE_START_RECEPTION;
        }
        break;
      }

      case CDC_STATE_START_RECEPTION:
        if (USBH_CDC_Receive(&hUsbHostFS, cdc_rx_buf, CDC_RX_BUF_SIZE) == USBH_OK)
        {
          cdc_last_rx_tick = HAL_GetTick();
          cdc_state = CDC_STATE_RUNNING;
        }
        break;

      case CDC_STATE_RUNNING:
        if ((HAL_GetTick() - cdc_last_rx_tick) > CDC_RX_TIMEOUT_MS)
        {
          printf("[WARN] CDC RX timeout, re-arming reception\r\n");
          cdc_state = CDC_STATE_START_RECEPTION;
        }
        break;

      default:
        cdc_state = CDC_STATE_IDLE;
        break;
    }
  }
  else
  {
    cdc_state = CDC_STATE_IDLE;
    cdc_rx_ready = 0U;
    cdc_line_coding_done = 0U;
  }
}

/**
  * @brief  SET_CONTROL_LINE_STATE class request (DTR | RTS), needed by the
  *         CH340 before it forwards data.
  */
static USBH_StatusTypeDef CDC_SetControlLineState(USBH_HandleTypeDef *phost, uint16_t state)
{
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
  phost->Control.setup.b.bRequest = CDC_REQ_SET_CONTROL_LINE_STATE;
  phost->Control.setup.b.wValue.w = state;
  phost->Control.setup.b.wIndex.w = 0x0000U;
  phost->Control.setup.b.wLength.w = 0U;

  return USBH_CtlReq(phost, NULL, 0U);
}

void USBH_CDC_LineCodingChanged(USBH_HandleTypeDef *phost)
{
  (void)phost;
  cdc_line_coding_done = 1U;
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  uint16_t len = USBH_CDC_GetLastReceivedDataSize(phost);

  if (len > 0U)
  {
    cdc_last_rx_tick = HAL_GetTick();
    cdc_rx_ready = 1U;
    IMU_ParseStream(cdc_rx_buf, len);
  }

  if (USBH_CDC_Receive(phost, cdc_rx_buf, CDC_RX_BUF_SIZE) != USBH_OK)
  {
    cdc_state = CDC_STATE_START_RECEPTION;
    cdc_rx_ready = 0U;
  }
}

/**
  * @brief  Poll the on-board REC button K3 (PA15, active low, 100 nF to GND
  *         on the board). Sampled every 20 ms, 3 identical samples required
  *         (60 ms debounce); a stable press toggles SD recording.
  */
static void rec_switch_poll(void)
{
  static uint32_t last_tick = 0u;
  static uint8_t  raw_history = 0xFFu;   /* bit0 = latest sample, 1 = released */
  static uint8_t  stable_state = 1u;     /* debounced state, 1 = released    */

  if ((HAL_GetTick() - last_tick) < 20u)
  {
    return;
  }
  last_tick = HAL_GetTick();

  raw_history = (uint8_t)((raw_history << 1) |
      ((HAL_GPIO_ReadPin(REC_SW_GPIO_Port, REC_SW_Pin) != GPIO_PIN_RESET) ? 1u : 0u));

  if ((raw_history & 0x07u) == 0x00u)
  {
    if (stable_state != 0u)
    {
      stable_state = 0u;   /* debounced press edge: toggle recording */
      if (Recorder_IsActive() != 0u)
      {
        Recorder_Stop();
      }
      else
      {
        Recorder_Start();
      }
    }
  }
  else if ((raw_history & 0x07u) == 0x07u)
  {
    stable_state = 1u;
  }
}

/**
  * @brief  TIM3 update (2000 Hz): bump the global sample sequence number and
  *         start a conversion on both AD7606 chips.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    g_sample_seq++;
    AD7606_StartConversion();
  }
}

/**
  * @brief  AD7606 BUSY falling edge: conversion finished. Read all 16
  *         channels right here (~120 us) and push the raw sample into the
  *         SD recorder ring, so no sample is lost while the main loop is
  *         busy with a FatFs/SDIO write. Both chips convert on the same
  *         CONVST pulse, so the chip #1 BUSY edge covers both.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == AD1_BUSY_Pin)
  {
    AD7606_ReadAll((int16_t *)g_adc_values);
    Recorder_PushAdc(g_sample_seq, g_adc_values);
    g_new_sample = 1u;
  }
}

/**
  * @brief  Raw IMU frame hook (overrides the weak one in imu_parser.c):
  *         log the untouched 26-byte payload with the current ADC tick.
  */
void IMU_RawFrameHook(uint8_t device_id, const uint8_t *payload, uint16_t len)
{
  if (len == 26u)
  {
    Recorder_PushImu(g_sample_seq, device_id, payload);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Cmd_RxByte(g_rx_byte);
    HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1u);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Serial_TxCpltHandler();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
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
  * @param  file: pointer to the file name
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
