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
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_cdc.h"
#include "imu_parser.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
  * @brief  CDC host application states.
  */
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
#define CDC_RX_BUF_SIZE                512U
#define CDC_RX_TIMEOUT_MS              2000U
#define IMU_PRINT_PERIOD_MS            10U
#define IMU_MONITOR_IDS                2U
#define CDC_REQ_SET_CONTROL_LINE_STATE 0x22U
#define CDC_CTRL_LINE_DTR_RTS          0x0003U
#define CDC_LINE_CODING_TIMEOUT_MS     2000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;

static uint8_t cdc_rx_buf[CDC_RX_BUF_SIZE];
static CDC_LineCodingTypeDef cdc_linecoding;
static volatile CDC_AppStateTypeDef cdc_state = CDC_STATE_IDLE;
static volatile uint8_t cdc_line_coding_done = 0;
static volatile uint8_t cdc_rx_ready = 0;
static volatile uint32_t cdc_last_rx_tick = 0;
static uint32_t cdc_line_coding_start_tick = 0;
static volatile uint32_t imu_print_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Send SET_CONTROL_LINE_STATE to the CDC-ACM device and assert DTR/RTS.
  *         The 9011RF receiver enumerates as CDC-ACM; some CDC-ACM-to-UART
  *         devices start output only after DTR is asserted, so set it here.
  */
static USBH_StatusTypeDef CDC_SetControlLineState(USBH_HandleTypeDef *phost, uint16_t state)
{
  phost->Control.setup.b.bmRequestType = USB_H2D |
                                         USB_REQ_TYPE_CLASS |
                                         USB_REQ_RECIPIENT_INTERFACE;
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

    if (!cdc_rx_ready)
    {
      cdc_rx_ready = 1U;
      imu_print_tick = HAL_GetTick();
    }

    /* Parse the received byte stream and store data per device_id. */
    IMU_ParseStream(cdc_rx_buf, len);
  }

  /* Re-arm the next bulk IN transfer. If it fails, let the state machine retry. */
  if (USBH_CDC_Receive(phost, cdc_rx_buf, CDC_RX_BUF_SIZE) != USBH_OK)
  {
    cdc_state = CDC_STATE_START_RECEPTION;
    cdc_rx_ready = 0U;
  }
}
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
  MX_USART1_UART_Init();

  /* Send a raw UART line first to verify the serial hardware/wiring/baudrate */
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n[SYS] UART1 OK\r\n", 18, 100);

  /* USER CODE BEGIN 2 */
  printf("\r\n[SYS] STM32F407 9011RF bridge started\r\n");
  printf("[SYS] USART1 printf retarget OK\r\n");

  MX_USB_HOST_Init();
  printf("[SYS] waiting for CDC USB device...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (Appli_state == APPLICATION_READY)
    {
      switch (cdc_state)
      {
        case CDC_STATE_IDLE:
          /* Configure 460800 8N1 and start the LineCoding request. */
          cdc_linecoding.b.dwDTERate = 460800U;
          cdc_linecoding.b.bCharFormat = 0U;
          cdc_linecoding.b.bParityType = 0U;
          cdc_linecoding.b.bDataBits = 8U;
          cdc_line_coding_done = 0U;
          cdc_line_coding_start_tick = HAL_GetTick();
          USBH_CDC_SetLineCoding(&hUsbHostFS, &cdc_linecoding);
          cdc_state = CDC_STATE_SET_LINE_CODING;
          break;

        case CDC_STATE_SET_LINE_CODING:
          if (cdc_line_coding_done)
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
          if (st == USBH_OK)
          {
            cdc_state = CDC_STATE_START_RECEPTION;
          }
          else if (st != USBH_BUSY)
          {
            /* The device may STALL or not support this request; continue anyway. */
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
          /* If no data arrives for too long, retry reception. */
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

    /* Output online IMU data at 100Hz while receiving. */
    if (cdc_rx_ready && ((HAL_GetTick() - imu_print_tick) >= IMU_PRINT_PERIOD_MS))
    {
      imu_print_tick = HAL_GetTick();
      for (uint8_t id = 0U; id < IMU_MONITOR_IDS; id++)
      {
        if (IMU_IsSlaveOnline(id))
        {
          IMU_Data_t *d = IMU_GetSlaveData(id);
          printf("[IMU%02u] A=%.3f %.3f %.3f g  G=%.2f %.2f %.2f dps  Ang=%.2f %.2f %.2f deg  Bat=%.2fV\r\n",
                 (unsigned)d->device_id,
                 d->accel_g[0], d->accel_g[1], d->accel_g[2],
                 d->gyro_dps[0], d->gyro_dps[1], d->gyro_dps[2],
                 d->angle_deg[0], d->angle_deg[1], d->angle_deg[2],
                 d->battery_voltage);
          IMU_ClearSlaveNewData(id);
        }
      }
    }
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
