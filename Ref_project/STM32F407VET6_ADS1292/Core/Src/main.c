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
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "ads1292.h"
#include "protocol.h"
#include "sd_recorder.h"
#include "imu_parser.h"
#include "imu_receiver.h"
#include "usb_host.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static volatile bool acquisition_active;
static volatile uint32_t sample_sequence;
static bool ads1292_ok;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void acquisition_update(void);
static void button_service(void);
static void led_service(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void acquisition_update(void)
{
  bool required = ads1292_ok
               && (protocol_is_streaming() || sd_recorder_is_accepting_isr());
  if (required && !acquisition_active)
  {
    sample_sequence = 0U;
    ads1292_start();
    acquisition_active = true;
  }
  else if (!required && acquisition_active)
  {
    acquisition_active = false;
    ads1292_stop();
  }
}

void app_set_host_streaming(bool enabled)
{
  (void)enabled;
  acquisition_update();
}

bool app_apply_sample_parameters(uint16_t rate, uint8_t pga,
                                 uint8_t channel_mask, uint8_t input_mode,
                                 uint8_t rld_enabled)
{
  bool rate_ok = rate == 125U || rate == 250U || rate == 500U
              || rate == 1000U || rate == 2000U || rate == 4000U
              || rate == 8000U;
  bool pga_ok = pga == 1U || pga == 2U || pga == 3U || pga == 4U
             || pga == 6U || pga == 8U || pga == 12U;
  if (!rate_ok || !pga_ok || input_mode > 2U || (channel_mask & 0x03U) == 0U)
  {
    return false;
  }
  /* SD文件头中的采样参数必须在整个文件期间保持不变。 */
  if (sd_recorder_state() == SD_RECORDER_ACTIVE
      || sd_recorder_state() == SD_RECORDER_STOPPING)
  {
    return false;
  }

  HAL_NVIC_DisableIRQ(ADS129X_DRDY_EXTI_IRQn);
  if (acquisition_active) ads1292_stop();
  ads1292_info.rate = rate;
  ads1292_info.pga = pga;
  ads1292_info.ch_en = channel_mask & 0x03U;
  ads1292_info.ch_sw = input_mode;
  ads1292_info.rld_en = rld_enabled != 0U;
  ads1292_update_sample_parameter();
  if (acquisition_active) ads1292_start();
  HAL_NVIC_EnableIRQ(ADS129X_DRDY_EXTI_IRQn);
  sd_recorder_init(rate, pga, channel_mask, rld_enabled);
  return true;
}

static void button_service(void)
{
  static GPIO_PinState previous_raw = GPIO_PIN_RESET;
  static GPIO_PinState stable_state = GPIO_PIN_RESET;
  static uint32_t changed_at;
  GPIO_PinState raw = HAL_GPIO_ReadPin(RECORD_BUTTON_GPIO_Port, RECORD_BUTTON_Pin);
  uint32_t now = HAL_GetTick();
  if (raw != previous_raw)
  {
    previous_raw = raw;
    changed_at = now;
  }
  if (raw != stable_state && (now - changed_at) >= 30U)
  {
    stable_state = raw;
    if (stable_state == GPIO_PIN_SET)
    {
      if (sd_recorder_state() == SD_RECORDER_ACTIVE)
      {
        sd_recorder_stop();
      }
      else if (sd_recorder_state() == SD_RECORDER_IDLE
               || sd_recorder_state() == SD_RECORDER_ERROR)
      {
        sd_recorder_init(ads1292_info.rate, ads1292_info.pga,
                         ads1292_info.ch_en, ads1292_info.rld_en);
        (void)sd_recorder_start();
      }
      acquisition_update();
    }
  }
}

static void led_service(void)
{
  GPIO_PinState output = GPIO_PIN_SET; /* LED低电平点亮 */
  sd_recorder_state_t state = sd_recorder_state();
  if (state == SD_RECORDER_ACTIVE || state == SD_RECORDER_STOPPING)
  {
    output = GPIO_PIN_RESET;
  }
  else if (state == SD_RECORDER_ERROR || !ads1292_ok)
  {
    output = ((HAL_GetTick() / 200U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(RECORD_LED_GPIO_Port, RECORD_LED_Pin, output);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  /* USER CODE BEGIN 2 */

  ads1292_ok = ads1292_init() != 0U;
  sd_recorder_init(ads1292_info.rate, ads1292_info.pga,
                   ads1292_info.ch_en, ads1292_info.rld_en);
  protocol_init();
  imu_receiver_init();
  MX_USB_HOST_Init();
  acquisition_active = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    button_service();
    sd_recorder_service(); /* SD优先，避免记录队列积压。 */
    protocol_service();
    imu_receiver_service();
    acquisition_update();
    led_service();
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADS129X_DRDY_Pin && acquisition_active)
  {
    emg_sample_t sample;
    ads1292_read_data(sample.channel);
    sample.sequence = sample_sequence++;
    sample.tick_ms = HAL_GetTick();
    (void)sd_recorder_push_isr(&sample);
    (void)protocol_push_isr(&sample);
  }
}

void imu_frame_received_callback(const imu_sample_t *sample)
{
  (void)sd_recorder_push_imu(sample);
  (void)protocol_push_imu(sample);
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
