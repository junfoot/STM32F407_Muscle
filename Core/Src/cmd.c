/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   Line-based string command interface on USART1. See cmd.h for the
  *          command list. Parsing runs in the main loop; the RX callback only
  *          collects bytes.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmd.h"
#include "dac8563.h"
#include "imu_parser.h"
#include "recorder.h"
#include "serial.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define CMD_LINE_LEN   64u
#define CMD_IMU_COUNT  4u
#define CMD_DMA_SIZE   256u
#define CMD_RX_RING_SIZE 512u

/* Private variables ---------------------------------------------------------*/
static uint8_t cmd_dma_buf[CMD_DMA_SIZE];
static uint8_t cmd_rx_ring[CMD_RX_RING_SIZE];
static volatile uint16_t cmd_rx_head = 0u;
static volatile uint16_t cmd_rx_tail = 0u;
static volatile uint32_t cmd_rx_dropped = 0u;
static volatile uint32_t cmd_rx_errors = 0u;
static char cmd_line[CMD_LINE_LEN];
static uint8_t cmd_len = 0u;
static uint8_t cmd_discard_line = 0u;

/* Private function prototypes -----------------------------------------------*/
static int  cmd_str_equal(const char *a, const char *b);
static void cmd_handle(char *line);
static void cmd_rx_push(const uint8_t *data, uint16_t len);
static void cmd_receive_start(void);

static void cmd_receive_start(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, cmd_dma_buf, CMD_DMA_SIZE) == HAL_OK)
  {
    /* IDLE and transfer-complete events are sufficient in normal mode. */
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void Cmd_Init(void)
{
  cmd_rx_head = 0u;
  cmd_rx_tail = 0u;
  cmd_rx_dropped = 0u;
  cmd_rx_errors = 0u;
  cmd_len = 0u;
  cmd_discard_line = 0u;
  cmd_receive_start();
}

static void cmd_rx_push(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0u; i < len; i++)
  {
    uint16_t head = cmd_rx_head;
    uint16_t next = (uint16_t)((head + 1u) % CMD_RX_RING_SIZE);

    if (next == cmd_rx_tail)
    {
      cmd_rx_dropped++;
      continue;
    }

    cmd_rx_ring[head] = data[i];
    __DMB();
    cmd_rx_head = next;
  }
}

void Cmd_RxEvent(uint16_t dma_position)
{
  uint16_t length = dma_position;

  if (length > CMD_DMA_SIZE)
  {
    length = CMD_DMA_SIZE;
  }

  if (length > 0u)
  {
    cmd_rx_push(cmd_dma_buf, length);
  }
  cmd_receive_start();
}

void Cmd_RxError(void)
{
  cmd_rx_errors++;
  (void)HAL_UART_AbortReceive(&huart1);
  cmd_receive_start();
}

void Cmd_Process(void)
{
  while (cmd_rx_tail != cmd_rx_head)
  {
    uint8_t byte = cmd_rx_ring[cmd_rx_tail];
    cmd_rx_tail = (uint16_t)((cmd_rx_tail + 1u) % CMD_RX_RING_SIZE);

    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
      if (cmd_discard_line != 0u)
      {
        cmd_discard_line = 0u;
        cmd_len = 0u;
      }
      else if (cmd_len > 0u)
      {
        cmd_line[cmd_len] = '\0';
        cmd_len = 0u;
        cmd_handle(cmd_line);
      }
    }
    else if (cmd_discard_line == 0u)
    {
      if (cmd_len < (CMD_LINE_LEN - 1u))
      {
        cmd_line[cmd_len++] = (char)byte;
      }
      else
      {
        cmd_len = 0u;
        cmd_discard_line = 1u;
      }
    }
  }
}

/**
  * @brief  Case-insensitive string compare.
  */
static int cmd_str_equal(const char *a, const char *b)
{
  while ((*a != '\0') && (*b != '\0'))
  {
    char ca = *a;
    char cb = *b;
    if ((ca >= 'a') && (ca <= 'z')) { ca = (char)(ca - 'a' + 'A'); }
    if ((cb >= 'a') && (cb <= 'z')) { cb = (char)(cb - 'a' + 'A'); }
    if (ca != cb)
    {
      return 0;
    }
    a++;
    b++;
  }
  return ((*a == '\0') && (*b == '\0')) ? 1 : 0;
}

/**
  * @brief  Parse a channel token: A, B or AB (case-insensitive).
  * @retval DAC8563_CH_A / DAC8563_CH_B / DAC8563_CH_BOTH, or 0xFF on error.
  */
static uint8_t cmd_parse_channel(const char *tok)
{
  if (cmd_str_equal(tok, "A"))  { return DAC8563_CH_A; }
  if (cmd_str_equal(tok, "B"))  { return DAC8563_CH_B; }
  if (cmd_str_equal(tok, "AB")) { return DAC8563_CH_BOTH; }
  return 0xFFu;
}

static void cmd_handle(char *line)
{
  char verb[16];
  char arg1[16];
  char arg2[24];
  int  fields;

  verb[0] = '\0';
  arg1[0] = '\0';
  arg2[0] = '\0';
  fields = sscanf(line, "%15s %15s %23s", verb, arg1, arg2);

  if (fields <= 0)
  {
    return;
  }

  if (cmd_str_equal(verb, "DAC"))
  {
    uint8_t ch;
    float   volts;
    float   applied_volts;
    char   *end = NULL;

    if (fields < 3)
    {
      printf("ERR: usage DAC <A|B|AB> <volts>\r\n");
      return;
    }
    ch = cmd_parse_channel(arg1);
    volts = strtof(arg2, &end);
    if ((ch == 0xFFu) || (end == arg2) || (*end != '\0'))
    {
      printf("ERR: usage DAC <A|B|AB> <volts>, volts in [0, 10]\r\n");
      return;
    }
    applied_volts = volts;
    if (applied_volts > DAC8563_VMAX) { applied_volts = DAC8563_VMAX; }
    if (applied_volts < DAC8563_VMIN) { applied_volts = DAC8563_VMIN; }
    DAC8563_SetVoltage(ch, applied_volts);
    printf("OK: DAC %s -> %.3f V\r\n", arg1, (double)applied_volts);
    return;
  }

  if (cmd_str_equal(verb, "DACR"))
  {
    uint8_t ch;
    long    code;
    char   *end = NULL;

    if (fields < 3)
    {
      printf("ERR: usage DACR <A|B|AB> <0..65535>\r\n");
      return;
    }
    ch = cmd_parse_channel(arg1);
    code = strtol(arg2, &end, 0);
    if ((ch == 0xFFu) || (end == arg2) || (*end != '\0') || (code < 0L) || (code > 65535L))
    {
      printf("ERR: usage DACR <A|B|AB> <0..65535>\r\n");
      return;
    }
    DAC8563_SetOutput(ch, (uint16_t)code);
    printf("OK: DAC %s -> code %ld\r\n", arg1, code);
    return;
  }

  if (cmd_str_equal(verb, "IMU"))
  {
    for (uint8_t id = 0u; id < CMD_IMU_COUNT; id++)
    {
      IMU_Data_t *d = IMU_GetSlaveData(id);
      printf("IMU%u %s: Ang=%.2f %.2f %.2f deg  Acc=%.3f %.3f %.3f g\r\n",
             (unsigned int)id,
             (IMU_IsSlaveOnline(id) != 0u) ? "online " : "offline",
             (double)d->angle_deg[0], (double)d->angle_deg[1], (double)d->angle_deg[2],
             (double)d->accel_g[0], (double)d->accel_g[1], (double)d->accel_g[2]);
    }
    return;
  }

  if (cmd_str_equal(verb, "REC"))
  {
    if (fields < 2)
    {
      printf("REC: %s, card=%s, requested=%s, button=%s, led=%s, %lu bytes written, %lu records dropped\r\n",
             (Recorder_IsActive() != 0u) ? "logging" : "idle",
             (Recorder_IsCardPresent() != 0u) ? "present" : "absent",
             (Recorder_IsRequested() != 0u) ? "on" : "off",
             (HAL_GPIO_ReadPin(REC_SW_GPIO_Port, REC_SW_Pin) == GPIO_PIN_RESET) ? "pressed" : "released",
             (HAL_GPIO_ReadPin(REC_LED_GPIO_Port, REC_LED_Pin) == GPIO_PIN_RESET) ? "on" : "off",
             (unsigned long)Recorder_GetBytesWritten(),
             (unsigned long)Recorder_GetDropped());
    }
    else if (cmd_str_equal(arg1, "START"))
    {
      Recorder_Start();
    }
    else if (cmd_str_equal(arg1, "STOP"))
    {
      Recorder_Stop();
    }
    else
    {
      printf("ERR: usage REC [START|STOP]\r\n");
    }
    return;
  }

  if (cmd_str_equal(verb, "HELP") || cmd_str_equal(verb, "?"))
  {
    printf("Commands (end with CR/LF):\r\n");
    printf("  DAC <A|B|AB> <volts>   set DAC output voltage, 0..10 V\r\n");
    printf("  DACR <A|B|AB> <code>   set DAC raw code, 0..65535\r\n");
    printf("  IMU                    show IMU0..3 online state and data\r\n");
    printf("  REC [START|STOP]       SD recorder status / control\r\n");
    printf("  HELP                   this list\r\n");
    printf("  UART diagnostics: RX errors=%lu, RX dropped=%lu, TX dropped=%lu bytes\r\n",
           (unsigned long)cmd_rx_errors,
           (unsigned long)cmd_rx_dropped,
           (unsigned long)Serial_GetDropCount());
    return;
  }

  printf("ERR: unknown command '%s', try HELP\r\n", verb);
}
