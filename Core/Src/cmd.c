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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define CMD_LINE_LEN   64u
#define CMD_IMU_COUNT  4u

/* Private variables ---------------------------------------------------------*/
static char    cmd_line[CMD_LINE_LEN];
static uint8_t cmd_len = 0u;
static volatile uint8_t cmd_ready = 0u;

/* Private function prototypes -----------------------------------------------*/
static int  cmd_str_equal(const char *a, const char *b);
static void cmd_handle(char *line);

void Cmd_RxByte(uint8_t byte)
{
  if (cmd_ready != 0u)
  {
    return;   /* previous line not consumed yet: drop input */
  }

  if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
  {
    if (cmd_len > 0u)
    {
      cmd_line[cmd_len] = '\0';
      cmd_ready = 1u;
    }
    return;
  }

  if (cmd_len < (CMD_LINE_LEN - 1u))
  {
    cmd_line[cmd_len++] = (char)byte;
  }
  else
  {
    cmd_len = 0u;   /* overlong line: discard */
  }
}

void Cmd_Process(void)
{
  char line[CMD_LINE_LEN];

  if (cmd_ready == 0u)
  {
    return;
  }

  __disable_irq();
  memcpy(line, cmd_line, (size_t)cmd_len + 1u);
  cmd_len = 0u;
  cmd_ready = 0u;
  __enable_irq();

  cmd_handle(line);
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
      printf("ERR: usage DAC <A|B|AB> <volts>, volts in [-10, 10]\r\n");
      return;
    }
    DAC8563_SetVoltage(ch, volts);
    printf("OK: DAC %s -> %.3f V\r\n", arg1, (double)volts);
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
      printf("REC: %s, requested=%s, button=%s, led=%s, %lu bytes written, %lu records dropped\r\n",
             (Recorder_IsActive() != 0u) ? "logging" : "idle",
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
    printf("  DAC <A|B|AB> <volts>   set DAC output voltage, -10..+10 V\r\n");
    printf("  DACR <A|B|AB> <code>   set DAC raw code, 0..65535\r\n");
    printf("  IMU                    show IMU0..3 online state and data\r\n");
    printf("  REC [START|STOP]       SD recorder status / control\r\n");
    printf("  HELP                   this list\r\n");
    printf("  TX drops: %lu bytes\r\n", (unsigned long)Serial_GetDropCount());
    return;
  }

  printf("ERR: unknown command '%s', try HELP\r\n", verb);
}
