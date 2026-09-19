/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    recorder.c
  * @brief   SD card recorder for EMG (AD7606) and IMU raw data.
  *          See recorder.h for the record format.
  *
  *          Recording starts automatically at power-up when a card is
  *          present; REC START / REC STOP / REC commands control it from the
  *          serial console. A 48 KB ring decouples the 2000 Hz producers
  *          from blocking FatFs/SDIO writes. f_sync() runs every 2 s to
  *          bound data loss on sudden power-down.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "recorder.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define REC_RING_SIZE        (48u * 1024u)
#define REC_WRITE_CHUNK      8192u
#define REC_SYNC_PERIOD_MS   2000u
#define REC_RETRY_PERIOD_MS  5000u

#define REC_TYPE_ADC         0xA1u
#define REC_TYPE_IMU         0xB1u

#define REC_ADC_RECORD_LEN   37u   /* type + u32 seq + 16 x int16 */
#define REC_IMU_RECORD_LEN   32u   /* type + u32 seq + u8 id + 26 raw */

/* Private variables ---------------------------------------------------------*/
static FATFS rec_fs;
static FIL   rec_file;

static uint8_t           ring[REC_RING_SIZE];
static volatile uint32_t ring_head = 0u;   /* consumer index (main loop) */
static volatile uint32_t ring_tail = 0u;   /* producer index (ISR)       */

static uint8_t  rec_active = 0u;
static uint32_t rec_dropped = 0u;
static uint32_t rec_bytes_written = 0u;
static uint32_t rec_sync_tick = 0u;
static uint32_t rec_retry_tick = 0u;
static char     rec_filename[16];
static uint8_t  rec_buf[REC_WRITE_CHUNK];   /* static: 8 KB must not sit on the stack */

/* Private function prototypes -----------------------------------------------*/
static void    ring_push(const uint8_t *data, uint32_t len);
static uint8_t rec_open_next_file(void);
static void    rec_close_file(void);
static uint8_t rec_drain(uint8_t flush_all);

/**
  * @brief  Append bytes to the ring; on overflow the record is dropped and
  *         counted. Safe to call from any ISR: the copy runs with interrupts
  *         masked (a few us).
  */
static void ring_push(const uint8_t *data, uint32_t len)
{
  uint32_t head;
  uint32_t tail;
  uint32_t free_bytes;
  uint32_t first;

  __disable_irq();

  head = ring_head;
  tail = ring_tail;
  free_bytes = (tail >= head) ? (REC_RING_SIZE - (tail - head) - 1u)
                              : (head - tail - 1u);
  if (free_bytes < len)
  {
    rec_dropped++;
    __enable_irq();
    return;
  }

  first = REC_RING_SIZE - tail;
  if (first > len)
  {
    first = len;
  }
  memcpy(&ring[tail], data, first);
  memcpy(ring, &data[first], len - first);
  ring_tail = (tail + len) % REC_RING_SIZE;

  __enable_irq();
}

void Recorder_PushAdc(uint32_t seq, const int16_t *ch16)
{
  uint8_t rec[REC_ADC_RECORD_LEN];

  rec[0] = REC_TYPE_ADC;
  rec[1] = (uint8_t)(seq & 0xFFu);
  rec[2] = (uint8_t)(seq >> 8);
  rec[3] = (uint8_t)(seq >> 16);
  rec[4] = (uint8_t)(seq >> 24);
  memcpy(&rec[5], ch16, 32u);

  ring_push(rec, REC_ADC_RECORD_LEN);
}

void Recorder_PushImu(uint32_t seq, uint8_t device_id, const uint8_t *payload26)
{
  uint8_t rec[REC_IMU_RECORD_LEN];

  rec[0] = REC_TYPE_IMU;
  rec[1] = (uint8_t)(seq & 0xFFu);
  rec[2] = (uint8_t)(seq >> 8);
  rec[3] = (uint8_t)(seq >> 16);
  rec[4] = (uint8_t)(seq >> 24);
  rec[5] = device_id;
  memcpy(&rec[6], payload26, 26u);

  ring_push(rec, REC_IMU_RECORD_LEN);
}

/**
  * @brief  Mount the volume and create the next free LOGxxxx.BIN with a
  *         32-byte format header. Retries are driven by Recorder_Process().
  */
static uint8_t rec_open_next_file(void)
{
  uint8_t  header[32];
  uint32_t n;
  UINT     bw;

  if (f_mount(&rec_fs, "", 1u) != FR_OK)
  {
    return 0u;
  }

  for (n = 0u; n < 10000u; n++)
  {
    FRESULT fr;

    (void)snprintf(rec_filename, sizeof(rec_filename), "LOG%04lu.BIN", (unsigned long)n);
    fr = f_open(&rec_file, rec_filename, FA_CREATE_NEW | FA_WRITE);
    if (fr == FR_OK)
    {
      break;
    }
    if (fr != FR_EXIST)
    {
      /* real I/O error (card removed mid-scan, full, ...): give up now */
      (void)f_mount(NULL, "", 0u);
      return 0u;
    }
  }
  if (n == 10000u)
  {
    (void)f_mount(NULL, "", 0u);
    return 0u;
  }

  memset(header, 0, sizeof(header));
  header[0] = 'E';
  header[1] = 'M';
  header[2] = 'G';
  header[3] = 'L';
  header[4] = 1u;                 /* format version, low byte  */
  header[5] = 0u;                 /* format version, high byte */
  header[6] = 16u;                /* ADC channels              */
  header[7] = 0u;
  header[8]  = (uint8_t)(2000u & 0xFFu);        /* ADC rate Hz, LE u32 */
  header[9]  = (uint8_t)(2000u >> 8);
  header[10] = 0u;
  header[11] = 0u;
  header[12] = 4u;                /* IMU count                 */

  if ((f_write(&rec_file, header, sizeof(header), &bw) != FR_OK) || (bw != sizeof(header)))
  {
    rec_close_file();
    return 0u;
  }

  rec_bytes_written = sizeof(header);
  rec_dropped = 0u;
  rec_sync_tick = HAL_GetTick();
  rec_active = 1u;

  return 1u;
}

static void rec_close_file(void)
{
  if (rec_active != 0u)
  {
    (void)f_sync(&rec_file);
    (void)f_close(&rec_file);
    printf("[REC] %s closed, %lu bytes, %lu records dropped\r\n",
           rec_filename,
           (unsigned long)rec_bytes_written,
           (unsigned long)rec_dropped);
  }
  rec_active = 0u;
  rec_retry_tick = HAL_GetTick();
  (void)f_mount(NULL, "", 0u);
}

uint8_t Recorder_Init(void)
{
  uint8_t ok = rec_open_next_file();
  if (ok == 0u)
  {
    rec_retry_tick = HAL_GetTick();
  }
  return ok;
}

/**
  * @brief  Drain the ring into the log file. With flush_all=0 only whole
  *         512-byte sectors are written (keeps FatFs fast); flush_all=1
  *         also writes the trailing partial chunk (used by Recorder_Stop).
  * @retval 1 while the file is still open, 0 if a write error closed it.
  */
static uint8_t rec_drain(uint8_t flush_all)
{
  uint32_t head;
  uint32_t tail;
  uint32_t pending;
  uint32_t chunk;
  uint8_t *buf = rec_buf;
  UINT     bw;

  for (;;)
  {
    __disable_irq();
    head = ring_head;
    tail = ring_tail;
    __enable_irq();

    pending = (tail >= head) ? (tail - head) : (REC_RING_SIZE - (head - tail));
    if (pending < 512u)
    {
      if ((flush_all == 0u) || (pending == 0u))
      {
        break;
      }
      chunk = pending;
    }
    else
    {
      chunk = pending;
      if (chunk > REC_WRITE_CHUNK)
      {
        chunk = REC_WRITE_CHUNK;
      }
      chunk &= ~511u;   /* whole sectors keep FatFs fast */
    }

    {
      uint32_t first = REC_RING_SIZE - head;
      if (first > chunk)
      {
        first = chunk;
      }
      memcpy(buf, &ring[head], first);
      memcpy(&buf[first], ring, chunk - first);
    }

    if ((f_write(&rec_file, buf, chunk, &bw) != FR_OK) || (bw != chunk))
    {
      printf("[REC] write error, card removed or full\r\n");
      rec_close_file();
      return 0u;
    }

    __disable_irq();
    ring_head = (ring_head + chunk) % REC_RING_SIZE;
    __enable_irq();

    rec_bytes_written += chunk;
  }
  return 1u;
}

void Recorder_Process(void)
{
  if (rec_active == 0u)
  {
    if ((HAL_GetTick() - rec_retry_tick) >= REC_RETRY_PERIOD_MS)
    {
      rec_retry_tick = HAL_GetTick();
      if (rec_open_next_file() != 0u)
      {
        printf("[REC] card detected, logging to %s\r\n", rec_filename);
      }
    }
    return;
  }

  if (rec_drain(0u) == 0u)
  {
    return;
  }

  if ((HAL_GetTick() - rec_sync_tick) >= REC_SYNC_PERIOD_MS)
  {
    rec_sync_tick = HAL_GetTick();
    (void)f_sync(&rec_file);
  }
}

void Recorder_Start(void)
{
  if (rec_active != 0u)
  {
    printf("[REC] already logging to %s\r\n", rec_filename);
    return;
  }
  if (rec_open_next_file() != 0u)
  {
    printf("[REC] logging to %s\r\n", rec_filename);
  }
  else
  {
    printf("[REC] start failed: no SD card or filesystem error\r\n");
  }
}

void Recorder_Stop(void)
{
  if (rec_active == 0u)
  {
    printf("[REC] not logging\r\n");
    return;
  }

  /* Flush everything still pending in the ring before closing */
  if (rec_drain(1u) != 0u)
  {
    rec_close_file();
  }
}

uint8_t Recorder_IsActive(void)
{
  return rec_active;
}

uint32_t Recorder_GetDropped(void)
{
  return rec_dropped;
}

uint32_t Recorder_GetBytesWritten(void)
{
  return rec_bytes_written;
}
