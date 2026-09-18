/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    imu_parser.c
  * @brief   WT9011DCL-RF default IMU frame parser implementation.
  *          Default frame format from RF host:
  *            [device_id][0x55][0x61][axL axH ayL ayH ... yawL yawH batL batH]
  *          Total payload after header: 26 bytes = 13 int16 little-endian fields.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "imu_parser.h"
#include "main.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define IMU_HEADER           0x55U
#define IMU_FLAG_IMU         0x61U
#define IMU_FRAME_DATA_LEN   26U

/* Private types -------------------------------------------------------------*/
typedef enum
{
  IMU_WAIT_HEADER = 0,
  IMU_WAIT_FLAG,
  IMU_DATA
} IMU_ParseState_t;

/* Private variables ---------------------------------------------------------*/
static IMU_ParseState_t parse_state = IMU_WAIT_HEADER;
static uint8_t            prev_byte = 0U;
static uint8_t            data_buf[IMU_FRAME_DATA_LEN];
static uint8_t            data_idx = 0U;

static IMU_Data_t         slave_data[IMU_MAX_SLAVES];
static uint32_t           new_data_mask = 0U;
static uint32_t           slave_last_rx_tick[IMU_MAX_SLAVES];

/* Private function prototypes -----------------------------------------------*/
static inline int16_t IMU_BytesToInt16(uint8_t lo, uint8_t hi);
static void           IMU_ParseFrame(uint8_t device_id);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Combine low and high bytes into a signed 16-bit integer.
  */
static inline int16_t IMU_BytesToInt16(uint8_t lo, uint8_t hi)
{
  return (int16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);
}

/**
  * @brief  Parse one complete frame and update the corresponding slave structure.
  */
static void IMU_ParseFrame(uint8_t device_id)
{
  uint8_t idx = device_id & (IMU_MAX_SLAVES - 1U);
  IMU_Data_t *d = &slave_data[idx];
  int i;

  d->device_id = device_id;

  d->accel[0] = IMU_BytesToInt16(data_buf[0],  data_buf[1]);
  d->accel[1] = IMU_BytesToInt16(data_buf[2],  data_buf[3]);
  d->accel[2] = IMU_BytesToInt16(data_buf[4],  data_buf[5]);

  d->gyro[0]  = IMU_BytesToInt16(data_buf[6],  data_buf[7]);
  d->gyro[1]  = IMU_BytesToInt16(data_buf[8],  data_buf[9]);
  d->gyro[2]  = IMU_BytesToInt16(data_buf[10], data_buf[11]);

  d->mag[0]   = IMU_BytesToInt16(data_buf[12], data_buf[13]);
  d->mag[1]   = IMU_BytesToInt16(data_buf[14], data_buf[15]);
  d->mag[2]   = IMU_BytesToInt16(data_buf[16], data_buf[17]);

  d->angle[0] = IMU_BytesToInt16(data_buf[18], data_buf[19]);
  d->angle[1] = IMU_BytesToInt16(data_buf[20], data_buf[21]);
  d->angle[2] = IMU_BytesToInt16(data_buf[22], data_buf[23]);

  d->battery_raw = IMU_BytesToInt16(data_buf[24], data_buf[25]);

  for (i = 0; i < 3; i++)
  {
    d->accel_g[i]   = (float)d->accel[i] / 32768.0f * 16.0f;
    d->gyro_dps[i]  = (float)d->gyro[i]  / 32768.0f * 2000.0f;
    d->angle_deg[i] = (float)d->angle[i] / 32768.0f * 180.0f;
  }

  d->battery_voltage = (float)d->battery_raw / 100.0f;

  new_data_mask |= (1UL << idx);
  slave_last_rx_tick[idx] = HAL_GetTick();

  IMU_RawFrameHook(device_id, data_buf, IMU_FRAME_DATA_LEN);
}

/**
  * @brief  Weak hook called with the raw 26-byte payload of every parsed
  *         IMU frame. Overridden by recorder.c for SD logging.
  */
__weak void IMU_RawFrameHook(uint8_t device_id, const uint8_t *payload, uint16_t len)
{
  (void)device_id;
  (void)payload;
  (void)len;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Feed the received byte stream into the parser.
  *         Caller: invoked from USBH_CDC_ReceiveCallback().
  */
void IMU_ParseStream(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    uint8_t b = data[i];

    switch (parse_state)
    {
      case IMU_WAIT_HEADER:
        if (b == IMU_HEADER)
        {
          parse_state = IMU_WAIT_FLAG;
        }
        else
        {
          /* The byte before 0x55 is treated as the next frame's device ID */
          prev_byte = b;
        }
        break;

      case IMU_WAIT_FLAG:
        if (b == IMU_FLAG_IMU)
        {
          data_idx = 0U;
          parse_state = IMU_DATA;
        }
        else if (b == IMU_HEADER)
        {
          /* Consecutive 0x55: keep the previous 0x55 as the last byte */
          prev_byte = IMU_HEADER;
        }
        else
        {
          prev_byte = b;
          parse_state = IMU_WAIT_HEADER;
        }
        break;

      case IMU_DATA:
        data_buf[data_idx++] = b;
        if (data_idx >= IMU_FRAME_DATA_LEN)
        {
          IMU_ParseFrame(prev_byte);
          parse_state = IMU_WAIT_HEADER;
        }
        break;

      default:
        parse_state = IMU_WAIT_HEADER;
        break;
    }
  }
}

/**
  * @brief  Get the latest data of the specified slave.
  */
IMU_Data_t *IMU_GetSlaveData(uint8_t device_id)
{
  return &slave_data[device_id & (IMU_MAX_SLAVES - 1U)];
}

/**
  * @brief  Check whether the specified slave has new data.
  */
uint8_t IMU_SlaveNewData(uint8_t device_id)
{
  uint8_t idx = device_id & (IMU_MAX_SLAVES - 1U);
  return (uint8_t)((new_data_mask >> idx) & 1UL);
}

/**
  * @brief  Clear the new-data flag of the specified slave.
  */
void IMU_ClearSlaveNewData(uint8_t device_id)
{
  uint8_t idx = device_id & (IMU_MAX_SLAVES - 1U);
  new_data_mask &= ~(1UL << idx);
}

/**
  * @brief  Check whether the specified slave is currently online.
  */
uint8_t IMU_IsSlaveOnline(uint8_t device_id)
{
  uint8_t idx = device_id & (IMU_MAX_SLAVES - 1U);
  return ((HAL_GetTick() - slave_last_rx_tick[idx]) < IMU_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}
