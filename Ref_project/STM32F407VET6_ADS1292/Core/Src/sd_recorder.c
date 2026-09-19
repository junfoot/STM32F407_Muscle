#include "sd_recorder.h"

#include <string.h>
#include "ff.h"
#include "sdio.h"

#define SD_QUEUE_CAPACITY       4097U
#define SD_BATCH_RECORDS        128U
#define SD_IMU_QUEUE_CAPACITY   257U
#define SD_IMU_BATCH_RECORDS    64U
#define SD_SYNC_PERIOD_MS       1000U
#define SD_FILENAME_ATTEMPTS    9999U

typedef struct
{
    char magic[8];
    uint16_t header_size;
    uint16_t record_size;
    uint32_t sample_rate;
    uint8_t pga;
    uint8_t channel_mask;
    uint8_t rld_enabled;
    uint8_t reserved0;
    uint32_t start_tick_ms;
    uint32_t reserved[122];
} emg_file_header_t;

typedef char emg_header_must_be_512_bytes[(sizeof(emg_file_header_t) == 512U) ? 1 : -1];
typedef char emg_record_must_be_16_bytes[(sizeof(emg_sample_t) == 16U) ? 1 : -1];

typedef struct
{
    char magic[8];
    uint16_t header_size;
    uint16_t record_size;
    uint32_t start_tick_ms;
    uint32_t accel_full_scale_g;
    uint32_t gyro_full_scale_dps;
    uint32_t angle_full_scale_deg;
    uint32_t battery_divisor;
    uint32_t reserved[120];
} imu_file_header_t;

typedef struct
{
    imu_sample_t *storage;
    uint16_t capacity;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t dropped;
} imu_queue_t;

typedef char imu_header_must_be_512_bytes[(sizeof(imu_file_header_t) == 512U) ? 1 : -1];
typedef char imu_record_must_be_40_bytes[(sizeof(imu_sample_t) == 40U) ? 1 : -1];

static FATFS file_system;
static FIL emg_file;
static FIL imu_file;
static sample_queue_t record_queue;
static emg_sample_t queue_storage[SD_QUEUE_CAPACITY];
static emg_sample_t write_batch[SD_BATCH_RECORDS];
static imu_queue_t imu_queue;
static imu_sample_t imu_queue_storage[SD_IMU_QUEUE_CAPACITY];
static imu_sample_t imu_write_batch[SD_IMU_BATCH_RECORDS];
static volatile sd_recorder_state_t recorder_state = SD_RECORDER_IDLE;
static uint32_t configured_rate;
static uint8_t configured_pga;
static uint8_t configured_channel_mask;
static uint8_t configured_rld;
static uint32_t last_sync_tick;
static uint32_t records_written;
static uint32_t imu_records_written;
static char current_filename[13];
static char current_imu_filename[13];
static bool emg_file_open;
static bool imu_file_open;
static uint32_t last_error_detail;

static void imu_queue_init(void)
{
    imu_queue.storage = imu_queue_storage;
    imu_queue.capacity = SD_IMU_QUEUE_CAPACITY;
    imu_queue.head = 0U;
    imu_queue.tail = 0U;
    imu_queue.dropped = 0U;
}

static uint16_t imu_queue_count(void)
{
    uint16_t head = imu_queue.head;
    uint16_t tail = imu_queue.tail;
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(imu_queue.capacity - tail + head);
}

static bool imu_queue_push(const imu_sample_t *sample)
{
    uint16_t next = (uint16_t)(imu_queue.head + 1U);
    if (next >= imu_queue.capacity) next = 0U;
    if (next == imu_queue.tail)
    {
        imu_queue.dropped++;
        return false;
    }
    imu_queue.storage[imu_queue.head] = *sample;
    __DMB();
    imu_queue.head = next;
    return true;
}

static bool imu_queue_pop(imu_sample_t *sample)
{
    uint16_t tail = imu_queue.tail;
    if (tail == imu_queue.head) return false;
    *sample = imu_queue.storage[tail];
    __DMB();
    tail++;
    if (tail >= imu_queue.capacity) tail = 0U;
    imu_queue.tail = tail;
    return true;
}

static void recorder_fail(uint8_t stage, FRESULT result)
{
    if (last_error_detail == 0U)
    {
        last_error_detail = ((uint32_t)stage << 8U) | (uint8_t)result;
    }
    if (emg_file_open)
    {
        (void)f_sync(&emg_file);
        (void)f_close(&emg_file);
        emg_file_open = false;
    }
    if (imu_file_open)
    {
        (void)f_sync(&imu_file);
        (void)f_close(&imu_file);
        imu_file_open = false;
    }
    (void)f_mount(0, "", 0U);
    recorder_state = SD_RECORDER_ERROR;
}

static FRESULT recorder_write(FIL *file, const void *data, UINT length)
{
    UINT written = 0U;
    FRESULT result = f_write(file, data, length, &written);
    if (result == FR_OK && written != length) result = FR_DISK_ERR;
    return result;
}

static bool recorder_make_filename(void)
{
    FILINFO info;
    uint32_t index;
    for (index = 1U; index <= SD_FILENAME_ATTEMPTS; index++)
    {
        current_filename[0] = 'E';
        current_filename[1] = 'M';
        current_filename[2] = 'G';
        current_filename[3] = (char)('0' + ((index / 1000U) % 10U));
        current_filename[4] = (char)('0' + ((index / 100U) % 10U));
        current_filename[5] = (char)('0' + ((index / 10U) % 10U));
        current_filename[6] = (char)('0' + (index % 10U));
        current_filename[7] = '.';
        current_filename[8] = 'B';
        current_filename[9] = 'I';
        current_filename[10] = 'N';
        current_filename[11] = '\0';
        memcpy(current_imu_filename, current_filename, sizeof(current_filename));
        current_imu_filename[0] = 'I';
        current_imu_filename[1] = 'M';
        current_imu_filename[2] = 'U';
        if (f_stat(current_filename, &info) == FR_NO_FILE
            && f_stat(current_imu_filename, &info) == FR_NO_FILE)
        {
            return true;
        }
    }
    return false;
}

void sd_recorder_init(uint32_t sample_rate, uint8_t pga, uint8_t channel_mask, uint8_t rld_enabled)
{
    if (emg_file_open || imu_file_open)
    {
        if (emg_file_open) (void)f_close(&emg_file);
        if (imu_file_open) (void)f_close(&imu_file);
        emg_file_open = false;
        imu_file_open = false;
        (void)f_mount(0, "", 0U);
    }
    configured_rate = sample_rate;
    configured_pga = pga;
    configured_channel_mask = channel_mask;
    configured_rld = rld_enabled;
    sample_queue_init(&record_queue, queue_storage, SD_QUEUE_CAPACITY);
    imu_queue_init();
    current_filename[0] = '\0';
    current_imu_filename[0] = '\0';
    records_written = 0U;
    imu_records_written = 0U;
    last_error_detail = 0U;
    emg_file_open = false;
    imu_file_open = false;
    recorder_state = SD_RECORDER_IDLE;
}

bool sd_recorder_start(void)
{
    emg_file_header_t header;
    imu_file_header_t imu_header;
    FRESULT result;
    if (recorder_state != SD_RECORDER_IDLE && recorder_state != SD_RECORDER_ERROR)
    {
        return false;
    }

    recorder_state = SD_RECORDER_IDLE;
    last_error_detail = 0U;
    sample_queue_clear(&record_queue);
    imu_queue_init();
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        (void)HAL_SD_DeInit(&hsd);
        if (HAL_SD_Init(&hsd) != HAL_OK)
        {
            last_error_detail = (1UL << 8U) | 0xFFU;
            recorder_state = SD_RECORDER_ERROR;
            return false;
        }
    }
    result = f_mount(&file_system, "", 1U);
    if (result != FR_OK)
    {
        recorder_fail(2U, result);
        return false;
    }
    if (!recorder_make_filename())
    {
        recorder_fail(3U, FR_EXIST);
        return false;
    }
    result = f_open(&emg_file, current_filename, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK)
    {
        recorder_fail(4U, result);
        return false;
    }
    emg_file_open = true;
    result = f_open(&imu_file, current_imu_filename, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK)
    {
        recorder_fail(5U, result);
        return false;
    }
    imu_file_open = true;

    memset(&header, 0, sizeof(header));
    memcpy(header.magic, "EMG2K01", 7U);
    header.header_size = (uint16_t)sizeof(header);
    header.record_size = (uint16_t)sizeof(emg_sample_t);
    header.sample_rate = configured_rate;
    header.pga = configured_pga;
    header.channel_mask = configured_channel_mask;
    header.rld_enabled = configured_rld;
    header.start_tick_ms = HAL_GetTick();
    memset(&imu_header, 0, sizeof(imu_header));
    memcpy(imu_header.magic, "IMU9011", 7U);
    imu_header.header_size = (uint16_t)sizeof(imu_header);
    imu_header.record_size = (uint16_t)sizeof(imu_sample_t);
    imu_header.start_tick_ms = header.start_tick_ms;
    imu_header.accel_full_scale_g = 16U;
    imu_header.gyro_full_scale_dps = 2000U;
    imu_header.angle_full_scale_deg = 180U;
    imu_header.battery_divisor = 100U;
    result = recorder_write(&emg_file, &header, sizeof(header));
    if (result != FR_OK)
    {
        recorder_fail(6U, result);
        return false;
    }
    result = recorder_write(&imu_file, &imu_header, sizeof(imu_header));
    if (result != FR_OK)
    {
        recorder_fail(7U, result);
        return false;
    }
    result = f_sync(&emg_file);
    if (result != FR_OK)
    {
        recorder_fail(8U, result);
        return false;
    }
    result = f_sync(&imu_file);
    if (result != FR_OK)
    {
        recorder_fail(9U, result);
        return false;
    }
    last_sync_tick = HAL_GetTick();
    recorder_state = SD_RECORDER_ACTIVE;
    return true;
}

void sd_recorder_stop(void)
{
    if (recorder_state == SD_RECORDER_ACTIVE)
    {
        recorder_state = SD_RECORDER_STOPPING;
    }
}

void sd_recorder_service(void)
{
    uint16_t count;
    uint16_t available;
    uint16_t imu_count;
    if (recorder_state != SD_RECORDER_ACTIVE && recorder_state != SD_RECORDER_STOPPING)
    {
        return;
    }

    available = sample_queue_count(&record_queue);
    count = (available >= SD_BATCH_RECORDS) ? SD_BATCH_RECORDS : available;
    if ((recorder_state == SD_RECORDER_ACTIVE) && (count < SD_BATCH_RECORDS))
    {
        if (imu_queue_count() < SD_IMU_BATCH_RECORDS
            && (HAL_GetTick() - last_sync_tick) >= SD_SYNC_PERIOD_MS)
        {
            FRESULT sync_result = f_sync(&emg_file);
            if (sync_result != FR_OK)
            {
                recorder_fail(10U, sync_result);
                return;
            }
            sync_result = f_sync(&imu_file);
            if (sync_result != FR_OK)
            {
                recorder_fail(11U, sync_result);
                return;
            }
            last_sync_tick = HAL_GetTick();
        }
        if (imu_queue_count() < SD_IMU_BATCH_RECORDS) return;
    }

    if (count > 0U)
    {
        uint16_t index;
        for (index = 0U; index < count; index++)
        {
            (void)sample_queue_pop(&record_queue, &write_batch[index]);
        }
        FRESULT write_result = recorder_write(&emg_file, write_batch,
                                               (UINT)(count * sizeof(emg_sample_t)));
        if (write_result != FR_OK)
        {
            recorder_fail(12U, write_result);
            return;
        }
        records_written += count;
    }

    available = imu_queue_count();
    imu_count = (available >= SD_IMU_BATCH_RECORDS) ? SD_IMU_BATCH_RECORDS : available;
    if (recorder_state == SD_RECORDER_ACTIVE && imu_count < SD_IMU_BATCH_RECORDS)
    {
        imu_count = 0U;
    }
    if (imu_count > 0U)
    {
        uint16_t index;
        for (index = 0U; index < imu_count; index++)
        {
            (void)imu_queue_pop(&imu_write_batch[index]);
        }
        FRESULT write_result = recorder_write(&imu_file, imu_write_batch,
                                               (UINT)(imu_count * sizeof(imu_sample_t)));
        if (write_result != FR_OK)
        {
            recorder_fail(13U, write_result);
            return;
        }
        imu_records_written += imu_count;
    }

    if ((recorder_state == SD_RECORDER_STOPPING)
        && sample_queue_count(&record_queue) == 0U && imu_queue_count() == 0U)
    {
        FRESULT emg_sync = f_sync(&emg_file);
        FRESULT imu_sync = f_sync(&imu_file);
        FRESULT emg_close = f_close(&emg_file);
        FRESULT imu_close = f_close(&imu_file);
        emg_file_open = false;
        imu_file_open = false;
        (void)f_mount(0, "", 0U);
        recorder_state = (emg_sync == FR_OK && imu_sync == FR_OK
                          && emg_close == FR_OK && imu_close == FR_OK)
                       ? SD_RECORDER_IDLE : SD_RECORDER_ERROR;
    }
}

bool sd_recorder_push_isr(const emg_sample_t *sample)
{
    return (recorder_state == SD_RECORDER_ACTIVE)
        ? sample_queue_push_isr(&record_queue, sample) : false;
}

bool sd_recorder_push_imu(const imu_sample_t *sample)
{
    return (recorder_state == SD_RECORDER_ACTIVE) ? imu_queue_push(sample) : false;
}

bool sd_recorder_is_accepting_isr(void)
{
    return recorder_state == SD_RECORDER_ACTIVE;
}

sd_recorder_state_t sd_recorder_state(void)
{
    return recorder_state;
}

uint32_t sd_recorder_dropped(void)
{
    return record_queue.dropped;
}

uint32_t sd_recorder_records_written(void)
{
    return records_written;
}

uint32_t sd_recorder_imu_dropped(void)
{
    return imu_queue.dropped;
}

uint32_t sd_recorder_imu_records_written(void)
{
    return imu_records_written;
}

uint32_t sd_recorder_error_detail(void)
{
    return last_error_detail;
}

const char *sd_recorder_filename(void)
{
    return current_filename;
}

const char *sd_recorder_imu_filename(void)
{
    return current_imu_filename;
}
