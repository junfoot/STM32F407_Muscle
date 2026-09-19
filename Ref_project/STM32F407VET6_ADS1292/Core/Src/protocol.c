#include "protocol.h"

#include <string.h>
#include "ads1292.h"
#include "imu_receiver.h"
#include "sd_recorder.h"
#include "usart.h"

#define FRAME_REQUEST_START     0xA5U
#define FRAME_REQUEST_END       0x5AU
#define FRAME_RESPONSE_START    0xAAU
#define FRAME_RESPONSE_END      0x55U
#define FRAME_OVERHEAD          8U
#define CMD_SW_VERSION          0x00U
#define CMD_HW_VERSION          0x01U
#define CMD_CONN_STATE          0x03U
#define CMD_SAMPLE_CONTROL      0x04U
#define CMD_SAMPLE_PARAMETER    0x05U
#define CMD_RAW_DATA            0x06U
#define CMD_SD_RECORD_CONTROL   0x07U
#define CMD_IMU_DATA            0x08U
#define CMD_IMU_STATUS          0x09U
#define UART_RX_SIZE            64U
#define UART_QUEUE_CAPACITY     513U
#define UART_SAMPLES_PER_FRAME  24U
#define UART_IMU_QUEUE_CAPACITY 129U
#define UART_IMU_PER_FRAME      4U

typedef struct
{
    imu_sample_t storage[UART_IMU_QUEUE_CAPACITY];
    uint16_t head;
    uint16_t tail;
    uint32_t dropped;
} uart_imu_queue_t;

extern void app_set_host_streaming(bool enabled);
extern bool app_apply_sample_parameters(uint16_t rate, uint8_t pga,
                                        uint8_t channel_mask, uint8_t input_mode,
                                        uint8_t rld_enabled);

static uint8_t uart_rx_dma[UART_RX_SIZE];
static uint8_t uart_rx_copy[UART_RX_SIZE];
static uint8_t rx_accumulator[UART_RX_SIZE * 2U];
static uint16_t rx_accumulator_length;
static volatile uint16_t uart_rx_length;
static volatile bool uart_rx_ready;
static volatile bool uart_tx_busy;
static volatile bool host_streaming;
static uint8_t uart_tx_frame[256];
static uint8_t response_payload[32];
static uint8_t response_command;
static uint8_t response_length;
static bool response_pending;
static sample_queue_t stream_queue;
static emg_sample_t stream_storage[UART_QUEUE_CAPACITY];
static uart_imu_queue_t imu_stream_queue;
static bool send_imu_next;

static void imu_stream_clear(void)
{
    imu_stream_queue.head = 0U;
    imu_stream_queue.tail = 0U;
    imu_stream_queue.dropped = 0U;
}

static bool imu_stream_push(const imu_sample_t *sample)
{
    uint16_t next = (uint16_t)(imu_stream_queue.head + 1U);
    if (next >= UART_IMU_QUEUE_CAPACITY) next = 0U;
    if (next == imu_stream_queue.tail)
    {
        imu_stream_queue.dropped++;
        return false;
    }
    imu_stream_queue.storage[imu_stream_queue.head] = *sample;
    __DMB();
    imu_stream_queue.head = next;
    return true;
}

static bool imu_stream_pop(imu_sample_t *sample)
{
    uint16_t tail = imu_stream_queue.tail;
    if (tail == imu_stream_queue.head) return false;
    *sample = imu_stream_queue.storage[tail];
    __DMB();
    tail++;
    if (tail >= UART_IMU_QUEUE_CAPACITY) tail = 0U;
    imu_stream_queue.tail = tail;
    return true;
}

static uint16_t imu_stream_count(void)
{
    uint16_t head = imu_stream_queue.head;
    uint16_t tail = imu_stream_queue.tail;
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(UART_IMU_QUEUE_CAPACITY - tail + head);
}

static void uart_receive_start(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma, sizeof(uart_rx_dma)) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

static uint16_t pack_frame(uint8_t command, const uint8_t *payload, uint16_t payload_length)
{
    uint16_t frame_length = (uint16_t)(payload_length + FRAME_OVERHEAD);
    uint16_t index;
    uint8_t check = 0U;
    uart_tx_frame[0] = FRAME_RESPONSE_START;
    uart_tx_frame[1] = (uint8_t)(frame_length & 0xFFU);
    uart_tx_frame[2] = (uint8_t)(frame_length >> 8);
    uart_tx_frame[3] = 0U;
    uart_tx_frame[4] = command;
    for (index = 1U; index <= 4U; index++) check ^= uart_tx_frame[index];
    uart_tx_frame[5] = check;
    if (payload_length > 0U) memcpy(&uart_tx_frame[6], payload, payload_length);
    check = 0U;
    for (index = 0U; index < payload_length; index++) check ^= payload[index];
    uart_tx_frame[6U + payload_length] = check;
    uart_tx_frame[7U + payload_length] = FRAME_RESPONSE_END;
    return frame_length;
}

static void queue_response(uint8_t command, const uint8_t *payload, uint8_t length)
{
    if (length > sizeof(response_payload)) length = sizeof(response_payload);
    if (length > 0U) memcpy(response_payload, payload, length);
    response_command = command;
    response_length = length;
    response_pending = true;
}

static void handle_command(uint8_t command, const uint8_t *data, uint16_t length)
{
    uint8_t reply[32];
    command &= 0x7FU;
    switch (command)
    {
        case CMD_SW_VERSION:
            reply[0] = 1U; reply[1] = 0U; reply[2] = 0U;
            queue_response(command, reply, 3U);
            break;
        case CMD_HW_VERSION:
            reply[0] = 1U; reply[1] = 0U;
            queue_response(command, reply, 2U);
            break;
        case CMD_CONN_STATE:
            reply[0] = (length > 0U) ? data[0] : (host_streaming ? 1U : 0U);
            queue_response(command, reply, 1U);
            break;
        case CMD_SAMPLE_CONTROL:
            if (length > 0U)
            {
                host_streaming = data[0] != 0U;
                if (!host_streaming)
                {
                    sample_queue_clear(&stream_queue);
                    imu_stream_clear();
                }
                app_set_host_streaming(host_streaming);
            }
            reply[0] = host_streaming ? 1U : 0U;
            queue_response(command, reply, 1U);
            break;
        case CMD_SAMPLE_PARAMETER:
            if (length >= 5U)
            {
                uint16_t rate = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                uint8_t rld = (length >= 6U) ? data[5] : 1U;
                (void)app_apply_sample_parameters(rate, data[2], data[3], data[4], rld);
            }
            reply[0] = (uint8_t)(ads1292_info.rate & 0xFFU);
            reply[1] = (uint8_t)(ads1292_info.rate >> 8);
            reply[2] = ads1292_info.pga;
            reply[3] = ads1292_info.ch_en;
            reply[4] = ads1292_info.ch_sw;
            reply[5] = ads1292_info.rld_en;
            queue_response(command, reply, 6U);
            break;
        case CMD_SD_RECORD_CONTROL:
        {
            uint32_t dropped;
            uint32_t written;
            if (length > 0U)
            {
                if (data[0] != 0U)
                {
                    if (sd_recorder_state() != SD_RECORDER_ACTIVE)
                    {
                        sd_recorder_init(ads1292_info.rate, ads1292_info.pga,
                                         ads1292_info.ch_en, ads1292_info.rld_en);
                        (void)sd_recorder_start();
                    }
                }
                else
                {
                    sd_recorder_stop();
                }
                /* Re-evaluate ADS acquisition exactly as the PA0 path does. */
                app_set_host_streaming(host_streaming);
            }
            memset(reply, 0, sizeof(reply));
            reply[0] = (uint8_t)sd_recorder_state();
            dropped = sd_recorder_dropped();
            reply[1] = (uint8_t)(dropped & 0xFFU);
            reply[2] = (uint8_t)((dropped >> 8) & 0xFFU);
            reply[3] = (uint8_t)((dropped >> 16) & 0xFFU);
            reply[4] = (uint8_t)((dropped >> 24) & 0xFFU);
            written = sd_recorder_records_written();
            reply[5] = (uint8_t)(written & 0xFFU);
            reply[6] = (uint8_t)((written >> 8) & 0xFFU);
            reply[7] = (uint8_t)((written >> 16) & 0xFFU);
            reply[8] = (uint8_t)((written >> 24) & 0xFFU);
            strncpy((char *)&reply[9], sd_recorder_filename(), 11U);
            queue_response(command, reply, 20U);
            break;
        }
        case CMD_IMU_STATUS:
        {
            uint32_t value;
            memset(reply, 0, sizeof(reply));
            reply[0] = imu_receiver_usb_state();
            reply[1] = imu_receiver_cdc_state();
            reply[2] = imu_receiver_connected() ? 1U : 0U;
            value = imu_frames_received();
            memcpy(&reply[4], &value, sizeof(value));
            value = imu_online_mask();
            memcpy(&reply[8], &value, sizeof(value));
            value = protocol_imu_dropped();
            memcpy(&reply[12], &value, sizeof(value));
            value = sd_recorder_imu_dropped();
            memcpy(&reply[16], &value, sizeof(value));
            value = imu_receiver_bytes_received();
            memcpy(&reply[20], &value, sizeof(value));
            value = sd_recorder_error_detail();
            memcpy(&reply[24], &value, sizeof(value));
            value = protocol_dropped();
            memcpy(&reply[28], &value, sizeof(value));
            queue_response(command, reply, 32U);
            break;
        }
        default:
            break;
    }
}

static uint16_t parse_rx(const uint8_t *buffer, uint16_t length)
{
    uint16_t offset = 0U;
    while ((uint16_t)(length - offset) >= FRAME_OVERHEAD)
    {
        uint16_t frame_length;
        uint16_t payload_length;
        uint16_t index;
        uint8_t header_check;
        uint8_t data_check = 0U;
        if (buffer[offset] != FRAME_REQUEST_START)
        {
            offset++;
            continue;
        }
        frame_length = (uint16_t)buffer[offset + 1U]
                     | ((uint16_t)buffer[offset + 2U] << 8);
        if (frame_length < FRAME_OVERHEAD || frame_length > sizeof(rx_accumulator))
        {
            offset++;
            continue;
        }
        if (frame_length > (uint16_t)(length - offset))
        {
            break;
        }
        header_check = buffer[offset + 1U] ^ buffer[offset + 2U]
                     ^ buffer[offset + 3U] ^ buffer[offset + 4U];
        payload_length = (uint16_t)(frame_length - FRAME_OVERHEAD);
        for (index = 0U; index < payload_length; index++)
        {
            data_check ^= buffer[offset + 6U + index];
        }
        if (buffer[offset + 5U] == header_check
            && buffer[offset + frame_length - 2U] == data_check
            && buffer[offset + frame_length - 1U] == FRAME_REQUEST_END)
        {
            handle_command(buffer[offset + 4U], &buffer[offset + 6U], payload_length);
            offset = (uint16_t)(offset + frame_length);
        }
        else
        {
            offset++;
        }
    }
    return offset;
}

void protocol_init(void)
{
    sample_queue_init(&stream_queue, stream_storage, UART_QUEUE_CAPACITY);
    imu_stream_clear();
    send_imu_next = true;
    host_streaming = false;
    uart_tx_busy = false;
    uart_rx_ready = false;
    rx_accumulator_length = 0U;
    response_pending = false;
    uart_receive_start();
}

void protocol_service(void)
{
    if (uart_rx_ready)
    {
        uint16_t length;
        __disable_irq();
        length = uart_rx_length;
        memcpy(uart_rx_copy, uart_rx_dma, length);
        uart_rx_ready = false;
        __enable_irq();
        if ((uint16_t)(rx_accumulator_length + length) > sizeof(rx_accumulator))
        {
            rx_accumulator_length = 0U;
        }
        memcpy(&rx_accumulator[rx_accumulator_length], uart_rx_copy, length);
        rx_accumulator_length = (uint16_t)(rx_accumulator_length + length);
        if (rx_accumulator_length > 0U)
        {
            uint16_t consumed = parse_rx(rx_accumulator, rx_accumulator_length);
            if (consumed > 0U)
            {
                rx_accumulator_length = (uint16_t)(rx_accumulator_length - consumed);
                memmove(rx_accumulator, &rx_accumulator[consumed], rx_accumulator_length);
            }
        }
        uart_receive_start();
    }

    if (uart_tx_busy) return;
    if (response_pending)
    {
        uint16_t frame_length = pack_frame(response_command, response_payload, response_length);
        response_pending = false;
        uart_tx_busy = HAL_UART_Transmit_DMA(&huart1, uart_tx_frame, frame_length) == HAL_OK;
        return;
    }
    if (host_streaming && imu_stream_count() > 0U
        && (sample_queue_count(&stream_queue) < UART_SAMPLES_PER_FRAME || send_imu_next))
    {
        uint8_t payload[UART_IMU_PER_FRAME * sizeof(imu_sample_t)];
        uint16_t count = imu_stream_count();
        uint16_t index;
        uint16_t frame_length;
        if (count > UART_IMU_PER_FRAME) count = UART_IMU_PER_FRAME;
        for (index = 0U; index < count; index++)
        {
            imu_sample_t sample;
            (void)imu_stream_pop(&sample);
            memcpy(&payload[index * sizeof(imu_sample_t)], &sample, sizeof(sample));
        }
        frame_length = pack_frame(CMD_IMU_DATA, payload,
                                  (uint16_t)(count * sizeof(imu_sample_t)));
        uart_tx_busy = HAL_UART_Transmit_DMA(&huart1, uart_tx_frame, frame_length) == HAL_OK;
        send_imu_next = false;
        return;
    }
    if (host_streaming && sample_queue_count(&stream_queue) >= UART_SAMPLES_PER_FRAME)
    {
        uint8_t payload[UART_SAMPLES_PER_FRAME * 8U + 1U];
        uint16_t index;
        for (index = 0U; index < UART_SAMPLES_PER_FRAME; index++)
        {
            emg_sample_t sample;
            (void)sample_queue_pop(&stream_queue, &sample);
            memcpy(&payload[index * 8U], sample.channel, 8U);
        }
        payload[UART_SAMPLES_PER_FRAME * 8U] = ads1292_loff_stat;
        index = pack_frame(CMD_RAW_DATA, payload, sizeof(payload));
        uart_tx_busy = HAL_UART_Transmit_DMA(&huart1, uart_tx_frame, index) == HAL_OK;
        send_imu_next = true;
    }
}

bool protocol_push_isr(const emg_sample_t *sample)
{
    return host_streaming ? sample_queue_push_isr(&stream_queue, sample) : false;
}

bool protocol_push_imu(const imu_sample_t *sample)
{
    return host_streaming ? imu_stream_push(sample) : false;
}

bool protocol_is_streaming(void)
{
    return host_streaming;
}

uint32_t protocol_dropped(void)
{
    return stream_queue.dropped;
}

uint32_t protocol_imu_dropped(void)
{
    return imu_stream_queue.dropped;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart->Instance == USART1 && !uart_rx_ready)
    {
        uart_rx_length = (size <= UART_RX_SIZE) ? size : UART_RX_SIZE;
        uart_rx_ready = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) uart_tx_busy = false;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1)
    {
        uart_tx_busy = false;
        uart_rx_ready = false;
        uart_receive_start();
    }
}
