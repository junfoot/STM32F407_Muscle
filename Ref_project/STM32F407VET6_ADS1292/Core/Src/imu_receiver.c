#include "imu_receiver.h"

#include <stdint.h>
#include "imu_parser.h"
#include "usb_host.h"
#include "usbh_cdc.h"

#define CDC_RX_BUFFER_SIZE              512U
#define CDC_RX_TIMEOUT_MS               2000U
#define CDC_LINE_CODING_TIMEOUT_MS      2000U
#define CDC_REQUEST_SET_CONTROL_LINE    0x22U
#define CDC_CONTROL_LINE_DTR_RTS        0x0003U
#define IMU_UART_BAUDRATE               460800U

typedef enum
{
    CDC_STATE_IDLE = 0,
    CDC_STATE_SET_LINE_CODING,
    CDC_STATE_SET_CONTROL_LINE,
    CDC_STATE_START_RECEPTION,
    CDC_STATE_RUNNING
} cdc_state_t;

extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;

static uint8_t cdc_rx_buffer[CDC_RX_BUFFER_SIZE];
static CDC_LineCodingTypeDef line_coding;
static volatile cdc_state_t cdc_state;
static volatile uint8_t line_coding_done;
static volatile uint8_t rx_ready;
static volatile uint32_t last_rx_tick;
static uint32_t line_coding_start_tick;
static volatile uint32_t bytes_received;

static USBH_StatusTypeDef set_control_line_state(USBH_HandleTypeDef *host, uint16_t state)
{
    host->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS
                                          | USB_REQ_RECIPIENT_INTERFACE;
    host->Control.setup.b.bRequest = CDC_REQUEST_SET_CONTROL_LINE;
    host->Control.setup.b.wValue.w = state;
    host->Control.setup.b.wIndex.w = 0U;
    host->Control.setup.b.wLength.w = 0U;
    return USBH_CtlReq(host, NULL, 0U);
}

void imu_receiver_init(void)
{
    imu_parser_init();
    cdc_state = CDC_STATE_IDLE;
    line_coding_done = 0U;
    rx_ready = 0U;
    last_rx_tick = 0U;
    bytes_received = 0U;
}

void imu_receiver_service(void)
{
    MX_USB_HOST_Process();
    if (Appli_state != APPLICATION_READY)
    {
        cdc_state = CDC_STATE_IDLE;
        line_coding_done = 0U;
        rx_ready = 0U;
        return;
    }

    switch (cdc_state)
    {
        case CDC_STATE_IDLE:
            line_coding.b.dwDTERate = IMU_UART_BAUDRATE;
            line_coding.b.bCharFormat = 0U;
            line_coding.b.bParityType = 0U;
            line_coding.b.bDataBits = 8U;
            line_coding_done = 0U;
            line_coding_start_tick = HAL_GetTick();
            (void)USBH_CDC_SetLineCoding(&hUsbHostFS, &line_coding);
            cdc_state = CDC_STATE_SET_LINE_CODING;
            break;

        case CDC_STATE_SET_LINE_CODING:
            if (line_coding_done
                || (HAL_GetTick() - line_coding_start_tick) > CDC_LINE_CODING_TIMEOUT_MS)
            {
                cdc_state = CDC_STATE_SET_CONTROL_LINE;
            }
            break;

        case CDC_STATE_SET_CONTROL_LINE:
        {
            USBH_StatusTypeDef status = set_control_line_state(&hUsbHostFS,
                                                               CDC_CONTROL_LINE_DTR_RTS);
            if (status != USBH_BUSY)
            {
                cdc_state = CDC_STATE_START_RECEPTION;
            }
            break;
        }

        case CDC_STATE_START_RECEPTION:
            if (USBH_CDC_Receive(&hUsbHostFS, cdc_rx_buffer,
                                 sizeof(cdc_rx_buffer)) == USBH_OK)
            {
                last_rx_tick = HAL_GetTick();
                cdc_state = CDC_STATE_RUNNING;
            }
            break;

        case CDC_STATE_RUNNING:
            if ((HAL_GetTick() - last_rx_tick) > CDC_RX_TIMEOUT_MS)
            {
                cdc_state = CDC_STATE_START_RECEPTION;
                rx_ready = 0U;
            }
            break;

        default:
            cdc_state = CDC_STATE_IDLE;
            break;
    }
}

bool imu_receiver_connected(void)
{
    return rx_ready != 0U;
}

uint8_t imu_receiver_usb_state(void)
{
    return (uint8_t)Appli_state;
}

uint8_t imu_receiver_cdc_state(void)
{
    return (uint8_t)cdc_state;
}

uint32_t imu_receiver_bytes_received(void)
{
    return bytes_received;
}

void USBH_CDC_LineCodingChanged(USBH_HandleTypeDef *host)
{
    (void)host;
    line_coding_done = 1U;
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *host)
{
    uint16_t length = USBH_CDC_GetLastReceivedDataSize(host);
    if (length > 0U)
    {
        last_rx_tick = HAL_GetTick();
        bytes_received += length;
        rx_ready = 1U;
        imu_parse_stream(cdc_rx_buffer, length);
    }
    if (USBH_CDC_Receive(host, cdc_rx_buffer, sizeof(cdc_rx_buffer)) != USBH_OK)
    {
        cdc_state = CDC_STATE_START_RECEPTION;
        rx_ready = 0U;
    }
}
