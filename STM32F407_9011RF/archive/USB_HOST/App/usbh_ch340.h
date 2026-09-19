/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbh_ch340.h
  * @brief   USB Host CH340/CH341 serial adapter class driver header.
  *          This is a minimal vendor-class driver used to receive data from the
  *          WT9011DCL-RF adapter (CH340 USB-to-UART) and forward it over USART1.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBH_CH340_H
#define __USBH_CH340_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"

/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_CH340_CLASS
  * @{
  */

/** @defgroup USBH_CH340_CORE
  * @brief CH340 host class public definitions.
  * @{
  */

/* CH340 interface descriptor constants */
#define CH340_INTERFACE_CLASS            0xFFU
#define CH340_INTERFACE_SUBCLASS         0x01U
#define CH340_INTERFACE_PROTOCOL         0x02U

/* CH340 vendor requests */
#define CH340_REQ_WRITE_REG              0x9AU
#define CH340_REQ_MODEM_CTRL             0xA4U
#define CH340_REQ_SERIAL_INIT            0xA1U

/* CH340 register / line-control values (8N1, TX/RX enabled) */
#define CH340_LCR_ENABLE_RX              0x80U
#define CH340_LCR_ENABLE_TX              0x40U
#define CH340_LCR_CS8                    0x03U
#define CH340_LCR_DEFAULT                (CH340_LCR_ENABLE_RX | \
                                          CH340_LCR_ENABLE_TX | \
                                          CH340_LCR_CS8)

/* CH340 modem control (DTR + RTS) */
#define CH340_MCR_RTS                    0x40U
#define CH340_MCR_DTR                    0x20U
#define CH340_MCR_DEFAULT                (CH340_MCR_RTS | CH340_MCR_DTR)

/**
  * @brief CH340 driver state machine states.
  */
typedef enum
{
  CH340_INIT_STATE = 0U,
  CH340_SET_BAUD_STATE,
  CH340_SET_LCR_STATE,
  CH340_SET_HANDSHAKE_STATE,
  CH340_READY_STATE,
  CH340_TRANSFER_STATE,
  CH340_ERROR_STATE
} CH340_StateTypeDef;

/**
  * @brief CH340 transmit sub-state.
  */
typedef enum
{
  CH340_TX_IDLE = 0U,
  CH340_TX_SEND,
  CH340_TX_WAIT
} CH340_TxStateTypeDef;

/**
  * @brief CH340 receive sub-state.
  */
typedef enum
{
  CH340_RX_IDLE = 0U,
  CH340_RX_RECV,
  CH340_RX_WAIT
} CH340_RxStateTypeDef;

/**
  * @brief CH340 class handle structure.
  */
typedef struct
{
  uint8_t              InPipe;
  uint8_t              OutPipe;
  uint8_t              InEp;
  uint8_t              OutEp;
  uint16_t             InEpSize;
  uint16_t             OutEpSize;
  uint8_t             *pTxData;
  uint8_t             *pRxData;
  uint32_t             TxDataLength;
  uint32_t             RxDataLength;
  uint16_t             baud_value;
  uint8_t              lcr;
  uint8_t              control;
  CH340_StateTypeDef   state;
  CH340_TxStateTypeDef tx_state;
  CH340_RxStateTypeDef rx_state;
} CH340_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBH_CH340_CORE_Exported_Variables
  * @{
  */
extern USBH_ClassTypeDef CH340_Class;
#define USBH_CH340_CLASS                 &CH340_Class
/**
  * @}
  */

/** @defgroup USBH_CH340_CORE_Exported_FunctionsPrototype
  * @{
  */
USBH_StatusTypeDef USBH_CH340_Receive(USBH_HandleTypeDef *phost,
                                      uint8_t *pbuff,
                                      uint32_t length);

USBH_StatusTypeDef USBH_CH340_Transmit(USBH_HandleTypeDef *phost,
                                       uint8_t *pbuff,
                                       uint32_t length);

uint16_t           USBH_CH340_GetLastReceivedDataSize(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef USBH_CH340_Stop(USBH_HandleTypeDef *phost);

void USBH_CH340_ReceiveCallback(USBH_HandleTypeDef *phost);
void USBH_CH340_TransmitCallback(USBH_HandleTypeDef *phost);
void USBH_CH340_ReadyCallback(USBH_HandleTypeDef *phost);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CH340_H */
