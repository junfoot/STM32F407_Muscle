/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbh_ch340.c
  * @brief   USB Host CH340/CH341 serial adapter class driver.
  *          This driver implements a small vendor-specific class used to talk
  *          to the CH340 USB-to-UART bridge inside the WT9011DCL-RF receiver.
  *          Received data is handed to the user via
  *          USBH_CH340_ReceiveCallback() where it can be forwarded to USART1.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbh_ch340.h"
#include "usbh_ioreq.h"

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
  * @brief CH340 host class implementation.
  * @{
  */

/** @defgroup USBH_CH340_CORE_Private_Defines
  * @{
  */
#define CH340_CLKRATE                    48000000U
/**
  * @}
  */

/** @defgroup USBH_CH340_CORE_Private_FunctionPrototypes
  * @{
  */
static USBH_StatusTypeDef USBH_CH340_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_CH340_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_CH340_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_CH340_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_CH340_SOFProcess(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef CH340_ControlOut(USBH_HandleTypeDef *phost,
                                           uint8_t request,
                                           uint16_t value,
                                           uint16_t index);

static USBH_StatusTypeDef CH340_ControlIn(USBH_HandleTypeDef *phost,
                                          uint8_t request,
                                          uint16_t value,
                                          uint16_t index,
                                          uint8_t *buf,
                                          uint16_t len);

static uint16_t CH340_CalcBaudValue(uint32_t baudrate);

static void CH340_ProcessTransmission(USBH_HandleTypeDef *phost);
static void CH340_ProcessReception(USBH_HandleTypeDef *phost);
/**
  * @}
  */

/** @defgroup USBH_CH340_CORE_Private_Variables
  * @{
  */
USBH_ClassTypeDef CH340_Class =
{
  "CH340",
  CH340_INTERFACE_CLASS,
  USBH_CH340_InterfaceInit,
  USBH_CH340_InterfaceDeInit,
  USBH_CH340_ClassRequest,
  USBH_CH340_Process,
  USBH_CH340_SOFProcess,
  NULL,
};
/**
  * @}
  */

/** @defgroup USBH_CH340_CORE_Private_Functions
  * @{
  */

/**
  * @brief  USBH_CH340_InterfaceInit
  *         Initialize the CH340 interface and open the bulk IN/OUT pipes.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CH340_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  uint8_t interface;
  uint8_t ep_ix;
  CH340_HandleTypeDef *CH340_Handle;
  USBH_InterfaceDescTypeDef *pif;
  USBH_EpDescTypeDef *pep;

  /* CH340 is a vendor-specific class. Use wildcards for SubClass/Protocol
     to tolerate small descriptor differences between CH340 variants. */
  interface = USBH_FindInterface(phost,
                                 CH340_INTERFACE_CLASS,
                                 0xFFU,
                                 0xFFU);

  if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES))
  {
    return USBH_FAIL;
  }

  pif = &phost->device.CfgDesc.Itf_Desc[interface];

  status = USBH_SelectInterface(phost, interface);
  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData = (CH340_HandleTypeDef *)USBH_malloc(sizeof(CH340_HandleTypeDef));
  CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if (CH340_Handle == NULL)
  {
    USBH_DbgLog("CH340: Cannot allocate memory for class handle.");
    return USBH_FAIL;
  }

  (void)USBH_memset(CH340_Handle, 0, sizeof(CH340_HandleTypeDef));

  /* Parse endpoints: keep only the bulk pair, ignore the interrupt endpoint. */
  pif = &phost->device.CfgDesc.Itf_Desc[interface];
  for (ep_ix = 0U; ep_ix < pif->bNumEndpoints; ep_ix++)
  {
    pep = &pif->Ep_Desc[ep_ix];

    if ((pep->bmAttributes & 0x03U) == USB_EP_TYPE_BULK)
    {
      if ((pep->bEndpointAddress & 0x80U) != 0U)
      {
        CH340_Handle->InEp = pep->bEndpointAddress;
        CH340_Handle->InEpSize = pep->wMaxPacketSize;
      }
      else
      {
        CH340_Handle->OutEp = pep->bEndpointAddress;
        CH340_Handle->OutEpSize = pep->wMaxPacketSize;
      }
    }
  }

  if ((CH340_Handle->InEp == 0U) || (CH340_Handle->OutEp == 0U))
  {
    (void)USBH_free(CH340_Handle);
    phost->pActiveClass->pData = NULL;
    return USBH_FAIL;
  }

  CH340_Handle->InPipe = USBH_AllocPipe(phost, CH340_Handle->InEp);
  CH340_Handle->OutPipe = USBH_AllocPipe(phost, CH340_Handle->OutEp);

  (void)USBH_OpenPipe(phost,
                      CH340_Handle->InPipe,
                      CH340_Handle->InEp,
                      phost->device.address,
                      phost->device.speed,
                      USB_EP_TYPE_BULK,
                      CH340_Handle->InEpSize);

  (void)USBH_OpenPipe(phost,
                      CH340_Handle->OutPipe,
                      CH340_Handle->OutEp,
                      phost->device.address,
                      phost->device.speed,
                      USB_EP_TYPE_BULK,
                      CH340_Handle->OutEpSize);

  (void)USBH_LL_SetToggle(phost, CH340_Handle->InPipe, 0U);
  (void)USBH_LL_SetToggle(phost, CH340_Handle->OutPipe, 0U);

  /* Fixed 460800 8N1 configuration for the WT9011DCL-RF receiver. */
  CH340_Handle->lcr = CH340_LCR_DEFAULT;
  CH340_Handle->control = CH340_MCR_DEFAULT;
  CH340_Handle->baud_value = CH340_CalcBaudValue(460800U);

  CH340_Handle->state = CH340_INIT_STATE;

  return USBH_OK;
}

/**
  * @brief  USBH_CH340_InterfaceDeInit
  *         De-initialize CH340 pipes and free the class handle.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CH340_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if (CH340_Handle != NULL)
  {
    if (CH340_Handle->InPipe != 0U)
    {
      (void)USBH_ClosePipe(phost, CH340_Handle->InPipe);
      (void)USBH_FreePipe(phost, CH340_Handle->InPipe);
      CH340_Handle->InPipe = 0U;
    }

    if (CH340_Handle->OutPipe != 0U)
    {
      (void)USBH_ClosePipe(phost, CH340_Handle->OutPipe);
      (void)USBH_FreePipe(phost, CH340_Handle->OutPipe);
      CH340_Handle->OutPipe = 0U;
    }

    (void)USBH_free(CH340_Handle);
    phost->pActiveClass->pData = NULL;
  }

  return USBH_OK;
}

/**
  * @brief  USBH_CH340_ClassRequest
  *         Issue class-specific requests. For CH340 all initialization is done
  *         in the background process, so just notify the core that we are ready.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CH340_ClassRequest(USBH_HandleTypeDef *phost)
{
  (void)phost;
  return USBH_OK;
}

/**
  * @brief  USBH_CH340_Process
  *         Background state machine: initialize CH340, then run bulk transfers.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CH340_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  USBH_StatusTypeDef req_status = USBH_OK;
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  switch (CH340_Handle->state)
  {
    case CH340_INIT_STATE:
      req_status = CH340_ControlOut(phost, CH340_REQ_SERIAL_INIT, 0U, 0U);
      if (req_status == USBH_OK)
      {
        CH340_Handle->state = CH340_SET_BAUD_STATE;
      }
      else if (req_status != USBH_BUSY)
      {
        CH340_Handle->state = CH340_ERROR_STATE;
      }
      break;

    case CH340_SET_BAUD_STATE:
      req_status = CH340_ControlOut(phost,
                                    CH340_REQ_WRITE_REG,
                                    0x1312U,
                                    CH340_Handle->baud_value);
      if (req_status == USBH_OK)
      {
        CH340_Handle->state = CH340_SET_LCR_STATE;
      }
      else if (req_status != USBH_BUSY)
      {
        CH340_Handle->state = CH340_ERROR_STATE;
      }
      break;

    case CH340_SET_LCR_STATE:
      req_status = CH340_ControlOut(phost,
                                    CH340_REQ_WRITE_REG,
                                    0x2518U,
                                    (uint16_t)CH340_Handle->lcr);
      if (req_status == USBH_OK)
      {
        CH340_Handle->state = CH340_SET_HANDSHAKE_STATE;
      }
      else if (req_status != USBH_BUSY)
      {
        CH340_Handle->state = CH340_ERROR_STATE;
      }
      break;

    case CH340_SET_HANDSHAKE_STATE:
      req_status = CH340_ControlOut(phost,
                                    CH340_REQ_MODEM_CTRL,
                                    (uint16_t)(~(uint16_t)CH340_Handle->control),
                                    0U);
      if (req_status == USBH_OK)
      {
        CH340_Handle->state = CH340_READY_STATE;
        USBH_CH340_ReadyCallback(phost);
      }
      else if (req_status != USBH_BUSY)
      {
        CH340_Handle->state = CH340_ERROR_STATE;
      }
      break;

    case CH340_READY_STATE:
      status = USBH_OK;
      break;

    case CH340_TRANSFER_STATE:
      CH340_ProcessTransmission(phost);
      CH340_ProcessReception(phost);
      break;

    case CH340_ERROR_STATE:
      req_status = USBH_ClrFeature(phost, 0x00U);
      if (req_status == USBH_OK)
      {
        CH340_Handle->state = CH340_INIT_STATE;
      }
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_CH340_SOFProcess
  *         SOF callback (unused).
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CH340_SOFProcess(USBH_HandleTypeDef *phost)
{
  (void)phost;
  return USBH_OK;
}

/**
  * @brief  CH340_ControlOut
  *         Send a vendor-specific control request with no data stage.
  * @param  phost: Host handle
  * @param  request: bRequest value
  * @param  value: wValue
  * @param  index: wIndex
  * @retval USBH Status
  */
static USBH_StatusTypeDef CH340_ControlOut(USBH_HandleTypeDef *phost,
                                           uint8_t request,
                                           uint16_t value,
                                           uint16_t index)
{
  phost->Control.setup.b.bmRequestType = USB_H2D |
                                         USB_REQ_TYPE_VENDOR |
                                         USB_REQ_RECIPIENT_DEVICE;

  phost->Control.setup.b.bRequest = request;
  phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = index;
  phost->Control.setup.b.wLength.w = 0U;

  return USBH_CtlReq(phost, NULL, 0U);
}

/**
  * @brief  CH340_ControlIn
  *         Send a vendor-specific control request with a device-to-host data stage.
  * @param  phost: Host handle
  * @param  request: bRequest value
  * @param  value: wValue
  * @param  index: wIndex
  * @param  buf: receive buffer
  * @param  len: receive length
  * @retval USBH Status
  */
static USBH_StatusTypeDef CH340_ControlIn(USBH_HandleTypeDef *phost,
                                          uint8_t request,
                                          uint16_t value,
                                          uint16_t index,
                                          uint8_t *buf,
                                          uint16_t len)
{
  phost->Control.setup.b.bmRequestType = USB_D2H |
                                         USB_REQ_TYPE_VENDOR |
                                         USB_REQ_RECIPIENT_DEVICE;

  phost->Control.setup.b.bRequest = request;
  phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = index;
  phost->Control.setup.b.wLength.w = len;

  return USBH_CtlReq(phost, buf, len);
}

/**
  * @brief  CH340_CalcBaudValue
  *         Compute the CH340 baud-rate register value for a requested baud rate.
  *         Algorithm derived from the Linux ch341 driver.
  * @param  baudrate: requested baud rate
  * @retval 16-bit value to be written via CH340_REQ_WRITE_REG
  */
static uint16_t CH340_CalcBaudValue(uint32_t baudrate)
{
  uint32_t clk = CH340_CLKRATE;
  uint32_t fact = 1U;
  uint32_t div;
  uint32_t clk_div;
  uint32_t min_rate;
  int ps;
  uint32_t rate1;
  uint32_t rate2;
  uint32_t rate_req;
  uint16_t value;

  /* Clamp to a sensible range to keep the math safe. */
  if (baudrate < 46U)
  {
    baudrate = 46U;
  }
  if (baudrate > 3000000U)
  {
    baudrate = 3000000U;
  }

  /* Find prescaler value ps (3..0). */
  for (ps = 3; ps >= 0; ps--)
  {
    min_rate = clk / ((1U << (12U - 3U * (uint32_t)ps - 1U)) * 512U);
    if (baudrate > min_rate)
    {
      break;
    }
  }
  if (ps < 0)
  {
    ps = 0;
  }

  clk_div = 1U << (12U - 3U * (uint32_t)ps - fact);
  div = clk / (clk_div * baudrate);

  /* Halve the base clock if the divisor is out of the valid range. */
  if ((div < 9U) || (div > 255U))
  {
    div /= 2U;
    clk_div *= 2U;
    fact = 0U;
  }

  if (div < 2U)
  {
    div = 2U;
  }

  /* Pick the divisor that gives the closest baud rate. */
  rate_req = 16U * baudrate;
  rate1 = (16U * clk) / (clk_div * div);
  rate2 = (16U * clk) / (clk_div * (div + 1U));

  if ((rate1 > rate_req) && ((rate1 - rate_req) >= (rate_req - rate2)))
  {
    div++;
  }

  value = (uint16_t)(((0x100U - div) << 8) |
                     (fact << 2) |
                     (uint32_t)ps);

  /* Bit 7 forces the CH340 to emit data as soon as it is received. */
  value |= 0x0080U;

  return value;
}

/**
  * @brief  USBH_CH340_Receive
  *         Start a bulk receive on the CH340 IN endpoint.
  * @param  phost: Host handle
  * @param  pbuff: receive buffer
  * @param  length: receive buffer length
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_CH340_Receive(USBH_HandleTypeDef *phost,
                                      uint8_t *pbuff,
                                      uint32_t length)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if ((CH340_Handle->state == CH340_READY_STATE) ||
      (CH340_Handle->state == CH340_TRANSFER_STATE))
  {
    if (CH340_Handle->rx_state == CH340_RX_IDLE)
    {
      CH340_Handle->pRxData = pbuff;
      CH340_Handle->RxDataLength = length;
      CH340_Handle->state = CH340_TRANSFER_STATE;
      CH340_Handle->rx_state = CH340_RX_RECV;
      status = USBH_OK;
    }
    else
    {
      status = USBH_BUSY;
    }
  }
  else if (CH340_Handle->state < CH340_READY_STATE)
  {
    status = USBH_BUSY;
  }
  else
  {
    status = USBH_FAIL;
  }

  return status;
}

/**
  * @brief  USBH_CH340_Transmit
  *         Start a bulk transmit on the CH340 OUT endpoint.
  * @param  phost: Host handle
  * @param  pbuff: transmit buffer
  * @param  length: transmit length
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_CH340_Transmit(USBH_HandleTypeDef *phost,
                                       uint8_t *pbuff,
                                       uint32_t length)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if ((CH340_Handle->state == CH340_READY_STATE) ||
      (CH340_Handle->state == CH340_TRANSFER_STATE))
  {
    if (CH340_Handle->tx_state == CH340_TX_IDLE)
    {
      CH340_Handle->pTxData = pbuff;
      CH340_Handle->TxDataLength = length;
      CH340_Handle->state = CH340_TRANSFER_STATE;
      CH340_Handle->tx_state = CH340_TX_SEND;
      status = USBH_OK;
    }
    else
    {
      status = USBH_BUSY;
    }
  }
  else if (CH340_Handle->state < CH340_READY_STATE)
  {
    status = USBH_BUSY;
  }
  else
  {
    status = USBH_FAIL;
  }

  return status;
}

/**
  * @brief  USBH_CH340_GetLastReceivedDataSize
  *         Return the size of the last completed bulk IN transfer.
  * @param  phost: Host handle
  * @retval Number of bytes received
  */
uint16_t USBH_CH340_GetLastReceivedDataSize(USBH_HandleTypeDef *phost)
{
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if (phost->gState == HOST_CLASS)
  {
    return (uint16_t)USBH_LL_GetLastXferSize(phost, CH340_Handle->InPipe);
  }

  return 0U;
}

/**
  * @brief  USBH_CH340_Stop
  *         Stop any active CH340 transfers.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_CH340_Stop(USBH_HandleTypeDef *phost)
{
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;

  if (phost->gState == HOST_CLASS)
  {
    CH340_Handle->state = CH340_READY_STATE;
    (void)USBH_ClosePipe(phost, CH340_Handle->InPipe);
    (void)USBH_ClosePipe(phost, CH340_Handle->OutPipe);
  }

  return USBH_OK;
}

/**
  * @brief  CH340_ProcessTransmission
  *         State machine for CH340 bulk OUT transfers.
  * @param  phost: Host handle
  * @retval None
  */
static void CH340_ProcessTransmission(USBH_HandleTypeDef *phost)
{
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;
  uint16_t len;

  switch (CH340_Handle->tx_state)
  {
    case CH340_TX_SEND:
      if (CH340_Handle->TxDataLength > CH340_Handle->OutEpSize)
      {
        len = CH340_Handle->OutEpSize;
      }
      else
      {
        len = (uint16_t)CH340_Handle->TxDataLength;
      }

      (void)USBH_BulkSendData(phost,
                              CH340_Handle->pTxData,
                              len,
                              CH340_Handle->OutPipe,
                              1U);

      CH340_Handle->tx_state = CH340_TX_WAIT;
      break;

    case CH340_TX_WAIT:
      URB_Status = USBH_LL_GetURBState(phost, CH340_Handle->OutPipe);

      if (URB_Status == USBH_URB_DONE)
      {
        if (CH340_Handle->TxDataLength > CH340_Handle->OutEpSize)
        {
          CH340_Handle->TxDataLength -= CH340_Handle->OutEpSize;
          CH340_Handle->pTxData += CH340_Handle->OutEpSize;
          CH340_Handle->tx_state = CH340_TX_SEND;
        }
        else
        {
          CH340_Handle->TxDataLength = 0U;
          CH340_Handle->tx_state = CH340_TX_IDLE;
          USBH_CH340_TransmitCallback(phost);
        }
      }
      else if (URB_Status == USBH_URB_NOTREADY)
      {
        CH340_Handle->tx_state = CH340_TX_SEND;
      }
      break;

    default:
      break;
  }
}

/**
  * @brief  CH340_ProcessReception
  *         State machine for CH340 bulk IN transfers.
  * @param  phost: Host handle
  * @retval None
  */
static void CH340_ProcessReception(USBH_HandleTypeDef *phost)
{
  CH340_HandleTypeDef *CH340_Handle = (CH340_HandleTypeDef *)phost->pActiveClass->pData;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;

  switch (CH340_Handle->rx_state)
  {
    case CH340_RX_RECV:
      (void)USBH_BulkReceiveData(phost,
                                 CH340_Handle->pRxData,
                                 (uint16_t)CH340_Handle->RxDataLength,
                                 CH340_Handle->InPipe);

      CH340_Handle->rx_state = CH340_RX_WAIT;
      break;

    case CH340_RX_WAIT:
      URB_Status = USBH_LL_GetURBState(phost, CH340_Handle->InPipe);

      if (URB_Status == USBH_URB_DONE)
      {
        CH340_Handle->rx_state = CH340_RX_IDLE;
        USBH_CH340_ReceiveCallback(phost);
      }
      else if (URB_Status == USBH_URB_ERROR)
      {
        CH340_Handle->rx_state = CH340_RX_IDLE;
        CH340_Handle->state = CH340_ERROR_STATE;
      }
      break;

    default:
      break;
  }
}

/**
  * @brief  USBH_CH340_ReceiveCallback
  *         Weak callback called when a bulk IN transfer completes.
  * @param  phost: Host handle
  * @retval None
  */
__weak void USBH_CH340_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  (void)phost;
}

/**
  * @brief  USBH_CH340_TransmitCallback
  *         Weak callback called when a bulk OUT transfer completes.
  * @param  phost: Host handle
  * @retval None
  */
__weak void USBH_CH340_TransmitCallback(USBH_HandleTypeDef *phost)
{
  (void)phost;
}

/**
  * @brief  USBH_CH340_ReadyCallback
  *         Weak callback called when the CH340 initialization is complete.
  * @param  phost: Host handle
  * @retval None
  */
__weak void USBH_CH340_ReadyCallback(USBH_HandleTypeDef *phost)
{
  (void)phost;
}

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
