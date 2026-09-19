/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   FatFs disk I/O glue for the on-board SD card. Block transfers use
  *          DMA so the 2 kHz AD7606 ISR cannot starve the SDIO FIFO; status
  *          checks, ready waits, aborts and retries remain time-bounded.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sdio.h"
#include "ff.h"
#include "diskio.h"

/* Private define ------------------------------------------------------------*/
#define SD_TIMEOUT       5000U
#define SD_RETRY_COUNT   3U

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT;

static DSTATUS SD_CheckStatus(void)
{
  if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
  {
    Stat &= (DSTATUS)~STA_NOINIT;
  }
  else
  {
    Stat |= STA_NOINIT;
  }
  return Stat;
}

static uint8_t SD_WaitTransfer(uint32_t timeout)
{
  uint32_t start = HAL_GetTick();

  while (hsd.State != HAL_SD_STATE_READY)
  {
    if ((HAL_GetTick() - start) >= timeout)
    {
      return 0U;
    }
  }

  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - start) >= timeout)
    {
      return 0U;
    }
  }
  return (hsd.ErrorCode == HAL_SD_ERROR_NONE) ? 1U : 0U;
}

/* Private functions ---------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  return SD_CheckStatus();
}

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  return SD_CheckStatus();
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  uint32_t attempt;

  if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  for (attempt = 0U; attempt < SD_RETRY_COUNT; attempt++)
  {
    if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)buff, sector, count) == HAL_OK)
    {
      if (SD_WaitTransfer(SD_TIMEOUT) != 0U)
      {
        return RES_OK;
      }
    }
    (void)HAL_SD_Abort(&hsd);
    HAL_Delay(2U);
  }
  return RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  uint32_t attempt;

  if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  for (attempt = 0U; attempt < SD_RETRY_COUNT; attempt++)
  {
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)buff, sector, count) == HAL_OK)
    {
      if (SD_WaitTransfer(SD_TIMEOUT) != 0U)
      {
        return RES_OK;
      }
    }
    (void)HAL_SD_Abort(&hsd);
    HAL_Delay(2U);
  }
  return RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  if (pdrv != 0U)
  {
    return RES_PARERR;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      return (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) ? RES_OK : RES_ERROR;

    case GET_SECTOR_COUNT:
      *(DWORD *)buff = hsd.SdCard.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512U;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = hsd.SdCard.LogBlockSize / 512U;
      if (*(DWORD *)buff == 0U)
      {
        *(DWORD *)buff = 1U;
      }
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
