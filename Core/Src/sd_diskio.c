/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   FatFs disk I/O glue for the on-board SD card, polling mode on
  *          top of HAL_SD. Writes may block for a few ms; the recorder ring
  *          buffer (recorder.c) absorbs this so no sample is lost.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sdio.h"
#include "ff.h"
#include "diskio.h"

/* Private define ------------------------------------------------------------*/
#define SD_TIMEOUT   30000U   /* ms */

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT;

/* Private functions ---------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  Stat = STA_NOINIT;

  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    return Stat;
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    return Stat;
  }

  Stat &= ~STA_NOINIT;
  return Stat;
}

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
  {
    Stat &= ~STA_NOINIT;
  }
  else
  {
    Stat |= STA_NOINIT;
  }
  return Stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != 0U) || (count == 0U))
  {
    return RES_PARERR;
  }

  if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)buff, sector, count, SD_TIMEOUT) != HAL_OK)
  {
    return RES_ERROR;
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != 0U) || (count == 0U))
  {
    return RES_PARERR;
  }

  if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, sector, count, SD_TIMEOUT) != HAL_OK)
  {
    return RES_ERROR;
  }
  return RES_OK;
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
      return RES_OK;

    case GET_SECTOR_COUNT:
      *(DWORD *)buff = hsd.SdCard.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512U;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = hsd.SdCard.LogBlockSize / 512U;
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
