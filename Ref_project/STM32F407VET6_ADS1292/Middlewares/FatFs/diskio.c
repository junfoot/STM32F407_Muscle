#include "diskio.h"
#include "sdio.h"

#define SD_TRANSFER_TIMEOUT_MS  30000U
#define SD_IO_RETRY_COUNT       3U

static volatile DSTATUS sd_status = STA_NOINIT;

static DSTATUS SD_CheckStatus(void)
{
    if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
    {
        sd_status &= (DSTATUS)~STA_NOINIT;
    }
    else
    {
        sd_status |= STA_NOINIT;
    }
    return sd_status;
}

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
    return (pdrv == 0U) ? SD_CheckStatus() : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    uint32_t attempt;
    if ((pdrv != 0U) || (buff == 0) || (count == 0U))
    {
        return RES_PARERR;
    }
    for (attempt = 0U; attempt < SD_IO_RETRY_COUNT; attempt++)
    {
        uint32_t start;
        if (HAL_SD_ReadBlocks(&hsd, buff, sector, count, SD_TRANSFER_TIMEOUT_MS) == HAL_OK)
        {
            start = HAL_GetTick();
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
            {
                if ((HAL_GetTick() - start) >= SD_TRANSFER_TIMEOUT_MS) break;
            }
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) return RES_OK;
        }
        (void)HAL_SD_Abort(&hsd);
        HAL_Delay(2U);
    }
    return RES_ERROR;
}

#if _USE_WRITE == 1
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    uint32_t attempt;
    if ((pdrv != 0U) || (buff == 0) || (count == 0U))
    {
        return RES_PARERR;
    }
    for (attempt = 0U; attempt < SD_IO_RETRY_COUNT; attempt++)
    {
        uint32_t start;
        if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, sector, count,
                               SD_TRANSFER_TIMEOUT_MS) == HAL_OK)
        {
            start = HAL_GetTick();
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
            {
                if ((HAL_GetTick() - start) >= SD_TRANSFER_TIMEOUT_MS) break;
            }
            if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) return RES_OK;
        }
        (void)HAL_SD_Abort(&hsd);
        HAL_Delay(2U);
    }
    return RES_ERROR;
}
#endif

#if _USE_IOCTL == 1
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    HAL_SD_CardInfoTypeDef info;
    if ((pdrv != 0U) || (sd_status & STA_NOINIT))
    {
        return RES_NOTRDY;
    }
    switch (cmd)
    {
        case CTRL_SYNC:
            return (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) ? RES_OK : RES_ERROR;
        case GET_SECTOR_COUNT:
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = info.LogBlockNbr;
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512U;
            return RES_OK;
        case GET_BLOCK_SIZE:
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = (info.LogBlockSize / 512U);
            if (*(DWORD *)buff == 0U) *(DWORD *)buff = 1U;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
#endif
