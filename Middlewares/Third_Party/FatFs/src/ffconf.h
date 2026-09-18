/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ffconf.h
  * @brief   FatFs R0.12c configuration for the EMG/IMU SD recorder
  *          (single SD volume, 8.3 names, no RTC timestamp).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef _FFCONF
#define _FFCONF 68300	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0
#define _FS_MINIMIZE	0
#define _USE_STRFUNC	0
#define _USE_FIND		0
#define _USE_MKFS		0
#define _USE_FASTSEEK	0
#define _USE_EXPAND		0
#define _USE_CHMOD		0
#define _USE_LABEL		0
#define _USE_FORWARD	0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	437
#define _USE_LFN	0
#define _MAX_LFN	255
#define _LFN_UNICODE	0
#define _STRF_ENCODE	3
#define _FS_RPATH	0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define _VOLUMES	1
#define _STR_VOLUME_ID	0
#define _VOLUME_STRS	"SD"
#define _MULTI_PARTITION	0
#define _MIN_SS		512
#define _MAX_SS		512
#define _USE_TRIM	0
#define _FS_NOFSINFO	0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define _FS_TINY	0
#define _FS_EXFAT	0

#define _FS_NORTC	1
#define _NORTC_MON	1
#define _NORTC_MDAY	1
#define _NORTC_YEAR	2026

#define _FS_LOCK	0
#define _FS_REENTRANT	0

#if _FS_REENTRANT
#define _FS_TIMEOUT		1000
#define _SYNC_t			NULL
#endif

#endif /* _FFCONF */
