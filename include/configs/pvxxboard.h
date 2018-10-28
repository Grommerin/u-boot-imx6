/*
 * Copyright (C) 2014 Eukréa Electromatique
 * Author: Eric Bénard <eric@eukrea.com>
 *
 * Configuration settings for the Strim PVxx boards
 *
 * based on mx6*sabre*.h which are :
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2018 LLC Strim.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __STRIMPVXXBOARD_CONFIG_H
#define __STRIMPVXXBOARD_CONFIG_H

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART2_BASE

#define PHYS_SDRAM_SIZE		        (1u * 1024 * 1024 * 1024)

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_PHYLIB

#define CONFIG_ARP_TIMEOUT              200UL

#define CONFIG_MXC_SPI
#define CONFIG_CMD_SPI
#define CONFIG_CMD_DATE
#ifdef  CONFIG_CMD_DATE
#define CONFIG_RTC_DS1390

#define CONFIG_DEFAULT_SPI_MODE     SPI_MODE_3
#define CONFIG_SYS_SPI_RTC_BUS      0
#define CONFIG_SYS_SPI_RTC_DEVID    1
#define CONFIG_SYS_SPI_RTC_SPEED    4000000
#endif

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			        (1 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_FDTFILE                  "imx6q-strimpvxx.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM        2
#define CONFIG_SYS_MMC_ENV_DEV          0   /* SDHC4 */
#define CONFIG_ENV_OFFSET               (6 * 64 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT        /* eMMC specific */
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_FDTFILE                  "imx6q-strimpvxx.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM        0
#define CONFIG_ENV_OFFSET               (768 * 1024)
#define CONFIG_ENV_SECT_SIZE            (8 * 1024)
#define CONFIG_ENV_SPI_BUS              CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS               CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE             CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ           CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_VIDEO_IPUV3

#define CONFIG_IPUV3_CLK                260000000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_ENV_VARS_UBOOT_CONFIG

#include "mx6_common.h"

#undef CONFIG_SYS_LONGHELP
#undef CONFIG_FRAMEBUFFER_CONSOLE
#undef CONFIG_FRAMEBUFFER_CONSOLE_DETECT_PRIMARY
#undef CONFIG_FONTS

/* 256M RAM (minimum), 32M uncompressed kernel, 16M compressed kernel, 1M fdt,
 * 1M script, 1M pxe and the ramdisk at the end */

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootargs=console=ttymxc1,115200 " \
    "video=mxcfb0:dev=ldb," \
    "1280x800M@60,if=RGB666 " \
    "root=/dev/mmcblk3p2 rootwait rw, rootfstype=ext4\0" \
    "mmcdev=0\0" \
    "mmcpart=1\0" \
    "imgfile=uImage\0" \
    "fdtfile=imx6q-strimpvxx.dtb\0" \
    "uimgaddr=0x10800000\0" \
    "bootcmd=run boot\0" \
    "loadcmd=ext4load\0" \
    "boot=mmc rescan; " \
    "${loadcmd} mmc ${mmcdev}:${mmcpart} ${uimgaddr} /${imgfile}; " \
    "${loadcmd} mmc ${mmcdev}:${mmcpart} ${loadaddr} /${fdtfile};" \
    "bootm ${uimgaddr} - ${loadaddr}\0"


#endif /* __STRIMPVXXBOARD_CONFIG_H */
