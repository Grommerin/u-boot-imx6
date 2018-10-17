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

#ifndef __STRIMBARS3000_CONFIG_H
#define __STRIMBARS3000_CONFIG_H

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		   UART2_BASE

#define PHYS_SDRAM_SIZE		           (1u * 1024 * 1024 * 1024)

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		   (10 * SZ_1M)

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_PHYLIB

#define CONFIG_ARP_TIMEOUT             200UL

#ifndef CONFIG_LEDS_COUNT
#define CONFIG_LEDS_COUNT              5
#endif
#ifndef CONFIG_LEDS_PULSE_US
#define CONFIG_LEDS_PULSE_US           60000
#endif
#ifndef CONFIG_LEDS_DELAY_MS
#define CONFIG_LEDS_DELAY_MS           500
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
#define CONFIG_ENV_SIZE			        (3 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_FDTFILE                  "imx6q-bars3000.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM        2
#define CONFIG_SYS_MMC_ENV_DEV          0   /* SDHC4 */
#define CONFIG_ENV_OFFSET               (512 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT        /* eMMC specific */
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_FDTFILE                  "imx6q-bars3000.dtb"
#define CONFIG_SYS_FSL_USDHC_NUM        0
#define CONFIG_ENV_OFFSET               (768 * 1024)
#define CONFIG_ENV_SECT_SIZE            (8 * 1024)
#define CONFIG_ENV_SPI_BUS              CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS               CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE             CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ           CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_ENV_VARS_UBOOT_CONFIG

#include "mx6_common.h"

#undef CONFIG_SYS_LONGHELP


/*
#define CONFIG_EXTRA_ENV_SETTINGS \
    "bootargs=console=ttymxc1,115200 root=/\0" \
    "bootnorm=led c; led p 2; bootm 0x12000000 0x12C00000 0x18000000\0" \
    "bootcmd=run boot\0" \
    "boot=run bootnorm\0" \

*/

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootargs=console=ttymxc1,115200 root=/dev/mmcblk3p1 rw quiet rootfstype=ext4\0" \
	"bootargs_r=console=ttymxc1,115200 init=/sbin/init " \
	           "root=/dev/mmcblk1p2 rootwait rw rootfstype=ext4\0" \
	"mmc_upd=1\0" \
    "uimgaddr=0x10800000\0" \
    "load_ftd=mmc rescan; mmc dev 0; mmc read ${loadaddr} 0x410 0x100\0" \
    "load_uimg=mmc rescan; mmc dev 0; mmc read ${uimgaddr} 0x510 0x4af0\0" \
    "load_kernel=mmc rescan; mmc dev 0; mmc read 0x10800000 0x410 0x4bf0; " \
                "setenv loadaddr 0x10800000; setenv uimgaddr 0x10820000\0" \
    "load_uimg_r=fatload mmc 1:1 ${uimgaddr} /uImage\0" \
    "load_ftd_r=fatload mmc 1:1 ${loadaddr} /imx6q-bars3000.dtb\0" \
    "load_ftd_err=led e d; delay; run load_ftd_err\0" \
    "load_uimg_err=led e k; delay; run load_uimg_err\0" \
    "restcmd=echo Restore u-boot from DRAM; led s; " \
            "mw 0x177fb00c 0x177fb02c; " \
            "mmc write 0x177fb000 0x0002 0x27e; led p 2" \
            "echo U-boot restored; led c\0" \
    "bootnorm=led c; " \
             "if run load_uimg ; then " \
                "if run load_ftd; then " \
                   "led p 1; bootm ${uimgaddr} - ${loadaddr}; " \
                "else run load_uimg_err; fi; " \
             "else run load_ftd_err; fi\0" \
    "boottest=if run load_kernel ; then " \
                "led p 1; bootm ${uimgaddr} - ${loadaddr}; " \
             "else run load_uimg_err; fi\0" \
    "bootcmd=run boot\0" \
    "boot=run bootnorm\0" \
    "bootr=mmc rescan; setenv bootargs ${bootargs_r}; led c; run load_uimg_r; " \
          "run load_ftd_r; led p 1; bootm ${uimgaddr} - ${loadaddr}\0" \
    "bootcmd_mfg=setenv boot echo \"Run console\"\0" \
    "keycmd_f=led r 5; if bkey; then setenv boot echo \"Run console\"; fi\0"

/*
    "boot=bootm 0x10800000 0x10C00000 0x107FFF00\0" \
    "bootcmd_mfg=run boot\0"
 */


#endif /* __STRIMBARS3000_CONFIG_H */
