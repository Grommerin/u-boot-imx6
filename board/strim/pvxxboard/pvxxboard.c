/*
 * Copyright (C) 2014 Eukréa Electromatique
 * Author: Eric Bénard <eric@eukrea.com>
 *         Fabio Estevam <fabio.estevam@freescale.com>
 *         Jon Nettleton <jon.nettleton@gmail.com>
 *         Nikita Divakov <n.divakov@strim-tech.com>
 *
 * based on sabresd.c which is :
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * and on hummingboard.c which is :
 * Copyright (C) 2013 SolidRun ltd.
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>.
 * Copyright (C) 2018 Nikita Divakov <n.divakov@strim-tech.com>.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
//#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
//#include <i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
//#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <rtc.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CLK_CTRL (PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST |			\
	PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)


int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return (0);
}

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}


//int mx6_rgmii_rework(struct phy_device *phydev)
//{
//	/* from linux/arch/arm/mach-imx/mach-imx6q.c :
//	 * Ar803x phy SmartEEE feature cause link status generates glitch,
//	 * which cause ethernet link down/up issue, so disable SmartEEE
//	 */
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x805d);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4003);
//
//	return (0);
//}

int board_phy_config(struct phy_device *phydev)
{
//	mx6_rgmii_rework(phydev);
//
//	if (phydev->drv->config) { phydev->drv->config(phydev); }

	return (0);
}

iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
	MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK | MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)
};

#ifdef CONFIG_FSL_ESDHC
#define USDHC2_CD_GPIO  IMX_GPIO_NR(1, 4)

struct fsl_esdhc_cfg usdhc_cfg[2] = {
    {.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 8},
    {.esdhc_base = USDHC2_BASE_ADDR, .bus_width = 4, .gp_cd = USDHC2_CD_GPIO},
};

int board_mmc_getcd(struct mmc *mmc)
{
    int ret = 0;
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
	    gpio_direction_input(USDHC2_CD_GPIO);
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
			ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return (ret);
}

int board_mmc_init(bd_t *bis)
{
	int ret = 0;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * StrimPVxx :
	 * mmc0                    uSDCard slot (bottom)
	 * mmc1                    eMMC
	 */
    imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

    ret = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
    if (ret) { return (ret); }
    ret = fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
    if (ret) { return (ret); }

	return (0);
}
#endif

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(SPI_PAD_CTRL),  /* RTC CS */
	MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(SPI_PAD_CTRL),   /* ADC CS */
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
    if ((bus != 0) || (cs > 1)) { return (-1); }
    return ((cs == 0) ? IMX_GPIO_NR(2, 30) :
           ((cs == 1) ? IMX_GPIO_NR(4, 10) : -1));
}

static void setup_spi(void)
{
    gpio_direction_output(IMX_GPIO_NR(2, 30), 1);
    gpio_direction_output(IMX_GPIO_NR(4, 10), 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));


}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	setbits_le32(&iomux->gpr[2], IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT);
	/* set backlight level to ON */
	gpio_direction_output(IMX_GPIO_NR(2, 10) , 1);
}

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* set backlight level to OFF */
	gpio_direction_output(IMX_GPIO_NR(2, 10) , 0);

	clrbits_le32(&iomux->gpr[2], IOMUXC_GPR2_LVDS_CH0_MODE_MASK);
}

//static void do_enable_hdmi(struct display_info_t const *dev)
//{
//	disable_lvds(dev);
//	imx_enable_hdmi_phy();
//}
//
//static int detect_i2c(struct display_info_t const *dev)
//{
//	return ((0 == i2c_set_bus_num(dev->bus)) && (0 == i2c_probe(dev->addr)));
//}

struct display_info_t const displays[] = {{
    .bus    = 0,
    .addr   = 0x0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode   = {
        .name           = "Mitsubishi-AA121TH11",
        .refresh        = 60,
        .xres           = 1280,
        .yres           = 800,
        .left_margin    = 20,
        .right_margin   = 70,
        .upper_margin   = 3,
        .lower_margin   = 15,
        .hsync_len      = 70,
        .vsync_len      = 5,
        .sync           = FB_SYNC_EXT,
        .vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
//	imx_setup_hdmi();

	/* Turn on LDB0, IPU,IPU DI0 clocks */
	setbits_le32(&mxc_ccm->CCGR3, MXC_CCM_CCGR3_LDB_DI0_MASK);

	/* set LDB0 clk select to 011/011 */
	clrsetbits_le32(&mxc_ccm->cs2cdr, MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK,
	                            (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET));

	setbits_le32(&mxc_ccm->cscmr2, MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV);

	setbits_le32(&mxc_ccm->chsccdr,
		     (CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET));

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	clrsetbits_le32(&iomux->gpr[3],
			IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
			IOMUXC_GPR3_HDMI_MUX_CTL_MASK,
			IOMUXC_GPR3_MUX_SRC_IPU1_DI0
			<< IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return (1);
}

int board_eth_init(bd_t *bis)
{
	return (0);
}

int board_early_init_f(void)
{
	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	/* power ON LCD */
	gpio_direction_output(IMX_GPIO_NR(1, 29) , 1);
	/* touch interrupt is an input */
	gpio_direction_input(IMX_GPIO_NR(6, 14));
	/* power ON backlight */
	gpio_direction_output(IMX_GPIO_NR(6, 15) , 1);
	/* set backlight level to off */
	gpio_direction_output(IMX_GPIO_NR(2, 10) , 0);
	setup_display();
#endif

	return (0);
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	return (0);
}

#ifdef CONFIG_CMD_BMODE

static const struct boot_mode strimpvxx_boot_modes[] = {
    {"emmc", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
    {"sd2",  MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
    rtc_run();

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(strimpvxx_boot_modes);
#endif

	return (0);
}

int checkboard(void)
{
	puts("Board: StrimPVxx\n");

	return (0);
}
