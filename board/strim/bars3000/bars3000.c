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
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
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

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |        \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |     \
    PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)


struct led_parms {
    unsigned num;   /* GPIO Number */
    unsigned v_on;  /* Active Value */
    unsigned v_off; /* Inactive Value */
    unsigned v_def; /* Default Value */
};
typedef struct led_parms led_cfg_t;

struct button_key {
    char const  *name;
    unsigned    gpnum;
    char        ident;
};
typedef struct button_key btn_cfg_t;


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

int board_phy_config(struct phy_device *phydev)
{
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
     * Strim Bars3000 :
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

iomux_v3_cfg_t const led_pads[] = {
    MX6_PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* LED_Power/Work */
    MX6_PAD_SD3_RST__GPIO7_IO08   | MUX_PAD_CTRL(NO_PAD_CTRL),      /* LED_GPS/GPRS */
    MX6_PAD_SD3_DAT4__GPIO7_IO01  | MUX_PAD_CTRL(NO_PAD_CTRL),      /* LED_WiFi */
    MX6_PAD_CSI0_DAT6__GPIO5_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* LED_CAN1 */
    MX6_PAD_CSI0_DAT5__GPIO5_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* LED_CAN2 */
};

iomux_v3_cfg_t const button_pads[] = {
    MX6_PAD_SD3_DAT1__GPIO7_IO05  | MUX_PAD_CTRL(BUTTON_PAD_CTRL),   /* Func */
};

led_cfg_t const led_gpios[] = {
    {IMX_GPIO_NR(1, 27), 1,  0,  1},
    {IMX_GPIO_NR(7, 8),  0,  1,  1},
    {IMX_GPIO_NR(7, 1),  1,  0,  0},
    {IMX_GPIO_NR(5, 24), 1,  0,  0},
    {IMX_GPIO_NR(5, 23), 1,  0,  0},
};

btn_cfg_t const buttons[] = {
    {"Func", IMX_GPIO_NR(7, 5),  'f'},
};


#define LEDS_CODE_ALL_ON        (0x1F)
#define LEDS_CODE_KERNEL        (0x07)
#define LEDS_CODE_DTB           (0x19)

#define LEDS_NO_TO_CODE(no)     (0x01 << (no & 0x7))
#define LEDS_CHR_TO_NUM(chr)    (chr & 0x0f)


static void setup_buttons(void)
{ imx_iomux_v3_setup_multiple_pads(button_pads, ARRAY_SIZE(button_pads)); }

static void setup_leds(void) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));
    for (i = 0; i < count; i++) {
        gpio_direction_output(led_gpios[i].num, led_gpios[i].v_off);
    }
}

static void leds_on(void) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    for (i = 0; i < count; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_on);
    }
}

static void leds_off(void) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    for (i = 0; i < count; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_off);
    }
}

static void leds_def(void) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    for (i = 0; i < count; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_def);
    }
}

static void leds_toggle(unsigned code) {
    int i, value, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    for (i = 0; i < count; i++) {
        if ((code & (1 << i)) != 0) {
            value = gpio_get_value(led_gpios[i].num);
            value = !value & 0x01;
            gpio_set_value(led_gpios[i].num, value);
        }
    }
}

static void leds_set(unsigned code) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    if (count > 8) { count = 8; }
    for (i = 0; i < count; i++) {
        if ((code & (1 << i)) != 0) {
            gpio_set_value(led_gpios[i].num, led_gpios[i].v_on);
        }
    }
}

static void leds_clr(unsigned code) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    for (i = 0; i < count; i++) {
        if ((code & (1 << i)) != 0) {
            gpio_set_value(led_gpios[i].num, led_gpios[i].v_off);
        }
    }
}


static char leds_get_code(void) {
    int i, value, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    char code = 0;
    for (i = 0; i < count; i++) {
        value = gpio_get_value(led_gpios[i].num);
        if (value == led_gpios[i].v_on) { code |= 1 << i; }
    }
    return (code);
}

static void leds_pulse(unsigned code, unsigned pulse_num, unsigned delay_time) {
    int i, count_pulse = pulse_num * 2;
    char state;
    if (pulse_num == 0) { return; }
    state = leds_get_code();
    leds_clr(code);
    for (i = 0; i < count_pulse; i++) {
        udelay(delay_time);
        leds_toggle(code);
    }
    leds_def();
    leds_set(state);
}

static void leds_run(unsigned delay_time) {
    int i, count = (sizeof(led_gpios) / sizeof(led_gpios[0]));
    char code = 0x01;
    leds_off();
    for (i = 0; i < count; i++) {
        leds_set(code);
        code |= (code << 1);
        udelay(delay_time);
    }
}

static int do_ledsw(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    unsigned cmd_vn = 0;
    unsigned def_pulses = 5;
    unsigned def_run_mul = 5;
    const char *cmd_v = NULL;
    const char *str_cmd_p = NULL;
    const char err_kernel[2] = "k\0";
    const char err_dtb[2] = "d\0";

    if ((argc < 2) || (argc > 3)) { return (CMD_RET_USAGE); }
    str_cmd_p = argv[1];
    if (argc == 3) { cmd_v = argv[2]; }
    /* parse the behavior */
    switch (*str_cmd_p) {
        case 's': {
            if (cmd_v == NULL) { leds_on(); }
            else {
                cmd_vn = LEDS_CHR_TO_NUM(*cmd_v);
                leds_set(LEDS_NO_TO_CODE(cmd_vn));
            }
            break;
        }
        case 'c': {
            if (cmd_v == NULL) { leds_def(); }
            else {
                cmd_vn = LEDS_CHR_TO_NUM(*cmd_v);
                leds_clr(LEDS_NO_TO_CODE(cmd_vn));
            }
            break;
        }
        case 't': {
            if (cmd_v == NULL) { leds_toggle(LEDS_CODE_ALL_ON); }
            else {
                cmd_vn = LEDS_CHR_TO_NUM(*cmd_v);
                leds_toggle(LEDS_NO_TO_CODE(cmd_vn));
            }
            break;
        }
        case 'p': {
            if (cmd_v == NULL) { cmd_vn = def_pulses; }
            else { cmd_vn = LEDS_CHR_TO_NUM(*cmd_v); }
            leds_pulse(LEDS_CODE_ALL_ON, cmd_vn, CONFIG_LEDS_PULSE_US);
            break;
        }
        case 'r': {
            if (cmd_v == NULL) { cmd_vn = def_run_mul; }
            else { cmd_vn = LEDS_CHR_TO_NUM(*cmd_v); }
            leds_run(CONFIG_LEDS_PULSE_US * cmd_vn);
            break;
        }
        case 'e': {
            if (cmd_v == NULL) { return (CMD_RET_USAGE); }
            if (strcmp(cmd_v, err_kernel) == 0) {
                leds_pulse(LEDS_CODE_KERNEL, 3, CONFIG_LEDS_PULSE_US * 2);
                leds_set(LEDS_CODE_KERNEL);
            } else if (strcmp(cmd_v, err_dtb) == 0) {
                leds_pulse(LEDS_CODE_DTB, 3, CONFIG_LEDS_PULSE_US * 2);
                leds_set(LEDS_CODE_DTB);
            } else { return (CMD_RET_USAGE); }
            break;
        }
        default: { return (CMD_RET_USAGE); }
    }
    return (0);
}


/***************************************************/

U_BOOT_CMD(
    ledsw, 3, 1,  do_ledsw,
    "set/clear/toggle/pulse/err/run board leds",
    "<s|c|t|p|e|r>\n"
        "    - set/clear/toggle/pulse/err/run board leds"
);


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
    return ((cs == 0) ? IMX_GPIO_NR(4, 10) :
           ((cs == 1) ? IMX_GPIO_NR(2, 30) : -1));
}

static void setup_spi(void)
{
    gpio_direction_output(IMX_GPIO_NR(4, 10), 1);
    gpio_direction_output(IMX_GPIO_NR(2, 30), 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));


}
#endif
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
    setup_leds();
    setup_buttons();

    leds_on();

	setup_iomux_uart();

	return (0);
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
	rtc_run();
#endif
	return (0);
}

#ifdef CONFIG_CMD_BMODE

static const struct boot_mode strimbars3000_boot_modes[] = {
    {"emmc", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
    {"sd2",  MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(strimbars3000_boot_modes);
#endif

	return (0);
}

int checkboard(void)
{
	puts("Board: Bars3000\n");

	return (0);
}
