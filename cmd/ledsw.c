/**
 * @file ledsw.c
 * @brief 
 * @date 13 апр. 2018 г.
 * @author: Nikita Divakov <n.divakov@strim-tech.com>
 * @copyright Strim-tech
 */

#include <common.h>
#include <command.h>
#include <ledsw.h>

#define LEDS_CODE_ALL_ON        (0x1F)
#define LEDS_CODE_KERNEL        (0x07)
#define LEDS_CODE_DTB           (0x19)

#define LEDS_NO_TO_CODE(no)     (0x01 << (no & 0x7))
#define LEDS_CHR_TO_NUM(chr)    (chr & 0x0f)


static led_cfg_t* led_gpios[];

void ledsw_setup_leds(led_cfg_t* pleds[])
{
    led_gpios = pleds;
}

static void leds_on(void) {
    int i;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_on);
    }
}

static void leds_off(void) {
    int i;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_off);
    }
}

static void leds_def(void) {
    int i;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        gpio_set_value(led_gpios[i].num, led_gpios[i].v_def);
    }
}

static void leds_toggle(unsigned code) {
    int i, value;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        if ((code & (1 << i)) != 0) {
            value = gpio_get_value(led_gpios[i].num);
            value = !value & 0x01;
            gpio_set_value(led_gpios[i].num, value);
        }
    }
}

static void leds_set(unsigned code) {
    int i;
    int count = CONFIG_LEDS_COUNT;
    if (count > 8) { count = 8; }
    for (i = 0; i < count; i++) {
        if ((code & (1 << i)) != 0) {
            gpio_set_value(led_gpios[i].num, led_gpios[i].v_on);
        }
    }
}

static void leds_clr(unsigned code)
{
    int i;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        if ((code & (1 << i)) != 0) {
            gpio_set_value(led_gpios[i].num, led_gpios[i].v_off);
        }
    }
}


static char leds_get_code(void)
{
    int i, value;
    char code = 0;
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
        value = gpio_get_value(led_gpios[i].num);
        if (value == led_gpios[i].v_on) { code |= 1 << i; }
    }
    return (code);
}

static void leds_pulse(unsigned code, unsigned pulse_num, unsigned delay_time)
{
    int i;
    int count_pulse = pulse_num * 2;
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

static void leds_run(unsigned delay_time)
{
    int i;
    char code = 0x01;
    leds_off();
    for (i = 0; i < CONFIG_LEDS_COUNT; i++) {
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
