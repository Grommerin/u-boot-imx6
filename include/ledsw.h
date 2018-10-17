/**
 * @file ledsw.h
 * @brief 
 * @date 13 апр. 2018 г.
 * @author: Nikita Divakov <n.divakov@strim-tech.com>
 * @copyright Strim-tech
 */

#ifndef INCLUDE_LEDSW_H_
#define INCLUDE_LEDSW_H_

struct led_parms {
    unsigned num;   /* GPIO Number */
    unsigned v_on;  /* Active Value */
    unsigned v_off; /* Inactive Value */
    unsigned v_def; /* Default Value */
};

typedef struct led_parms led_cfg_t;


void ledsw_setup_leds(led_cfg_t* pleds);

#endif /* INCLUDE_LEDSW_H_ */
