/*
 * pwm.h
 *
 *  Created on: Oct 20, 2017
 *      Author: Konrad Traczyk
 */

#ifndef HARDWARE_PWM_H_
#define HARDWARE_PWM_H_

#include <stdint-gcc.h>

typedef struct
{
    uint8_t  gpio_pin;
    uint8_t  duty_cycle_percent;
    uint32_t duty_cycle_ticks;
    uint32_t frequency_ticks;
}pwm_params_t;

extern pwm_params_t pwm_module[3];

#endif /* HARDWARE_PWM_H_ */
