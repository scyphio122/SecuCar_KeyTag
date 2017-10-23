/*
 * pwm.c
 *
 *  Created on: Oct 20, 2017
 *      Author: Konrad Traczyk
 */

#include "pwm.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "pinout.h"

pwm_params_t pwm_module[3];

#define PWM_PRESCALER                   (PWM_PRESCALER_PRESCALER_DIV_16)
#define PWM_MS_TO_TICKS(x)              ((16000000/(1 << PWM_PRESCALER)*x)/1000)


void PwmInit()
{
    NRF_PWM1->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;

    NRF_PWM1->PRESCALER = PWM_PRESCALER;

    NRF_PWM1->INTENSET = PWM_INTENSET_PWMPERIODEND_Enabled << PWM_INTENSET_PWMPERIODEND_Pos;

    NRF_PWM1->PSEL.OUT[0] = RED_LED | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM1->PSEL.OUT[1] = GREEN_LED | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM1->PSEL.OUT[2] = BLUE_LED | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

}

void PwmSetPeriod(uint8_t pwmIndex, uint16_t timeMs)
{
    pwm_module[pwmIndex].frequency_ticks = PWM_MS_TO_TICKS(timeMs);

    NRF_PWM1->COUNTERTOP = pwm_module[pwmIndex].duty_cycle_ticks;
}

void PwmSetDutyCycle(uint8_t pwmIndex, uint8_t dutyPercentage)
{

}

void PwmStart()
{
    NRF_PWM1->ENABLE = PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos;

void PwmStop()
{
    NRF_PWM1->ENABLE = PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos;
}

