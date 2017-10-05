/*
 * RTC.h
 *
 *  Created on: Sep 9, 2017
 *      Author: root
 */

#ifndef HARDWARE_RTC_H_
#define HARDWARE_RTC_H_

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "settings.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#define RTC0_PRESCALER				0
#define RTC0_FREQUENCY   			(32768/(RTC0_PRESCALER + 1))
#define RTC0_MS_TO_TICKS(time_ms)	(time_ms*RTC0_FREQUENCY/1000)
#define RTC0_US_TO_TICKS(time_us)	(time_us*RTC0_FREQUENCY/1000000)

#define RTC1_PRESCALER				0
#define RTC1_FREQUENCY   			(32768/(RTC1_PRESCALER + 1))
#define RTC1_MS_TO_TICKS(time_ms)	(time_ms*RTC1_FREQUENCY/1000)
#define RTC1_US_TO_TICKS(time_us)	(time_us*RTC1_FREQUENCY/1000000)

#define RTC2_PRESCALER				0
#define RTC2_FREQUENCY   			(32768/(RTC2_PRESCALER + 1))
#define RTC2_MS_TO_TICKS(time_ms)	(time_ms*RTC2_FREQUENCY/1000)
#define RTC2_US_TO_TICKS(time_us)	(time_us*RTC2_FREQUENCY/1000000)

typedef enum
{
	E_RTC_OK = 0,
	E_RTC_REG_NOT_EXISTING,
	E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE,
	E_RTC_TIMEOUT,
}RTC_Error_e;

typedef struct
{
	bool activeFlag;
	bool timeoutTriggeredFlag;
}rtc_timeout_t;

RTC_Error_e RTCInit(NRF_RTC_Type* RTC);
RTC_Error_e RTCEnableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber);
RTC_Error_e RTCDisableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber);
RTC_Error_e RTCStart(NRF_RTC_Type* RTC);
RTC_Error_e RTCStop(NRF_RTC_Type* RTC);

RTC_Error_e RTCDelay(NRF_RTC_Type* RTC, uint32_t time_ticks);
RTC_Error_e RTCTimeout(NRF_RTC_Type* RTC, uint32_t time_ticks, uint8_t* outTimeoutId);
RTC_Error_e RTCClearTimeout(NRF_RTC_Type* RTC, uint8_t timeoutId);

#endif /* HARDWARE_RTC_H_ */
