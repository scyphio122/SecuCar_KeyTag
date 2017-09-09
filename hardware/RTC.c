/*
 * RTC.c
 *
 *  Created on: Sep 9, 2017
 *      Author: root
 */

#include "RTC.h"
#include "nrf52.h"
#include "core_cm4.h"

#define DELAY_REGISTER_MAIN		0
#define	TIMEOUT_REGISTER_MAIN 	1

#if !SOFTDEVICE_ENABLED
void RTC0_IRQHandler()
{

}
#endif

volatile bool rtc1_delay_completed_flag;					/**< Delay flag - set to true if the delay has been completed */
volatile bool rtc1_timeout_triggered_flag;					/**< Timeout flag - set to true if the timeout has been triggered */

volatile bool rtc2_delay_completed_flag;					/**< Delay flag - set to true if the delay has been completed */
volatile bool rtc2_timeout_triggered_flag;					/**< Timeout flag - set to true if the timeout has been triggered */

void RTC1_IRQHandler()
{
	if (NRF_RTC1->EVENTS_COMPARE[DELAY_REGISTER_MAIN])			/*< RTC DELAY */
	{
		NRF_RTC1->EVENTS_COMPARE[DELAY_REGISTER_MAIN] = 0;
		rtc1_delay_completed_flag = true;
	}
	else
	if (NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN])		/*< RTC TIMEOUT */
	{
		NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN] = 0;
		rtc1_timeout_triggered_flag = true;
	}
	else
	if (NRF_RTC1->EVENTS_COMPARE[2])
	{
		NRF_RTC1->EVENTS_COMPARE[2] = 0;
	}
	else
	if (NRF_RTC1->EVENTS_COMPARE[3])
	{
		NRF_RTC1->EVENTS_COMPARE[3] = 0;
	}
}

void RTC2_IRQHandler()
{
	if (NRF_RTC2->EVENTS_COMPARE[DELAY_REGISTER_MAIN])			/*< RTC DELAY */
	{
		NRF_RTC2->EVENTS_COMPARE[DELAY_REGISTER_MAIN] = 0;
		rtc2_delay_completed_flag = true;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN])		/*< RTC TIMEOUT */
	{
		NRF_RTC2->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN] = 0;
		rtc2_timeout_triggered_flag = true;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[2])
	{
		NRF_RTC2->EVENTS_COMPARE[2] = 0;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[3])
	{
		NRF_RTC2->EVENTS_COMPARE[3] = 0;
	}

}

RTC_Error_e RTCDelay(NRF_RTC_Type* RTC, uint32_t time_ticks)
{
	bool* delayFlag = NULL;

	if (RTC == NRF_RTC1)
	{
		rtc1_delay_completed_flag = false;
		delayFlag = &rtc1_delay_completed_flag
	}
	else
	if (RTC == NRF_RTC2)
	{
		rtc2_delay_completed_flag = false;
		delayFlag = &rtc2_delay_completed_flag;
	}

	RTCEnableComparingReg(RTC, DELAY_REGISTER_MAIN);
	RTC->CC[DELAY_REGISTER_MAIN] = RTC->COUNTER + time_ticks;

	while (!(*delayFlag))
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}

	RTCDisableComparingReg(RTC, DELAY_REGISTER_MAIN);
	return E_RTC_OK;
}

RTC_Error_e RTCTimeout(NRF_RTC_Type* RTC, uint32_t time_ticks)
{
	bool* timoutFlag = NULL;

	if (RTC == NRF_RTC1)
	{
		rtc1_timeout_triggered_flag = false;
		timoutFlag = &rtc1_timeout_triggered_flag
	}
	else
	if (RTC == NRF_RTC2)
	{
		rtc2_timeout_triggered_flag = false;
		timoutFlag = &rtc2_timeout_triggered_flag;
	}

	RTCEnableComparingReg(RTC, TIMEOUT_REGISTER_MAIN);
	RTC->CC[TIMEOUT_REGISTER_MAIN] = RTC->COUNTER + time_ticks;

	while (!(*timoutFlag))
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}
	RTCDisableComparingReg(RTC, TIMEOUT_REGISTER_MAIN);

	return E_RTC_TIMEOUT;
}

RTC_Error_e RTCClearTimeout(NRF_RTC_Type* RTC)
{
	RTC->EVTENCLR &= ~(1 << (TIMEOUT_REGISTER_MAIN + RTC_EVTENSET_COMPARE0_Pos));
	RTCDisableComparingReg(RTC, TIMEOUT_REGISTER_MAIN);
	if (RTC == NRF_RTC1)
	{
		rtc1_timeout_triggered_flag = false;
	}
	else
	if (RTC == NRF_RTC2)
	{
		rtc2_timeout_triggered_flag = false;
	}

	return E_RTC_OK;
}

RTC_Error_e RTCInit(NRF_RTC_Type* RTC)
{
	RTC_Error_e retval = 0;

#if !SOFTDEVICE_ENABLED
	if (RTC == NRF_RTC0)
		return E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE;
#endif

#if SOFTDEVICE_ENABLED
	if (RTC == NRF_RTC1)
	{
		sd_nvic_SetPriority(RTC1_IRQn, RTC1_PRIORITY);
		sd_nvic_EnableIRQ(RTC1_IRQn);
	}

	if (RTC == NRF_RTC2)
	{
		sd_nvic_SetPriority(RTC2_IRQn, RTC2_PRIORITY);
		sd_nvic_EnableIRQ(RTC2_IRQn);
	}
#else
	if (RTC == NRF_RTC0)
	{
		NVIC_SetPriority(RTC1_IRQn, RTC0_PRIORITY);
		NVIC_EnableIRQ(RTC1_IRQn);
	}

	if (RTC == NRF_RTC1)
	{
		NVIC_SetPriority(RTC1_IRQn, RTC1_PRIORITY);
		NVIC_EnableIRQ(RTC1_IRQn);
	}

	if (RTC == NRF_RTC2)
	{
		NVIC_SetPriority(RTC2_IRQn, RTC2_PRIORITY);
		NVIC_EnableIRQ(RTC2_IRQn);
	}
#endif

	RTCStop(RTC);

	RTC->EVTENCLR |= RTC_EVTENCLR_TICK_Enabled << RTC_EVTENCLR_TICK_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE1_Enabled << RTC_EVTENCLR_COMPARE1_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE2_Enabled << RTC_EVTENCLR_COMPARE2_Pos;

	if (RTC != NRF_RTC0)
		RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE3_Enabled << RTC_EVTENCLR_COMPARE3_Pos;

	switch (RTC)
	{

		case NRF_RTC0:
		{
			RTC->PRESCALER = RTC0_PRESCALER;
		}break;

		case NRF_RTC1:
		{
			RTC->PRESCALER = RTC1_PRESCALER;
		}break;

		case NRF_RTC2:
		{
			RTC->PRESCALER = RTC2_PRESCALER;
		}break;

		default:
			return E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE;
	}

	RTCStart(RTC);

	return E_RTC_OK;
}

RTC_Error_e RTCEnableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber)
{
	switch (registerNumber)
	{
		case 0:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;
		}break;

		case 1:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE1_Enabled << RTC_EVTENSET_COMPARE1_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE1_Enabled << RTC_INTENSET_COMPARE1_Pos;
		}break;

		case 2:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE2_Enabled << RTC_EVTENSET_COMPARE2_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE2_Enabled << RTC_INTENSET_COMPARE2_Pos;
		}break;

		case 3:
		{
			if (RTC == NRF_RTC0)
				return E_RTC_REG_NOT_EXISTING;

			RTC->EVTENSET |= RTC_EVTENSET_COMPARE3_Enabled << RTC_EVTENSET_COMPARE3_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE3_Enabled << RTC_INTENSET_COMPARE3_Pos;
		}break;
	}

	return E_RTC_OK;
}

RTC_Error_e RTCDisableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber)
{
	switch (registerNumber)
	{
		case 0:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE0_Enabled << RTC_INTENCLR_COMPARE0_Pos;
		}break;

		case 1:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE1_Enabled << RTC_EVTENCLR_COMPARE1_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE1_Enabled << RTC_INTENCLR_COMPARE1_Pos;
		}break;

		case 2:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE2_Enabled << RTC_EVTENCLR_COMPARE2_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE2_Enabled << RTC_INTENCLR_COMPARE2_Pos;
		}break;

		case 3:
		{
			if (RTC == NRF_RTC0)
				return E_RTC_REG_NOT_EXISTING;

			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE3_Enabled << RTC_EVTENCLR_COMPARE3_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE3_Enabled << RTC_INTENCLR_COMPARE3_Pos;
		}break;
	}

	return E_RTC_OK;
}

RTC_Error_e RTCStart(NRF_RTC_Type* RTC)
{
	RTC->TASKS_START = 1;

	return E_RTC_OK;
}

RTC_Error_e RTCStop(NRF_RTC_Type* RTC)
{
	RTC->TASKS_STOP = 1;

	return E_RTC_OK;
}
