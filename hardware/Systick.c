#include "Systick.h"
#include "core_cm4.h"
#include <stdbool.h>
#include "system_nrf52.h"

#define SYSTICK_CLOCK_FREQ			(64000000ul)
#define SYSTICK_MS_TO_TICKS(x)		((x)/(SYSTICK_CLOCK_FREQ/1000))

static uint32_t systickCnt;
static bool		systickTimeoutFlag;


void Systick_IRQHandler()
{
	systickCnt--;
	if(systickCnt == 0)
	{
		systickTimeoutFlag = true;
	}
}

void SystickInit()
{
	SysTickConfig(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, 2);
	NVIC_EnableIRQ(SysTick_IRQn);
}

void SystickDelayMs(uint32_t timeMs)
{
	systickCnt = SYSTICK_MS_TO_TICKS(timeMs);
	systickTimeoutFlag = false;

	while(!systickTimeoutFlag)
	{
		__WFE();
	}
}


