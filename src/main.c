/*
 ============================================================================
 Name        : main.c
 Author      : Konrad Traczyk
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C
 ============================================================================
 */

#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf52.h"
#include "core_cm4.h"
#include "Systick.h"

/*
 *
 * Print a greeting message on standard output and exit.
 *
 * On embedded platforms this might require semi-hosting or similar.
 *
 * For example, for toolchains derived from GNU Tools for Embedded,
 * to enable semi-hosting, the following was added to the linker:
 *
 * --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
 *
 * Adjust it for other toolchains.
 *
 */

/*void HardFault_Handler()
{
	uint32_t hfsr = SCB->HFSR;
	while(1);
}*/

void NVICInit()
{
	NVIC_SetPriorityGrouping(4);
}

int
main(void)
{
	nrf_gpio_cfg_output(17);
	NVICInit();
	SystickInit();


	while(1)
	{
		nrf_gpio_pin_toggle(17);
		SystickDelayMs(1000);
	}
  //printf("Hello ARM World!" "\n");
  return 0;
}
