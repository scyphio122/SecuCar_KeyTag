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
#include <stdint-gcc.h>
#include "nrf_gpio.h"
#include "nrf52.h"
#include "core_cm4.h"
#include "Systick.h"
#include "nrf_sdm.h"
#include "settings.h"
#include "nrf_nvic.h"
#include "UART.h"
#include "nrf52_bitfields.h"

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

nrf_nvic_state_t nrf_nvic_state = {0};

void POWER_CLOCK_IRQHandler()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	nrf_gpio_cfg_output(20);
	nrf_gpio_pin_clear(20);
}

void SDFaultHandler(uint32_t id, uint32_t pc, uint32_t info)
{
	nrf_gpio_cfg_output(19);
	nrf_gpio_pin_clear(19);
	while(1)
	{

	}
}

void NVICInit()
{
	__enable_irq();
	NVIC_SetPriorityGrouping(0);
}

uint8_t buf[64] = {0};

int
main(void)
{

#if SOFTDEVICE_ENABLED
	nrf_clock_lf_cfg_t clockConfig;

	clockConfig.source = NRF_CLOCK_LF_SRC_XTAL;
	clockConfig.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
	clockConfig.rc_ctiv = 0;
	clockConfig.rc_temp_ctiv = 0;

	uint32_t retval = sd_softdevice_enable(&clockConfig, SDFaultHandler);
#endif

	SystickInit();

	UartConfig(UART_BAUDRATE_BAUDRATE_Baud9600, UART_CONFIG_PARITY_Included, UART_CONFIG_HWFC_Disabled);
	UartEnable();
	UartSendDataSync("Hello World, it's nRF52!", sizeof("Hello World, it's nRF52!"));


//	UartReadDataEndCharSync(buf, '\n');
	while(1)
	{

		UartReadDataEndCharSync(buf, '\r');
		UartSendDataSync("\r\n", 3);
		UartSendDataSync(buf, sizeof(buf));

	}

  return 0;
}
