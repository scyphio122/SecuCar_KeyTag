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
#include "nrf52_bitfields.h"
#include "nrf_soc.h"
#include "core_cm4.h"
#include "nrf_sdm.h"
#include "settings.h"
#include "nrf_nvic.h"
#include "ble_common.h"
#include "advertising.h"
#include <string.h>
#include "RTC.h"
#include "ble_uart_service.h"
#include "nrf_sdh_soc.h"
#include "crypto.h"
#include "internal_flash.h"
#include "Systick.h"
#include "pinout.h"
#include "nfc.h"

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

/**@brief Function for handling SOC events.
 *
 * @param[in]   evt_id      SOC stack event id.
 * @param[in]   p_context   Unused.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    SD_flash_operation_callback(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);

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

__attribute__((optimize("O0")))
int main(void)
{
#if SOFTDEVICE_ENABLED
    BleStackInit();

	GapParamsInit();
	GattInit();
//	ConnParamsInit();
	ServicesInit();
	AdvertisingInit();
	SystickInit();

//	AdvertisingStart();

	nrf_gpio_cfg_output(BLUE_LED);
	nrf_gpio_cfg_output(GREEN_LED);
    nrf_gpio_cfg_output(RED_LED);

    nrf_gpio_pin_set(BLUE_LED);
    nrf_gpio_pin_set(GREEN_LED);
    nrf_gpio_pin_set(RED_LED);

    nrf_gpio_pin_clear(BLUE_LED);
//    nrf_gpio_pin_set(BLUE_LED);

    NfcInit();

#endif

    // Check if the Main Key exists
   /* if (!CryptoCheckMainKey())
    {

    }*/

    while(1)
    {
        sd_app_evt_wait();
/*        SystickDelayMs(1000);

        nrf_gpio_pin_clear(RED_LED);
        nrf_gpio_pin_set(RED_LED);
        SystickDelayMs(1000);
        nrf_gpio_pin_clear(BLUE_LED);
        nrf_gpio_pin_set(BLUE_LED);
        SystickDelayMs(1000);
        nrf_gpio_pin_clear(GREEN_LED);
        nrf_gpio_pin_set(GREEN_LED);*/

    }

	
  return 0;
}
