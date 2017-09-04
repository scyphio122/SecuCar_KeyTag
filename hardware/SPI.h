/*
 * SPI.h
 *
 *  Created on: Sep 3, 2017
 *      Author: root
 */

#ifndef HARDWARE_SPI_H_
#define HARDWARE_SPI_H_

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include <stdint-gcc.h>


typedef enum
{
	E_SPI_SUCCESS,
	E_SPI_TIMEOUT
}E_SPI_Errors;

typedef enum
{
	E_SPI_FREQUENCY_125_kHz = SPI_FREQUENCY_FREQUENCY_K125,
	E_SPI_FREQUENCY_250_kHz = SPI_FREQUENCY_FREQUENCY_K250,
	E_SPI_FREQUENCY_500_kHz = SPI_FREQUENCY_FREQUENCY_K500,
	E_SPI_FREQUENCY_1_MHz 	= SPI_FREQUENCY_FREQUENCY_M1,
	E_SPI_FREQUENCY_2_MHz 	= SPI_FREQUENCY_FREQUENCY_M2,
	E_SPI_FREQUENCY_4_MHz 	= SPI_FREQUENCY_FREQUENCY_M4,
	E_SPI_FREQUENCY_8MHz  	= SPI_FREQUENCY_FREQUENCY_M8
}E_SPI_Frequency;

typedef enum
{
	E_SPI_REGULAR,
	E_SPI_MASTER
}E_SPI_Type;

void SPI_Config(NRF_SPI_Type* spi,
				E_SPI_Frequency frequency,
				uint8_t bytes_order,
				uint8_t sck_phase,
				uint8_t sck_polarity);

void SPI_Enable(NRF_SPI_Type* spi);

void SPI_Disable(NRF_SPI_Type* spi);

E_SPI_Errors SPI_Write(NRF_SPI_Type* spi, uint8_t* in_buf, uint16_t data_size);

E_SPI_Errors SPI_Read(NRF_SPI_Type* spi, uint8_t* out_buf, uint16_t data_size);



#endif /* HARDWARE_SPI_H_ */
