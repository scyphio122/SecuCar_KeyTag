/*
 * pinout.h
 *
 *  Created on: Jun 15, 2017
 *      Author: root
 */

#ifndef INC_PINOUT_H_
#define INC_PINOUT_H_

/**< ########### UART ############# **/
#define UART_RX_PIN 	8
#define UART_TX_PIN		6
#define UART_RTS_PIN	13
#define UART_CTS_PIN	14

/**< ########## SPI ############# **/
#define SPI0_CS_PIN		(uint32_t)0xFFFFFFFF
#define SPI0_SCK_PIN	(uint32_t)6
#define SPI0_MISO_PIN	(uint32_t)7
#define SPI0_MOSI_PIN	(uint32_t)8

#define SPI1_CS_PIN		(uint32_t)0xFFFFFFFF
#define SPI1_SCK_PIN	(uint32_t)0xFFFFFFFF
#define SPI1_MISO_PIN	(uint32_t)0xFFFFFFFF
#define SPI1_MOSI_PIN	(uint32_t)0xFFFFFFFF

#define SPI2_CS_PIN		(uint32_t)0xFFFFFFFF
#define SPI2_SCK_PIN	(uint32_t)0xFFFFFFFF
#define SPI2_MISO_PIN	(uint32_t)0xFFFFFFFF
#define SPI2_MOSI_PIN	(uint32_t)0xFFFFFFFF

#endif /* INC_PINOUT_H_ */
