/*
 * UART.h
 *
 *  Created on: Jun 14, 2017
 *      Author: root
 */

#ifndef HARDWARE_UART_H_
#define HARDWARE_UART_H_

typedef enum
{
	UART_NO_ERROR,
	UART_INT_BUF_OVRFLW,

}e_uart_error;

void UartConfig(uint32_t baudrateBitfield, uint32_t parity, uint32_t hardwareFlowControl);
void UartEnable();
void UartDisable();

e_uart_error UartSendDataSync(uint8_t* dataToSend, uint32_t dataSize);
e_uart_error UartReadDataNumberSync(uint8_t* dataBuffer, uint32_t dataSize);
e_uart_error UartReadDataEndCharSync(uint8_t* dataBuffer, uint8_t endChar);



#endif /* HARDWARE_UART_H_ */