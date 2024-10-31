/*
 * UART.h
 *
 *  Created on: 2 Nov 2022
 *      Author: Technikum Wien
 */

#ifndef UART_H_
#define UART_H_

#include "platform.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/***
 * @brief: functions to initialise the UART Interrupt routines
 * @params: None
 * @return: void
 */
void asclin3_Tx_ISR(void);
void asclin3_Rx_ISR(void);
void asclin3_Er_ISR(void);

/***
 * @brief: initialises the UART module
 * @params: None
 * @returns: void
 */
void initUART(void);

/***
 * @brief: a wrapper function of the IfxAsclin_Asc_blockingWrite function
 * it sends one byte via UART to the receiver
 * @params: uint8, the byte to be transfered
 * @return: void
 */
void uart_blockingWrite(uint8 byte);

/***
 * @brief: a wrapper function of the IfxAsclin_Asc_write function
 * it sends a char array to the receiver
 * @params: uint8 pointer: the char array to be transfered
 * @params: Ifx_SizeT: the size of the char array
 * @return: void
 */
void uart_sendMessage(uint8 *data, Ifx_SizeT size);
#endif /* UART_H_ */
