/*
simple asynchronus USART module
Transmit only for debug

TX: PD3
*/

#ifndef UART_H_
#define UART_H_

#include "system.h"


//sets up uart
// for fosc = 8khz & 9600 baud
// -> baud = 51
void UART_Init(unsigned int baud);

//Transmits a byte
void UART_Transmit( unsigned char data);

//Transmits a string of characters
void UART_Write(char string[32]);

// 
// and prints "hello world"
void UART_Test();

#endif