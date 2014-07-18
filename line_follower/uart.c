/*
 * uart.c
 *
 * Created: 6/06/1996 06:06:06 a.m.
 *  Author: Sam
 */

#include "clock.h"
#include "system.h"
#include <avr/io.h>
#include <stdio.h>

void UART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRR1H = (unsigned char)(baud>>8);
	UBRR1L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);
	// Set no parity
	UCSR1C &= ~(3<<UPM10);
	//Set to async
	UCSR1C &= ~(3<<UMSEL10);
}

void UART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

//Writes a buffer to serial.
void UART_Write(char string[32])
{
	for (int i = 0; i < 32; i++)
	{
		if(string[i] == 0)
			break;
		else
			UART_Transmit(string[i]);
	}
}

void UART_Test()
{

	char str[] = "Hello World!\n";
	int length = 13;
	for(int i = 0; i < length; i++)
		UART_Transmit(str[i]);

}