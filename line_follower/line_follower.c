/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 *
 * Comparator does funny stuff with portb, for PorbD io, 
 * check comparator.c
 */


#include "config.h"

//Parameters
#ifdef ENABLE_UART
#define UART_ENABLED 1
#else
#define UART_ENABLED 0
#endif
#define UART_PERIOD (CLOCK_RATE_HZ/UART_RATE)
#define SAMPLE_PERIOD (CLOCK_RATE_HZ/SAMPLE_RATE)


//System Includes
#include "system.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
//Peripheral Includes
#include "motor.h"
#include "led.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "circBuf.h"



int main(void)
{
	

	
	system_init();
	clock_init();
	led_init();	led_set(0x01); //show life
	UART_Init(BAUD); UART_Write("Init"); //Show UART life
	//motor_init();
	adc_init();
	
	//Disable output on analog pins 
	DDRD &= ~(BIT(4)|BIT(2)); DDRC &= ~BIT(2);  // TODO: MOVE TO ADC_INIT()
	//Enable Analog pins
	adc_enable(AIN1); adc_enable(AIN2);	adc_enable(AIN3);
	//Initialize signal conditioning arrays
	circBuf_t aLeft; circBuf_t aRight; circBuf_t aFront;
	
	uint16_t a_in1 = 0;
	uint16_t a_in2 = 0;
	uint16_t a_in3 = 0;
		
	//Initialise UART output buffer
	char buffer[UART_BUFF_SIZE] = {0};

	//Initialize scheduler variables
	uint32_t t = 0;	
	uint32_t sample_t_last = 0;
	uint32_t UART_t_last = 0;


	clock_set_ms(0);
	sei(); // Enable all interrupts
	UART_Write("ialized\n");
	

	short i = 0;
	while(1)
	{                                                                                                                         
		
// 		motor_set(i, i);
// 		_delay_ms(1);
// 		i++;
// 		if (i > 255)
// 			i = 0;

		/*MOTOR_TEST*/
// 		PORTB |= BIT(1)|BIT(7);
// 		PORTB &= ~BIT(6);
// 		
// 		PORTD |= BIT(0);
// 		PORTB |= BIT(5);
// 		PORTC &= BIT(7);
		//  		t = clock_get_ms();
// 		
//  		if((t%SAMPLE_PERIOD == 0) & (t!=sample_t_last))
// 		{
// 			//TODO: unblock function if necessary. 
//  			a_in1 = adc_measure(AIN1);
// 			_delay_us(100);
// 			a_in2 = adc_measure(AIN2);
// 			_delay_us(100);
// 			a_in3 = adc_measure(AIN3);
// 			sample_t_last = t;
// 		}
// 		
// 		if((t%UART_PERIOD == 0) & (t != UART_t_last) & UART_ENABLED)
// 		{
// 			sprintf(buffer, "\n %u, %u, %u", a_in1, a_in2, a_in3);
// 			UART_Write(buffer);
// 		}
		
	}
}
//65535