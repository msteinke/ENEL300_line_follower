/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 *
 * Comparator does funny stuff with portb, for PorbD io, 
 * check comparator.c
 */

//portb LED bitmask
#define PBLED 0x9C
#define BAUD 51 //uart baud rate 9600

#include <avr/io.h>
#include <avr/interrupt.h> 
#include "system.h"
#include "motor.h"
#include "comparator.h"
#include "clock.h"
//#include "sensor.h"
#include "uart.h"
#include "adc.h"



void led_set_one(char led, char state)
{

	{
		PORTB &= ~BIT(led);
		PORTB |= (state<<led);
	}
}

void led_set(char p)
{
	PORTB &= ~PBLED; //KILL LEDS
	// set relevent bits
	PORTB |= ((p << 4)&BIT(7))|	//shift 4th bit to 7th
		((p<<2)&(BIT(2)|
		BIT(3)|
		BIT(4)));		//& 0-2 -> 2->4
		
		//PB2, PB3, PB4, PB7
}

void led_init(void)
{
	DDRB |= PBLED;
}

int main(void)
{
	
	
	system_init();
	//motor_init();
	clock_init();
	sensor_init();
	//UART_Init(BAUD);
	led_init();
	adc_init();
	
	adc_enable(AIN1_MUX);
	

	
	//UART_Write("Initialising...");
	
    //cli(); /*// disable all interrupts
	sei(); /**/// Enable all interrupts
	clock_enable_interrupt();
	
	led_set(0x01); //show everyone I'm alive.
	
	//comparator_test(AIN2_MUX);
	
	//DDRD = 0xFF; //Necissary?
	
	//UART_Write("Complete\n");


	volatile short i = 0;
	DDRB |= (1<<PB3) | (1<<PB4);
	clock_set_ms(0);


	int t = 0;
	int t_last = 0;
	byte j = 0;
	
	for (int i = 0; i < 16000; i++)
	{
		continue;
	}
	while(1)
	{
		unsigned long start = adc_measure(AIN1_MUX);

		{
		}
		t = clock_get_ms();
		if( (t % 500) == 0 & t!=t_last)
		{
			PORTB ^= (1<<3);
			t_last = t;
			if( adc_measure(AIN1_MUX) >= start)
				PORTB |= (1<<7);
			else
				PORTB &= ~(1<<7);

		}
		
	}
}
//65535