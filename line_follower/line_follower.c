/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 */


#include <avr/io.h>
#include <avr/interrupt.h> 
#include "system.h"
//#include "motor.h"
//#include "comparator.h"

int main(void)
{
	system_init();
	//comparator_init();
	
    cli(); // disable all interrupts
	//sei(); // Enable all interrupts
	
	//DDRD |= PD2; // LED on PD2
	
	DDRD = 0x00;
	ACSR = (0<<ACD) | // enable comparator
			(0<<ACBG) | // use AIN0 as comparator input
			(0<<ACIE) | // ACIE: Analog Comparator Interrupt Enable
			(0<<ACIC); // ACIC: Analog Comparator Input Capture Enable
		
	//ACMUX = 0x00; // use AIN1
		
	DIDR1 = 0xFF; // disable digital input on all AINx pins
		
	
	DDRB |= (1<<PB7) | (1<<PB6);
	
	while(1)
	{
		if ( (ACSR & (1<<ACO) ) )
		{
			PORTB |= (1<<PB7);
		}
		else
		{
			PORTB &= ~(1<<PB7);
		}
		//PORTB = (ACSR<<2);
		
		PORTB ^= (1<<PB6);
	}
}
//65535