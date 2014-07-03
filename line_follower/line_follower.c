/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 */


#include <avr/io.h>
#include <avr/interrupt.h> 
//#include <avr/wdt.h>
#include "system.h"
#include "motor.h"

// usefull macro
#define BIT(X) (1 << (X))

int main(void)
{
	
	system_init();
	motor_init();
	
    cli(); // disable all interrupts
	//sei(); // Enable all interrupts
	
	DDRD |= PD2; // LED on PD2
	//PORTD |= PD2; // turn on LED
	
	long i = 0;
	short j = -127;
	short k = 1;
	while(1)
	{
		PORTD ^= PD2; // toggle an LED on PD2
		
		motor_set(j, j);
		
		for (i = 0; i < 5000; i++)
		{
			continue;
		}
		
		if (j == 127)
		{
			k = -1;
		}
		else if (j == -127)
		{
			k = 1;
		}
		
		j += k;
	}
}
