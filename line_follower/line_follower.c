/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 */


#include <avr/io.h>
#include <avr/interrupt.h> 
#include "system.h"
#include "motor.h"
//#include "comparator.h"

int main(void)
{
	system_init();
	//motor_init();
	
    //cli(); // disable all interrupts
	sei(); // Enable all interrupts
	
	//motor_test();
	
	
	
	DDRB |= (1<<PB6);
	
	while(1)
	{
		
		PORTB ^= (1<<PB6);
	}
}
//65535