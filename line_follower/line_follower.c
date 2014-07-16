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
//#include "clock.h"
#include "sensor.h"

int main(void)
{
	DDRB |= (1<<PB6);
	system_init();
	motor_init();
	//clock_init();
	//sensor_init();
	
    //cli(); // disable all interrupts
	sei(); // Enable all interrupts
	//PORTB |= (1<<PB6);
	
	//motor_test();
	//clock_test();
	//sensor_test(AIN3_MUX);
	
	
	
	
	
	
	while(1)
	{
		
		//PORTB ^= (1<<PB6);
		motor_set(-128, -128);
		for (i = 0; i < 100000; i++)
		{
			continue;
		}
		
		motor_set(-128, 128);
		for (i = 0; i < 100000; i++)
		{
			continue;
		}
	}
}
//65535