/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 */


#include <avr/io.h>
#include "system.h"

int main(void)
{
	system_init ();
 
	DDRD = 0xFF;
 
	volatile long i = 0;
 
	while(1)
	{
		//TODO:: Please write your application code
		PORTD--;
 
		for (i = 0; i < 10000; i++)
		{
			continue;
		}
	}
}