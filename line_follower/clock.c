/*
 * clock.c
 *
 * Created: 6/07/2014 12:16:38 p.m.
 *  Author: martin
 */ 

#include "clock.h"
#include "system.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/* Global variable to hold the time
 max size is 2^32 - 1 = 4294967295 ms
                      = 4294967 s
					  = 71582 minutes
					  = 1193 hours
					  = 49 days.
 */
static volatile unsigned long g_time_ms;

ISR(TIMER1_COMPA_vect)
{
	g_time_ms++;
}

void clock_set_ms(unsigned long time)
{
	g_time_ms = time;
}

void clock_init(void)
{
	// clear the time
	clock_set_ms(0);
	
	// Configure PWM timer1
	DDRC |= (1<<6) | (1<<5); 
	
	TCCR1A = (1<<COM1A1) |
			(0<<COM1A0) |
			(0<<COM1B1) |
			(0<<COM1B0) |
			(0<<COM1C1) |
			(0<<COM1C0) |
			(0<<WGM11) | // CTC mode 4
			(0<<WGM10);  // CTC mode 4
	
	TCCR1B = //(0<<FOC0A) | // must disable these for PWM
			//(0<<FOC0B) | // must disable these for PWM
			(0<<WGM13) | // CTC mode 4
			(1<<WGM12) | // CTC mode 4
			(0<<CS12) | // divide clock by 64
			(1<<CS11) | // divide clock by 64
			(1<<CS10); //divide clock by 64
	
	//TIMSK1 = 0x00; // disable interrupts
	TIMSK1 = (0<<ICIE1) | // Timer/Countern, Input Capture Interrupt Enable
			(0<<OCIE1C) | // Timer/Countern, Output Compare C Match Interrupt Enable
			(0<<OCIE1B) | // Timer/Countern, Output Compare B Match Interrupt Enable
			(1<<OCIE1A) | // Timer/Countern, Output Compare A Match Interrupt Enable
			(0<<TOIE1); // Timer/Countern, Overflow Interrupt Enable
	
	OCR1A = CLOCK_TIMER1_COMPARE_VALUE;
	//OCR1A = (unsigned short) (F_CPU / CLOCK_TIMER1_PRESCALER) / CLOCK_RATE_HZ;
	//OCR1A = 125;
	//OCR1AL = 125;
	
}

unsigned long clock_get_ms(void)
{
	return g_time_ms;
}


/*
from table 11-1, Reset and Interrupt Vectors:
16 $001E TIMER1 COMPA Timer/Counter1 Compare Match A


TIMSK1 - Timer/Counter1 Interrupt Mask Register
		ICIEn:  Timer/Countern, Input Capture Interrupt Enable
		OCIEnC: Timer/Countern, Output Compare C Match Interrupt Enable
		OCIEnB: Timer/Countern, Output Compare B Match Interrupt Enable
		OCIEnA: Timer/Countern, Output Compare A Match Interrupt Enable
		TOIEn:  Timer/Countern, Overflow Interrupt Enable
		
		
		
		
TIFR1 - Timer/Counter1 Interrupt Flag Register
		ICFn:  Timer/Countern, Input Capture Flag
		OCFnC: Timer/Countern, Output Compare C Match Flag
		OCFnB: Timer/Counter1, Output Compare B Match Flag
		OCF1A: Timer/Counter1, Output Compare A Match Flag
		TOVn:  Timer/Countern, Overflow Flag
		
*/