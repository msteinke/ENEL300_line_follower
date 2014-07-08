/*
 * clock.h
 *
 * Created: 6/07/2014 12:16:25 p.m.
 *  Author: martin
 *
 * This module is intended to work with motor.c
 */ 


#ifndef CLOCK_H_
#define CLOCK_H_

#include "system.h"

#ifndef CLOCK_RATE
	#define CLOCK_RATE_HZ 1000
#endif

#define CLOCK_TIMER1_SIZE 65535
// F_CPU is defined in system.h
#define CLOCK_TIMER1_PRESCALER 64
#define CLOCK_TIMER1_COMPARE_VALUE  (F_CPU / CLOCK_TIMER1_PRESCALER) / CLOCK_RATE_HZ
/* tick rate = F_CPU / CLOCK_TIMER1_PRESCALER (ticks per second)
 * compare value = tick rate (ticks per second) / clock rate (ticks per second)
 *               = tick rate / clock rate (no units)
 
   interrupt rate = tick rate (ticks per second) / compare value (ticks per interrupt)
                  = tick rate / compare value (interrupts per second)
 
    | prescaler	| compare value	|
	|   1		| 8000			|
	|	8		| 1000			|
	|	64		| 125			|
	|	256		| 31.25			|
	|	1024	| 7.8125		|
 */


void clock_init(void);

unsigned long clock_get_ms(void);

void clock_set_ms(unsigned long time);

void clock_test(void);


#endif /* CLOCK_H_ */