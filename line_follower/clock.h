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


#ifndef CLOCK_RATE
	#define CLOCK_RATE 1000
#endif



void clock_init(void);

int clock_get_ticks(void);

int clock_get_ms(void);





#endif /* CLOCK_H_ */