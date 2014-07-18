/*
 * comparator.h
 *
 * Created: 1/07/2014 5:07:26 p.m.
 *  Author: martin
 */ 


#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "system.h"
//typedef unsigned char bool;



/** Initialize Analog comparator on AINx
	@param none
	@return none */
void comparator_init();

/** Changes the selected comparator pin
	@param mux - configure mux register
	@return none */
void comparator_mux(byte mux);

/** Read the comparator value. If the voltage on 
	AIN0 (PD1) is higher than on AINx then this
	returns 1.
	@param mux - chose which pin to measure on
	@return - the comparator value, ACO. */
bool comparator_higher_than(byte mux);

/** Turns off the comparator. this may save
	power or you may wish to do this if you
	want to use the pins for something else.
	@param none
	@return none */
void comparator_disable(void);


/** Simple test program for this module.
    Place a voltage on AIN0 (PD1)
	and another voltage on a mux pin
	and an LED on pin PB7 and PB6.
	@param none
	@return none */
void comparator_test(byte mux);



#endif /* COMPARATOR_H_ */