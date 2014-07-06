/*
 * comparator.c
 *
 * Created: 1/07/2014 5:07:16 p.m.
 *  Author: martin
 */ 

#include "comparator.h"
#include "system.h"
#include <avr/io.h>
//#include "pio.h"



/** Initialize Analog comparator on AINx
	@param none
	@return none */
void comparator_init(short mux)
{
	DDRD = 0x00;
	ACSR = (0<<ACD) | // enable comparator
			(0<<ACBG) | // use AIN0 as comparator input
			(0<<ACIE) | // ACIE: Analog Comparator Interrupt Enable
			(0<<ACIC); // ACIC: Analog Comparator Input Capture Enable
			
	comparator_mux(mux);
	
	DIDR1 = 0xFF; // disable digital input on all AINx pins
	
	DDRD &= ~(1<<PD1); // make AIN0 (PD1) an input
}

/** Changes the selected comparator pin
	@param mux - configure mux register
	@return none */
void comparator_mux(short mux)
{
	// have to use a pointer because
	// ACMUX  is not defined for reasons
	// that are beyond my knowledge...
	char* acmux;
	acmux = 0x7D;
	*acmux = mux;
	
	//ACMUX = mux;
}

/** Read the comparator value. If the voltage on 
	AIN0 (PD1) is higher than on AINx then this
	returns 1.
	@param mux - chose which pin to measure on
	@return - the comparator value, ACO. */
bool comparator_higher_than(short mux)
{
	comparator_mux(mux);
	
	return ACSR & (1<<ACO);
}


/** Turns off the comparator. this may save
	power or you may wish to do this if you
	want to use the pins for something else.
	@param none
	@return none */
void comparator_disable(void)
{
	ACSR |= (1<<ACD);
}



/** Simple test program for this module.
    Place a voltage on AIN0 (PD1)
	and another voltage on a mux pin
	and an LED on pin PB7 and PB6.
	@param none
	@return none */
void comparator_test(short mux)
{
	DDRB |= (1<<PB7) | (1<<PB6);
	
	comparator_init(mux);
	
	while(1)
	{
		if ( comparator_higher_than(mux) )
		{
			PORTB |= (1<<PB7);
		}
		else
		{
			PORTB &= ~(1<<PB7);
		}
	
		PORTB ^= (1<<PB6);
	}
}

/*
ACSR - Analog Comparator Control and Status Register
		ACD:	Analog Comparator Disable
		ACBG:	Analog Comparator Bandgap Select
		ACO:	Analog Comparator Output
		ACI:	Analog Comparator Interrupt Flag
		ACIE:	Analog Comparator Interrupt Enable
		ACIC:	Analog Comparator Input Capture Enable
		ACIS1:	Analog Comparator Interrupt Mode Select
		ACIS0:	Analog Comparator Interrupt Mode Select
			
ACMUX - Analog Comparator Input Multiplexer
		CMUX2 |  CMUX1  | CMUX0 | Comparator Input	| External Pin
			0 |   0		|   0	| AIN1				| PD2
			0 |   0		|   1	| AIN2				| PC2
			0 |   1		|   0	| AIN3				| PD4
			0 |   1		|   1	| AIN4				| PD5
			1 |   0		|   0	| AIN5				| PD6
			1 |   0		|   1	| AIN6				| PD7
			1 |   1		|   0	| Reserved			|
			1 |   1		|   1	|					|
								|------- AIN0 ------| PD1

DIDR1 - Digital Input Disable Register 1 (set this to 1 to save power)
		AIN6D, AIN5D, AIN4D, AIN3D, AIN2D, AIN1D, AIN0D


*/