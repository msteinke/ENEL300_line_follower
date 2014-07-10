/*
 * sensor.c
 *
 * Created: 4/07/2014 12:50:58 p.m.
 *  Author: martin
 */ 

#include "sensor.h"
#include "comparator.h"
#include "system.h"
#include "pio.h"

// delay AC read after switching mux
#define PROPOGATION_DELAY 40

/** Initialize the comparator and GPIO
	@param none
	@return none*/
void sensor_init(void)
{
	comparator_init();
	pio_config_set(SENSOR_EMUX_SELECT_PIN, PIO_OUTPUT_LOW);
	pio_config_set(SENSOR_EMUX_VREF_PIN, PIO_INPUT);
}

/** A delay loop, for a few clock cycles, to allow
	time for the external mux to switch reference
	voltages. This is a must!!
	@param none
	@return none*/
void sensor_propogation_delay(void)
{
	for (volatile short i = 0; i < PROPOGATION_DELAY; i++)
	{
		continue;
	}
}


/** determines if the measured voltage is darker
	than grey.
	@param internal mux pin
	@return none*/
bool higher_than_grey(byte acmux)
{
	pio_output_high(SENSOR_EMUX_SELECT_PIN); // 1.3V
	sensor_propogation_delay();
	
	return comparator_higher_than(acmux);
}

/** Determines if the measured voltage is lighter
	than grey.
	@param none
	@return none*/
bool lower_than_grey(byte acmux)
{
	pio_output_low(SENSOR_EMUX_SELECT_PIN); // 3.1V
	sensor_propogation_delay();
		
	return !comparator_higher_than(acmux);
}

/** Determine if a ACMUX pin reads
	WHITE, GREY or BLACK.
	@param ACMUX pin
	@return colour type*/
colour_t sensor_read(byte acmux)
{
	if (lower_than_grey(acmux))
	{
		return WHITE;
	}
	else if (higher_than_grey(acmux))
	{
		return BLACK;
	}
	else
	{
		return GREY; 
	}
}
	

/** Simple test program to see if this module
	works or not. Requires (atleast) 3 LEDs.
	@param ACMUX pin that has POT connected to it
	@return none*/
void sensor_test(byte acmux)
{
	pio_config_set(PIO_DEFINE(PORT_B, 6), PIO_OUTPUT_LOW);
	
	pio_config_set(PIO_DEFINE(PORT_B, 5), PIO_OUTPUT_LOW); // white
	pio_config_set(PIO_DEFINE(PORT_B, 4), PIO_OUTPUT_LOW); // grey
	pio_config_set(PIO_DEFINE(PORT_B, 3), PIO_OUTPUT_LOW); // black
	
	//comparator_init(acmux);
	colour_t data = GREY;
	while(1)
	{
		data = sensor_read(acmux);
		
		if (data == WHITE)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 5));
			pio_output_low(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 3));
			
			pio_output_low(PIO_DEFINE(PORT_B, 6));
		}
		else if (data == BLACK)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 3));
			pio_output_low(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 5));
			
			pio_output_low(PIO_DEFINE(PORT_B, 6));
		}
		else if (data == GREY)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 5));
			pio_output_low(PIO_DEFINE(PORT_B, 3));
			
			pio_output_low(PIO_DEFINE(PORT_B, 6));
		}
		else
		{
			pio_output_high(PIO_DEFINE(PORT_B, 6));
			
			pio_output_low(PIO_DEFINE(PORT_B, 3));
			pio_output_low(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 5));
		}
		
		//pio_output_toggle (PIO_DEFINE(PORT_B, 6));
	}
}