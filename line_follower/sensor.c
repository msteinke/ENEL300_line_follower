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

void sensor_init(void)
{
	comparator_init();
	pio_config_set(SENSOR_EMUX_SELECT_PIN, PIO_OUTPUT_LOW);
	pio_config_set(SENSOR_EMUX_VREF_PIN, PIO_INPUT);
}

bool higher_than_grey(byte acmux)
{
	pio_output_high(SENSOR_EMUX_SELECT_PIN); // 1.3V
	
	return comparator_higher_than(acmux);
}

bool lower_than_grey(byte acmux)
{
	pio_output_low(SENSOR_EMUX_SELECT_PIN); // 3.1V
	
	return !comparator_higher_than(acmux);
}

colour_t sensor_read(byte acmux)
{
	//comparator_higher_than(sensorPin);
	
	// is it more than grey?
	// is it less than grey?
	// else it is grey
	
	//pio_output_high(SENSOR_EMUX_SELECT_PIN); // 1.3V
	//pio_output_low(SENSOR_EMUX_SELECT_PIN); // 3.1V
	
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
	

void sensor_test(byte acmux)
{
	pio_config_set(PIO_DEFINE(PORT_B, 6), PIO_OUTPUT_LOW);
	
	pio_config_set(PIO_DEFINE(PORT_B, 5), PIO_OUTPUT_LOW); // white
	pio_config_set(PIO_DEFINE(PORT_B, 4), PIO_OUTPUT_LOW); // grey
	pio_config_set(PIO_DEFINE(PORT_B, 3), PIO_OUTPUT_LOW); // black
	
	//comparator_init(acmux);
	
	while(1)
	{
		
		if (sensor_read(acmux) == WHITE)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 5));
			pio_output_low(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 3));
			
			pio_output_low(PIO_DEFINE(PORT_B, 6));
		}
		else if (sensor_read(acmux) == GREY)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 5));
			pio_output_low(PIO_DEFINE(PORT_B, 3));
			
			pio_output_low(PIO_DEFINE(PORT_B, 6));
		}
		else if (sensor_read(acmux) == BLACK)
		{
			pio_output_high(PIO_DEFINE(PORT_B, 3));
			pio_output_low(PIO_DEFINE(PORT_B, 4));
			pio_output_low(PIO_DEFINE(PORT_B, 5));
			
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