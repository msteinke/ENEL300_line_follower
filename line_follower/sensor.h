/*
 * sensor.h
 *
 * Created: 4/07/2014 12:51:10 p.m.
 *  Author: martin
 */ 
/*
	AIN ---> PD1
	AIN1_MUX = 0x00, // Pin PD2
	AIN2_MUX = 0x01, // Pin PC2
	AIN3_MUX = 0x02, // Pin PD4
	AIN4_MUX = 0x03, // Pin PD5
	AIN5_MUX = 0x04, // Pin PD6
	AIN6_MUX = 0x05, // Pin PD7
	
	
	Instead of using an external multiplexer, we could use GPIO
	pins to power each IR sensor. i.e a transistor without an
	applied voltage sends out zero volts. 
	*/

#ifndef SENSOR_H_
#define SENSOR_H_

#include "comparator.h"

// IR sensor pin locations on ACMUX
#define SENSOR_LEFT_PIN   AIN5_MUX
#define SENSOR_MIDDLE_PIN AIN4_MUX
#define SENSOR_RIGHT_PIN  AIN3_MUX

// Voltage reference pins on external mux
#define SENSOR_EMUX_SELECT_PIN PIO_DEFINE(PORT_D, 2)
#define SENSOR_EMUX_VREF_PIN   PIO_DEFINE(PORT_D, 1) /*this is always PD1 (AIN0)*/

// Definitions for the three colours that must be detected.
typedef enum {WHITE, GREY, BLACK} colour_t; 

/** Initialize the comparator and GPIO
	@param none
	@return none*/
void sensor_init(void);

/** Determine if a ACMUX pin reads
	WHITE, GREY or BLACK.
	@param ACMUX pin
	@return colour type*/
colour_t sensor_read(byte acmux);

/** Simple test program to see if this module
	works or not. Requires (atleast) 3 LEDs.
	@param ACMUX pin that has POT connected to it
	@return none*/
void sensor_test(byte acmux);



#endif /* SENSOR_H_ */