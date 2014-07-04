/*
 * comparator.h
 *
 * Created: 1/07/2014 5:07:26 p.m.
 *  Author: martin
 */ 


#ifndef COMPARATOR_H_
#define COMPARATOR_H_

// must do this because it's not defined
// else where for me.
//#ifndef ACMUX
	#define ACMUX _SFR_IO8(0x7D)
//#endif

typedef unsigned short bool;

enum adc_pins {
	AIN1_MUX = 0x00, // Pin PD2
	AIN2_MUX = 0x01, // Pin PC2
	AIN3_MUX = 0x02, // Pin PD4
	AIN4_MUX = 0x03, // Pin PD5
	AIN5_MUX = 0x04, // Pin PD6
	AIN6_MUX = 0x05, // Pin PD7
};

/** Initialize Analog comparator on AIN1
	@param none
	@return none */
void comparator_init(void);

/** Changes the selected comparator pin
	@param mux - configure mux register
	@return none */
void comparator_mux(short mux);

/** Read the comparator value 
	@param mux - chose which pin to measure on
	@return - the comparator value, ACO. */
bool comparator_read(short mux);

/** Simple test program for this module.
    Place a voltage on AIN0 (PD1)
	and another voltage on AIN1 (PD2)
	and an LED on pin PB7.
	@param mux - configure mux register
	@return none */
void comparator_test(short mux);



#endif /* COMPARATOR_H_ */