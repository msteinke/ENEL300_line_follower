/*
led.h

Sam Stephenson

20/7/14

//portb LED bitmask
#define PBLED 0x9C
*/

#ifndef LED_H
#define LED_H

#include "system.h"
#ifdef AVR
#include <avr/io.h>
#endif

// Set one LED
// led: bit mask of LED
// state: true=on false=off
void led_set_one(char led, char state);

// Sets all LEDs
// P is the 4 bit bit pattern 
// inidicating LED states
// (can be used to display 4 bit binary
void led_set(char p);

// Initialises GPIO for LED operation
void led_init(void);

#endif