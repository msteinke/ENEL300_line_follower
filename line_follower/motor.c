/*
 * motor.c
 *
 * Created: 1/07/2014 4:52:17 p.m.
 *  Author: martin
 */ 


#include "motor.h"
#include "system.h"
#include "pio.h"
#include <avr/io.h>

// maybe move this to system.h?!!
#define LEFT_MOTOR_FWD_PIN 0
#define LEFT_MOTOR_RVS_PIN 0
#define RIGHT_MOTOR_FWD_PIN 0
#define RIGHT_MOTOR_RVS_PIN 0

/** Initialize motor PWM timers
	@param none
	@return none */
void motor_init(void)
{
	/*
	TCCR0A |= (1<<COM0A1) | 
			(1<<COM0A0) | 
			(1<<COM0B1) | 
			(0<<COM0B0) | 
			(0<<WGM02) |  //PWM phase correct mode
			(0<<WGM01) | 
			(1<<WGM00);
			
	TCCR0B |= (0<<FOC0A) | // must disable these for PWM
			(0<<FOC0B) |
			(0<<WGM02) |
			(1<<CS02) | // divide clock by 1024
			(0<<CS01) |
			(1<<CS00);
	
	OCR0A = 85; // PWM duty cycle on PB7
	OCR0B = 170; // PWM duty cycle on PD0
	
	TIMSK0 = 0x00; // disable interrupts
	*/
	//DDRB |=(1<<PB3);
	
	DDRB |= (1<<7);
	DDRC |= (1<<6) | (1<<5);
	DDRD |= (1<<0);
	
	TCCR0A = (1<<COM0A1) |
			(0<<COM0A0) |
			(1<<COM0B1) |
			(1<<COM0B0) |
			(1<<WGM02) |  // PWM phase correct mode 5
			(0<<WGM01) | // PWM phase correct mode 5
			(1<<WGM00);  // PWM phase correct mode 5
		
	TCCR0B = (0<<FOC0A) | // must disable these for PWM
			(0<<FOC0B) | // must disable these for PWM
			(1<<WGM02) | // PWM phase correct mode 5
			(1<<CS02) | // divide clock by 1024
			(0<<CS01) | // divide clock by 1024
			(1<<CS00); //divide clock by 1024
		
	OCR0A = 85; // PWM duty cycle on PB7
	OCR0B = 170; // PWM duty cycle on PD0
	
	TIMSK0 = 0x00; // disable interrupts
	//TIMSK0 = 0x03; // enable interrupts
}


/** Control each motor from -127 to +128.
	@param left_speed
	@param right_speed */
void motor_set(short left_speed, short right_speed)
{
	//asdf
}

/** calls motor_set and sets each motor speed
	to zero.
	@param none
	@return none */
void motor_stop(void)
{
	motor_set(0,0);
}

/*
The Timer/Counter (TCNT0) and Output Compare Registers (OCR0A and OCR0B) are 8-bit
registers.

TCCR0A - Timer/Counter Control Register A
TCCR0B - Timer/Counter Control Register B
TCNT0 - Timer/Counter Register
OCR0A - Output Compare Register A
OCR0B - Output Compare Register B
TIMSK0 - Timer/Counter Interrupt Mask Register
TIFR0 - Timer/Counter 0 Interrupt Flag Register

http://extremeelectronics.co.in/avr-tutorials/pwm-signal-generation-by-using-avr-timers-part-ii/


OC.0A - PB7*
OC.0B - PD0
OC.1A - PC6
OC.1B - PC5

OC.1C - PB7*

*/