/*
 * motor.c
 *
 * Created: 1/07/2014 4:52:17 p.m.
 *  Author: martin
 */ 


#include "motor.h"
#include "system.h"
//#include "pio.h"
#include <avr/io.h>
#include "pio.h"

#define MOTOR_INIT_DUTY_CYCLE 0


// check that everything has been defined
#ifndef MOTOR_PIN_L_FWD
#error "MOTOR_PIN_L_FWD is undefined"
#endif

#ifndef MOTOR_PIN_L_RVSE
#error "MOTOR_PIN_L_RVSE is undefined"
#endif

#ifndef MOTOR_PIN_R_FWD
#error "MOTOR_PIN_R_FWD is undefined"
#endif

#ifndef MOTOR_PIN_R_RVSE
#error "MOTOR_PIN_R_RVSE is undefined"
#endif


// figure out which PWM register goes
// with which assigned pin. 
#if MOTOR_PIN_L_PWM == PB7
    #define LEFT_DUTY OCR0A
#elif MOTOR_PIN_L_PWM == PD0
    #define LEFT_DUTY OCR0B
#else 
    #error "MOTOR_PIN_L_PWM is undefined"
#endif

#if MOTOR_PIN_R_PWM == PB7
    #define RIGHT_DUTY OCR0A
#elif MOTOR_PIN_R_PWM == PD0
    #define RIGHT_DUTY OCR0B
#else
    #error "MOTOR_PIN_R_PWM is undefined"
#endif




/** Initialize PWM channels on timer0;
	PB7 and PD0.
	@param none
	@return none */
void motor_init_timer0(void)
{
	// Configure PWM.    i/o pin:
	DDRB |= (1<<PB7); // OC.0A
	DDRD |= (1<<PD0); // OC.0B
	
	TCCR0A = (1<<COM0A1) |
			(0<<COM0A0) |
			(1<<COM0B1) |
			(0<<COM0B0) |
			(0<<WGM01) | // PWM phase correct mode 5
			(1<<WGM00);  // PWM phase correct mode 5
	
	TCCR0B = //(0<<FOC0A) | // must disable these for PWM
			//(0<<FOC0B) | // must disable these for PWM
			(0<<WGM02) | // PWM phase correct mode 5
			(0<<CS02) | // divide clock by 64
			(1<<CS01) | // divide clock by 64
			(1<<CS00); //divide clock by 64
	
	OCR0A = MOTOR_INIT_DUTY_CYCLE; // PWM duty cycle on PB7 %85
	OCR0B = MOTOR_INIT_DUTY_CYCLE; // PWM duty cycle on PD0 %170
	
	TIMSK0 = 0x00; // disable interrupts
	//TIMSK0 = 0x03; // enable interrupts
	
}


/** Initialize motor PWM timers
	@param none
	@return none */
void motor_init(void)
{
	pio_config_set (MOTOR_PIN_L_FWD, PIO_OUTPUT_LOW);
	pio_config_set (MOTOR_PIN_L_RVSE, PIO_OUTPUT_LOW);
	pio_config_set (MOTOR_PIN_R_FWD, PIO_OUTPUT_LOW);
	pio_config_set (MOTOR_PIN_R_RVSE, PIO_OUTPUT_LOW);
	
	motor_init_timer0();
	//motor_init_timer1();
	
}


/** Control motor from -255 to +255.
	@param motor
	@param duty 
	@param direction */
void motor_set_one(byte motor, byte duty, byte direction)
{
	if (motor == MOTOR_LEFT && direction == MOTOR_FWD)
	{
		LEFT_DUTY = duty;
		
		// BE CAREFUL!!
		// Set low before high otherwise you'll make
		// a short circuit inside the driver.
		pio_output_low(MOTOR_PIN_L_RVSE);
		pio_output_high(MOTOR_PIN_L_FWD);
	}
	else if (motor == MOTOR_LEFT && direction == MOTOR_RVSE)
	{
		LEFT_DUTY = duty;
		
		// BE CAREFUL!!
		// Set low before high otherwise you'll make
		// a short circuit inside the driver.
		pio_output_low(MOTOR_PIN_L_FWD);
		pio_output_high(MOTOR_PIN_L_RVSE);
	}
	else if (motor == MOTOR_RIGHT && direction == MOTOR_FWD)
	{
		RIGHT_DUTY = duty;
		
		// BE CAREFUL!!
		// Set low before high otherwise you'll make
		// a short circuit inside the driver.
		pio_output_low(MOTOR_PIN_R_RVSE);
		pio_output_high(MOTOR_PIN_R_FWD);
	}
	else if (motor == MOTOR_RIGHT && direction == MOTOR_RVSE)
	{
		RIGHT_DUTY = duty;
		
		// BE CAREFUL!!
		// Set low before high otherwise you'll make
		// a short circuit inside the driver.
		pio_output_low(MOTOR_PIN_R_FWD);
		pio_output_high(MOTOR_PIN_R_RVSE);
	}
}

/** Control each motor from -255 to +255.
	@param left_speed
	@param right_speed */
void motor_set(short left_speed, short right_speed)
{
	// do left motor first
	if (left_speed >= 0)
	{
		motor_set_one(MOTOR_LEFT, left_speed, MOTOR_FWD);
	}
	else if (left_speed < 0)
	{
		motor_set_one(MOTOR_LEFT, -left_speed, MOTOR_RVSE);
	}
	
	// now let's do the right motor
	if (right_speed >= 0)
	{
		motor_set_one(MOTOR_RIGHT, right_speed, MOTOR_FWD);
	}
	else if (right_speed < 0)
	{
		motor_set_one(MOTOR_RIGHT, -right_speed, MOTOR_RVSE);
	}
}

/** calls motor_set and sets each motor speed
	to zero.
	@param none
	@return none */
void motor_stop(void)
{
	motor_set(0,0);
}


/** Simple test program that pulses the PWM
    channels so that it is obvious if it works.
	An LED is on PB6 for debugging.
	@param none
	@return none */
void motor_test(void)
{
		long i = 0;
		
		short j = -255;
		short k = 1;
		
		short x = 59;
		short y = 1;
		
		DDRB |= (1<<PB6);
		
		while(1)
		{
			// set the PWM duty cycle.
			motor_set(j, x);
			
			// delay loop
			for (i = 0; i < 10000; i++)
			{
				continue;
			}
			
			//
			if (j == 255)
			{
				k = -1;
			}
			else if (j == -255)
			{
				k = 1;
			}
			//
			if (x == 255)
			{
				y = -1;
			}
			else if (x == -255)
			{
				y = 1;
			}
			
			// increment duty cycle
			j += k;
			x += y;
			
			PORTB ^= (1<<PB6);
		}
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