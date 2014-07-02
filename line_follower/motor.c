/*
 * motor.c
 *
 * Created: 1/07/2014 4:52:17 p.m.
 *  Author: martin
 */ 


#include "motor.h"
#include "system.h"
#include "pio.h"

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
	//asdf
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