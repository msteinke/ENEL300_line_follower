/*
 * motor.h
 *
 * Created: 1/07/2014 4:52:05 p.m.
 *  Author: martin
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

/** Initialize motor PWM timers
	@param none
	@return none */
void motor_init(void);


/** Control each motor from -127 to +128.
	@param left_speed
	@param right_speed */
void motor_set(short left_speed, short right_speed);

/** calls motor_set and sets each motor speed
	to zero.
	@param none
	@return none */
void motor_stop(void);

/** Simple test program that pulses the PWM
    channels so that it is obvious if it works.
	An LED is on PB6 foor debugging.
	@param none
	@return none */
void motor_test(void);

#endif /* MOTOR_H_ */