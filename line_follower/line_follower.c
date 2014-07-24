/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 *
 * Comparator does funny stuff with portb, for PorbD io, 
 * check comparator.c
 */

#include "config.h"

#define TRUE 1
#define FALSE 0

//Config.h Dependant Parameters
#ifdef ENABLE_UART
#define UART_ENABLED 1
#else
#define UART_ENABLED 0
#endif

#define UART_PERIOD (CLOCK_RATE_HZ/UART_RATE)

#define SAMPLE_PERIOD (CLOCK_RATE_HZ/SAMPLE_RATE)

 #define MAZE_LOGIC_PERIOD (CLOCK_RATE_HZ/MAZE_LOGIC_RATE)

 //Application specific enums
 typedef enum{IDLE, SWEEP_LEFT, SWEEP_RIGHT, ON_WHITE, ON_BLACK} action;


//System Includes
#include "system.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
//Peripheral Includes
#include "motor.h"
#include "led.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "circBuf.h"


 #define CHANNEL_SENSOR_LEFT AIN1
 #define CHANNEL_SENSOR_RIGHT AIN2
 #define CHANNEL_SENSOR_FRONT AIN3


//PROTOTYPES
//PID numerical integral and derivative functions
int16_t integrate(int16_t* integrator, uint16_t current_value, uint16_t last_value, int16_t delta_t);
int16_t derivative(uint16_t current_value, uint16_t last_value, uint16_t delta_t);

//regulating function. limiting the abs value of value to limit
int16_t regulate(uint16_t value, uint16_t limit);
int16_t regulate_within(uint16_t value, uint16_t lower, uint16_t upper);


level level_get(int16_t value);
//Checks for rising and falling edges on quantises sensor levels
level_action action_get(level current, level last);

//quantises an analog value from sensor into levels defined in config.h
bool is_black(int16_t value)
{
	return level_get(value) == BLACK;
}

bool is_grey(int16_t value)
{
	return level_get(value) == GREY;
}

bool is_white(int16_t value)
{
	return level_get(value) == WHITE;
}

//hard turn one way or the other
void sweep_left(int16_t turn_speed);
void sweep_right(int16_t turn_speed);

//reads sensors
void sensor_update(uint8_t channel, circBuf_t* readings, uint16_t* sensor_value);

// win the maze game
void maze_completed(void);


int main(void)
{
	system_init();
	clock_init();
	led_init();	led_set(0x01); //show life
	UART_Init(BAUD); UART_Write("\nInit"); //Show UART life
	motor_init();
	adc_init();
	
	
	//Enable Analog pins
	adc_enable(CHANNEL_SENSOR_LEFT); 
    adc_enable(CHANNEL_SENSOR_RIGHT);	
    adc_enable(CHANNEL_SENSOR_FRONT);

    //Sensor state variables
    level left_level; level right_level; level front_level; 
	//senor previous state variables for edge detection
	level left_last_level; level right_last_level; level front_last_level;
	// sensor action variables
	level_action left_action = NC; 
	level_action right_action = NC; 
	level_action front_action = NC;
	
	//Sensor value variables
	uint16_t sensor_left_value = 0; uint16_t sensor_right_value  = 0; uint16_t sensor_front_value  = 0;

	//Analog inputSignal conditioning arrays
	circBuf_t left_buffer; circBuf_t right_buffer; 	circBuf_t front_buffer;

    //Initialise sensor averaging buffers
	initCircBuf(&left_buffer, ROLLING_AVERAGE_LENGTH);
	initCircBuf(&right_buffer, ROLLING_AVERAGE_LENGTH);
	initCircBuf(&front_buffer, ROLLING_AVERAGE_LENGTH);

		
	//UART output buffer
	char buffer[UART_BUFF_SIZE] = {0};

	//=====Application specific variables=====								//TODO: initialise circbuff
	circBuf_t sweep_times;
	initCircBuf(&sweep_times, SWEEP_TIME_MEMORY_LENGTH);
	short sweep_del_t_last = 0;
	short sweep_end_t_last = 0;
	
	//time when front sensor begins to see grey.
	uint32_t grey_time_start = 0;

	bool sweep_ended = FALSE;
	//set high if the front sensor crosses the line
	bool front_crossed_black = FALSE; 
	//set high if front finds finish line
	bool front_crossed_grey = FALSE;

	bool sensor_update_serviced = TRUE;
	
	bool buffer_updated = FALSE;
	
	
	action current_action = IDLE;
	
	int16_t forward_speed = DEFAULT_FORWARD_SPEED;
	int16_t turn_speed = DEFAULT_SPEED;
	
	//PID variables (not used)
	int16_t error = 0;
	int16_t error_last = 0;
	int16_t error_integrator = 0;
	
	int16_t control = 0;
	
	int16_t left_turn_speed = 0;
	int16_t right_turn_speed = 0;
	
	//Scheduler variables
	uint32_t t = 0;	

	//Loop control time variables
	uint32_t maze_logic_t_last = 0;
	uint32_t sample_t_last = 0;
	uint32_t UART_t_last = 0;


	clock_set_ms(0);
	sei(); // Enable all interrupts
	UART_Write("ialized\n");

	//wait for start command
	DDRD &= ~BIT(7);
	PORTD |= BIT(7);
	
	
	//motor_set(128, 128);
	while((PIND & BIT(7)))
	{
		continue;
	}
	
	

	/*
    //set initial state
	current_action = SWEEP_LEFT;
	sweep_left(DEFAULT_SPEED);	
	*/
	
	while(1)
	{
		                                                                                                                         
		t = clock_get_ms();
		
		//check if a sensor update has occured
		if ((sensor_update_serviced == FALSE) && 
			(t%MAZE_LOGIC_PERIOD == 0) && (t != maze_logic_t_last))
		{
			sensor_update_serviced = TRUE;

			// finishing condition is a grey read for a set period
			if(is_grey(sensor_front_value) && front_crossed_grey == FALSE)
			{
				front_crossed_grey = TRUE;
				grey_time_start = t;	                                   //TODO: adjust so that finishing condition is a 1/2 whole sweeps on grey line
			}
			else if (is_grey(sensor_front_value) && front_crossed_grey == TRUE)
			{
				//
				if ((grey_time_start + GREY_TIME) <= t )
				{
					// Finish line found. Stop robot.
					maze_completed(); // wait for button push
					front_crossed_grey = FALSE;
				}

				//front_crossed_grey = FALSE;
			}
			else
			{
				front_crossed_grey = FALSE;
			}
			
			//check for false finish line
			if(is_black(sensor_front_value)&&front_crossed_black == FALSE)
			{
				front_crossed_black = TRUE;
				if(front_crossed_grey)
					front_crossed_grey = FALSE; //false alarm
			}
	
							
					
			//when both rear sensors go black, this indicates an intersection (turns included).
			//try turning left.
			// ERROR: this statement never turns right. Something is wrong!
			/*if(is_black(sensor_left_value) && is_black(sensor_right_value))
			{
				sweep_left(255);				
				PORTB |= BIT(3);
				PORTB |= BIT(4);
				current_action = ON_BLACK;
			}*/
			
			//when both sensors are completely white this indicates a dead end or a tape-gap
			else if (is_white(sensor_left_value) && is_white(sensor_right_value))
			{
				PORTB &= ~BIT(3);
				PORTB &= ~BIT(4);
				current_action = ON_WHITE;
				//Check if the front sensor is on black, or has been during the last sweep.
				if(is_black(sensor_front_value) | front_crossed_black)
					motor_set(turn_speed, turn_speed);
				else if (is_white(sensor_front_value))
					motor_set(turn_speed, -turn_speed);
			}										
						
			else if (sensor_left_value + SENSOR_TOLLERANCE < sensor_right_value)
			{
				PORTB &= ~BIT(3);
				PORTB |= BIT(4);
				if (current_action == SWEEP_LEFT)
					sweep_ended = TRUE;
				current_action = SWEEP_RIGHT;
				motor_set(forward_speed + turn_speed, forward_speed);
			}
			else if(sensor_right_value + SENSOR_TOLLERANCE< sensor_left_value)
			{
				PORTB |= BIT(3);
				PORTB &= ~BIT(4);			
				if (current_action == SWEEP_RIGHT)
					sweep_ended = TRUE;
				current_action = SWEEP_LEFT;
				motor_set(forward_speed, forward_speed + turn_speed);
			}
			
			
			//sprintf(buffer, "%u, %u \n", sensor_left_value, sensor_right_value);
			//UART_Write(buffer);

            //If a new sweep started this cycle, find how long it took
            if (sweep_ended)
            {
				sweep_ended = FALSE;
				if (front_crossed_black)
					front_crossed_black = FALSE;
					/*
				if (front_crossed_grey)	//stop
					while(1)
					{
						continue;
					}
					*/
				sweep_del_t_last = t - sweep_end_t_last;
				sweep_end_t_last = t;
				writeCircBuf(&sweep_times, sweep_del_t_last);
				
				//adjust turn_speed for battery level.
				if (sweep_del_t_last > IDEAL_SWEEP_TIME)
				{
					UART_Write("| - |");
					//forward_speed += 5;
					turn_speed += 5;
				}					
				if (sweep_del_t_last < IDEAL_SWEEP_TIME)
				{
					UART_Write("| + |");
					//forward_speed -= 5;
					turn_speed -= 5;
				}					
					
				turn_speed = regulate_within(turn_speed, MIN_TURN_SPEED, MAX_TURN_SPEED);
				
			}
		}
		// Paced loop tasks
		
		//Sensor value update
 		if((t%SAMPLE_PERIOD == 0) & (t!=sample_t_last))
		{
            sample_t_last = t;
            //read in analog values
            sensor_update(CHANNEL_SENSOR_LEFT, &left_buffer, &sensor_left_value );
            sensor_update(CHANNEL_SENSOR_RIGHT, &right_buffer, &sensor_right_value );
            sensor_update(CHANNEL_SENSOR_FRONT, &front_buffer, &sensor_front_value );
			sensor_update_serviced = FALSE;

		}
		
		//display debug information
		
		if((t%UART_PERIOD == 0) & (t != UART_t_last) & UART_ENABLED)
		{
			UART_t_last = t;
			
			if(current_action == SWEEP_LEFT)
			{
				UART_Write("sweep left");
			}
			if(current_action == SWEEP_RIGHT)
			{
				UART_Write("sweep right");
			}
			if(current_action == ON_BLACK)
			{
				UART_Write("on black");
			}
			if(current_action == ON_WHITE)
			{
				UART_Write("on white");
			}
			sprintf(buffer, "| sweep time: %u turn_speed : %u", sweep_del_t_last, turn_speed);
			UART_Write(buffer);
			UART_Write("\n");
			/*
			sprintf(buffer, "Sensors: %u, %u, %u, %u \n", sensor_left_value , sensor_right_value , sensor_front_value);
			UART_Write(buffer);
			buffer_updated = FALSE;
			*/


			
 		}
		
		 
		
	}
}

int16_t integrate(int16_t* integrator, uint16_t current_value, uint16_t last_value, int16_t delta_t)
{
	*integrator += (current_value-last_value)*delta_t;
	*integrator = regulate(integrator, WINDUP_LIMIT);
	return integrator;
}

int16_t derivative(uint16_t current_value, uint16_t last_value, uint16_t delta_t)
{
	return (current_value-last_value)/delta_t;
}

int16_t regulate(uint16_t value, uint16_t limit)
{
	if (value > limit)
		return limit;
	else if (value < -limit)
		return -limit;
	else
		return value;
}

int16_t regulate_within(uint16_t value, uint16_t lower, uint16_t upper)
{
	if (value > upper)
		return upper;
	if (value < lower)
		return lower;
	else
		return value;
}

level level_get(int16_t value)
{
	if (value < GREY_THRESHOLD)
		return WHITE;
	else if (value < BLACK_THRESHOLD)
		return GREY;
	else
		return BLACK;
}

level_action action_get(level current, level last)
{
	if(current < last)
		return FALLEN;
	else if(current > last)
		return RISEN;
	else
		return NC;
}



void sweep_left(int16_t turn_speed)
{  
    motor_set(turn_speed*0*FF/100, turn_speed);
}

void sweep_right(int16_t turn_speed)
{
    motor_set(turn_speed, turn_speed*0*FF/100);
}


void sensor_update(uint8_t channel, circBuf_t* readings, uint16_t* sensor_value)
{
    //Read in analog values
	uint16_t new_value = adc_measure(channel);
    //Read value to be replaced from circular buffer
    short val_replaced = readings->data[readings->windex];
    //Remove value to be replaced from current average
    *sensor_value -= val_replaced/ROLLING_AVERAGE_LENGTH;
    //add new value to average and replace old value in buffer
    *sensor_value += new_value/ROLLING_AVERAGE_LENGTH;
    writeCircBuf(readings, new_value);
}

void maze_completed(void)
{
	motor_stop();
	led_set(0x0F);
	while((PIND & BIT(7)))
	{
		continue;
	}
}