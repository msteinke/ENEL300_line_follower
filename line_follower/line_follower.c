/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 *
 * Comparator does funny stuff with portb, for PorbD io, 
 * check comparator.c
 */
/*
#include "config.h"

//System Includes
#include "system.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor.h"



int main(void)
{
	system_init();
	sei();
	motor_init();
	//adc_init();
	motor_testb();
	
	while (1)
	{
		continue;
	}
}	
*/


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


//Parameters
#ifdef ENABLE_UART
#define UART_ENABLED 1
#else
#define UART_ENABLED 0
#endif
#define UART_PERIOD (CLOCK_RATE_HZ/UART_RATE)
#define SAMPLE_PERIOD (CLOCK_RATE_HZ/SAMPLE_RATE)

 //Application specific enums
 typedef enum{IDLE, SWEEP_LEFT, SWEEP_RIGHT} action;


/*bitfield variable
* 	Bit one indicates posibility of forward travel (1 = possible)
	Bit two indicates left possible
	Bit three indicates right possible
*/
/*
 typedef enum{
 	STRAIGHT_POSSBLE = BIT(0),
 	STRAIGHT_NOT_POSSIBLE = ~Bit(0),
 	LEFT_POSSIBLE BIT(1),
 	LEFT_NOT_POSSLBE ~BIT(1),
 	RIGHT_POSSIBLE BIT(2),
 	RIGHT_NOT_POSSIBLE ~BIT(2),
 	UNKNOWN BIT(7)
 } position;
*/

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
int16_t integrate(int16_t* integrator, uint16_t current_value, uint16_t last_value, int16_t delta_t);
int16_t derivative(uint16_t current_value, uint16_t last_value, uint16_t delta_t);
int16_t regulate(int16_t value, uint16_t limit);
level level_get(uint16_t value);
level_action action_get(level current, level last);

void sweep_left(int16_t speed);
void sweep_right(int16_t speed);

void sensor_update(uint8_t channel, circBuf_t* readings, uint16_t* sensor_value);




int main(void)
{
	system_init();
	clock_init();
	led_init();	led_set(0x01); //show life
	UART_Init(BAUD); UART_Write("\nInit"); //Show UART life
	motor_init();
	adc_init();
	//motor_test();
	
	//Enable Analog pins
	adc_enable(CHANNEL_SENSOR_LEFT); 
    adc_enable(CHANNEL_SENSOR_RIGHT);	
    adc_enable(CHANNEL_SENSOR_FRONT);

    //Sensor state variables
    level left; level right; level front; 
	//senor previous state variables for edge detection
	level left_last; level right_last; level front_last;
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

	bool sweep_ended = FALSE;

	bool sensor_update_serviced = TRUE;
	
	bool buffer_updated = FALSE;
	
	
	action current_action = IDLE;
	//position current_position = UNKNOWN;
	
	int16_t error = 0;
	int16_t error_last = 0;
	int16_t error_integrator = 0;
	
	int16_t control = 0;
	
	int16_t left_speed = 0;
	int16_t right_speed = 0;
	
	//Scheduler variables
	uint32_t t = 0;	

	//Loop control time variables
	uint32_t time_check_t_last = 0;
	uint32_t sample_t_last = 0;
	uint32_t UART_t_last = 0;


	clock_set_ms(0);
	sei(); // Enable all interrupts
	UART_Write("ialized\n");

	//wait for start command
	//make sure the pullups enabled
	DDRD &= ~BIT(7);
	PORTD |= BIT(7);
	
	while((PIND & BIT(7)))
	{
		continue;
	}


    //set initial state
	current_action = SWEEP_LEFT;
	sweep_left(DEFAULT_SPEED);	

	
	while(1)
	{
		                                                                                                                         
		t = clock_get_ms();
		
		//Check for sweep action taking an unexpected amount of time. but dont check the very first sweep. 
/*
		if ((t != time_check_t_last) & (t > 2000))
		{
			time_check_t_last = t;
			if ((t - sweep_end_t_last) > (sweep_del_t_last + SWEEP_TIME_TOLLERANCE))
			{
				motor_stop();
				
				//turn possible
				if (current_action == SWEEP_LEFT)
					//current_position |= LEFT_POSSIBLE;
					//can go left so do so.
					sweep_left(DEFAULT_SPEED);
				else if (current_action == SWEEP_RIGHT)
					//current_position |= RIGHT_POSSIBLE;
					if(level_get(sensor_front_value) == BLACK)
						sweep_left(DEFAULT_SPEED);
					else
						sweep_right(DEFAULT_SPEED);	
						*			
			}
		}		
		*/

		//check if sensor update has any relevant changes
		if (sensor_update_serviced == FALSE)
		{
			sensor_update_serviced = TRUE;
			
			error = sensor_left_value - sensor_right_value; //error for proportional control
			
			control = error*KP + derivative(error, error_last, SAMPLE_PERIOD)*KD +
							integrate(&error_integrator, error, error_last, SAMPLE_PERIOD);
			
			//need to go right, slow down right motor

			left_speed = control*KP;
			right_speed =  -control*KP;
			
			left_speed = regulate(left_speed, 255);
			right_speed = regulate(right_speed, 255);



			sprintf(buffer, "left speed: %d, right speed: %d control: %d \n", left_speed, right_speed, control);
			UART_Write(buffer);
						
			//No motor control following, motor controll is done by PID, furthur on
			if (sensor_left_value + SENSOR_TOLLERANCE < sensor_right_value)
			{
				if (current_action == SWEEP_LEFT)
					sweep_ended = TRUE;
				UART_Write("sweep right: ");
				current_action = SWEEP_RIGHT;
			}
			else if(sensor_right_value + SENSOR_TOLLERANCE< sensor_left_value)
			{				
				if (current_action == SWEEP_RIGHT)
					sweep_ended = TRUE;
				UART_Write("sweep left: ");
				current_action = SWEEP_LEFT;
			}
			
			motor_set(left_speed, right_speed);
			
			//sprintf(buffer, "%u, %u \n", sensor_left_value, sensor_right_value);
			//UART_Write(buffer);

            //If a new sweep started this cycle, find how long it took
            if (sweep_ended)
            {
				sweep_ended = FALSE;
				sweep_del_t_last = t - sweep_end_t_last; 
				sweep_end_t_last = t;
				//sprintf(buffer, "sweep took: %u left: %u right: %u \n ", sweep_del_t_last, sensor_left_value, sensor_right_value);
				//UART_Write(buffer);
				writeCircBuf(&sweep_times, sweep_del_t_last);
			}
		}
		
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
		
		//display degug information
		/*
		if((t%UART_PERIOD == 0) & (t != UART_t_last) & UART_ENABLED)
		{
			

			
			//sprintf(buffer, "%u, %u, %u, %u \n", sensor_left_value , sensor_right_value , sensor_front_value, sweep_del_t_last);
			if (buffer_updated)
			{
				sprintf(buffer, "Sensors: %u, %u, %u, %u \n", sensor_left_value , sensor_right_value , sensor_front_value);
				UART_Write(buffer);
				buffer_updated = FALSE;

			}
			
 		}
		*/
		 
		
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

int16_t regulate(int16_t value, uint16_t limit)
{
	if (value > limit)
		return limit;
	else if (value < -limit)
		return -limit;
	else
		return value;
}

level level_get(uint16_t value)
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



void sweep_left(int16_t speed)
{  
    motor_set(speed*FF/100, speed);
}

void sweep_right(int16_t speed)
{
    motor_set(speed, speed*FF/100);
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
//65535