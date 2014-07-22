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

 #define SWEEP_TOLLERANCE 50 //miliseconds of sweep tollerenace before edge is not where expected

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
 typedef enum{
 	STRAIGHt_POSSBLE BIT(0),
 	STRAIGHT_NOT_POSSIBLE ~Bit(0),
 	LEFT_POSSIBLE BIT(1),
 	LEFT_NOT_POSSLBE ~BIT(1),
 	RIGHT_POSSIBLE BIT(2),
 	RIGHT_NOT_POSSIBLE ~BIT(2),
 	UNKNOWN BIT(7)
 } position;


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



int main(void)
{
	

	
	system_init();
	clock_init();
	led_init();	led_set(0x01); //show life
	UART_Init(BAUD); UART_Write("Init"); //Show UART life
	motor_init();
	adc_init();
	
	//Enable Analog pins
	adc_enable(AIN1); adc_enable(AIN2);	adc_enable(AIN3);
	//Analog input variables
	uint16_t a_in1 = 0;	uint16_t a_in2 = 0;	uint16_t a_in3 = 0;
	//Analog inputSignal conditioning arrays
	circBuf_t aLeft; circBuf_t aRight; circBuf_t aFront;			//TODO: initialise circbuffs
		
	//UART output buffer
	char buffer[UART_BUFF_SIZE] = {0};

	//=====Application specific variables=====								//TODO: initialise circbuff
	circBuf_t sweep_times;
	short del_t_last();

	bool sensors_updated = false;
	
	action current_action = IDLE;
	position current_position = UNKNOWN;


	//Scheduler variables
	uint32_t t = 0;	
	uint32_t sample_t_last = 0;
	uint32_t UART_t_last = 0;


	clock_set_ms(0);
	sei(); // Enable all interrupts
	UART_Write("ialized\n");
	

	short i = 0;
	while(1)
	{                                                                                                                         
		t = clock_get_ms();
		
		//Check if time threshold has been exceeded 
		if ((t - t_last) > (del_t_last + SWEEP_TOLLERANCE)
		{
			del_l_last = t - t_last;

			//turn possible
			if (currenct_action == SWEEP_LEFT)
																	//TODO: update information on current position

		}

		//check if sensor update has any relevant changes
		if (sensors_updated == true)
		{
			continue;
																	//TODO: add flag modificatoin for changed state
		}
		
		//Sensor update
 		if((t%SAMPLE_PERIOD == 0) & (t!=sample_t_last))
		{
			sensor_update(&ain_1, &a_in2, &ain_3);
			sample_t_last = t;
			sensors_updated = true;
		}
		
		//display degug information
		if((t%UART_PERIOD == 0) & (t != UART_t_last) & UART_ENABLED)
		{

			sprintf(buffer, "\r %u", t);
			UART_Write(buffer);
 		}
		
	}
}


void sensor_update(short* a_in1, short* a_in2, short* ain3)
{
	a_in1* = adc_measure(AIN1);
	_delay_us(20);
	a_in2* = adc_measure(AIN2);
	_delay_us(20);
	a_in3* = adc_measure(AIN3);
}
//65535