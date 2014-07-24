#ifndef CONFIG_H
#define CONFIG_H

//===========CONFIG=============
//Configuration for system settings

#ifndef F_CPU
#define F_CPU 8000000
#endif

//ADC paramers
#ifndef SAMPLE_RATE
#define SAMPLE_RATE 200
#endif

// buffer lengths

#ifndef ROLLING_AVERAGE_LENGTH
#define ROLLING_AVERAGE_LENGTH 5
#endif

#ifndef SWEEP_TIME_MEMORY_LENGTH 
#define SWEEP_TIME_MEMORY_LENGTH 5
#endif


//PID constants (not used)

#define KP 20/100
#define KI 1/100
#define KD 0

#define WINDUP_LIMIT 50

// Maze solving constants

#define SENSOR_TOLLERANCE 0

#define DEFAULT_FORWARD_SPEED 100
#define DEFAULT_SPEED 100
#define MIN_TURN_SPEED 100
#define MAX_TURN_SPEED 150
#define FF 0 //forwardization factor %

#define IDEAL_SWEEP_TIME 180
#define SWEEP_TIME_TOLLERANCE 50 //miliseconds of sweep tollerenace before edge is not where expected


typedef enum{BLACK, GREY, WHITE} level;
typedef enum{NC, FALLEN, RISEN} level_action;
	
#define GREY_THRESHOLD 20
#define BLACK_THRESHOLD 240



//UART settings for debugging
#ifndef UART_PARAMS
#define UART_PARAMS

#define ENABLE_UART
#define UART_RATE 4 //update frequency
#define UART_BUFF_SIZE 200
#define BAUD 51 //uart baud rate 9600

#endif


// Pin Allocations
#define MOTOR_PIN_L_FWD PIO_DEFINE(PORT_B, 1)
#define MOTOR_PIN_L_RVSE PIO_DEFINE(PORT_B, 6)
#define MOTOR_PIN_R_FWD PIO_DEFINE(PORT_B, 5)
#define MOTOR_PIN_R_RVSE PIO_DEFINE(PORT_C, 7)

#define MOTOR_PIN_L_PWM PIO_DEFINE(PORT_D, 0)
#define MOTOR_PIN_R_PWM PIO_DEFINE(PORT_B, 7)



/* Pin allocation for two motors. Choose 
   from PB7 and PD0 for PWM. Any GPIO pin for 
   motor direction. */


#endif