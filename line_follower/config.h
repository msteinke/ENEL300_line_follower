#ifndef CONFIG_H
#define CONFIG_H

//===========CONFIG=============
//Configuration for system settings

#ifndef F_CPU
#define F_CPU 8000000
#endif

//ADC sample rate
#ifndef SAMPLE_RATE
#define SAMPLE_RATE 200
#endif

//Rate at which sensors are checked for 
#ifndef MAZE_LOGIC_RATE 
#define MAZE_LOGIC_RATE 100 
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

//State enums
typedef enum{IDLE, SWEEP_LEFT, SWEEP_RIGHT, ON_WHITE, ON_BLACK, TURNING_LEFT, TRYING_LEFT} action;



#define DEFAULT_FORWARD_SPEED 70
#define DEFAULT_SPEED 170


#define MIN_TURN_SPEED 170
#define MAX_TURN_SPEED (255 - DEFAULT_FORWARD_SPEED)

#define IDEAL_SWEEP_TIME 400

#define GREY_TIME 200
#define LOST_TIME 500

typedef enum{BLACK, GREY, WHITE} level;
typedef enum{NC, FALLEN, RISEN} level_action;


#define SENSOR_TOLLERANCE 0

// GREY_THRESHOLD used to be 20
#define GREY_THRESHOLD 60
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