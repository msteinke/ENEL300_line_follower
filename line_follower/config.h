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

//regularity to check for relevant changes
//in sensor state
#ifndef MAZE_LOGIC_RATE 
#define MAZE_LOGIC_RATE 100 
#endif

//UART settings for debugging. disabled because we're not debugging
#ifndef UART_PARAMS
#define UART_PARAMS

//#define ENABLE_UART
#define UART_RATE 4 //update frequency
#define UART_BUFF_SIZE 200
#define BAUD 51 //uart baud rate 9600

#endif

// buffer lengths
#ifndef ROLLING_AVERAGE_LENGTH
#define ROLLING_AVERAGE_LENGTH 5
#endif

#ifndef SWEEP_TIME_MEMORY_LENGTH 
#define SWEEP_TIME_MEMORY_LENGTH 5
#endif

// Maze solving constants
//Colour quantity enums
typedef enum{BLACK, GREY, WHITE} level;
	
//level thresholds, defining 
//black grey and white states
#define GREY_THRESHOLD 60
#define BLACK_THRESHOLD 240

//allowable error in sensor readings
#define SENSOR_TOLLERANCE 0

//State enums to manage robot actions
typedef enum{IDLE, SWEEP_LEFT, SWEEP_RIGHT, ON_WHITE, ON_BLACK, TURNING_LEFT, TRYING_LEFT} action;

//speed values, adjusted on the fly for battery voltage
#define DEFAULT_FORWARD_SPEED 50
#define DEFAULT_SPEED 170

#define MIN_TURN_SPEED 150
#define MAX_TURN_SPEED (255 - DEFAULT_FORWARD_SPEED)

#define IDEAL_SWEEP_TIME 400

//time for which front sensor must read grey to qualify stopping
#define GREY_TIME 200







#endif