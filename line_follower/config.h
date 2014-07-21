#ifndef CONFIG_H
#define CONFIG_H

//===========CONFIG=============
//Configuration for system settings

#ifndef F_CPU
#define F_CPU 8000000
#endif

//ADC paramers
#define SAMPLE_RATE 50

#define AIN_CIRCBUF_SIZE 10

//UART settings for debugging
#define ENABLE_UART
#define UART_RATE 4 //update frequency
#define UART_BUFF_SIZE 32
#define BAUD 51 //uart baud rate 9600


// Pin Allocations




/* Pin allocation for two motors. Choose 
   from PB7 and PD0 for PWM. Any GPIO pin for 
   motor direction. */


#endif
