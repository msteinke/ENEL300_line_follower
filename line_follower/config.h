#ifndef CONFIG_H
#define CONFIG_H

//===========CONFIG=============
//Configuration for system settings

//ADC paramers
#define SAMPLE_RATE 50

#define AIN_CIRCBUF_SIZE 10

//UART settings for debugging
#define ENABLE_UART
#define UART_RATE 4 //update frequency
#define UART_BUFF_SIZE 32
#define BAUD 51 //uart baud rate 9600

//==========END-CONFIG==========

#endif
