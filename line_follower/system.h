/** @file   system.h
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief  System specific definitions
*/
#ifndef SYSTEM_H
#define SYSTEM_H

/* Data typedefs.  */
#include <stdint.h>

typedef unsigned short bool;
typedef unsigned char byte;

/* Useful macros.  */
#define BIT(X) (1 << (X))

#define ARRAY_SIZE(ARRAY) (sizeof (ARRAY) / sizeof (ARRAY[0]))

#define  __unused__ __attribute__ ((unused))


/* Clock frequency Hz.  */
#define F_CPU 8000000

// THESE ACTUALLY DONT BELONG HERE
// Comparator pins
#define AIN1 = 0X01 //PIN PD2
#define AIN2 = 0X02 //PIN PD3
#define AIN3 = 0X03 //PIN PD4
#define AIN4 = 0X04 //PIN PD5
#define AIN5 = 0X05 //PIN PD6
#define AIN6 = 0X06 //PIN PD7
#define AIN7 = 0X07 //PIN PD7



/* Button.  */
#define BUTTON1 0
#define BUTTON1_PIO PIO_DEFINE(PORT_D, 7)



#define LED0 0
#define LED0_PIO PIO_DEFINE(PORT_B, 0)

/* LED (active high).  */
#define LED1 0
#define LED1_PIO PIO_DEFINE(PORT_C, 2)



/* Pin allocation for two motors. Choose 
   from PB7 and PD0 for PWM. Any GPIO pin for 
   motor direction. */
#define MOTOR_PIN_L_FWD PIO_DEFINE(PORT_D, 4)
#define MOTOR_PIN_L_RVSE PIO_DEFINE(PORT_D, 3)
#define MOTOR_PIN_R_FWD PIO_DEFINE(PORT_D, 2)
#define MOTOR_PIN_R_RVSE PIO_DEFINE(PORT_D, 1)

#define MOTOR_PIN_L_PWM PIO_DEFINE(PORT_B, 7)
#define MOTOR_PIN_R_PWM PIO_DEFINE(PORT_D, 0)










void system_init (void);

#endif
