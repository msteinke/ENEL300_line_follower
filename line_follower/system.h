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





/* Button.  */
#define BUTTON1 0
#define BUTTON1_PIO PIO_DEFINE(PORT_D, 7)



#define LED0 0
#define LED0_PIO PIO_DEFINE(PORT_B, 0)

/* LED (active high).  */
#define LED1 0
#define LED1_PIO PIO_DEFINE(PORT_C, 2)



void system_init (void);

#endif
