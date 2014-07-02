/*
 * line_follower.c
 *
 * Created: 1/07/2014 3:42:31 p.m.
 *  Author: martin
 */


#include <avr/io.h>
#include <avr/interrupt.h> 
#include <avr/wdt.h>

// usefull macro
#define BIT(X) (1 << (X))

int main(void)
{	
	// ************************************************
	// Configure PWM            i/o pin:
	DDRB |= (1<<PB7); //			OC.0A
	//DDRC |= (1<<PC6) | (1<<PC5); //	OC.1A and OC.1B
	DDRD |= (1<<PD0); //			OC.0B
	
	TCCR0A = (1<<COM0A1) |
			(0<<COM0A0) |
			(1<<COM0B1) |
			(0<<COM0B0) |
			//(1<<WGM02) |  // PWM phase correct mode 5
			(0<<WGM01) | // PWM phase correct mode 5
			(1<<WGM00);  // PWM phase correct mode 5
		
	TCCR0B = //(0<<FOC0A) | // must disable these for PWM
			//(0<<FOC0B) | // must disable these for PWM
			(0<<WGM02) | // PWM phase correct mode 5
			(0<<CS02) | // divide clock by 64
			(1<<CS01) | // divide clock by 64
			(1<<CS00); //divide clock by 64
		
	OCR0A = 85; // PWM duty cycle on PB7 %85
	OCR0B = 170; // PWM duty cycle on PD0 %170
		
	TIMSK0 = 0x00; // disable interrupts
	//TIMSK0 = 0x03; // enable interrupts
	// ------------------------------------------------
	
	
	
	// ************************************************
	// Configure System Clock and watchdog
	CLKPR = BIT (CLKPCE);
	CLKPR = 0;
	
	//wdt_reset ();

    //MCUSR &= ~BIT (WDRF); // Clear WDRF in MCUSR.
    
	//Write logical one to WDCE and WDE and keep old prescaler
    //setting to prevent unintentional time-out.
    //WDTCSR |= BIT (WDCE) | BIT (WDE);
	
	//WDTCSR = 0x00; // Turn off watchdog timer
	// ------------------------------------------------
	
	
	
    cli(); // disable all interrupts
	//sei(); // Enable all interrupts
	
	
	
	DDRD |= PD2; // LED on PD2
	//PORTD |= PD2; // turn on LED
 
 
	while(1)
	{
		PORTD ^= PD2; // toggle an LED on PD2
	}
}


/*
#include <avr/io.h>
#include "system.h"
#include <avr/interrupt.h> 
#include "motor.h"

int main(void)
{
	DDRD |= 0x02;
	PORTD |= 0x02;
	cli(); // disable all interrupts
	//sei(); // Enable all interrupts
	system_init();
	motor_init();
 
	//volatile long i = 0;
 
	while(1)
	{
		//TODO:: Please write your application code
		
		PORTD ^= 0x02; // toggle an LED on PD2
		//PORTD = TCNT0 
 
		for (i = 0; i < 10000; i++)
		{
			continue;
		}
	}
}*/




/*
void init(void) {
	DDRB = 0xff;
	PORTB = 0x00;

	TCCR0A = (1 << WGM01);
	// 1024 prescaler
	TCCR0B = ((1 << CS02) | (1 << CS00));
	// Interrupt every 4096 clocks
	OCR0A = 85;
	// Enable timer compare match interrupt
	TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
	PORTB++;
}

int main(void) {
	init();

	sei();

	while(1);

	return 0;
}
*/