
#include "system.h"
#include "led.h"

#define PBLED 0x1C
#define PCLED 0x80


void led_set_one(char led, char state)
{

	{
		PORTB &= ~BIT(led);
		PORTB |= (state<<led);
	}
}

void led_set(char p)
{
	PORTB &= ~PBLED; //KILL LEDS
	// set relevent bits
	PORTB |= ((p << 4)&BIT(7))|	//shift 4th bit to 7th
	((p<<2)&(BIT(2)|
	BIT(3)|
	BIT(4)));		//& 0-2 -> 2->4
	
	//PB2, PB3, PB4, PB7
}

void led_init(void)
{
	DDRB |= PBLED;
	DDRC |= BIT(7);
}