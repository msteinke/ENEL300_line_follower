/*
 * clock.c
 *
 * Created: 6/07/2014 12:16:38 p.m.
 *  Author: martin
 */ 


/** Initialize PWM channels on timer1;
	PC5 and PC6.
	@param none
	@return none */
/*void motor_init_timer1(void)
{
	// Configure PWM 
	// pins: OC.1A & OC.1B
	//DDRC |= (1<<PC6) | (1<<PC5); 
	DDRC |= (1<<6) | (1<<5); 
	
	TCCR1A = (1<<COM1A1) |
			(0<<COM1A0) |
			(1<<COM1B1) |
			(0<<COM1B0) |
			(0<<COM1C1) |
			(0<<COM1C0) |
			(0<<WGM11) | // PWM phase correct 8bit mode 1
			(1<<WGM10);  // PWM phase correct 8bitmode 1
	
	TCCR1B = //(0<<FOC0A) | // must disable these for PWM
			//(0<<FOC0B) | // must disable these for PWM
			(0<<WGM13) |
			(0<<WGM12) | // PWM phase correct 8bitmode 1
			(0<<CS12) | // divide clock by 64
			(1<<CS11) | // divide clock by 64
			(1<<CS10); //divide clock by 64
	
	// write to the lower half of the 16 bit register.
	OCR1AL = MOTOR_INIT_DUTY_CYCLE; // PWM duty cycle on PB7 %85
	OCR1BL = MOTOR_INIT_DUTY_CYCLE; // PWM duty cycle on PD0 %170
	
	TIMSK1 = 0x00; // disable interrupts
	//TIMSK1 = 0x03; // enable interrupts
	
}*/