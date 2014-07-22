/** @file   adc.c
    @author M. P. Hayes, UCECE
    @date   23 April 2013
    @brief  Simple ADC using a comparator and RC network.
*/
#include "avr/io.h"
#include "pio.h"
//#include "delay.h"
#include "adc.h"
 
/**
   This requires a capacitor connected from AIN0 (P1-3) to ground
   (P1-1) and a resistor connected from AIN0 (P1-3) to PD5 (P1-7).
   The unknown input voltage is applied to AIN3 (P1-6).
 
   It operates by applying a fixed voltage from PD5 (5 V, assuming the
   UCFK4 is powered from USB) to the capacitor and timing how long it
   takes for the capacitor voltage to exceed the unknown voltage using
   the comparator. 
 
   Here's the typical usage for two channels.:
 
   adc_init ();
   adc_enable (ADC_CHANNEL1);
   adc_enable (ADC_CHANNEL2);
 
   adc_measure (ADC_CHANNEL1);
 
   wait until capacitor discharged
 
   adc_measure (ADC_CHANNEL2);
 
   Note adc_measure blocks until the voltage on the capacitor exceeds
   the voltage being measured.  adc_measure cannot be called again
   until the capacitor is discharged.  This could be greatly sped up
   by using another PIO directly connected to the capacitor.  Better
   still, since PD1 shares the same pin as AIN0, it can be configured
   as a low output to more rapidly discharge the capacitor.
 
   From the IBIS file it appears that a PIO can sink/source 60 mA
   (typ) at 25 deg. C.  The datasheet says that the maximum PIO DC
   current is 40 mA.
 
 */
#define ADC_MAX 254

#ifndef PC6_PIO
#define PC6_PIO PIO_DEFINE(PORT_D, 5)
#endif

#ifndef ADC_CHARGE_PIO
#define ADC_CHARGE_PIO PC6_PIO
#endif

#ifndef PD1_PIO
#define PD1_PIO PIO_DEFINE(PORTD, 1)
#endif

#ifndef ADC_MEA_PIO
#define ADC_MEA_PIO PD1_PIO
#endif
 
#ifndef ADC_CAPACITANCE
#define ADC_CAPACITANCE 4.7e-9
#endif
 
#ifndef ADC_RESISTANCE
#define ADC_RESISTANCE 37e3
#endif
 
#define ADC_TIME_CONSTANT_US (ADC_CAPACITANCE * ADC_RESISTANCE * 1e6)
 
/* This is not defined in io32u2.h  */
#ifndef ACMUX
#define ACMUX _SFR_MEM8(0x7D)
#endif
 
void adc_init(void)
{
    pio_config_set(ADC_CHARGE_PIO, PIO_OUTPUT_LOW);
 
    /* Disable digital input for AIN0.  */
    DIDR1 |= BIT(0);
	
 
    /* By default the analog comparator is enabled but let's enable it.  */
    ACSR &= BIT(7);
	
	 //Set ADC muxed input pins as inputs
	 DDRD &= ~(BIT(4)|BIT(2));
	 DDRC &= ~BIT(2);
	
	//Set disable output on ADC measurment pin
	pio_config_set(ADC_MEA_PIO, PIO_INPUT);
	
	//Set charge capacitor as an output
	pio_config_set(ADC_CHARGE_PIO, PIO_OUTPUT_LOW);

}
 
 
void adc_enable(uint8_t adc_channel)
{
   /* Disable digital input for selected channel.  */
    DIDR1 |= BIT(adc_channel);
}
 
 
void adc_disable(uint8_t adc_channel)
{
   /* Enable digital input for selected channel.  */
    DIDR1 &= ~BIT(adc_channel);
}
 
 
uint16_t adc_measure(uint8_t adc_channel)
{
    uint16_t count;
 
    count = 0;
 
    /* Select desired channel.  */
    ACMUX = adc_channel - 1;
	
	//pio_config_set(ADC_MEA_PIO, PIO_INPUT);
	DDRD &= ~BIT(1);
 
    //pio_output_high (ADC_CHARGE_PIO);
	PORTD |= BIT(5);
	
    while (! (ACSR & BIT(ACO)))
    {
		count++;
		if (count > ADC_MAX)
		{
			break;
		}
	}		
 
	//Stop charging capacitor
    pio_output_low (ADC_CHARGE_PIO);
	
	
	//Discharge capacitor through PD0
	//pio_config_set(ADC_MEA_PIO, PIO_OUTPUT_LOW);
	PORTD &= ~BIT(1);
	DDRD |= BIT(1); 
 
    return count;
}