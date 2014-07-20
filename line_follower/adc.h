#ifndef ADC_H_
#define ADC_H_

#include "system.h"




//
void adc_init(void);


//
void adc_enable(uint8_t adc_channel);


//
void adc_disable(uint8_t adc_channel);


//
uint16_t adc_measure(uint8_t adc_channel);

#endif