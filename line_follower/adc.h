#ifndef ADC_H_
#define ADC_H_

// THESE ACTUALLY DONT BELONG HERE
// Comparator pins
#define AIN1 0X01 //PIN PD2
#define AIN2 0X02 //PIN PC2
#define AIN3 0X03 //PIN PD3
#define AIN4 0X04 //PIN PD4
#define AIN5 0X05 //PIN PD5
#define AIN6 0X06 //PIN PD6

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