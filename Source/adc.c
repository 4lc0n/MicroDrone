/*
 * adc.c
 *
 *  Created on: Jul 30, 2019
 *      Author: h2l
 */

#include "adc.h"
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>


void adc_init(uint8_t ain, bool interrupt){
	//set pin as input
	DDRC &= ~(0x01 << ain);
	//disable pullup
	PORTC &= ~(0x01 << ain);
	//
	ADMUX = ( ain & 0x0F) | (0x01 << 6);
	if(interrupt){
		ADCSRA |= (0x01 << 3);
	}
	else{
		ADCSRA &= ~(0x01 << 3);
	}
	//enable adc, clear interrupt, set prescaler to 128 (needed)
	ADCSRA |= (0x01 << 7) | (0x01 << 4) | (0x07 << 0);



}
uint16_t adc_read(void){
	uint16_t result = 0;
	//wait until previous conversion is finished
	while(ADCSRA & (0x01 << 6));

	//activate conversion
	ADCSRA |= (0x01 << 6) | (0x01 << 4);
	//wait...
	while(ADCSRA & (0x01 << 6));
	//return 12 bit value
	result = ADC;
	return result;

}


uint8_t adc_read8(void){
	//wait until previous conversion is finished
	while(ADCSRA & (0x01 << 6));

	//activate conversion
	ADCSRA |= (0x01 << 6);
	//wait...
	while(ADCSRA & (0x01 << 6));

	//only return high bits -> only 8 bits needed
	return ADC >> 2;

}
void adc_change_channel(uint8_t ain){
	//set pin as input
	DDRC &= ~(0x01 << ain);
	//disable pullup
	PORTC &= ~(0x01 << ain);

	//wait until last conversion finished
	while(ADCSRA & (0x01 << 6));
	ADMUX &= ~(0x0F);
	ADMUX |= (ain & 0x0F);
}


