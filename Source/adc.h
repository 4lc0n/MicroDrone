/*
 * adc.h
 *
 *  Created on: Jul 30, 2019
 *      Author: h2l
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>

void adc_init(uint8_t ain, bool interrupt);
uint16_t adc_read(void);
uint8_t adc_read8(void);

void adc_change_channel(uint8_t ain);


#endif /* ADC_H_ */
