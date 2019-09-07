/*
 * uart.h
 *
 *  Created on: Sep 6, 2019
 *      Author: h2l
 */

#ifndef UART_H_
#define UART_H_


void setup_uart();

void put_uart(char c);
int serial_write(char* p);
char get_uart();

void put_int(int i);
void put_uint(unsigned int u);
void put_float(float f, uint8_t decimal);

#endif /* UART_H_ */
