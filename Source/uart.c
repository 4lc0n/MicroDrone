/*
 * uart.c
 *
 *  Created on: Sep 6, 2019
 *      Author: h2l
 */

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include "uart.h"


//init for uart: Baud 9600, 8N1
void setup_uart(){
	UBRR0L = 103;
	UBRR0H = 0;
	UCSR0B |= (0x01 << RXEN0) | (0x01 << TXEN0);
	UCSR0C |= (0x3 << UCSZ00);

}


//puts a char on uart
void put_uart(char c){
	while(!(UCSR0A & (0x01 << UDRE0)));
	UDR0 = c;
}


//writes a null-terminated string to uart0
int serial_write(char* p){
	int counter = 0;
	while(*p != 0){
		put_uart(*p);
		p++;
		counter++;
	}
	put_uart('\n');
	put_uart('\r');
	return counter;
}


//fetches a char from uart buffer (actually, no buffer)
char get_uart(){
	while( !(UCSR0A & (0x01 << RXC0)));
	return UDR0;
}





void put_uint(unsigned int u){
	unsigned int compare = 10000;
	unsigned int temp = 0;
	bool first = true;
	while(u < compare && compare >= 10)compare /= 10;

	do{
		if(first){
			first = false;

		}
		else{
			compare /= 10;

		}
		if(u >= compare){
			temp = u / compare;
			put_uart('0' + temp);
			u = u - (temp * compare);
		}
		else{
			put_uart('0');
		}


	}while(compare > 1);

}

void put_int(int i){
	unsigned int compare = 10000;
	unsigned int temp = 0;
	bool first = true;
	if(i < 0){
		put_uart('-');
		i *= -1;
	}
	while(i < compare && compare >= 10)compare /= 10;

	do{
		if(first){
			first = false;

		}
		else{
			compare /= 10;

		}
		if(i >= compare){
			temp = i / compare;
			put_uart('0' + temp);
			i = i - (temp * compare);
		}
		else{
			put_uart('0');
		}


	}while(compare > 1);

}


void put_float(float f, uint8_t decimal){


}
