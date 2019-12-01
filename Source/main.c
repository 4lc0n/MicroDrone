/*
 * main.c
 *
 *  Created on: Sep 5, 2019
 *      Author: h2l
 *
 *      Code for a tiny drone controlled with a atmega328p @ 16 MHz
 *      RC: pins:			D9 PB1, D10 PB2, D12 PB4, D2 PD2, D4 PD4, D7 PD7
 *      Motor output pins. 	VR: D5 PD5 , HR: D6 PD6, HL D11 PB3,  VL D3, PD3
 *      Battery Pin: 		A0 PC0
 *      I2C:				A4 (SDA) PC4, A5 (SCL) PC5
 *      LED: 				D8 PB0 (doesn't work? )
 *
 *      for special purpose: D13
 *
 *
 *      Used Timers:
 *      Timer 1: Motor PWM
 *      Timer 2: RC Input Capture
 *      Timer 3: Motor PWM
 *
 *		Motor HR: 0A
 *		Motor VR: 0B
 *		Motor HL: 2A
 *		Motor VL: 2B
 *
 *
 *					 --__--
 *	    Reset	PC6 |1   28| PC5  	SCL
 *	    Rx		PD0 |2   27| PC4 	SDA
 *	    Tx		PD1 |3 	 26| PC3
 *	    RC		PD2 |4	 25| PC2
 *	    M-VL	PD3 |5	 24| PC1
 *	    RC		PD4 |6	 23| PC0	Battery
 *	    +		VCC |7	 22| GND	-
 *	    -		GND |8	 21| AREF	+
 *	    XTAL	PB6 |9	 20| VCC	+
 *	    XTAL	PB7 |10	 19| PB5	Special purpose D13
 *	    M-VR	PD5 |11	 18| PB4	RC
 *	    M-HR	PD6 |12  17| PB3	Motor HL
 *	    RC		PD7 |13  16| PB2	RC
 *	    LED		PB0 |14	 15| PB1	RC
 *					 ------
 *
 *		Todo check leds
 *		Done activate watchdog for emergency reset with check on reset cause on startup
 *		Done reset watchdog on every loop
 *		Done implement tick timer for constant loop time (250 Hz) (used timer0_ovf % 31)
 *		Todo implement error codes for light
 *		Todo implement WS2812 LED for status
 *
 *
 *		Experimental: Ch5, CH6 für PID tuning:
 *		CH5: P
 *		CH6: D
 *
 */


#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <signal.h>
#include <avr/wdt.h>

#include "i2cmaster.h"
#include "uart.h"
#include "adc.h"
//#include "ws2812_config.h"
//#include "light_ws2812.h"

#define GYRO_ADRESS (0x68 << 1)

/*
 * Variables
 */

float pid_p_gain_roll = 1.6;               //Gain setting for the roll P-controller (2.4 / 1.5)
float pid_i_gain_roll = 0.001;              //Gain setting for the roll I-controller (0.002)
float pid_d_gain_roll = 2;                //Gain setting for the roll D-controller (3.2)
const int pid_max_roll = 150;                    //Maximum output of the PID-controller (+/-)


float pid_p_gain_pitch = 1.6;  		//Gain setting for the pitch P-controller
float pid_i_gain_pitch = 0.001;  		//Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 2;  		//Gain setting for the pitch D-controller.
const int pid_max_pitch = 150;          		//Maximum output of the PID-controller (+/-)

const float pid_p_gain_yaw = 1.0;                //Gain setting for the pitch P-controller. //4.0
const float pid_i_gain_yaw = 0.0003;               //Gain setting for the pitch I-controller. //0.02
const float pid_d_gain_yaw = 1.0;                //Gain setting for the pitch D-controller.
const int pid_max_yaw = 150;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start = 0;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

int gyro_pitch, gyro_roll, gyro_yaw;
int gyro_axis[4], gyro_axis_cal[4];

uint8_t gyro_value[16];

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;



int loop_counter = 0;
volatile unsigned int tick = 0;



volatile uint16_t rc_signal[7];





void setup_interrupts();
void setup_timers();
void setup_gyro();
void read_gyro();
void calculate_pid();

int main(){

	wdt_disable();


	//setup outputs for Motor control, LED, Misc
	DDRB |= (0x01 << 0) | (0x01 << 3) | (0x01 << 5);
	DDRD |= (0x01 << 6) | (0x01 << 5) | (0x01 << 3);

	setup_timers();

	setup_interrupts();
	uint8_t reset_cause = MCUSR;
	MCUSR &= 0xF8;
	if(reset_cause == 0x08){
		//watchdog timer reset: fast setup
		gyro_axis_cal[0] = 0;
		gyro_axis_cal[1] = 0;
		gyro_axis_cal[2] = 0;
		gyro_axis_cal[3] = 0;

		i2c_init();
		setup_gyro();
		adc_init(0, false);
		sei();
		//jump right into the main loop
	}
	else{
		//normal startup routine

		setup_uart();
		serial_write("uart initialized");
		i2c_init();

		i2c_start(GYRO_ADRESS+I2C_WRITE);
		i2c_write(117);
		i2c_rep_start(GYRO_ADRESS+I2C_READ);
		char temp = i2c_readNak();
		i2c_stop();
		serial_write("Gyro ID:");
		put_int(temp);
		serial_write("");
		if(temp != 0x70){
			serial_write("gyro error!");
			while(1){
				_delay_ms(50);
				PORTB ^= (0x01 << PB5);
			}
		}

		serial_write("setting up gyro");
		setup_gyro();

		gyro_axis_cal[0] = 0;
		gyro_axis_cal[1] = 0;
		gyro_axis_cal[2] = 0;

		start = 0;
		int32_t gyro_axis_cal32[3] = {0};
		serial_write("claibrating gyro offset...");
		for(cal_int = 0; cal_int < 2000; cal_int++){
			read_gyro();
			gyro_axis_cal32[0] += gyro_pitch;
			gyro_axis_cal32[1] += gyro_roll;
			gyro_axis_cal32[2] += gyro_yaw;

			if(cal_int % 200)PORTB ^= (0x01 << PB5);
			_delay_us(900);
		}

		for(int i = 0; i < 3; i++){
			gyro_axis_cal32[i] /= 2000;
			gyro_axis_cal[i] = gyro_axis_cal32[i];
		}






		serial_write("initialize adc for battery voltage");

		adc_init(0, false);
		//4.2V: 859
		//3.3V: 675
		//3.0V: 618

		battery_voltage = adc_read();
		if(battery_voltage < 675){
			serial_write("battery voltage too low...\a");
			put_uint(battery_voltage);
			while(1){
				_delay_ms(200);
				PORTB ^= (0x01 << PB5);
			}
		}


		sei();
		serial_write("waiting for rc_signal");
		while(rc_signal[1] < 990);

		serial_write("ok \nstarting loop");

	}

	//setup watchdog to reset after ___ s
	//set to reset, prescaler 16k WDT Oszillator cycles (0.125ms)
	//WDTCSR = (0x01 << 3) | (0x03 << 0);
	wdt_enable(WDTO_120MS);
	while(1){

	//convert timer input value to value in µs
	receiver_input_channel_1 = rc_signal[1] / 2;
	receiver_input_channel_2 = rc_signal[2] / 2;
	receiver_input_channel_3 = rc_signal[3] / 2;
	receiver_input_channel_4 = rc_signal[4] / 2;






/*
put_uint(receiver_input_channel_1);
put_uart('\t');
put_uint(receiver_input_channel_2);
put_uart('\t');
put_uint(receiver_input_channel_3);
put_uart('\t');
put_uint(receiver_input_channel_4);
put_uart('\t');
/*
put_uint((unsigned int) (pid_p_gain_pitch * 100));
put_uart('\t');
put_uint((unsigned int) (pid_d_gain_pitch * 100));
put_uart('\t');
*/

		read_gyro();
		//serial_write("gyro read finished");
		gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 65.5) * 0.2);            //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 65.5) * 0.2);         //Gyro pid input is deg/sec.
		gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 65.5) * 0.2);               //Gyro pid input is deg/sec.
/*
put_int((int)gyro_roll_input);
put_uart('\t');
put_int((int)gyro_pitch_input);
put_uart('\t');
put_int((int)gyro_yaw_input);
put_uart('\t');
*/
		//For starting the motors: throttle low and yaw left (step 1).
		if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)
			start = 1;
		//When yaw stick is back in the center position start the motors (step 2).
		if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
			start = 2;
			//Reset the pid controllers for a bumpless start.
			pid_i_mem_roll = 0;
			pid_last_roll_d_error = 0;
			pid_i_mem_pitch = 0;
			pid_last_pitch_d_error = 0;
			pid_i_mem_yaw = 0;
			pid_last_yaw_d_error = 0;
		}
		//Stopping the motors: throttle low and yaw right.
		if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950){
			start = 0;


		}

		//The PID set point in degrees per second is determined by the roll receiver input.
		//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_roll_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(receiver_input_channel_1 > 1508)pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
		else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;

		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;
		//We need a little dead band of 16us for better results.
		//invert signal bc .
		if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = -(receiver_input_channel_2 - 1508)/3.0;
		else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = -(receiver_input_channel_2 - 1492)/3.0;

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
		if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = -(receiver_input_channel_4 - 1508)/3.0;
		else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = -(receiver_input_channel_4 - 1492)/3.0;
		}
		//PID inputs are known. So we can calculate the pid output.
		calculate_pid();

		//serial_write("pids calculated");

		throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

		if (start == 2){                                                          //The motors are started.
		if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
		esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
		esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
		esc_3 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
		esc_4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

/*
put_int(pid_output_pitch);
put_uart('\t');
put_int(pid_output_roll);
put_uart('\t');
put_int(pid_output_yaw);
put_uart('\t');
*/

		if (esc_1 < 1050) esc_1 = 1050;                                         //Keep the motors running.
		if (esc_2 < 1050) esc_2 = 1050;                                         //Keep the motors running.
		if (esc_3 < 1050) esc_3 = 1050;                                         //Keep the motors running.
		if (esc_4 < 1050) esc_4 = 1050;                                         //Keep the motors running.

		if(esc_1 > 1600)esc_1 = 1600;                                           //Limit the esc-1 pulse to 2000us.
		if(esc_2 > 1600)esc_2 = 1600;                                           //Limit the esc-2 pulse to 2000us.
		if(esc_3 > 1600)esc_3 = 1600;                                           //Limit the esc-3 pulse to 2000us.
		if(esc_4 > 1600)esc_4 = 1600;                                           //Limit the esc-4 pulse to 2000us.
		}

		else{
		esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
		esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
		esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
		esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
		}

		OCR0B = (esc_1 - 1000)/4;
		OCR0A = (esc_2 - 1000)/4;
		OCR2A = (esc_3 - 1000)/4;
		OCR2B = (esc_4 - 1000)/4;
/*
put_uint(esc_1);
put_uart('\t');
put_uint(esc_2);
put_uart('\t');
put_uint(esc_3);
put_uart('\t');
put_uint(esc_4);
put_uart('\t');
*/

		loop_counter++;

		battery_voltage = adc_read();
		if(battery_voltage < 675){
			if(loop_counter % 200){
				PORTB ^= (0x01 << PB5);
			}
		}
		//put_uint(battery_voltage);
		//serial_write("");

		//reset watchdog timer
		wdt_reset();
		//watchdog timer reset indicator
		if(reset_cause == 0x80)
			if(loop_counter % 25){
				PORTB ^= (0x01 << PB5);
			}
		//
		while((tick % 31) != 0);
	}

	return 0;
}






void setup_timers(){
	//setup timer 0: PWM for both pins, prescaler 8 -> 2MHz Clock -> 7812 Hz
	TCCR0A = 0xA3; // @suppress("Symbol is not resolved")
	TCCR0B = 0x02; // @suppress("Symbol is not resolved")

	OCR0A = 0;
	OCR0B = 0;
	//enable overflow interrupt for loop time reference
	TIMSK0 |= (0x01 << 0);

	//setup timer 2:

	TCCR2A = 0xA3;
	TCCR2B = 0x02;

	OCR2A = 0;
	OCR2B = 0;


	//setup timer 1 for input capture:
	//clock divider: 8
	TCCR1B |= (0x01 << 1);

}

void setup_interrupts(){
	PCICR |= (0x01 << 2) | (0x01 << 0);
	PCMSK2 |= (0x01 << 2) | (0x01 << 4) | (0x01 << 7);
	PCMSK0 |= (0x01 << 4) | (0x01 << 2) | (0x01 << 1);
}

void setup_gyro(){
	//write to power management regeister to activate all sensors
	i2c_start(GYRO_ADRESS+I2C_WRITE);
	i2c_write(108);
	i2c_write(0x00);
	i2c_stop();


	// set gyro to 500 dps
	i2c_start(GYRO_ADRESS+I2C_WRITE);
	i2c_write(27);
	i2c_write(0x08);
	i2c_stop();

	//set accle to 4g
	i2c_start(GYRO_ADRESS+I2C_WRITE);
	i2c_write(28);
	i2c_write(0x08);
	i2c_stop();


}

void read_gyro(){

	i2c_start(GYRO_ADRESS+I2C_WRITE);
	i2c_write(59);
	i2c_rep_start(GYRO_ADRESS+I2C_READ);

	for(int i = 0; i < 14; i++){
		gyro_value[i] = i2c_readAck();
	}
	i2c_readNak();
	i2c_stop();




	gyro_pitch = (int)(gyro_value[8] << 8 | gyro_value[9]) ;
	gyro_roll = (int)(gyro_value[10] << 8 | gyro_value[11]);
	gyro_yaw = (int)(gyro_value[12] << 8 | gyro_value[13]);

	//if started subtract offset
	if(cal_int == 2000){
		gyro_pitch -= gyro_axis_cal[0];
		gyro_roll -= gyro_axis_cal[1];
		gyro_yaw -= gyro_axis_cal[2];
	}
}


void calculate_pid(){
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}


ISR(PCINT2_vect){


	uint16_t delta = 0;
	static bool last_channel = false;

	//if fourth timer is high: clear timer, save time
	if(PIND & (0x01 << 2)){
		delta = TCNT1 - current_time;
		current_time =TCNT1;
		if(delta > 1800 && delta < 4200)
			rc_signal[3] = delta;
	}
	//if fifth timer is high: clear timer, save time, activate last_channel state
	else if(PIND & (0x01 << 4)){
		delta = TCNT1 - current_time;
		current_time =TCNT1;
		if(delta > 1800 && delta < 4200)
			rc_signal[4] = delta;
	}
	//when the last channel switches to low and last_channel is active: save time
	else if(PIND & (0x01 << 7)){
		delta = TCNT1 - current_time;
		current_time =TCNT1;
		if(delta > 1800 && delta < 4200)
			rc_signal[5] = delta;
		last_channel = true;
	}
	else if(last_channel && !(PIND & (0x01 << 7))){
		delta = TCNT1 - current_time;

		if(delta > 1800 && delta < 4200)
			rc_signal[6] = delta;
		last_channel = false;



	}

}

ISR(PCINT0_vect){

	uint16_t delta = 0;

	//if first channel is high: start timer
	if(PINB & (0x01 << 1)){
		TCNT1 = 0;
		current_time = 0;

	}
	//if second channel is high: clear timer, save time
	else if(PINB & (0x01 << 2)){
		delta = TCNT1 - current_time;
		current_time =TCNT1;
		if(delta > 1800 && delta < 4200)
			rc_signal[1] = delta;
	}
	//if third timer is high: clear timer, save time;
	else if(PINB & (0x01 << 4)){
		delta = TCNT1 - current_time;
		current_time =TCNT1;
		if(delta > 1800 && delta < 4200)
			rc_signal[2] = delta;
	}



}

ISR(TIMER0_OVF_vect){
	tick++;
}



