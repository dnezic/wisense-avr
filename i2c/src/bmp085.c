/*
 * bmp085.c
 *
 * Copyright 2014, Drazen Nezic, www.svesoftware.com
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *  Created on: Apr 11, 2014
 *      Author: dnezic
 */

#include <avr/builtins.h>
#include <avr/common.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <twimaster.h>
#include <bmp085.h>
#include <string.h>
#include <avr/eeprom.h>

#define BMP085_ADDR  0x77

#define TWI_GEN_CALL         0x00  // The General Call address is 0

unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg) {
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	asm("nop");
	return TWIerrorMsg;
}

void bmp085_writemem(uint8_t reg, uint8_t value) {
	unsigned const char SLAVE_ADDR = BMP085_ADDR;
	send_data_pair(SLAVE_ADDR, reg, value);
}

void blink_for(uint8_t n) {
	DDRA |= (1 << PORTA1);

	PORTA |= (1 << PORTA1);
	_delay_us(1000000);
	PORTA &= ~(1 << PORTA1);
	_delay_us(1000000);
	PORTA |= (1 << PORTA1);
	_delay_us(1000000);
	PORTA &= ~(1 << PORTA1);
	_delay_us(1000000);

	for (uint8_t i = 0; i < n; i++) {
		PORTA |= (1 << PORTA1);
		_delay_us(500000);
		PORTA &= ~(1 << PORTA1);
		_delay_us(500000);

	}

}

/*
 * i2c read
 */
void bmp085_readmem(uint8_t reg, uint8_t buff[], uint8_t bytes) {
	receive_bytes_pair(BMP085_ADDR, reg, bytes, &buff[0]);
}

#if BMP085_FILTERPRESSURE == 1
#define BMP085_AVARAGECOEF 21
static long k[BMP085_AVARAGECOEF];
long bmp085_avaragefilter(long input) {
	uint8_t i = 0;
	long sum = 0;
	for (i = 0; i < BMP085_AVARAGECOEF; i++) {
		k[i] = k[i + 1];
	}
	k[BMP085_AVARAGECOEF - 1] = input;
	for (i = 0; i < BMP085_AVARAGECOEF; i++) {
		sum += k[i];
	}
	return (sum / BMP085_AVARAGECOEF);
}
#endif

/*
 * read calibration registers
 */
void bmp085_getcalibration() {
	uint8_t buff[2];
	memset(buff, 0, sizeof(buff));
	bmp085_readmem(BMP085_REGAC1, buff, 2);
	bmp085_regac1 = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGAC2, buff, 2);
	bmp085_regac2 = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGAC3, buff, 2);
	bmp085_regac3 = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGAC4, buff, 2);
	bmp085_regac4 = ((unsigned int) buff[0] << 8 | ((unsigned int) buff[1]));
	bmp085_readmem(BMP085_REGAC5, buff, 2);
	bmp085_regac5 = ((unsigned int) buff[0] << 8 | ((unsigned int) buff[1]));
	bmp085_readmem(BMP085_REGAC6, buff, 2);
	bmp085_regac6 = ((unsigned int) buff[0] << 8 | ((unsigned int) buff[1]));
	bmp085_readmem(BMP085_REGB1, buff, 2);
	bmp085_regb1 = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGB2, buff, 2);
	bmp085_regb2 = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGMB, buff, 2);
	bmp085_regmb = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGMC, buff, 2);
	bmp085_regmc = ((int) buff[0] << 8 | ((int) buff[1]));
	bmp085_readmem(BMP085_REGMD, buff, 2);
	bmp085_regmd = ((int) buff[0] << 8 | ((int) buff[1]));
}

/*
 * get raw temperature as read by registers, and do some calculation to convert it
 */
void bmp085_getrawtemperature() {
	uint8_t buff[2];
	memset(buff, 0, sizeof(buff));
	long ut, x1, x2;

	//read raw temperature
	bmp085_writemem(BMP085_REGCONTROL, BMP085_REGREADTEMPERATURE);
	_delay_ms(5); // min. 4.5ms read Temp delay
	bmp085_readmem(BMP085_REGCONTROLOUTPUT, buff, 2);
	ut = ((long) buff[0] << 8 | ((long) buff[1])); //uncompensated temperature value

	//calculate raw temperature
	x1 = ((long) ut - bmp085_regac6) * bmp085_regac5 >> 15;
	x2 = ((long) bmp085_regmc << 11) / (x1 + bmp085_regmd);
	bmp085_rawtemperature = x1 + x2;
}

/*
 * get raw pressure as read by registers, and do some calculation to convert it
 */
void bmp085_getrawpressure() {
	uint8_t buff[3];
	memset(buff, 0, sizeof(buff));
	long up, x1, x2, x3, b3, b6, p;
	unsigned long b4, b7;

#if BMP085_AUTOUPDATETEMP == 1
	bmp085_getrawtemperature();
#endif

	//read raw pressure
	bmp085_writemem(BMP085_REGCONTROL,
	BMP085_REGREADPRESSURE + (BMP085_MODE << 6));
	_delay_ms(2 + (3 << BMP085_MODE));
	bmp085_readmem(BMP085_REGCONTROLOUTPUT, buff, 3);
	up = ((((long) buff[0] << 16) | ((long) buff[1] << 8) | ((long) buff[2]))
			>> (8 - BMP085_MODE)); // uncompensated pressure value

	//calculate raw pressure
	b6 = bmp085_rawtemperature - 4000;
	x1 = (bmp085_regb2 * (b6 * b6) >> 12) >> 11;
	x2 = (bmp085_regac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long) bmp085_regac1) * 4 + x3) << BMP085_MODE) + 2) >> 2;
	x1 = (bmp085_regac3 * b6) >> 13;
	x2 = (bmp085_regb1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (bmp085_regac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) up - b3) * (50000 >> BMP085_MODE);
	p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	bmp085_rawpressure = p + ((x1 + x2 + 3791) >> 4);

#if BMP085_FILTERPRESSURE == 1
	bmp085_rawpressure = bmp085_avaragefilter(bmp085_rawpressure);
#endif
}

/*
 * get celsius temperature
 */
double bmp085_gettemperature() {
	bmp085_getrawtemperature();
	double temperature = ((bmp085_rawtemperature + 8) >> 4);
	temperature = temperature / 10;
	return temperature;
}

/*
 * get pressure
 */
int32_t bmp085_getpressure() {
	bmp085_getrawpressure();
	return bmp085_rawpressure + BMP085_UNITPAOFFSET;
}

/*
 * get altitude
 */
double bmp085_getaltitude() {
	bmp085_getrawpressure();
	return ((1 - pow(bmp085_rawpressure / (double) 101325, 0.1903))
			/ 0.0000225577) + BMP085_UNITMOFFSET;
}

void i2c_initialize() {
	USI_TWI_Master_Initialise();
}

/*
 * init bmp085
 */
void bmp085_init() {
	i2c_initialize();
	_delay_us(10);
	bmp085_getcalibration(); //get calibration data
	bmp085_getrawtemperature(); //update raw temperature, at least the first time

#if BMP085_FILTERPRESSURE == 1
	//initialize the avarage filter
	uint8_t i = 0;
	for (i = 0; i < BMP085_AVARAGECOEF; i++) {
		bmp085_getrawpressure();
	}
#endif
}

void blink_fast() {
	DDRA |= (1 << PORTA1);
	PORTA |= (1 << PORTA1);
	_delay_us(100000);
	PORTA &= ~(1 << PORTA1);
	_delay_us(100000);
	PORTA |= (1 << PORTA1);
	_delay_us(100000);
	PORTA &= ~(1 << PORTA1);

}

void blink_slow() {
	DDRA |= (1 << PORTA1);
	PORTA |= (1 << PORTA1);
	_delay_us(1000000);
	PORTA &= ~(1 << PORTA1);
	_delay_us(1000000);
	PORTA |= (1 << PORTA1);
	_delay_us(1000000);
	PORTA &= ~(1 << PORTA1);

}

uint8_t bmp085_readall(BMP085_DATA_t *data) {
	//uint8_t debug_counter = 0;
	//eeprom_update_byte((uint8_t *) EEPROM_ADDRESS, ++debug_counter);
	bmp085_init();

	long pressure = bmp085_getpressure();

	data->pressure_4 = (uint8_t) (pressure & 0xff);
	data->pressure_3 = (uint8_t) ((pressure >> 8) & 0xff);
	data->pressure_2 = (uint8_t) ((pressure >> 16) & 0xff);
	data->pressure_1 = (uint8_t) ((pressure >> 24) & 0xff);
	data->temperature_integral = (uint8_t) bmp085_gettemperature();
	data->temperature_decimal = 0;

	if (USI_TWI_Get_Write_Oks() == 0) {
		//blink_slow();
		//blink_slow();
		//blink_slow();
		USI_TWI_ResetStats();
		return 1;
	}

	//blink_fast();
	return 0;
}

