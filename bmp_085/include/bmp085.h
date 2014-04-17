/*
 * bmp085.h
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
 *  Created on: Apr 12, 2014
 *      Author: dnezic
 */

#ifndef BMP085_H_
#define BMP085_H_

#define BMP085_I2CINIT 1

#define BMP085_REGAC1 0xAA
#define BMP085_REGAC2 0xAC
#define BMP085_REGAC3 0xAE
#define BMP085_REGAC4 0xB0
#define BMP085_REGAC5 0xB2
#define BMP085_REGAC6 0xB4
#define BMP085_REGB1 0xB6
#define BMP085_REGB2 0xB8
#define BMP085_REGMB 0xBA
#define BMP085_REGMC 0xBC
#define BMP085_REGMD 0xBE
#define BMP085_REGCONTROL 0xF4 //control
#define BMP085_REGCONTROLOUTPUT 0xF6 //output 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB
#define BMP085_REGREADTEMPERATURE 0x2E //read temperature
#define BMP085_REGREADPRESSURE 0x34 //read pressure

//modes
#define BMP085_MODEULTRALOWPOWER 0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define BMP085_MODESTANDARD 1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define BMP085_MODEHIGHRES 2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define BMP085_MODEULTRAHIGHRES 3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25

//autoupdate temperature enabled
#define BMP085_AUTOUPDATETEMP 1 //autoupdate temperature every read

//setup parameters
#define BMP085_MODE BMP085_MODEULTRALOWPOWER //define a mode
#define BMP085_UNITPAOFFSET 0 //define a unit offset (pa)
#define BMP085_UNITMOFFSET 0 //define a unit offset (m)

//average filter
#define BMP085_FILTERPRESSURE 0 //avarage filter for pressure

//variables
int bmp085_regac1, bmp085_regac2, bmp085_regac3, bmp085_regb1, bmp085_regb2,
		bmp085_regmb, bmp085_regmc, bmp085_regmd;
unsigned int bmp085_regac4, bmp085_regac5, bmp085_regac6;
long bmp085_rawtemperature, bmp085_rawpressure;

typedef struct
{
	int8_t temperature_integral;
	uint8_t temperature_decimal;
	uint8_t pressure_1;
	uint8_t pressure_2;
	uint8_t pressure_3;
	uint8_t pressure_4;
} BMP085_DATA_t;

typedef struct
{
	uint8_t read_errors;
	uint8_t write_errors;
	uint8_t total_errors;
} BMP085_ERROR_t;


//functions
extern void bmp085_init();
extern int32_t bmp085_getpressure();
extern double bmp085_getaltitude();
extern double bmp085_gettemperature();
extern BMP085_ERROR_t bmp085_readall(BMP085_DATA_t *data);

#endif /* BMP085_H_ */
