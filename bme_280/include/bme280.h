/*
 * bme280.h
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
 *  Created on: Jan 10, 2016
 *      Author: dnezic
 */

#ifndef BME280_H_
#define BME280_H_

#define BME280_addr    0x76
#define MODE 1

#define __BME280_SLEEP 0
#define __BME280_FORCED 1
#define __BME280_NORMAL 3
#define __BME280_CONTROL 0xF4  // Control measurement
#define __BME280_CONTROL_HUM 0xF2  // Control humidity
#define __BME280_TEMPDATA 0xFA  // Temperature
#define __BME280_PRESSURE_DATA 0xF7  // Pressure
#define __BME280_HUMIDITY_DATA 0xFD  // Humidity
#define __BME280_STATUS 0xF3  // Status
#define __BME280_DATA_REG 0xF7  // Start of data registers (support shadowing)

typedef struct
{
	int8_t temperature_integral;
	uint8_t temperature_decimal;
	uint8_t pressure_1;
	uint8_t pressure_2;
	uint8_t pressure_3;
	uint8_t humidity;
} BME280_DATA_t;

typedef struct
{
	uint8_t read_errors;
	uint8_t write_errors;
	uint8_t total_errors;
} BME280_ERROR_t;

extern BME280_ERROR_t bme280_readall(BME280_DATA_t *data);

#endif /* BME280_H_ */
