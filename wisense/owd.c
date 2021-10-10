/*
 * owd.c
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
 *  Created on: Jan 24, 2016
 *      Author: dnezic
 */

#include <owd.h>
#include <stdarg.h>

void owd_init() {
	DEBUG_DDR |= 1 << DEBUG_PIN;
}

void digital_write(uint8_t value) {
	if (value == 1) {
		DEBUG_PORT_OUT |= 1 << DEBUG_PIN;
	} else {
		DEBUG_PORT_OUT &= ~(1 << DEBUG_PIN);
	}
}

void reset() {
	digital_write(1);
	_delay_us(N * 10 + T);
	digital_write(0);
	_delay_us(N + T);
}

void transmit(uint8_t c) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		int bit = (c >> i) & (0x01);
		if (bit == 0) {
			digital_write(1);
			_delay_us(N + T);
		} else {
			digital_write(1);
			_delay_us(2 * N + T);
		}

		digital_write(0);
		_delay_us(N + T);
	}
}

void send_text(char *ch) {
	char c;
	reset();
	do {
		c = *(ch++);
		if (c != 0) {
			transmit(c);
		}
	} while (c != 0);
}


char buffer[64];

void owd_printf(const char *format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 63, format, args);
	send_text(buffer);
	va_end(args);
}

