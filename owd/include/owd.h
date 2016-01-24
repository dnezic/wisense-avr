/*
 * owd.h
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

#ifndef OWD_H_
#define OWD_H_

#include <avr/io.h>
#include <util/delay.h>


#define N 400
#define T 200

#ifdef __AVR_ATtiny861A__
#define DEBUG_PIN PORTA6
#define DEBUG_DDR DDRA
#define DEBUG_PORT_OUT PORTA
#endif

#endif /* OWD_H_ */

extern void send_text(char *ch);
extern void owd_init();
extern void owd_printf(const char *format, ...);
