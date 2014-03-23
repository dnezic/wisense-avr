/*

 nRF24L01+ with the ATtiny85 or ATtiny84

 Based on:
 http://www.insidegadgets.com/2012/08/22/using-the-nrf24l01-wireless-module/
 http://gizmosnack.blogspot.gr/2013/04/tutorial-nrf24l01-and-avr.html
 https://github.com/stanleyseow/arduino-nrf24l01
 Made to be compatible with stanleyseow libs for RPi.

 Author:
 dnezic, (http://www.svesoftware.com)

 */

/*
 * already set as environment variable
 * #define F_CPU 8000000UL
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <nRF24L01.h>
#include <mirf.h>
#include <voltage.h>
#include <temp.h>

#define mirf_CH			0x50

#ifdef __AVR_ATtiny84__
#define WDTCR WDTCSR
#endif

#ifdef __AVR_ATtiny85__
#endif


// ATtiny25/45/85 Pin map
//                                 +-\/-+
//          Reset/Ain0 (D 5) PB5  1|o   |8  Vcc
//  nRF24L01 CE - Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 - nRF24L01 SCK
// nRF24L01 CSN - Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1 - nRF24L01 MO
//                           GND  4|    |5  PB0 (D 0) pwm0 - nRF24L01 MI
//                                 +----+
// ATtiny84 Pin map
//                                 +-\/-+
//                           Vcc  1|o   |8  GND
//                           PB0  2|    |9  PA0
//                           PB1  3|    |10 PA1
//                           PB3  4|    |11 PA2
//                           PB2  5|    |12 PA3
//                           PA7  6|    |13 PA4
//                           PA6  7|    |14 PA5
//                                 +----+

/* flags controlling the sleep process */
volatile boolean f_wdt = 1;
volatile uint8_t wait_counter = 30;
volatile uint8_t wait_cycles = 32;

// watchdog interrupt
ISR(WDT_vect) {

	f_wdt = 1;  // set global flag
	wait_counter = wait_counter + 1;
	// most important: set auto reset bit for interrupt instead of reset
	WDTCR |= _BV(WDIE);
	/*
	 from datasheet 8.5.2:
	 If WDE is set, WDIE is automatically cleared by hardware when a time-out occurs. This is useful for keeping the
	 Watchdog Reset security while using the interrupt. After the WDIE bit is cleared, the next time-out will generate a
	 reset. To avoid the Watchdog Reset, WDIE must be set after each interrupt.
	 */

}

// put mcu in sleep (power save mode)
void system_sleep() {

	cbi(ADCSRA, ADEN);  // switch ADC OFF
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here (most conservative)
	sleep_bod_disable(); // probably not supported
	sleep_mode()
	;  // system sleeps here
	   // (includes sleep disable and enable)
	sbi(ADCSRA, ADEN);						// switch ADC ON

	// EXTREME: all ports to low inputs? TODO:

}

// setup timer
void setup_watchdog() {

	cli();
	// disable interrupts, sets SREG 7th bit to 0

	/*
	 In safety level 1, WDE is overridden by WDRF in MCUSR. See “MCUSR – MCU Status Register” on page 44 for
	 description of WDRF. This means that WDE is always set when WDRF is set. To clear WDE, WDRF must be
	 cleared before disabling the Watchdog with the procedure described above. This feature ensures multiple resets
	 during conditions causing failure, and a safe start-up after the failure.
	 */

	MCUSR &= ~(1 << WDRF);// clear the watchdog reset, this is required to disable already active timer
						  // if reset of device happened. TODO: the purpose is not clear

	WDTCR |= (1 << WDCE) | (1 << WDE);// set up watchdog timer, in same operation
	WDTCR |= (1 << WDP3) | (1 << WDP0);	// timer goes off every 8 seconds
	// ? are this two lines above equivalent to: wdt_enable(WDTO_8S); ?

	WDTCR |= _BV(WDIE);					// do-not-reset flag
	sei();
	// enable interrupts, sets SREG 7th bit to 1

}

void setup() {
	spi_init();
	setup_watchdog();
	mirf_init();
}

int main(void) {

	uint8_t counter = 0;
	const uint8_t BUFFERSIZE = 8;
	long voltage = 0;
	byte RADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };
	byte TADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };

	setup();
	// some delay to setup mirf
	_delay_ms(50);

	mirf_config(TADDR, RADDR, mirf_CH, BUFFERSIZE);

	while (1) {

		if (f_wdt == 1) {// wait for timed out watchdog / flag is set when a watchdog timeout occurs

			f_wdt = 0;			// reset flag
			if (wait_counter >= wait_cycles) {

				wait_counter = 0;


				// delay
				_delay_ms(100);

				voltage = read_vcc();

				DHT22_DATA_t data;


				cli();
				powerOnDHT22();
				DHT22_ERROR_t error = readDHT22(&data);
				powerOffDHT22();
				sei();



				uint8_t buffer[8] = { voltage >> 8, voltage & 0xff,
						data.humidity_integral, data.humidity_decimal,
						data.temperature_integral, data.temperature_decimal, error,
						counter };

				mirf_flush_rx_tx();					// flush TX/RX
				mirf_send(buffer, BUFFERSIZE);

				// unnecessary delay ?
				_delay_ms(100);

				// If maximum retries were reached, reset MAX_RT
				/* FIXME: we are doing broadcast, so we don't use retries */
				if (mirf_max_rt_reached()) {
					mirf_config_register(STATUS, 1 << MAX_RT);
					/* led flash:
					 mirf_CSN_lo;
					 _delay_ms(1000);
					 mirf_CSN_hi;*/

				}
				/* FIXME: removed for testing */
				//mirf_powerdown();	// put device in power down mode
			}
			counter = counter + 1;
			system_sleep();
		}

	}
}

