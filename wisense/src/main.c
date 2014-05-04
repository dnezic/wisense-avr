/*

 nRF24L01+ tested with ATtiny85, ATtiny84A and ATtiny861A

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
 *
 * #define F_CPU 1000000UL
 * #define F_CPU 8000000UL
 *
 */

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <bmp085.h>
#include <mirf.h>
#include <nRF24L01.h>
#include <spi.h>
#include <util.h>
#include <voltage.h>

#define mirf_CH			0x50

#define DEBUG 0

#ifdef __AVR_ATtiny84__
#define WDTCR WDTCSR
#endif

#ifdef __AVR_ATtiny85__
#endif

#ifdef __AVR_ATtiny861A__
/* used to power on BMP085 using PN2222. */
#define SWITCH_PIN PORTB3
#define SWITCH_DDR DDRB
#define SWITCH_PORT PORTB
#endif

#ifdef DEBUG
#define EEPROM_RESET_COUNTER 50
#define EEPROM_ADDRESS_2 51
#define EEPROM_ADDRESS_3 52
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
//           nRF24L01 CE     PB0  2|    |9  PA0
//           nRF24L01 CSN    PB1  3|    |10 PA1
//                           PB3  4|    |11 PA2
//                           PB2  5|    |12 PA3
//                           PA7  6|    |13 PA4 nRF24L01 SCK
//           nRF24L01 MI     PA6  7|    |14 PA5 nRF24L01 MO
//                                 +----+
//
// ATtiny861 Pin map
//                                 +-\/-+
//           nRF24L01 MI     PB0  1|o   |20  PA0
//           nRF24L01 MO     PB1  2|    |19  PA1
//           nRF24L01 SCK    PB2  3|    |18  PA3
//                           PB3  4|    |17  PA3
//                           VCC  5|    |16  AGND
//                           GND  6|    |15  AVCC
//           nRF24L01 CE     PB4  7|    |14  PA4
//           nRF24L01 CSN    PB5  8|    |13  PA5
//                           PB6  9|    |12  PA6
//                           PB7 10|    |11  PA7
//                                 +----+

/* flags controlling the sleep process */
volatile boolean f_wdt = 1;
volatile uint8_t wait_counter = 62;
/* wait for approximately 10 minutes. */
volatile uint8_t wait_cycles = 64;

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

	/* all pins low */
	PORTA = 0x00;
	PORTB = 0x00;
	DDRA = 0xff;
	DDRB = 0xff;
	PORTA = 0x00;
	PORTB = 0x00;

	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here (most conservative)
	sleep_bod_disable()
	; // probably not supported
	sleep_mode()
	;  // system sleeps here
	   // (includes sleep disable and enable)

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
						  // if reset of device happened. TODO: the exact purpose is not clear to me

	WDTCR |= (1 << WDCE) | (1 << WDE);// set up watchdog timer, in same operation
	WDTCR |= (1 << WDP3) | (1 << WDP0);	// timer goes off every 8 seconds
	// ? are this two lines above equivalent to: wdt_enable(WDTO_8S); ?

	WDTCR |= _BV(WDIE);					// do-not-reset flag
	sei();
	// enable interrupts, sets SREG 7th bit to 1

}

/* initialize SPI on PORTB */
void setup_mirf() {
	spi_init();
	mirf_init();
	_delay_ms(50);
}

#define BUFFERSIZE 15

int main(void) {

	uint8_t counter = 0;

#if DEBUG
	uint8_t flow_byte;
	uint8_t flow_byte_aux = 0;
	uint8_t reset_counter = 0;
#endif

	//const uint8_t BUFFERSIZE = 16;
	long voltage = 0;
	byte RADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };
	byte TADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };

	setup_watchdog();

#if DEBUG
	/* store reset/power counter into eeprom */
	reset_counter = eeprom_read_byte((uint8_t *) EEPROM_RESET_COUNTER);
	reset_counter = reset_counter + 1;
	eeprom_update_byte((uint8_t *) EEPROM_RESET_COUNTER, reset_counter);
#endif

	while (1) {

		if (f_wdt == 1) {// wait for timed out watchdog / flag is set when a watchdog timeout occurs

			f_wdt = 0;			// reset flag
			if (wait_counter >= wait_cycles) {

#if DEBUG
				flow_byte = 0;
				flow_byte_aux = 0;

#endif

				/* important to be called first to get accurate readings */
				voltage = read_vcc();

#if DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				/* power on BMP085 */

				SWITCH_DDR |= (1 << SWITCH_PIN);
				SWITCH_PORT |= (1 << SWITCH_PIN);

				/* wait for BMP085 to start. */
				_delay_ms(1500);

				BMP085_DATA_t bmp_data;
				BMP085_ERROR_t error = bmp085_readall(&bmp_data);

				/* power off BMP085 */
				SWITCH_PORT &= ~(1 << SWITCH_PIN);

				setup_mirf();

				mirf_config(TADDR, RADDR, mirf_CH, BUFFERSIZE);
				mirf_config_register(STATUS, 1 << MAX_RT);

				wait_counter = 0;

#if DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				uint8_t buffer[BUFFERSIZE] = { voltage >> 8, voltage & 0xff,
						bmp_data.temperature_integral,
						bmp_data.temperature_decimal, bmp_data.pressure_1,
						bmp_data.pressure_2, bmp_data.pressure_3,
						bmp_data.pressure_4, 0, 0, 0, 0, 0, counter,
						error.total_errors };

				mirf_flush_rx_tx();					// flush TX/RX
				mirf_send(buffer, BUFFERSIZE);

				// unnecessary delay ?
				_delay_ms(50);

#if DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				// If maximum retries were reached, reset MAX_RT
				/* FIXME: we are doing broadcast with no ACK, so why sometimes program enters in this block? */
				if (mirf_max_rt_reached()) {
					mirf_config_register(STATUS, 1 << MAX_RT);
#if DEBUG
					eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_3,
							++flow_byte_aux);
#endif
				}

				mirf_powerdown();	// put device in power down mode

#if DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif
			}
			counter = counter + 1;
			system_sleep();
		}

	}
}

