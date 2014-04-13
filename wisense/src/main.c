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
#include <avr/eeprom.h>
#include <nRF24L01.h>
#include <mirf.h>
#include <voltage.h>
#include <bmp085.h>

#define mirf_CH			0x50
#define EEPROM_ADDRESS_1 50
#define EEPROM_ADDRESS_2 51
#define EEPROM_ADDRESS_3 52
#define EEPROM_RESET_COUNTER 53

//#define DEBUG 0

#ifdef __AVR_ATtiny84__
#define WDTCR WDTCSR
#endif

#ifdef __AVR_ATtiny85__
#endif

#ifdef __AVR_ATtiny861A__
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
//            CE             PB0  2|    |9  PA0
//            CSN            PB1  3|    |10 PA1
//                           PB3  4|    |11 PA2
//                           PB2  5|    |12 PA3
//                           PA7  6|    |13 PA4 nRF24L01 SCK
//           nRF24L01 MI     PA6  7|    |14 PA5 nRF24L01 MO
//                                 +----+

/* flags controlling the sleep process */
volatile boolean f_wdt = 1;
volatile uint8_t wait_counter = 29;
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
	// all inputs low

	/* all pins low */
	PORTA = 0x00;
	PORTB = 0x00;
	DDRA = 0xff;
	DDRB = 0xff;
	PORTA = 0x00;
	PORTB = 0x00;
	// pin input high
	//DDRA &= ~(1 << (PINA2));
	//PINA |= (1 << (PINA2));

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);// sleep mode is set here (most conservative)
	sleep_bod_disable()
	; // probably not supported
	sleep_mode()
	;  // system sleeps here
	   // (includes sleep disable and enable)
	sbi(ADCSRA, ADEN);						// switch ADC ON

	// EXTREME: all ports to low inputs? TODO:

}

//void blink() {
//	DDRA |= (1 << PORTA1);
//	PORTA |= (1 << PORTA1);
//	_delay_us(1000000);
//	PORTA &= ~(1 << PORTA1);
//	_delay_us(1000000);
//	PORTA |= (1 << PORTA1);
//	_delay_us(1000000);
//	PORTA &= ~(1 << PORTA1);
//
//}

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

void setup_mirf() {
	spi_init();
	mirf_init();
	_delay_ms(50);
}

int main(void) {

	uint8_t counter = 0;

	uint8_t flow_byte;
	uint8_t flow_byte_aux = 0;
	uint8_t reset_counter = 0;

	const uint8_t BUFFERSIZE = 11;
	long voltage = 0;
	byte RADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };
	byte TADDR[] = { 0xe3, 0xf0, 0xf0, 0xf0, 0xf0 };

	setup_watchdog();

#ifdef DEBUG
	/* store reset/power counter into eeprom */
	reset_counter = eeprom_read_byte((uint8_t *) EEPROM_RESET_COUNTER);
	reset_counter = reset_counter + 1;
	eeprom_update_byte((uint8_t *) EEPROM_RESET_COUNTER, reset_counter);
#endif

	while (1) {

		if (f_wdt == 1) {// wait for timed out watchdog / flag is set when a watchdog timeout occurs

			f_wdt = 0;			// reset flag
			if (wait_counter >= wait_cycles) {

				//blink();

#ifdef DEBUG
				flow_byte = 0;
				flow_byte_aux = 0;

#endif

#ifdef DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				DDRA |= (1 << PORTA1);
				PORTA |= (1 << PORTA1);
				_delay_ms(300);

				BMP085_DATA_t bmp_data;
				sei();
				uint8_t bmp_status = bmp085_readall(&bmp_data);

				PORTA &= ~(1 << PORTA1);

				_delay_ms(50);

				setup_mirf();

				mirf_config(TADDR, RADDR, mirf_CH, BUFFERSIZE);
				mirf_config_register(STATUS, 1 << MAX_RT);

				wait_counter = 0;

#ifdef DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				voltage = read_vcc();

#ifdef DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				uint8_t buffer[11] = { voltage >> 8, voltage & 0xff, bmp_data.temperature_integral, bmp_data.temperature_decimal, bmp_data.pressure_1, bmp_data.pressure_2,
						bmp_data.pressure_3, bmp_data.pressure_4, counter, reset_counter, bmp_status};

				mirf_flush_rx_tx();					// flush TX/RX
				mirf_send(buffer, BUFFERSIZE);

				// unnecessary delay ?
				_delay_ms(100);

#ifdef DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif

				// If maximum retries were reached, reset MAX_RT
				/* FIXME: we are doing broadcast, so we don't use retries */

				if (mirf_max_rt_reached()) {
					mirf_config_register(STATUS, 1 << MAX_RT);
					/* led flash:
					 mirf_CSN_lo;
					 _delay_ms(1000);
					 mirf_CSN_hi;*/

#ifdef DEBUG
					eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_3,
							++flow_byte_aux);
#endif
				}

				mirf_powerdown();	// put device in power down mode

#ifdef DEBUG
				eeprom_update_byte((uint8_t *) EEPROM_ADDRESS_2, ++flow_byte);
#endif
			}
			counter = counter + 1;
			system_sleep();
		}

	}
}

