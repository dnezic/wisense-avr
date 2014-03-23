/*
 ATtiny85 SPI
 */

#include "spi.h"

// SPI transfer 1 byte and return the result
uint8_t spi_transfer(uint8_t data) {
	USIDR = data;
	USISR = _BV(USIOIF); // clear flag

	while ((USISR & _BV(USIOIF)) == 0) { // in three-wire mode USIOIF determines when the transfer is completed
		USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
	}
	return USIDR;
}

// Write data using SPI
void spi_write_data(uint8_t * dataout, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		spi_transfer(dataout[i]);
	}
}

// Read data using SPI
void spi_read_data(uint8_t * datain, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		datain[i] = spi_transfer(0x00);
	}
}

// Initialise the SPI
void spi_init(void) {

#ifdef __AVR_ATtiny85__
	DDRB |= (1 << PB2); // SPI CLK
	DDRB |= (1 << PB1); // SPI DO
	DDRB &= ~(1 << PB0); // SPI DI
	PORTB |= (1 << PB0); // SPI DI
#endif

#ifdef __AVR_ATtiny84__
	DDRA |= (1<<PINA4); // SPI CLK
	DDRA |= (1<<PINA5);// SPI DO
	DDRA &= ~(1<<PINA6);// SPI DI
	PORTA |= (1<<PINA6);// SPI DI
#endif

}

