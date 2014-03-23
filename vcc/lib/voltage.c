/*
* voltage.c
*
* Created: 29.1.2014. 20:51:10
*  Author: drazen
*/


#include <voltage.h>

long read_vcc() {

	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	long result;

	#ifdef __AVR_ATtiny85__
	ADMUX = _BV(MUX3) | _BV(MUX2);
	#endif

	#ifdef __AVR_ATtiny84__
	ADMUX = _BV(MUX5) | _BV(MUX0);
	#endif

	
	_delay_ms(10); // Wait for Vref to settle
	/* enable ADC !*/
	ADCSRA |= _BV(ADEN);

	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring
	result = ADCL;
	result |= ADCH<<8;
	result = 1126400L / result;
	return result;
}
