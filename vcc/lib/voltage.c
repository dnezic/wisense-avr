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

	/* enable ADC !*/
	ADCSRA |= _BV(ADEN);

	#ifdef __AVR_ATtiny85__
	ADMUX = _BV(MUX3) | _BV(MUX2);
	#endif

	#ifdef __AVR_ATtiny84__
	ADMUX = _BV(MUX5) | _BV(MUX0);
	#endif

    #ifdef __AVR_ATtiny861A__
	ADMUX = _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif
	

	_delay_ms(10); // Wait for Vref to settle

	double average = 0;

	for(uint8_t i = 0; i < 20; i ++) {
		ADCSRA |= _BV(ADSC); // Start conversion
		while (bit_is_set(ADCSRA,ADSC)); // measuring
		result = ADCL;
		result |= ADCH<<8;
		result = 1126400L / result;
		if(i >= 5) {
			average = average + result;
		}
	}



	ADCSRA &= ~_BV(ADEN);
	return average / 15.0;
}
