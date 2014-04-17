/*
* temp.h
*
* Currently works only for attiny84
*/


#ifndef DHT22_H_
#define DHT22_H_

#include <avr/io.h>
#include <util/delay.h>

#define OUTPUT_RAW_VALUES 0

#define DHT22_DATA_BIT_COUNT 40

/* Configure port and pin */

/* AVR has to have fuse:
 * Divide clock by 8 internally; [CKDIV8=0] unset,
 * #> -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
 * So it runs with F_CPU set to 8000000L
 */
#ifdef __AVR_ATtiny861A__
#define DHT22_PIN PINB6
#define DHT22_DDR DDRB
#define DHT22_PORT_OUT PORTB
#define DHT22_PORT_IN PINB
#endif


typedef enum
{
	DHT_ERROR_NONE = 0,
	DHT_BUS_HUNG = 1,
	DHT_ERROR_NOT_PRESENT = 2,
	DHT_ERROR_ACK_TOO_LONG = 3,
	DHT_ERROR_SYNC_TIMEOUT = 4,
	DHT_ERROR_DATA_TIMEOUT = 5,
	DHT_ERROR_CHECKSUM = 6,
} DHT22_ERROR_t;

// Output format structure.
#if(OUTPUT_RAW_VALUES==0)
typedef struct
{
	int8_t temperature_integral;
	uint8_t temperature_decimal;
	uint8_t humidity_integral;
	uint8_t humidity_decimal;
} DHT22_DATA_t;
#else
typedef struct
{
	int16_t raw_temperature;
	uint16_t raw_humidity;
} DHT22_DATA_t;
#endif

extern DHT22_ERROR_t readDHT22(DHT22_DATA_t* data);

#endif /* TEMP_H_ */
