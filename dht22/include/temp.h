/*
* temp.h
*
* Currently works only for attiny84
*/


#ifndef TEMP_H_
#define TEMP_H_

#include <avr/io.h>
#include <util/delay.h>

#define OUTPUT_RAW_VALUES 0

#define DHT22_DATA_BIT_COUNT 40

/* Configure port and pin */
#define DHT22_PIN PINA1
#define DHT22_POWER PINA2
#define DHT22_DDR DDRA
#define DHT22_PORT_OUT PORTA
#define DHT22_PORT_IN PINA

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
extern void powerOnDHT22();
extern void powerOffDHT22();


#endif /* TEMP_H_ */
