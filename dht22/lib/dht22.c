/*
 * temp.c
 *
 * Created: 29.1.2014. 20:49:51
 *  Author: drazen
 */
#include <dht22.h>

DHT22_ERROR_t readDHT22(DHT22_DATA_t* data) {


	uint8_t retryCount = 0;
	uint8_t csPart1, csPart2, csPart3, csPart4;
	uint16_t rawHumidity = 0;
	uint16_t rawTemperature = 0;
	uint8_t checkSum = 0;
	uint8_t i;

	// Pin needs to start HIGH, wait until it is HIGH with a timeout
	retryCount = 0;
	DHT22_DDR &= ~(1 << ( DHT22_PIN));
	do {
		if (retryCount > 125)
			return DHT_BUS_HUNG;
		retryCount++;
		_delay_us(2);
	} while (!( DHT22_PORT_IN & (1 << DHT22_PIN))); //!DIRECT_READ(reg, bitmask)

	// Send the activate pulse
	DHT22_PORT_OUT &= ~(1 << ( DHT22_PIN));    //DIRECT_WRITE_LOW(reg, bitmask);
	DHT22_DDR |= 1 << ( DHT22_PIN); //DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
	_delay_ms(2);                                           // spec is 1 to 10ms
	DHT22_DDR &= ~(1 << ( DHT22_PIN));  // Switch back to input so pin can float
	DHT22_PORT_OUT |= (1 << ( DHT22_PIN)); // Enable pullup.

	// Find the start of the ACK signal
	retryCount = 0;
	do {
		if (retryCount > 25)              //(Spec is 20 to 40 us, 25*2 == 50 us)
				{
			//                      data->retryCount = retryCount;
			return DHT_ERROR_NOT_PRESENT;
		}
		retryCount++;
		_delay_us(2);
	} while ( DHT22_PORT_IN & (1 << DHT22_PIN)); // While pin is 1.

	// Here sensor responded pulling the line down DHT22_PIN = 0

	// Find the transition of the ACK signal
	retryCount = 0;
	do {
		if (retryCount > 50)                   //(Spec is 80 us, 50*2 == 100 us)
				{
			//data->retryCount = retryCount;
			return DHT_ERROR_ACK_TOO_LONG;
		}
		retryCount++;
		_delay_us(2);
	} while (!(DHT22_PORT_IN & (1 << DHT22_PIN)));

	// Here sensor pulled up DHT22_PIN = 1

	// Find the end of the ACK signal
	retryCount = 0;
	do {
		if (retryCount > 50)                   //(Spec is 80 us, 50*2 == 100 us)
				{
			return DHT_ERROR_ACK_TOO_LONG;
		}
		retryCount++;
		_delay_us(2);
	} while ( DHT22_PORT_IN & (1 << DHT22_PIN));
	// Here sensor pulled down to start transmitting bits.

	// Read the 40 bit data stream
	for (i = 0; i < DHT22_DATA_BIT_COUNT; i++) {
		// Find the start of the sync pulse
		retryCount = 0;
		do {
			if (retryCount > 35)                //(Spec is 50 us, 35*2 == 70 us)
					{
				return DHT_ERROR_SYNC_TIMEOUT;
			}
			retryCount++;
			_delay_us(2);
		} while (!(DHT22_PORT_IN & (1 << DHT22_PIN)));

		// Measure the width of the data pulse
		retryCount = 0;
		do {
			if (retryCount > 50)               //(Spec is 80 us, 50*2 == 100 us)
					{
				return DHT_ERROR_DATA_TIMEOUT;
			}
			retryCount++;
			_delay_us(2);
		} while ( DHT22_PORT_IN & (1 << DHT22_PIN));

		// Identification of bit values.
		if (retryCount > 20) // Bit is 1: 20*2 = 40us (specification for bit 0 is 26 a 28us).
				{
			if (i < 16) // Humidity
					{
				rawHumidity |= (1 << (15 - i));
			}
			if ((i > 15) && (i < 32))  // Temperature
					{
				rawTemperature |= (1 << (31 - i));
			}
			if ((i > 31) && (i < 40))  // CRC data
					{
				checkSum |= (1 << (39 - i));
			}
		}
	}
	// translate bitTimes
	// 26~28us == logical 0
	// 70us    == logical 1
	// here threshold is 40us

	// calculate checksum
	csPart1 = rawHumidity >> 8;
	csPart2 = rawHumidity & 0xFF;
	csPart3 = rawTemperature >> 8;
	csPart4 = rawTemperature & 0xFF;

	if (checkSum == ((csPart1 + csPart2 + csPart3 + csPart4) & 0xFF)) {
#if(OUTPUT_RAW_VALUES==0)
		// raw data to sensor values
		data->humidity_integral = (uint8_t) (rawHumidity / 10);
		data->humidity_decimal = (uint8_t) (rawHumidity % 10);

		if (rawTemperature & 0x8000) // Check if temperature is below zero, non standard way of encoding negative numbers!
				{
			rawTemperature &= 0x7FFF; // Remove signal bit
			data->temperature_integral = (int8_t) (rawTemperature / 10) * -1;
			data->temperature_decimal = (uint8_t) (rawTemperature % 10);
		} else {
			data->temperature_integral = (int8_t) (rawTemperature / 10);
			data->temperature_decimal = (uint8_t) (rawTemperature % 10);
		}
#else
		if(rawTemperature & 0x8000) // Check if temperature is below zero, non standard way of encoding negative numbers!
		{
			rawTemperature &= 0x7FFF; // Remove signal bit
			data->raw_temperature = ((int16_t)rawTemperature) * -1;
		} else
		{
			data->raw_temperature = rawTemperature;
		}
		data->raw_humidity = rawHumidity;
#endif

		return DHT_ERROR_NONE;
	}
	return DHT_ERROR_CHECKSUM;
}

