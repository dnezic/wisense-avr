wisense-avr
===========

Software for wireless sensor unit. Unit is composed of AVR, nrf24l01+ wireless module and some sensors.
General idea is that unit costs as low as possible and to consume small amount of electricity.
Unit is currently powered using 2 AA batteries.

Overview
------------------
Project is suited for ATTINY861. You can also use ATTINY84 or ATTINY85 but without some components. ATTINY861 has two USI channels, so it can use at the same time SPI and I2C interfaces.
However, all modules within project work with all three chips, just with different settings.

Basic idea is that AVR reads data from BME280 chip and transmits data periodically using nRF24L01P+ chip to the central station.

Basic module description
------------------------

### avr
contains main program

### nRF24L01P
library for wireless communication.
Currently, for ATTINY861, **USIPP** register is set to 0, so PORTB has to be used.
nRF24L01P+ is set to use **250kbps** with maximum power in order to achieve better range. Dynamic payload is disabled and auto-acknowledge from receiver is disabled.

### vcc
Reads vCC voltage measured by AVR itself. It can help to monitor battery power depletion.

### dht22
Driver for DHT22 sensor. DHT22 has to be powered with voltage greater than 3V to function properly. Also clock of AVR has to be set to 8000000L.
Not used any more.

### i2c
Module for i2c master communication in standard 100KHz mode. AVR is the master. For ATTINY861 PORTA is used for USI interface, **USIPP** is set to **1**.

### bme_280
Module for communication with BME280 sensor using I2C protocol.

### bmp_085
Module for communication with BMP085 sensor using I2C protocol.
Not used any more.


Building project
------------------

Projects should be imported and built in Eclipse CDT with AVR programming support.
Check if environment variable **F_CPU** is set to 8000000UL on all projects.


Programming of fuses
--------------------------------------------

First, read fuses to check internal clock divider settings:

```bash
avrdude -c usbasp -p t84 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U lock:r:-:h -v
```

Calculate fuses using this website: [http://www.engbedded.com/fusecalc](http://www.engbedded.com/fusecalc).
Divide clock by 8 internally; [CKDIV8=0] should be unchecked:

####Write fuses

```bash
avrdude -c usbasp -p t84 -U lfuse:w:0xe2:m
```

####Flash program into the chip

```bash
cd wisense/Release
avrdude -p t861 -c usbasp -e -U flash:w:avr.hex
```
