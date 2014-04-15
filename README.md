wisense-avr
===========

Software for wireless sensor unit. Unit is composed of AVR, nrf24l01+ wireless module and some sensors.
General idea is that unit costs as low as possible and to consume small amount of electricity.
Unit is currently powered using 2 AA batteries. 

Overview
------------------
Project is suited for ATTINY861. You can also use ATTINY84 since ATTINY85 has no enough pins to communicate with DHT22 chip unless RESET pin is "fused" to be "normal" IO pin.
However, all modules within project work with all three chips, just with different settings.
So for ATTINY85, please configure software to use RESET pin for communication.
You need to program fuses correctly in ATTINY85 to change function of RESET pin and to disable
future programming (except of high voltage programming).

Basic idea is that AVR reads data from DHT22 chip and transmits data periodically using nRF24L01P+ chip to the central station. This idea had to be currently abandoned because DHT22 requires at least 3V in order to function normally. Step-Up converters of decent quality with low stand-by current consumption are more expensive than other components alltogether. Currently BMP085 is used with barometric pressure sensor and *not so accurate* temperature sensor.
Because of that, analog temperature sensor will be added in the future - or *not yet available* BMP sensor with humidity sensing too. That is the reason why ATTINY861 is used, because it has two USI ports, one is used for SPI and other for I2C communication.


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

### i2c
Module for i2c master communication in standard 100KHz mode. AVR is the master. For ATTINY861 PORTA is used for USI interface, **USIPP** is set to **1**.

### bmp_085
Module for communication with BMP085 sensor using I2C protocol.

Building project
------------------

Projects should be imported and built in Eclipse CDT with AVR programming support.
Check if environemnt variable **F_CPU** is set to 1000000UL.
In order to use DHT22 module, **F_CPU** has to be set to 8000000L, and clock divisor by 8 disabled by fuse programming.


Programming of fuses (only if DHT22 is used)
--------------------------------------------

First, read fuses to check internal clock divider settings:

```bash
avrdude -c usbasp -p t84 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U lock:r:-:h -v
```

Calculate fuses using this website: [http://www.engbedded.com/fusecalc](http://www.engbedded.com/fusecalc).
Divide clock by 8 internally; [CKDIV8=0] should be unchecked:

####Write fuses

```bash
avrdude -c usbasp -p t84 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
```

####Flash program into the chip

```bash
cd wisense/Release
avrdude -p t84 -c usbasp -e -U flash:w:avr.hex
```
