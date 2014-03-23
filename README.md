wisense-avr
===========

Software for wireless sensor unit.

Overview
------------------
Project is suited for ATTINY84 since ATTINY85 has no enough pins to communicate with DHT22 chip.
If you want to use ATTINY85, please configure software to use RESET pin for communication.
You need to program fuses correctly in ATTINY85 to change function of RESET pin and to disable
future programming (except of high voltage programming).

ATTINY84 reads data from DHT22 chip and transmitts data periodically using nRF24L01P+ chip to the central station.


Building project
------------------

Projects should be imported and built in Eclipse CDT with AVR programming support.
Check if environemnt variable **F_CPU** is set to 8000000UL.


Programming
------------------

First, read fuses to check internal clock divider settings:

```bash
avrdude -c usbasp -p t84 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U lock:r:-:h -v
```

Calculate fuses using this website: [http://www.engbedded.com/fusecalc](http://www.engbedded.com/fusecalc).
Divide clock by 8 internally; [CKDIV8=0] should be unchecked:

```bash
avrdude -c usbasp -p t84 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
```

```bash
#> cd wisense/Release
#> avrdude -p t84 -c usbasp -e -U flash:w:avr.hex
```