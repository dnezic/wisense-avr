MCU=attiny861
F_CPU=8000000
MCU_DEF=__AVR_ATtiny861A__
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -D${MCU_DEF} -I/usr/avr/sys-root/include/ -Iwisense/.
TARGET=main
SRCS=wisense/twimaster.c wisense/spi.c wisense/mirf.c wisense/bme280.c wisense/voltage.c wisense/main.c

all:
		${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
		${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

flash:
		avrdude -p ${MCU} -c usbasp -U flash:w:${TARGET}.hex:i -F -P usb

clean:
		rm -f *.bin *.hex