CC=avr-gcc
CFLAGS += -DF_CPU=16000000UL
CFLAGS += -mmcu=atmega644p
CFLAGS += -Os
all: main.o
	avr-gcc -mmcu=atmega644p -o firmware $^
	avr-objcopy -O ihex -R .eeprom firmware firmware.hex
	avr-size --mcu=atmega644p -C firmware

fuses:
	avrdude -c usbasp -p atmega644p -P usb -U lfuse:w:0xff:m -U hfuse:w:0xd7:m
