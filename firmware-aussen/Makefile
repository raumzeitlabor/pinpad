CC=avr-gcc
CFLAGS += -DF_CPU=16000000UL
CFLAGS += -mmcu=atmega168
CFLAGS += -Os
all: main.o
	avr-gcc -mmcu=atmega168 -o firmware $^
	avr-objcopy -O ihex -R .eeprom firmware firmware.hex
	avr-size --mcu=atmega168 -C firmware
