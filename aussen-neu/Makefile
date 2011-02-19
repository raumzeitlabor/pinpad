CC = avr-gcc

MCU := atmega644p
MHZ := 16000000UL
ADDRESS := 1

# CFLAGS for ATmega
CFLAGS += -Wall
CFLAGS += -std=c99
CFLAGS += -DF_CPU=${MHZ}
CFLAGS += -DMCU=${MCU}
CFLAGS += -mmcu=${MCU}
CFLAGS += -Os

CFLAGS += -I../lib
CFLAGS += -DMYADDRESS=${ADDRESS}

#.SILENT:

.PHONY: clean

all: firmware.hex

firmware.hex: main.o lcd.o
	$(CC) -mmcu=atmega644p -o $(shell basename $@ .hex).bin $^
	avr-objcopy -O ihex -R .eeprom $(shell basename $@ .hex).bin $@
	avr-size --mcu=${MCU} -C $(shell basename $@ .hex).bin

clean:
	rm -f *.o

program:
	sudo avrdude -c usbasp -p atmega644p -P usb -U flash:w:firmware.hex:i

fuses:
	sudo avrdude -c usbasp -p atmega644p -P usb -U lfuse:w:0x9f:m -U hfuse:w:0xd9:m
