all: bootldr communication reception ohc ohc-arduino-8mhz ohc-arduino-16mhz

.PHONY: docs bootldr communication reception ohc ohc-arduino-8mhz ohc-arduino-16mhz
KILOLIB = build/kilolib.a
communication: build/communication.elf build/communication.hex build/communication.lss
reception: build/reception.elf build/reception.hex build/reception.lss

CC = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

PFLAGS = -P usb -c avrispmkII # user to reprogram OHC
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -DF_CPU=8000000
ASFLAGS = $(CFLAGS)

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0  

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

%.eep: %.elf
	$(AVROC) -O ihex $(EEPROM) $< $@

%.bin: %.elf
	$(AVROC) -O binary $(FLASH) $< $@ 

build:
	mkdir -p $@

$(KILOLIB): kilolib.o message_crc.o message_send.o | build
	$(AVRAR) rcs $@ kilolib.o message_crc.o message_send.o 
	rm -f *.o

build/communication.elf: communication.c $(KILOLIB) | build
	$(CC) $(CFLAGS) -o $@ $< $(KILOLIB) 

build/reception.elf: reception.c $(KILOLIB) | build
	$(CC) $(CFLAGS) -o $@ $< $(KILOLIB) 

program-communication: build/communication.hex
	$(AVRUP) -p m328p $(PFLAGS) -U "flash:w:build/communication.hex:i" -U "flash:w:build/bootldr.hex"

program-reception: build/reception.hex
	$(AVRUP) -p m328p $(PFLAGS) -U "flash:w:build/reception.hex:i" -U "flash:w:build/bootldr.hex"

docs:
	cat message.h kilolib.h message_crc.h | grep -v "^\#" > docs/kilolib.h
	(cd docs; doxygen)

clean:
	rm -fR build
