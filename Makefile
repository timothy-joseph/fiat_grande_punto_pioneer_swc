CC=avr-gcc
CFLAGS=-Os -DF_CPU=16000000L -mmcu=atmega328p -Wall -Wextra -std=c99 -Iinclude
LDFLAGS=-mmcu=atmega328p -Wall -Wextra

DEV=/dev/ttyUSB0
BAUD_SERIAL=9600

BIN_FILE = firmware.bin
HEX_FILE = firmware.hex
CFILES = swc.c

OFILES_DIR = build/
OFILES = $(CFILES:%.c=build/%.o)

build: $(HEX_FILE)

$(HEX_FILE): $(BIN_FILE)
	avr-objcopy -O ihex -R .eeprom $(BIN_FILE) $(HEX_FILE)

$(BIN_FILE): $(OFILES)
	$(CC) -o $@ $(OFILES) $(LDFLAGS)

$(OFILES): build/%.o : %.c

build/%.o : %.c
	mkdir -p $(@D)
	$(CC) -c $(CFLAGS) $^ -o $@

clean:
	rm -rf $(OFILES) $(HEX_FILE) $(BIN_FILE) $(OFILES_DIR)

flash: $(HEX_FILE)
	avrdude -F -V -c arduino -p ATMEGA328P -P $(DEV) -b 115200 -U flash:w:$(HEX_FILE)

monitor:
	screen $(DEV) $(BAUD_SERIAL)
