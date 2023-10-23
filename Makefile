default:
	avr-gcc -Os -DF_CPU=16000000L -mmcu=atmega328p -c -o swc.o swc.c
	avr-gcc -mmcu=atmega328p -o swc.bin swc.o -Wall -Wextra
	avr-objcopy -O ihex -R .eeprom swc.bin swc.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:swc.hex
