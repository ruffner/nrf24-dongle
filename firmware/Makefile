DEVICE  = attiny84
F_CPU   = 1000000
# internal 8Mhz, divide by 8, preserve EEPROM through chip erase
FUSE_L  = 0x62
FUSE_H  = 0xd7
AVRDUDE = avrdude -c usbtiny -p $(DEVICE)

#CFLAGS  = -Iusbdrv -I. -DDEBUG_LEVEL=0
CFLAGS  = -I. -DDEBUG_LEVEL=0
OBJECTS =  nrf24-dongle.o

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)
COMPILEPP = avr-g++ -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ....... to build nrf24-dongle.hex"
	@echo "make program ... to flash fuses and firmware"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash nrf24-dongle.hex without compiling"
	@echo "make clean ..... to delete objects and hex file"

hex: nrf24-dongle.hex

program: flash fuse

# rule for programming fuse bits if they are defined
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
                { echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m
read:
	$(AVRDUDE) -U hfuse:r:high.txt:r -U lfuse:r:low.txt:r

# rule for uploading firmware:
flash: nrf24-dongle.hex
	$(AVRDUDE) -U flash:w:nrf24-dongle.hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f *.o *.hex *.elf

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

# for VUSB integration
#usbdrv:
#	cp -r ../../../usbdrv .

nrf24-dongle.elf: $(OBJECTS)
	$(COMPILE) -o nrf24-dongle.elf $(OBJECTS)

nrf24-dongle.hex: nrf24-dongle.elf
	rm -f nrf24-dongle.hex nrf24-dongle.eep.hex
	avr-objcopy -j .text -j .data -O ihex nrf24-dongle.elf nrf24-dongle.hex
	avr-size nrf24-dongle.hex

# debugging targets:

disasm:	nrf24-dongle.elf
	avr-objdump -d nrf24-dongle.elf
