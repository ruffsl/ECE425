###############################################################################
# Makefile for the project MoveSquare
###############################################################################

## General Flags
PROJECT = MoveSquare
MCU = atmega324p
TARGET = MoveSquare.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99      -DF_CPU=20000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -W1,-u,vfprintf -Wl,-Map=MoveSquare.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Users\whitemrj\Documents\GitHub\ECE425\Labs\Lab01\MoveSquare\..\..\..\..\..\..\..\..\Desktop\AVR\lib-includes" 

## Library Directories
LIBDIRS = -L"C:\Users\solorzaa\Desktop\AVR" 

## Libraries
LIBS = -lcapi324v221 -lm -lprintf_flt 

## Objects that must be built in order to link
OBJECTS = MoveSquare.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) MoveSquare.hex MoveSquare.eep MoveSquare.lss## Compile
MoveSquare.o: ../MoveSquare.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) MoveSquare.elf dep/* MoveSquare.hex MoveSquare.eep MoveSquare.lss MoveSquare.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

