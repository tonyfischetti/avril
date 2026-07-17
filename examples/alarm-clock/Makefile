MCU      := atmega328p
F_CPU    := 16000000UL
HAL_LOC  := ../..

SRC      := main.cpp
SRC      += $(wildcard $(HAL_LOC)/src/*.cpp)
SRC      += $(wildcard $(HAL_LOC)/src/utils/*.cpp)
SRC      += $(wildcard $(HAL_LOC)/src/utils/*.S)

AVRDUDE    ?= avrdude
# override at the prompt: make fuses PROGRAMMER=usbasp
PROGRAMMER ?= usbtiny

CXX      := avr-g++
CXXFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU) \
            -I$(HAL_LOC)/include -I$(HAL_LOC)/include/utils \
            -Os -Wall -std=gnu++2a -flto \
            -ffunction-sections -fdata-sections \
            -Wextra -Wpedantic -Wshadow -Wsign-conversion \
            -Wunreachable-code -Weffc++ -Wmissing-braces \
            -Wnon-virtual-dtor -Winline -Wnormalized -Wold-style-cast \
            -Wreturn-local-addr -Wsuggest-final-types \
            -Wsuggest-override -Wvirtual-move-assign -Wuninitialized \
            -Wl,--gc-sections

all: build/main.hex

build:
	mkdir -p build

build/main.elf: $(SRC) | build
	$(CXX) $(CXXFLAGS) $(SRC) -o $@
	avr-size -G --mcu=$(MCU) $@

build/main.hex: build/main.elf
	avr-objcopy -O ihex -R .eeprom $< $@

.PHONY: flash
flash: build/main.hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:w:$<:i

# Fuses for this example: ATmega328P on a 16 MHz crystal (matches
# F_CPU above). Fuses persist across chip erases -- set them once per
# chip. NEVER program RSTDISBL or unprogram SPIEN: either one locks
# you out of ISP (recovery needs a high-voltage programmer). Selecting
# an external clock source with nothing attached also bricks the chip
# until you feed it a clock on XTAL1.
#
# ACTIVE:  lfuse=0xFF  hfuse=0xD9  efuse=0xFD
#   lfuse 0xFF: low-power crystal oscillator, 8-16 MHz (CKSEL=1111),
#               slow-rising-power startup (SUT=11), CKDIV8 off (run at
#               full crystal speed), CKOUT off
#   hfuse 0xD9: SPIEN on (ISP stays alive), no bootloader (BOOTRST
#               off -- reset lands at our code), EESAVE off, WDTON off
#   efuse 0xFD: brown-out detector at 2.7 V -- the right call with an
#               SD card attached: a sagging supply mid-write corrupts
#               cards, and BOD resets first
.PHONY: fuses
fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xFD:m
#
# Alternatives (swap into the line above; each documents its tradeoff):
#   Factory-fresh chip (internal 8 MHz RC, CKDIV8 on -> 1 MHz). What a
#   new chip ships with; listed so you can recognize/restore it. F_CPU
#   would have to become 1000000UL:
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0x62:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m
#   Internal 8 MHz RC, no crystal needed (rebuild with
#   F_CPU := 8000000UL; the RC is +/-10%, fine for I2C/SPI, marginal
#   for UART):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xD9:m -U efuse:w:0xFD:m
#   BOD off (saves ~20 uA -- for battery builds WITHOUT an SD card):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m
#   EESAVE on (EEPROM survives chip erase -- if you store settings):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xFF:m -U hfuse:w:0xD1:m -U efuse:w:0xFD:m

clean:
	rm -rf build

.PHONY: all clean
