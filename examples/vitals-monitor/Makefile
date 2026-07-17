MCU      := attiny84
F_CPU    := 8000000UL
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

# Fuses for this example: ATtiny84 on its internal 8 MHz RC (matches
# F_CPU above; no crystal, which is the point on a 14-pin part).
# Fuses persist across chip erases -- set once per chip. NEVER program
# RSTDISBL or unprogram SPIEN: either locks you out of ISP (recovery
# needs a high-voltage programmer).
#
# ACTIVE:  lfuse=0xE2  hfuse=0xDD  efuse=0xFF
#   lfuse 0xE2: internal 8 MHz RC (CKSEL=0010), slow-rising-power
#               startup, CKDIV8 off -- a factory chip has CKDIV8 ON
#               and runs at 1 MHz, which is why fresh tinies seem
#               mysteriously 8x slow
#   hfuse 0xDD: brown-out detector at 2.7 V (BODLEVEL=101); SPIEN on,
#               RSTDISBL off, EESAVE off
#   efuse 0xFF: self-programming off (default)
.PHONY: fuses
fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFF:m
#
# Alternatives (swap into the line above; each documents its tradeoff):
#   Factory-fresh chip (internal 8 MHz with CKDIV8 -> 1 MHz). Listed
#   so you can recognize/restore it; F_CPU would be 1000000UL:
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0x62:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
#   BOD off (saves ~20 uA continuously -- tempting for exactly this
#   battery example, at the cost of undefined behavior as the pack
#   dies instead of a clean reset):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
#   BOD at 1.8 V (run the pack all the way down; the LCD will give up
#   long before the AVR does):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFF:m

clean:
	rm -rf build

.PHONY: all clean
