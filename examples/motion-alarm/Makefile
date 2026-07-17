MCU      := attiny85
F_CPU    := 8000000UL
HAL_LOC  := ../..

# two sensor variants, one circuit: pir.cpp (room alarm, ~58 uA) or
# sw18015p.cpp (bike/tamper alarm, ~8 uA). Build the other with
#   make VARIANT=sw18015p
VARIANT  ?= pir

SRC      := $(VARIANT).cpp
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

all: build/$(VARIANT).hex

build:
	mkdir -p build

build/$(VARIANT).elf: $(SRC) | build
	$(CXX) $(CXXFLAGS) $(SRC) -o $@
	avr-size -G --mcu=$(MCU) $@

build/$(VARIANT).hex: build/$(VARIANT).elf
	avr-objcopy -O ihex -R .eeprom $< $@

.PHONY: flash
flash: build/$(VARIANT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:w:$<:i

# Fuses for this example: ATtiny85 on its internal 8 MHz RC. Fuses
# persist across chip erases -- set once per chip. NEVER program
# RSTDISBL or unprogram SPIEN: either locks you out of ISP (recovery
# needs a high-voltage programmer).
#
# ACTIVE:  lfuse=0xE2  hfuse=0xDF  efuse=0xFF
#   lfuse 0xE2: internal 8 MHz RC, CKDIV8 off (factory chips ship
#               with CKDIV8 ON and run 8x slow)
#   hfuse 0xDF: BOD *off* -- deliberately, for this example: BOD
#               burns ~20 uA continuously, a third of this alarm's
#               entire sleep budget. The tradeoff: as the 9 V finally
#               dies the chip browns out unsupervised instead of
#               resetting cleanly -- acceptable for a buzzer, wrong
#               for anything writing flash/EEPROM/SD
#   efuse 0xFF: self-programming off (default)
.PHONY: fuses
fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
#
# Alternatives (swap into the line above; each documents its tradeoff):
#   Factory-fresh chip (internal 8 MHz with CKDIV8 -> 1 MHz). Listed
#   so you can recognize/restore it; F_CPU would be 1000000UL:
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0x62:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
#   BOD at 1.8 V (clean resets at the cost of ~20 uA -- if the alarm
#   ever grows state worth protecting):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFF:m
#   BOD at 2.7 V (the choice for SD/EEPROM-writing builds):
#	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xE2:m -U hfuse:w:0xDD:m -U efuse:w:0xFF:m

clean:
	rm -rf build

.PHONY: all clean
