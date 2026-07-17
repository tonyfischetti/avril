MCU      := attiny84
F_CPU    := 8000000UL
HAL_LOC  := ../..

SRC      := main.cpp
SRC      += $(wildcard $(HAL_LOC)/src/*.cpp)
SRC      += $(wildcard $(HAL_LOC)/src/utils/*.cpp)
SRC      += $(wildcard $(HAL_LOC)/src/utils/*.S)

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

clean:
	rm -rf build

.PHONY: all clean
