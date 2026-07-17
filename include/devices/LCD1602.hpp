#pragma once

#include "common.hpp"

#include <stdint.h>
#include <avr/pgmspace.h>

#include "../comms/i2c.hpp"

/**
 * 16x2 character LCD (HD44780 controller) behind the ubiquitous
 * PCF8574 I2C "backpack" -- the classic $2 way to get text output on
 * two wires. Works on all three MCUs, since the I2C module does.
 *
 * How the backpack works: the PCF8574 is just an 8-bit I2C GPIO
 * expander wired to the LCD in 4-bit mode --
 *
 *     P0 = RS   (0 command, 1 data)
 *     P1 = RW   (kept 0: this driver is write-only)
 *     P2 = EN   (data latches on EN's falling edge)
 *     P3 = backlight transistor
 *     P4..P7 = D4..D7 (the high nibble of the HD44780 data bus)
 *
 * -- so every LCD byte becomes four I2C bytes (two nibbles, each with
 * an EN pulse), sent as ONE I2C transaction. At 100 kHz that is ~450 us
 * per character: a full 32-character screen repaints in ~15 ms, and
 * the HD44780's ~37 us instruction time is hidden entirely inside the
 * bus time, so no per-character delays are needed. Only clear()/home()
 * (~1.6 ms inside the controller) get an explicit wait.
 *
 * Addresses: PCF8574 backpacks sit at 0x27 (A-variant chips at 0x3F)
 * with all address jumpers open. A wrong address comes back as
 * Result::NACK_ADDR from begin() -- and the I2C module's ping() makes
 * a fine bus scanner when in doubt.
 *
 * Errors are HAL::Comms::I2C::Result, passed straight through: the
 * only thing that can fail here is the bus.
 */

namespace HAL {
namespace Devices {

template<typename I2cBus, uint8_t addr7 = 0x27>
struct LCD1602 {

    using Result = HAL::Comms::I2C::Result;

  private:
    static constexpr uint8_t RS { 0x01 };
    static constexpr uint8_t EN { 0x04 };
    static constexpr uint8_t BL { 0x08 };

    // the backlight bit must ride along with every write, so its
    // current state is remembered here
    static inline uint8_t blMask { BL };

    // one HD44780 byte = two nibbles on P4..P7, each latched by an EN
    // falling edge; all four backpack states go in a single I2C
    // transaction
    static Result sendByte(uint8_t val, bool dataP) {
        uint8_t rs { dataP ? RS : static_cast<uint8_t>(0) };
        uint8_t hi { static_cast<uint8_t>((val & 0xF0) | rs | blMask) };
        uint8_t lo { static_cast<uint8_t>(((val << 4) & 0xF0) | rs
                                          | blMask) };
        uint8_t seq[4] { static_cast<uint8_t>(hi | EN), hi,
                         static_cast<uint8_t>(lo | EN), lo };
        return I2cBus::write(addr7, seq, 4);
    }

    static Result command(uint8_t c) { return sendByte(c, false); }

    // during the reset dance the controller is still in 8-bit mode and
    // only the high nibble counts, so single-nibble writes are needed
    static Result nibble(uint8_t nib) {
        uint8_t v { static_cast<uint8_t>(((nib << 4) & 0xF0) | blMask) };
        uint8_t seq[2] { static_cast<uint8_t>(v | EN), v };
        return I2cBus::write(addr7, seq, 2);
    }

  public:
    static Result begin() {
        I2cBus::begin();
        // power-on: the controller wants >= 40 ms before it will listen
        __builtin_avr_delay_cycles((F_CPU / 1000UL) * 50UL);

        // the datasheet's reset-by-instruction dance: 0x3 three times
        // (works whatever half-state the controller woke up in), then
        // 0x2 to drop into 4-bit mode
        Result r { nibble(0x03) };
        if (r != Result::OK) return r;   // usually NACK_ADDR: wrong addr7
        __builtin_avr_delay_cycles((F_CPU / 1000UL) * 5UL);
        nibble(0x03);
        __builtin_avr_delay_cycles((F_CPU / 1000UL) * 5UL);
        nibble(0x03);
        __builtin_avr_delay_cycles((F_CPU / 1000000UL) * 150UL);
        nibble(0x02);
        __builtin_avr_delay_cycles((F_CPU / 1000000UL) * 150UL);

        command(0x28);   // function set: 4-bit, 2 lines, 5x8 font
        command(0x08);   // display off while configuring
        r = clear();
        if (r != Result::OK) return r;
        command(0x06);   // entry mode: advance right, no shift
        return command(0x0C);   // display on, cursor off, blink off
    }

    static Result clear() {
        Result r { command(0x01) };
        // clear/home are the two slow instructions (~1.6 ms internally)
        __builtin_avr_delay_cycles((F_CPU / 1000UL) * 2UL);
        return r;
    }

    static Result home() {
        Result r { command(0x02) };
        __builtin_avr_delay_cycles((F_CPU / 1000UL) * 2UL);
        return r;
    }

    static Result setCursor(uint8_t col, uint8_t row) {
        // row 1 starts at DDRAM 0x40 on a 16x2
        return command(static_cast<uint8_t>(
            0x80 | (col + (row ? 0x40 : 0))));
    }

    // display / cursor / blink in one call (they share a register)
    static Result setDisplay(bool onP, bool cursorP, bool blinkP) {
        return command(static_cast<uint8_t>(
            0x08 | (onP ? 0x04 : 0) | (cursorP ? 0x02 : 0)
                 | (blinkP ? 0x01 : 0)));
    }

    static Result backlight(bool onP) {
        blMask = onP ? BL : 0;
        // no EN pulse needed: just repaint the expander's outputs
        uint8_t v { blMask };
        return I2cBus::write(addr7, &v, 1);
    }

    static Result write(char c) {
        return sendByte(static_cast<uint8_t>(c), true);
    }

    static Result print(const char* s) {
        while (*s) {
            Result r { write(*s++) };
            if (r != Result::OK) return r;
        }
        return Result::OK;
    }

    // PROGMEM twin, same convention as the UART: wrap literals in
    // PSTR() and they cost flash only
    static Result print_P(const char* s) {
        uint8_t c;
        while ((c = pgm_read_byte(s++)) != 0) {
            Result r { sendByte(c, true) };
            if (r != Result::OK) return r;
        }
        return Result::OK;
    }

    static Result print(uint32_t n) {
        char buf[11];
        char* p { buf + 10 };
        *p = '\0';
        do {
            *--p = static_cast<char>('0' + static_cast<uint8_t>(n % 10));
            n /= 10;
        } while (n != 0);
        return print(p);
    }

    static Result print(int32_t n) {
        if (n < 0) {
            Result r { write('-') };
            if (r != Result::OK) return r;
            return print(0U - static_cast<uint32_t>(n));
        }
        return print(static_cast<uint32_t>(n));
    }

    // custom 5x8 glyphs, slots 0..7: glyph[i] holds row i's five
    // pixels in its low bits. Print one with write(char(slot))
    static Result createChar(uint8_t slot, const uint8_t glyph[8]) {
        Result r { command(static_cast<uint8_t>(
            0x40 | ((slot & 0x07) << 3))) };
        if (r != Result::OK) return r;
        for (uint8_t i = 0; i < 8; ++i) {
            r = sendByte(glyph[i], true);
            if (r != Result::OK) return r;
        }
        // leave CGRAM: point back at the display
        return setCursor(0, 0);
    }

};

}
}
