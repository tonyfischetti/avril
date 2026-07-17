#pragma once

#include "common.hpp"

#include <stdint.h>
#include <avr/io.h>

#include "gpio.hpp"

/**
 * Blocking SPI master.
 *
 * SPI is barely a protocol -- a shift register with a clock -- so this
 * is a thin module: configure once at compile time, then transfer()
 * bytes (full duplex: every byte out is a byte in). There is nothing to
 * time out on: the master generates the clock, so a transfer cannot
 * hang the way an I2C or SD wait can.
 *
 * Chip select is deliberately NOT managed here. CS is a per-*device*
 * concern, not a per-bus one: drive it with a GPIO<pin> in the app
 * (low = selected, for virtually every part).
 *
 * Backends, per the allow-list:
 *
 *   - ATmega328P: the hardware SPI peripheral. All four modes, both
 *     bit orders, clock dividers /2../128. Fixed pins: MOSI=PB3(17),
 *     MISO=PB4(18), SCK=PB5(19). THE SS TRAP: in master mode, if
 *     SS(PB2, 16) is an *input* and something pulls it low, the
 *     hardware silently demotes the peripheral to slave (clears MSTR).
 *     begin() therefore sets SS as an output; it is then free to be
 *     used as an ordinary GPIO -- e.g. as a device's CS.
 *
 *   - ATtiny84/85: the USI in three-wire mode, clock strobed by
 *     software (the datasheet's "SPI Master Operation Example").
 *     Modes 0 and 1 only, MSB first only -- that is all the USI can
 *     do; asking for more fails the build. Fixed pins:
 *       tiny85: DO=PB1(6), DI=PB0(5), USCK=PB2(7)
 *       tiny84: DO=PA5(8), DI=PA6(7), USCK=PA4(9)
 *     Note DO/DI are the USI's names, not the SPI ones: DO is this
 *     chip's output (connect to the slave's MOSI/SDI), DI its input
 *     (from the slave's MISO/SDO).
 *
 * maxHz is a ceiling, not a promise: the fastest achievable rate at or
 * below it is chosen. On the 328P that means the smallest hardware
 * divider that fits; on the tinies the strobe loop's natural rate is
 * ~F_CPU/12, and compile-time delay padding is inserted only when
 * maxHz asks for slower (e.g. an SD card's 400 kHz initialization).
 */

namespace HAL {
namespace Comms {
namespace SPI {

enum class Mode : uint8_t { M0, M1, M2, M3 };  // CPOL = bit1, CPHA = bit0

enum class BitOrder : uint8_t { MSB_FIRST, LSB_FIRST };

#if defined(__AVR_ATmega328P__)

namespace detail {

struct ClockOption {
    uint8_t divider;
    bool    spi2x;
    uint8_t spr_bits;
};

// descending speed; the first divider that gets under maxHz wins
constexpr ClockOption clock_options[] = {
    {   2, true,  0 },
    {   4, false, 0 },
    {   8, true,  1 },
    {  16, false, 1 },
    {  32, true,  2 },
    {  64, false, 2 },
    { 128, false, 3 },
};

template<uint32_t maxHz>
constexpr int selectClockIndex() {
    for (uint8_t i = 0;
         i < sizeof(clock_options) / sizeof(clock_options[0]); ++i) {
        if (F_CPU / clock_options[i].divider <= maxHz) return i;
    }
    return -1;
}

}

template<uint32_t maxHz,
         Mode     mode  = Mode::M0,
         BitOrder order = BitOrder::MSB_FIRST>
struct Master {

    static void begin() {
        constexpr int idx = detail::selectClockIndex<maxHz>();
        static_assert(idx >= 0,
                "maxHz is below F_CPU/128, the slowest the hardware "
                "SPI divider can go");
        constexpr detail::ClockOption opt = detail::clock_options[idx];

        // MOSI, SCK as outputs; MISO stays an input; SS must be an
        // output or a stray low on it demotes master to slave (see
        // header comment)
        HAL::GPIO::GPIO<17>::setOutput();   // MOSI = PB3
        HAL::GPIO::GPIO<19>::setOutput();   // SCK  = PB5
        HAL::GPIO::GPIO<16>::setOutput();   // SS   = PB2

        constexpr uint8_t modeBits = static_cast<uint8_t>(
            ((static_cast<uint8_t>(mode) & 0x02) ? (1 << CPOL) : 0) |
            ((static_cast<uint8_t>(mode) & 0x01) ? (1 << CPHA) : 0));
        constexpr uint8_t orderBit =
            (order == BitOrder::LSB_FIRST) ? (1 << DORD) : 0;

        SPCR = static_cast<uint8_t>((1 << SPE) | (1 << MSTR)
                                    | modeBits | orderBit
                                    | (opt.spr_bits & 0x03));
        SPSR = opt.spi2x ? (1 << SPI2X) : 0;
    }

    // full duplex: clocks `out` onto MOSI and returns what arrived on
    // MISO meanwhile. Cannot hang -- the master owns the clock
    static uint8_t transfer(uint8_t out) {
        SPDR = out;
        while (!(SPSR & (1 << SPIF))) {}
        return SPDR;
    }

    static void    write(uint8_t out) { static_cast<void>(transfer(out)); }
    static uint8_t read()             { return transfer(0xFF); }

};

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__)

namespace detail {

// the strobe loop below costs roughly this many cycles per clock edge
// even with no delay padding; only pad for the remainder. Estimated
// low on purpose: erring slow keeps maxHz an honest ceiling
constexpr uint32_t STROBE_OVERHEAD_CYCLES { 5 };

template<uint32_t maxHz>
constexpr uint32_t delayCyclesPerEdge() {
    // ceil() so the achieved rate never exceeds the ceiling
    constexpr uint32_t halfPeriod = (F_CPU + 2UL * maxHz - 1)
                                    / (2UL * maxHz);
    return halfPeriod > STROBE_OVERHEAD_CYCLES
        ? halfPeriod - STROBE_OVERHEAD_CYCLES
        : 0;
}

}

template<uint32_t maxHz,
         Mode     mode  = Mode::M0,
         BitOrder order = BitOrder::MSB_FIRST>
struct Master {

    static_assert(mode == Mode::M0 || mode == Mode::M1,
            "the USI can only do SPI modes 0 and 1 (clock idles low)");
    static_assert(order == BitOrder::MSB_FIRST,
            "the USI shifts MSB-first only");

#if defined(__AVR_ATtiny85__)
    static constexpr uint8_t doPin   { 6 };  // PB1
    static constexpr uint8_t usckPin { 7 };  // PB2
#else
    static constexpr uint8_t doPin   { 8 };  // PA5
    static constexpr uint8_t usckPin { 9 };  // PA4
#endif

    // three-wire mode, shift register clocked externally (USICS1) but
    // strobed by software (USICLK), clock line toggled by USITC. For
    // SPI mode 1 the sampling edge flips: USICS0 joins in, per the
    // datasheet's mode table
    static constexpr uint8_t strobe {
        (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC)
        | ((mode == Mode::M1) ? (1 << USICS0) : 0)
    };

    static void begin() {
        // DO and USCK driven by the USI once USIWM0 is set, but their
        // DDR bits are still ours to set; USCK's PORT bit stays 0 so
        // the clock idles low (modes 0/1). DI remains an input
        HAL::GPIO::GPIO<doPin>::setOutput();
        HAL::GPIO::GPIO<usckPin>::setOutput();
        USICR = static_cast<uint8_t>(strobe & ~(1 << USITC));
    }

    static uint8_t transfer(uint8_t out) {
        USIDR = out;
        USISR = (1 << USIOIF);   // clear the flag, zero the edge counter
        do {
            USICR = strobe;      // one clock edge per write; the
                                 // counter overflows after 16 = 8 bits
            if constexpr (detail::delayCyclesPerEdge<maxHz>() > 0) {
                __builtin_avr_delay_cycles(
                    detail::delayCyclesPerEdge<maxHz>());
            }
        } while (!(USISR & (1 << USIOIF)));
        return USIDR;
    }

    static void    write(uint8_t out) { static_cast<void>(transfer(out)); }
    static uint8_t read()             { return transfer(0xFF); }

};

#endif

}
}
}
