#pragma once

#include "common.hpp"

#include <stdint.h>
#include <avr/io.h>

/**
 * Blocking 10-bit analog reads. All three MCUs carry the same
 * successive-approximation ADC core; a conversion takes 13 ADC clocks
 * (~110 us at the 125 kHz the prescaler lands on) and cannot hang, so
 * there is no timeout machinery -- the simplest peripheral in the HAL.
 *
 * (Namespace note: this is HAL::Analog, not HAL::ADC, because avr-libc
 * defines ADC as a macro for the result register. The macro wins.)
 *
 * The three classic gotchas, all handled internally:
 *   - ADCL must be read before ADCH: the low-byte read latches the
 *     pair, backwards reads tear the value;
 *   - the first conversion after a reference change reads garbage
 *     while the reference settles -- reference changes are tracked and
 *     a discard conversion inserted automatically;
 *   - a pin used for analog input should have its digital input
 *     buffer disabled (DIDR0), or it burns current sitting at
 *     mid-rail -- read<pin>() sets the bit as it goes.
 *
 * Channels are addressed by PHYSICAL PIN, like GPIO<pin>; a pin with
 * no ADC behind it fails the build. References are compile-time
 * (setReference<Ref::...>()) so an unsupported one -- INTERNAL_2V56
 * exists only on the ATtiny85 -- is also a build error.
 *
 * Two bonus meters, no external parts:
 *   - readVccMillivolts(): measures the internal 1.1 V bandgap
 *     *against Vcc* and solves backwards -- a battery gauge for free.
 *     The bandgap is +/-10% chip-to-chip (calibrate the constant per
 *     board if you care).
 *   - readTemperature(): the on-die sensor, raw. Roughly 1 LSB/degC
 *     with a large uncalibrated per-chip offset: good for "warmer
 *     than before", not for degrees.
 */

namespace HAL {
namespace Analog {

enum class Ref : uint8_t {
    VCC,            // supply rail (the usual choice)
    INTERNAL_1V1,   // on-die bandgap
    INTERNAL_2V56,  // ATtiny85 only
    EXTERNAL        // the AREF pin
};

namespace detail {

// the ADC wants 50-200 kHz for full 10-bit accuracy; pick the
// smallest divider that gets under 200 kHz (=> the fastest legal one)
constexpr uint8_t selectPrescalerBits() {
    // ADPS2:0 -> division factor
    constexpr uint16_t divs[] { 2, 4, 8, 16, 32, 64, 128 };
    constexpr uint8_t  bits[] { 1, 2, 3, 4,  5,  6,  7  };
    for (uint8_t i = 0; i < 7; ++i) {
        if (F_CPU / divs[i] <= 200000UL) return bits[i];
    }
    return 0;   // caught by the static_assert in begin()
}

// physical pin -> ADC channel, per MCU; -1 = that pin has no ADC
constexpr int8_t channelFor(uint8_t pin) {
#if defined(__AVR_ATtiny85__)
    if (pin == 1) return 0;   // PB5 (reset -- usable with care)
    if (pin == 7) return 1;   // PB2
    if (pin == 3) return 2;   // PB4
    if (pin == 2) return 3;   // PB3
    return -1;
#elif defined(__AVR_ATtiny84__)
    // PA0..PA7 are physical 13 down to 6, channels 0..7 in order
    if (pin >= 6 && pin <= 13) return static_cast<int8_t>(13 - pin);
    return -1;
#elif defined(__AVR_ATmega328P__)
    // PC0..PC5 are physical 23..28, channels 0..5
    if (pin >= 23 && pin <= 28) return static_cast<int8_t>(pin - 23);
    return -1;
#endif
}

// the DIDR0 bit for a channel (the tiny85 scrambles them)
constexpr uint8_t didrBit(uint8_t channel) {
#if defined(__AVR_ATtiny85__)
    return channel == 0 ? 5
         : channel == 1 ? 2
         : channel == 2 ? 4
         :                3;
#else
    return channel;   // tiny84 and 328P: bit == channel
#endif
}

constexpr uint8_t refBits(Ref r) {
#if defined(__AVR_ATtiny85__)
    return r == Ref::VCC           ? 0x00
         : r == Ref::EXTERNAL      ? (1 << REFS0)
         : r == Ref::INTERNAL_1V1  ? (1 << REFS1)
         :                           static_cast<uint8_t>((1 << REFS2)
                                                          | (1 << REFS1));
#elif defined(__AVR_ATtiny84__)
    return r == Ref::VCC           ? 0x00
         : r == Ref::EXTERNAL      ? (1 << REFS0)
         :                           (1 << REFS1);
#elif defined(__AVR_ATmega328P__)
    return r == Ref::EXTERNAL      ? 0x00
         : r == Ref::VCC           ? (1 << REFS0)
         :                           static_cast<uint8_t>((1 << REFS1)
                                                          | (1 << REFS0));
#endif
}

// internal channels: the 1.1 V bandgap and the temperature sensor
#if defined(__AVR_ATtiny85__)
constexpr uint8_t BANDGAP_MUX { 0x0C };
constexpr uint8_t TEMP_MUX    { 0x0F };
#elif defined(__AVR_ATtiny84__)
constexpr uint8_t BANDGAP_MUX { 0x21 };
constexpr uint8_t TEMP_MUX    { 0x22 };
#elif defined(__AVR_ATmega328P__)
constexpr uint8_t BANDGAP_MUX { 0x0E };
constexpr uint8_t TEMP_MUX    { 0x08 };
#endif

// reference changes need a settling conversion; remember what the
// last real conversion used. 0xFF = "never converted"
inline uint8_t lastRefBits { 0xFF };
inline uint8_t curRefBits  { 0x00 };   // set properly by begin()

// noinline: `inline` is for linkage; a busy-wait body called from
// several places should exist once (and -Winline agrees)
__attribute__((noinline)) inline uint16_t convert() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}
    uint8_t lo { ADCL };   // ADCL first: this read latches the pair
    return static_cast<uint16_t>(lo
               | (static_cast<uint16_t>(ADCH) << 8));
}

// select, settle if the reference changed, convert
__attribute__((noinline)) inline uint16_t readMux(uint8_t mux) {
    ADMUX = static_cast<uint8_t>(curRefBits | mux);
    if (curRefBits != lastRefBits) {
        static_cast<void>(convert());   // discard: reference settling
        lastRefBits = curRefBits;
    }
    return convert();
}

}

inline void begin() {
    constexpr uint8_t adps = detail::selectPrescalerBits();
    static_assert(adps != 0,
            "no ADC prescaler lands in the 50-200 kHz window at this "
            "F_CPU");
    detail::curRefBits  = detail::refBits(Ref::VCC);
    detail::lastRefBits = 0xFF;
    ADCSRA = static_cast<uint8_t>((1 << ADEN) | adps);
}

// compile-time reference selection, so asking a chip for a reference
// it doesn't have is a build error rather than a wrong voltage
template<Ref r>
inline void setReference() {
#if !defined(__AVR_ATtiny85__)
    static_assert(r != Ref::INTERNAL_2V56,
            "the 2.56 V reference exists only on the ATtiny85");
#endif
    detail::curRefBits = detail::refBits(r);
    // settling handled automatically on the next read
}

// 0..1023 against the current reference; blocking, ~110 us
// (~220 us when a reference change forces a settling conversion)
template<uint8_t physicalPin>
inline uint16_t read() {
    constexpr int8_t ch = detail::channelFor(physicalPin);
    static_assert(ch >= 0, "that physical pin has no ADC channel");
    // stop the digital input buffer from burning current at mid-rail;
    // idempotent, one sbi
    DIDR0 |= static_cast<uint8_t>(
        1 << detail::didrBit(static_cast<uint8_t>(ch)));
    return detail::readMux(static_cast<uint8_t>(ch));
}

// 8-bit variant for when 256 steps is plenty (a volume pot, say):
// same conversion, top 8 bits
template<uint8_t physicalPin>
inline uint8_t read8() {
    return static_cast<uint8_t>(read<physicalPin>() >> 2);
}

// the famous free battery gauge: sample the 1.1 V bandgap AGAINST Vcc
// and solve backwards (Vcc = 1.1 * 1024 / reading). +/-10% chip to
// chip until you calibrate the constant
inline uint16_t readVccMillivolts() {
    detail::curRefBits = detail::refBits(Ref::VCC);
    uint16_t raw { detail::readMux(detail::BANDGAP_MUX) };
    if (raw == 0) return 0;
    return static_cast<uint16_t>((1100UL * 1024UL) / raw);
}

// raw on-die temperature (needs -- and selects -- the 1.1 V
// reference; the next read() re-settles automatically). ~1 LSB/degC
// with a big uncalibrated offset: relative readings only
inline uint16_t readTemperature() {
    detail::curRefBits = detail::refBits(Ref::INTERNAL_1V1);
    return detail::readMux(detail::TEMP_MUX);
}

}
}
