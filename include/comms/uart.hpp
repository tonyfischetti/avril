#pragma once

#include "common.hpp"

#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

/**
 * Blocking, transmit-oriented debug UART (ATmega328P only; the ATtiny84
 * and ATtiny85 have no USART).
 *
 * The baud divisor is computed at compile time with proper rounding, and
 * double-speed mode (U2X) is selected automatically when it gives a
 * smaller rate error (this is what makes e.g. 115200 @ 16 MHz work: the
 * truncated normal-mode divisor is off by +8.5%, the rounded U2X one by
 * -2.1%). Rates that cannot be achieved within 2.5% -- or that need more
 * than UBRR's 12 bits -- fail the build.
 *
 * Blocking is a feature here: no ISR, no buffer, no lost messages when
 * the chip resets two instructions later. The costs are known and
 * bounded (one byte is ~87 us at 115200). Two things to know:
 *
 *   - String literals passed to print() live in .data: flash AND RAM,
 *     copied at startup. On a 2 KB-RAM part, debug strings are the
 *     classic silent RAM eater. Use print_P(PSTR("...")) to keep them
 *     in flash only.
 *   - printByte() waits for the *buffer* (UDRE0), not the *shift
 *     register* (TXC0), so the final frame is still on the wire when
 *     the last print() returns. Call flush() before sleeping or
 *     resetting, or the last character arrives mangled.
 */

namespace HAL {
namespace Comms {
namespace UART {

// compile only where a USART exists: an allow-list, not a deny-list,
// so a newly supported MCU without a USART gets an empty namespace
// instead of a wall of undefined-register errors
#if defined(__AVR_ATmega328P__)

namespace detail {

struct BaudConfig {
    uint16_t ubrr;
    bool     u2x;
    uint32_t actualBaud;
};

constexpr uint32_t absDiff(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

template<uint32_t baud>
constexpr BaudConfig selectBaudConfig() {
    // rounded-to-nearest (UBRR+1) for normal (/16) and U2X (/8) modes
    constexpr uint32_t divN = (F_CPU + 8UL * baud) / (16UL * baud);
    constexpr uint32_t divU = (F_CPU + 4UL * baud) / (8UL  * baud);
    constexpr uint32_t actN = divN ? F_CPU / (16UL * divN) : 0;
    constexpr uint32_t actU = divU ? F_CPU / (8UL  * divU) : 0;

    // prefer normal mode on ties: U2X halves the receiver's sampling
    if (divN && absDiff(actN, baud) <= absDiff(actU, baud)) {
        return { static_cast<uint16_t>(divN - 1), false, actN };
    }
    return { static_cast<uint16_t>(divU - 1), true, actU };
}

// flush() must not wait on TXC0 if nothing was ever transmitted -- the
// flag only latches after a first completed frame, so a pre-first-print
// flush() would spin forever. One byte of state buys away the deadlock
inline bool anythingSentP { false };

constexpr uint8_t hexDigit(uint8_t nibble) {
    return static_cast<uint8_t>(nibble < 10 ? '0' + nibble
                                            : 'A' + (nibble - 10));
}

}

template<uint32_t BaudRate>
inline void init() {
    static_assert(BaudRate > 0, "Baud rate must be > 0");

    constexpr detail::BaudConfig cfg =
        detail::selectBaudConfig<BaudRate>();

    static_assert(cfg.ubrr <= 4095,
            "Baud rate too slow for the 12-bit UBRR register at this F_CPU");
    static_assert(detail::absDiff(cfg.actualBaud, BaudRate) * 1000UL
                      / BaudRate <= 25,
            "Baud rate not achievable within 2.5% at this F_CPU");

    UBRR0H = static_cast<uint8_t>(cfg.ubrr >> 8);
    UBRR0L = static_cast<uint8_t>(cfg.ubrr);

    UCSR0A = cfg.u2x ? (1 << U2X0) : 0;

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

    // Enable transmitter
    //  TODO  parameterize
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

// noinline (here and below): `inline` is for linkage, not a request to
// inline -- a busy-wait body called from a dozen places should exist
// once. It also keeps -Winline quiet now that GCC's cost model declines
// to inline these at their call-site count
__attribute__((noinline)) inline void printByte(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1<<UDRE0))) {}

    // clear the sticky TXC0 (write-one-to-clear) as the byte is queued,
    // so flush() measures THIS transmission, not a long-finished one.
    // Keep only the R/W bits (U2X0, MPCM0); the datasheet wants the RX
    // status flags written as zero
    UCSR0A = static_cast<uint8_t>(
        (UCSR0A & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0));
    detail::anythingSentP = true;

    // Put data into buffer, sends the data
    UDR0 = data;
}

// block until the shift register has pushed the final frame onto the
// wire. printByte() only waits for the buffer, so without this a
// goToSleep() (or a watchdog reboot) right after a print mangles the
// last character mid-frame
inline void flush() {
    if (!detail::anythingSentP) return;  // TXC0 never latches otherwise
    while (!(UCSR0A & (1 << TXC0))) {}
}

__attribute__((noinline)) inline void print(const char* str) {
    while (*str) {
        printByte(static_cast<uint8_t>(*str++));
    }
}

// PROGMEM variant: `flashStr` must point into flash -- wrap literals in
// PSTR(), e.g. print_P(PSTR("booted")). Unlike print("booted"), the
// string then costs flash only, not a startup-copied RAM shadow
__attribute__((noinline)) inline void print_P(const char* flashStr) {
    uint8_t c;
    while ((c = pgm_read_byte(flashStr++)) != 0) {
        printByte(c);
    }
}

// digits are peeled off least-significant-first into the back of a
// small stack buffer. ~40 bytes of code; the snprintf("%lu") this
// replaced dragged avr-libc's formatted-print machinery (~1.4 KB of
// flash) into every build that printed a number
__attribute__((noinline)) inline void print(uint32_t n) {
    char buf[11];             // 4294967295 is 10 digits + NUL
    char* p { buf + 10 };
    *p = '\0';
    do {
        *--p = static_cast<char>('0' + static_cast<uint8_t>(n % 10));
        n /= 10;
    } while (n != 0);
    print(p);
}

inline void print(int32_t n) {
    if (n < 0) {
        printByte(static_cast<uint8_t>('-'));
        // negate in unsigned space: -INT32_MIN overflows int32_t, but
        // 0 - 0x80000000u is 2147483648, exactly the digits we want
        print(0U - static_cast<uint32_t>(n));
    } else {
        print(static_cast<uint32_t>(n));
    }
}

// exact-match overloads for the 16-bit types so that plain `int`
// arguments (int == int16_t on AVR) resolve without ambiguity between
// the 32-bit signed and unsigned printers. uint8_t/int8_t arguments
// promote to int and land on the int16_t overload, printing their
// numeric value; use printByte() to send a raw character instead
inline void print(uint16_t n) { print(static_cast<uint32_t>(n)); }
inline void print(int16_t n)  { print(static_cast<int32_t>(n));  }

// fixed-width uppercase hex, no prefix: two digits per byte, so
// printHex(PINB) reads like the datasheet. Widths compose: the 16- and
// 32-bit versions print 4 and 8 digits
__attribute__((noinline)) inline void printHex(uint8_t n) {
    printByte(detail::hexDigit(static_cast<uint8_t>(n >> 4)));
    printByte(detail::hexDigit(static_cast<uint8_t>(n & 0x0F)));
}

inline void printHex(uint16_t n) {
    printHex(static_cast<uint8_t>(n >> 8));
    printHex(static_cast<uint8_t>(n));
}

inline void printHex(uint32_t n) {
    printHex(static_cast<uint16_t>(n >> 16));
    printHex(static_cast<uint16_t>(n));
}

namespace detail {
__attribute__((noinline)) inline void newline() {
    printByte(static_cast<uint8_t>('\r'));
    printByte(static_cast<uint8_t>('\n'));
}
}

// the println family is noinline too: each is a two-call wrapper that
// GCC stops inlining as call sites accumulate, and -Winline would
// then flag them one by one as programs grow
__attribute__((noinline)) inline void println(const char* str) {
    print(str); detail::newline();
}
__attribute__((noinline)) inline void println_P(const char* s) {
    print_P(s); detail::newline();
}
__attribute__((noinline)) inline void println(uint32_t n) {
    print(n); detail::newline();
}
__attribute__((noinline)) inline void println(int32_t n) {
    print(n); detail::newline();
}
__attribute__((noinline)) inline void println(uint16_t n) {
    print(n); detail::newline();
}
__attribute__((noinline)) inline void println(int16_t n) {
    print(n); detail::newline();
}
__attribute__((noinline)) inline void printlnHex(uint8_t n) {
    printHex(n); detail::newline();
}
__attribute__((noinline)) inline void printlnHex(uint16_t n) {
    printHex(n); detail::newline();
}
__attribute__((noinline)) inline void printlnHex(uint32_t n) {
    printHex(n); detail::newline();
}

#endif

}
}
}
