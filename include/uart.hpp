#pragma once

#include "common.hpp"

#include <stdio.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


/**
 * Blocking, transmit-oriented debug UART (ATmega328P only; the ATtiny85
 * has no USART).
 *
 * The baud divisor is computed at compile time with proper rounding, and
 * double-speed mode (U2X) is selected automatically when it gives a
 * smaller rate error (this is what makes e.g. 115200 @ 16 MHz work: the
 * truncated normal-mode divisor is off by +8.5%, the rounded U2X one by
 * -2.1%). Rates that cannot be achieved within 2.5% -- or that need more
 * than UBRR's 12 bits -- fail the build.
 */

namespace HAL {
namespace UART {

#if defined(__AVR_ATtiny85__)
// #warning "HAL::UART is not supported on ATtiny85"
#else

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

inline void printByte(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1<<UDRE0))) {}

    // Put data into buffer, sends the data
    UDR0 = data;
}

inline void print(const char* str) {
    while (*str) {
        printByte(static_cast<uint8_t>(*str++));
    }
}

inline void print(uint32_t n) {
    char buf[11];   // 4294967295 is 10 digits; snprintf NUL-terminates
    snprintf(buf, sizeof(buf), "%lu", n);
    print(buf);
}

inline void println(const char* str) {
    print(str);
    print("\r\n");
}

inline void println(uint32_t n) {
    print(n);
    print("\r\n");
}

#endif

}
}
