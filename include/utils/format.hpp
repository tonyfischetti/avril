#pragma once

#include <stdint.h>

/**
 * Sink-agnostic number formatting: every converter writes into a
 * caller's buffer, and every sink (UART, LCD, a log record) prints
 * the result. Before this existed, the decimal digit-peel lived in
 * three places, the INT32_MIN negation trick in two, hex digits in
 * two, and six examples each grew their own zero-padded `print2` --
 * this is all of them, once.
 *
 * Deliberately dependency-free (just <stdint.h>): these functions
 * are pure computation, which also makes them HOST-TESTABLE -- see
 * tests/format_test.cpp, which checks every edge against the host's
 * snprintf. If you touch this file, run that.
 *
 * Termination conventions, worth reading once:
 *   - u32()/s32() return a NUL-terminated C string (a pointer into
 *     the caller's buffer -- digits are peeled from the back, so the
 *     string starts mid-buffer).
 *   - fixed()/hex8/16/32()/bin8() are FIELD writers: they emit
 *     exactly N characters and do NOT terminate -- they exist to
 *     drop digits into records and displays, where the field width
 *     is the contract.
 */

namespace HAL {
namespace Utils {
namespace Fmt {

namespace detail {

// the one digit-peel loop. noinline: `inline` is for linkage; a
// loop called from every numeric printer should exist once
__attribute__((noinline)) inline char* peelDecimal(char* end,
                                                   uint32_t n) {
    *end = '\0';
    char* p { end };
    do {
        *--p = static_cast<char>('0' + static_cast<uint8_t>(n % 10));
        n /= 10;
    } while (n != 0);
    return p;
}

}

// "4294967295" needs 10 digits + NUL: buf[11]
inline const char* u32(char (&buf)[11], uint32_t n) {
    return detail::peelDecimal(buf + 10, n);
}

// sign + 10 digits + NUL: buf[12]. THE INT32_MIN TRICK lives here,
// once: -n overflows int32_t at the minimum, but negating in
// unsigned space (0U - uint32_t(n)) yields 2147483648 exactly
inline const char* s32(char (&buf)[12], int32_t n) {
    uint32_t m { n < 0 ? 0U - static_cast<uint32_t>(n)
                       : static_cast<uint32_t>(n) };
    char* p { detail::peelDecimal(buf + 11, m) };
    if (n < 0) *--p = '-';
    return p;
}

// zero-padded fixed-width field, most significant digit first;
// values wider than `width` are truncated to their LOW digits
// (i.e. value mod 10^width) -- the right behavior for clock digits
// and record fields. Writes exactly `width` chars, no terminator
__attribute__((noinline)) inline void fixed(char* dst, uint32_t v,
                                            uint8_t width) {
    for (uint8_t i = width; i > 0; --i) {
        dst[i - 1] = static_cast<char>('0'
                         + static_cast<uint8_t>(v % 10));
        v /= 10;
    }
}

constexpr char hexDigit(uint8_t nibble) {
    return static_cast<char>(nibble < 10 ? '0' + nibble
                                         : 'A' + (nibble - 10));
}

// fixed-width uppercase hex fields, no prefix, no terminator
inline void hex8(char* dst, uint8_t v) {
    dst[0] = hexDigit(static_cast<uint8_t>(v >> 4));
    dst[1] = hexDigit(static_cast<uint8_t>(v & 0x0F));
}

inline void hex16(char* dst, uint16_t v) {
    hex8(dst,     static_cast<uint8_t>(v >> 8));
    hex8(dst + 2, static_cast<uint8_t>(v));
}

inline void hex32(char* dst, uint32_t v) {
    hex16(dst,     static_cast<uint16_t>(v >> 16));
    hex16(dst + 4, static_cast<uint16_t>(v));
}

// eight '0'/'1' characters, MSB first, no terminator -- the register
// dump's native tongue
__attribute__((noinline)) inline void bin8(char* dst, uint8_t v) {
    for (uint8_t i = 0; i < 8; ++i) {
        dst[i] = (v & (0x80 >> i)) ? '1' : '0';
    }
}

}
}
}
