#pragma once

#if !defined(__AVR_ATtiny85__) && !defined(__AVR_ATmega328P__)
#error "Unsupported MCU: This code only supports ATtiny85 and ATmega328P."
#endif

constexpr bool HIGH {  true };
constexpr bool LOW  { false };

#define READ_VOLATILE_U32(var, dest)   \
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { \
        (dest) = (var);                 \
    }

#define WRITE_VOLATILE_U32(var, value)  \
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { \
        (var) = (value);                \
    }
