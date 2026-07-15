#pragma once

#include "common.hpp"
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>


namespace HAL {
namespace GPIO {

enum class Port : uint8_t { A, B, C, D, Invalid };

struct PinInfo {
    Port port;
    uint8_t bit;
    uint8_t pcintNumber;
    uint8_t pcicrBit;
};

#if defined(__AVR_ATtiny85__)
constexpr PinInfo pinTable[8] = {
    { Port::B, 5, 5, PCIE },          //  1 RESET (PB5)
    { Port::B, 3, 3, PCIE },          //  2 PB3
    { Port::B, 4, 4, PCIE },          //  3 PB4
    { Port::Invalid, 0, 0xFF, 0xFF }, //  4 GND
    { Port::B, 0, 0, PCIE },          //  5 PB0
    { Port::B, 1, 1, PCIE },          //  6 PB1
    { Port::B, 2, 2, PCIE },          //  6 PB2
    { Port::Invalid, 0, 0xFF, 0xFF }, //  8 VCC
};
#elif defined(__AVR_ATtiny84__)
// PA0-7 are PCINT0-7 (PCMSK0, PCIE0, PCINT0_vect);
// PB0-3 are PCINT8-11 (PCMSK1, PCIE1, PCINT1_vect).
// PCMSK1 bit positions coincide with the PB bit numbers, so `mask`
// serves both the port registers and the PCINT mask register
constexpr PinInfo pinTable[14] = {
    { Port::Invalid, 0, 0xFF, 0xFF }, //  1 VCC
    { Port::B, 0,  8, PCIE1 },        //  2 PB0 (XTAL1)
    { Port::B, 1,  9, PCIE1 },        //  3 PB1 (XTAL2)
    { Port::B, 3, 11, PCIE1 },        //  4 PB3 (RESET)
    { Port::B, 2, 10, PCIE1 },        //  5 PB2
    { Port::A, 7,  7, PCIE0 },        //  6 PA7
    { Port::A, 6,  6, PCIE0 },        //  7 PA6
    { Port::A, 5,  5, PCIE0 },        //  8 PA5
    { Port::A, 4,  4, PCIE0 },        //  9 PA4
    { Port::A, 3,  3, PCIE0 },        // 10 PA3
    { Port::A, 2,  2, PCIE0 },        // 11 PA2
    { Port::A, 1,  1, PCIE0 },        // 12 PA1
    { Port::A, 0,  0, PCIE0 },        // 13 PA0
    { Port::Invalid, 0, 0xFF, 0xFF }, // 14 GND
};
#elif defined(__AVR_ATmega328P__)
constexpr PinInfo pinTable[28] = {
    { Port::Invalid, 0, 0xFF, 0xFF }, //  1 RESET
    { Port::D, 0, 16, PCIE2 },        //  2 PD0
    { Port::D, 1, 17, PCIE2 },        //  3 PD1
    { Port::D, 2, 18, PCIE2 },        //  4 PD2
    { Port::D, 3, 19, PCIE2 },        //  5 PD3
    { Port::D, 4, 20, PCIE2 },        //  6 PD4
    { Port::Invalid, 0, 0xFF, 0xFF }, //  7 VCC
    { Port::Invalid, 0, 0xFF, 0xFF }, //  8 GND
    { Port::Invalid, 0, 0xFF, 0xFF }, //  9 XTAL1
    { Port::Invalid, 0, 0xFF, 0xFF }, // 10 XTAL2
    { Port::D, 5, 21, PCIE2 },        // 11 PD5
    { Port::D, 6, 22, PCIE2 },        // 12 PD6
    { Port::D, 7, 23, PCIE2 },        // 13 PD7
    { Port::B, 0, 0,  PCIE0 },        // 14 PB0
    { Port::B, 1, 1,  PCIE0 },        // 15 PB1
    { Port::B, 2, 2,  PCIE0 },        // 16 PB2
    { Port::B, 3, 3,  PCIE0 },        // 17 PB3
    { Port::B, 4, 4,  PCIE0 },        // 18 PB4
    { Port::B, 5, 5,  PCIE0 },        // 19 PB5
    { Port::Invalid, 0, 0xFF, 0xFF }, // 20 AVCC
    { Port::Invalid, 0, 0xFF, 0xFF }, // 21 AREF
    { Port::Invalid, 0, 0xFF, 0xFF }, // 22 GND
    { Port::C, 0, 8,  PCIE1 },        // 23 PC0
    { Port::C, 1, 9,  PCIE1 },        // 24 PC1
    { Port::C, 2, 10, PCIE1 },        // 25 PC2
    { Port::C, 3, 11, PCIE1 },        // 26 PC3
    { Port::C, 4, 12, PCIE1 },        // 27 PC4
    { Port::C, 5, 13, PCIE1 },        // 28 PC5
};
#endif

template<uint8_t physicalPin>
struct GPIO {

#if defined(__AVR_ATtiny85__)
    static_assert(physicalPin >= 1 && physicalPin <= 8,
            "Invalid pin number for ATTiny85");
#elif defined(__AVR_ATtiny84__)
    static_assert(physicalPin >= 1 && physicalPin <= 14,
            "Invalid pin number for ATTiny84");
#elif defined(__AVR_ATmega328P__)
    static_assert(physicalPin >= 1 && physicalPin <= 28,
            "Invalid pin number for ATMega328P");
#endif

    static constexpr PinInfo info = pinTable[physicalPin-1];

    static_assert(info.port != Port::Invalid,
            "Physical pin is not a usable GPIO (power/crystal pin)");

    static constexpr uint8_t mask { static_cast<uint8_t>(1 << info.bit) };

  private:
    // SFR addresses aren't constant expressions, so they can't live in a
    // constexpr table; instead the register *lvalue* is selected by
    // if-constexpr on the (constexpr) port. Everything folds at compile
    // time: no RAM, no indirection, and single-instruction sbi/cbi where
    // the register is in low I/O space
    static volatile uint8_t& ddrReg() {
#if defined(__AVR_ATtiny85__)
        return DDRB;
#elif defined(__AVR_ATtiny84__)
        if constexpr (info.port == Port::A) return DDRA;
        else                                return DDRB;
#elif defined(__AVR_ATmega328P__)
        if constexpr      (info.port == Port::B) return DDRB;
        else if constexpr (info.port == Port::C) return DDRC;
        else                                     return DDRD;
#endif
    }

    static volatile uint8_t& portReg() {
#if defined(__AVR_ATtiny85__)
        return PORTB;
#elif defined(__AVR_ATtiny84__)
        if constexpr (info.port == Port::A) return PORTA;
        else                                return PORTB;
#elif defined(__AVR_ATmega328P__)
        if constexpr      (info.port == Port::B) return PORTB;
        else if constexpr (info.port == Port::C) return PORTC;
        else                                     return PORTD;
#endif
    }

    static volatile uint8_t& pinReg() {
#if defined(__AVR_ATtiny85__)
        return PINB;
#elif defined(__AVR_ATtiny84__)
        if constexpr (info.port == Port::A) return PINA;
        else                                return PINB;
#elif defined(__AVR_ATmega328P__)
        if constexpr      (info.port == Port::B) return PINB;
        else if constexpr (info.port == Port::C) return PINC;
        else                                     return PIND;
#endif
    }

    static volatile uint8_t& pcmskReg() {
#if defined(__AVR_ATtiny85__)
        return PCMSK;
#elif defined(__AVR_ATtiny84__)
        if constexpr (info.port == Port::A) return PCMSK0;
        else                                return PCMSK1;
#elif defined(__AVR_ATmega328P__)
        if constexpr      (info.port == Port::B) return PCMSK0;
        else if constexpr (info.port == Port::C) return PCMSK1;
        else                                     return PCMSK2;
#endif
    }

  public:
    static inline void setOutput() { ddrReg()  |=  mask; }
    static inline void setInput()  { ddrReg()  &= static_cast<uint8_t>(~mask); }
    static inline void setHigh()   { portReg() |=  mask; }
    static inline void setLow()    { portReg() &= static_cast<uint8_t>(~mask); }
    static inline void toggle()    { portReg() ^=  mask; }
    static inline bool read()      { return pinReg() & mask; }

    static inline void setInputPullup() { setInput(); setHigh(); }

    static inline void enablePCINT() {
        // the tinys put the enable bits in GIMSK, the 328P in PCICR;
        // pcicrBit is per-port everywhere except the tiny85 (one port)
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny84__)
        GIMSK |= (1 << info.pcicrBit);
#elif defined(__AVR_ATmega328P__)
        PCICR |= (1 << info.pcicrBit);
#endif
        pcmskReg() |= mask;
    }
};


}
}
