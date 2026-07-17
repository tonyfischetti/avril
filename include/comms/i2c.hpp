#pragma once

#include "common.hpp"

#include <stdint.h>
#include <avr/io.h>
#include <util/twi.h>

#include "gpio.hpp"

/**
 * Blocking I2C (TWI) master.
 *
 * The API is transaction-shaped: write() a buffer to a 7-bit address,
 * read() one back, or writeRead() for the ubiquitous sensor idiom
 * (write a register pointer, repeated start, read the data). Every
 * function returns a Result -- I2C, unlike SPI, can fail (a missing
 * device NACKs its address, a wedged one stretches the clock forever),
 * so every wait in here is bounded and a dead bus comes back as
 * Result::TIMEOUT instead of a hung main loop.
 *
 * Addresses are SEVEN-bit (the datasheet-style address, 0x00..0x7F);
 * the R/W bit is this module's business. If a part's datasheet says
 * "write address 0x78 / read address 0x79", the 7-bit address is 0x3C.
 *
 * Pull-ups are mandatory physics: I2C lines are open-drain, only ever
 * driven low, and something must pull them up. Fit external resistors
 * (4.7k to VCC is the classic value at 100 kHz).
 *
 * Backends, per the allow-list:
 *
 *   - ATmega328P: the hardware TWI peripheral. Fixed pins SDA=PC4(27),
 *     SCL=PC5(28). The internal pull-ups are enabled by default
 *     (template param) -- at ~35k they can carry a short bench wire at
 *     100 kHz, but they are marginal; fit real resistors for anything
 *     that leaves the bench. Clock stretching is handled in hardware.
 *
 *   - ATtiny84/85: a bit-banged open-drain master on ANY two pins
 *     (template params). "High" is a released (input) pin that the
 *     external pull-up raises -- the internal pull-ups cannot be used
 *     here, because open-drain emulation keeps PORT at 0 so that DDR
 *     alone toggles between drive-low and release. External resistors
 *     are therefore REQUIRED, not just recommended. Clock stretching
 *     is honored: after releasing SCL the code waits (bounded) for the
 *     line to actually rise.
 *
 * maxHz is a ceiling, not a promise, as in the SPI module: the 328P
 * picks the largest TWBR-derived rate at or below it; the bit-banged
 * backend times half-periods to the ceiling and the loop overhead only
 * slows it further.
 *
 * Master only, and deliberately blocking -- one transaction at a time,
 * no ISR, errors where you can see them. (An I2C *slave* is a planned,
 * very different module: it is ISR-driven by nature.)
 */

namespace HAL {
namespace Comms {
namespace I2C {

enum class Result : uint8_t {
    OK,
    NACK_ADDR,   // nobody home at that address
    NACK_DATA,   // device rejected a data byte
    BUS_ERROR,   // lost arbitration or an illegal bus state
    TIMEOUT      // a wait never completed (wedged device, missing
                 // pull-ups, shorted line)
};

namespace detail {

// bounds every wait loop. Iterations, not time: at a few cycles per
// spin this is tens of milliseconds -- generous enough for a slave
// legitimately stretching the clock through a conversion, short
// enough that a wedged bus doesn't feel like a crash
constexpr uint16_t TIMEOUT_SPINS { 50000 };

}

#if defined(__AVR_ATmega328P__)

namespace detail {

constexpr uint32_t twbrFor(uint32_t maxHz) {
    // SCL = F_CPU / (16 + 2*TWBR) with the prescaler at 1; round up so
    // the achieved rate stays at or below the ceiling
    return (F_CPU / maxHz > 16) ? ((F_CPU / maxHz - 16) + 1) / 2 : 0;
}

// noinline: `inline` is for linkage; a busy-wait loop called from a
// dozen places should exist once (and -Winline agrees)
__attribute__((noinline)) inline bool waitTWINT() {
    uint16_t spins { TIMEOUT_SPINS };
    while (!(TWCR & (1 << TWINT))) {
        if (--spins == 0) return false;
    }
    return true;
}

}

template<uint32_t maxHz = 100000, bool useInternalPullupsP = true>
struct Master {

    static void begin() {
        constexpr uint32_t twbr = detail::twbrFor(maxHz);
        static_assert(twbr >= 10,
                "maxHz too fast: the datasheet wants TWBR >= 10 in "
                "master mode (~444 kHz ceiling at 16 MHz)");
        static_assert(twbr <= 255,
                "maxHz too slow: TWBR overflows (~30 kHz floor at "
                "16 MHz)");
        if constexpr (useInternalPullupsP) {
            // ~35k: enough for a short bench wire at 100 kHz, marginal
            // beyond that -- fit real resistors (see header comment)
            HAL::GPIO::GPIO<27>::setInputPullup();   // SDA = PC4
            HAL::GPIO::GPIO<28>::setInputPullup();   // SCL = PC5
        }
        TWSR = 0;   // prescaler = 1
        TWBR = static_cast<uint8_t>(twbr);
        TWCR = (1 << TWEN);
    }

    // START (or repeated START -- the hardware reports which and both
    // are accepted) followed by the address byte
    static Result startAddress(uint8_t addr7, bool readP) {
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
        if (!detail::waitTWINT()) return Result::TIMEOUT;
        uint8_t st { static_cast<uint8_t>(TW_STATUS) };
        if (st != TW_START && st != TW_REP_START) {
            stop();
            return Result::BUS_ERROR;
        }
        TWDR = static_cast<uint8_t>((addr7 << 1) | (readP ? 1 : 0));
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!detail::waitTWINT()) return Result::TIMEOUT;
        st = static_cast<uint8_t>(TW_STATUS);
        if (st == TW_MT_SLA_ACK || st == TW_MR_SLA_ACK) return Result::OK;
        stop();   // free the bus whatever went wrong
        if (st == TW_MT_SLA_NACK || st == TW_MR_SLA_NACK) {
            return Result::NACK_ADDR;
        }
        return Result::BUS_ERROR;   // includes lost arbitration
    }

    static void stop() {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        // TWSTO self-clears when the STOP has been transmitted; there
        // is no TWINT to wait on, so bound this one by iteration too
        uint16_t spins { detail::TIMEOUT_SPINS };
        while ((TWCR & (1 << TWSTO)) && --spins) {}
    }

    static Result write(uint8_t addr7, const uint8_t* data, uint8_t len,
                        bool stopP = true) {
        Result r { startAddress(addr7, false) };
        if (r != Result::OK) return r;
        for (uint8_t i = 0; i < len; ++i) {
            TWDR = data[i];
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!detail::waitTWINT()) { stop(); return Result::TIMEOUT; }
            if (TW_STATUS != TW_MT_DATA_ACK) {
                stop();
                return Result::NACK_DATA;
            }
        }
        if (stopP) stop();   // stopP=false: hold the bus for a
                             // repeated start (see writeRead)
        return Result::OK;
    }

    static Result read(uint8_t addr7, uint8_t* data, uint8_t len,
                       bool stopP = true) {
        if (len == 0) return Result::OK;
        Result r { startAddress(addr7, true) };
        if (r != Result::OK) return r;
        for (uint8_t i = 0; i < len; ++i) {
            // ACK every byte except the last: the NACK is how a master
            // tells the slave to stop sending
            bool lastP { i == static_cast<uint8_t>(len - 1) };
            TWCR = static_cast<uint8_t>((1 << TWINT) | (1 << TWEN)
                                        | (lastP ? 0 : (1 << TWEA)));
            if (!detail::waitTWINT()) { stop(); return Result::TIMEOUT; }
            uint8_t expected {
                lastP ? static_cast<uint8_t>(TW_MR_DATA_NACK)
                      : static_cast<uint8_t>(TW_MR_DATA_ACK) };
            if (TW_STATUS != expected) { stop(); return Result::BUS_ERROR; }
            data[i] = TWDR;
        }
        if (stopP) stop();
        return Result::OK;
    }

    // the sensor idiom: write a register pointer, repeated START (no
    // STOP in between -- a STOP would let another master barge in and,
    // on many parts, reset the register pointer), then read
    static Result writeRead(uint8_t addr7,
                            const uint8_t* wr, uint8_t wlen,
                            uint8_t* rd, uint8_t rlen) {
        Result r { write(addr7, wr, wlen, false) };
        if (r != Result::OK) return r;
        return read(addr7, rd, rlen, true);
    }

    // address probe: OK means somebody ACKed. Loop it over 0x08..0x77
    // and you have a bus scanner
    static Result ping(uint8_t addr7) {
        Result r { startAddress(addr7, false) };
        if (r == Result::OK) stop();
        return r;
    }

};

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__)

template<uint8_t sdaPin, uint8_t sclPin, uint32_t maxHz = 100000>
struct Master {

    using SDA = HAL::GPIO::GPIO<sdaPin>;
    using SCL = HAL::GPIO::GPIO<sclPin>;

    // open-drain emulation: PORT stays 0 forever, so DDR alone toggles
    // between driving low (output) and released/high (input, pulled up
    // by the EXTERNAL resistor). This is why the internal pull-ups are
    // unusable here: they'd need PORT=1, which would actively drive
    // the line high the moment DDR flips to output
    static void begin() {
        SDA::setInput();
        SDA::setLow();     // PORT=0 while an input: pullup off, and a
        SCL::setInput();   // future setOutput() drives low immediately
        SCL::setLow();
    }

  private:
    static void halfBit() {
        // to the ceiling; call overhead only slows the bus further,
        // which is the honest direction
        __builtin_avr_delay_cycles(F_CPU / (2UL * maxHz));
    }

    // release SCL, then wait for it to actually rise: a slave holding
    // it low is clock stretching (legal), a slave holding it low
    // forever is wedged (the bounded wait turns that into TIMEOUT).
    // noinline on the bit primitives: each is called ~20 times per
    // transferred byte across the byte routines; one copy each keeps
    // the whole backend a few hundred bytes instead of a few thousand
    __attribute__((noinline)) static bool sclReleaseWait() {
        SCL::setInput();
        uint16_t spins { detail::TIMEOUT_SPINS };
        while (!SCL::read()) {
            if (--spins == 0) return false;
        }
        return true;
    }

    __attribute__((noinline)) static bool writeBit(bool bit) {
        if (bit) SDA::setInput(); else SDA::setOutput();
        halfBit();
        if (!sclReleaseWait()) return false;
        halfBit();
        SCL::setOutput();
        return true;
    }

    __attribute__((noinline)) static bool readBit(bool& bit) {
        SDA::setInput();   // release: the slave owns SDA now
        halfBit();
        if (!sclReleaseWait()) return false;
        bit = SDA::read();
        halfBit();
        SCL::setOutput();
        return true;
    }

    // SDA falls while SCL is high. Written to be valid both from a
    // quiet bus and as a repeated START mid-transaction (release SDA,
    // release SCL, then the defining falling edge)
    __attribute__((noinline)) static bool startCondition() {
        SDA::setInput();
        halfBit();
        if (!sclReleaseWait()) return false;
        halfBit();
        SDA::setOutput();
        halfBit();
        SCL::setOutput();
        return true;
    }

    // SDA rises while SCL is high; leaves the bus idle
    __attribute__((noinline)) static bool stopCondition() {
        SDA::setOutput();
        halfBit();
        if (!sclReleaseWait()) return false;
        halfBit();
        SDA::setInput();
        halfBit();
        return true;
    }

    // 8 data bits MSB-first, then the 9th clock where the slave ACKs
    // (pulls SDA low) or NACKs (leaves it high)
    __attribute__((noinline)) static Result writeByteAck(uint8_t b) {
        for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
            if (!writeBit((b & mask) != 0)) return Result::TIMEOUT;
        }
        bool nackP { false };
        if (!readBit(nackP)) return Result::TIMEOUT;
        return nackP ? Result::NACK_DATA : Result::OK;
    }

    __attribute__((noinline)) static bool readByteWithAck(uint8_t& out,
                                                          bool ackP) {
        out = 0;
        for (uint8_t i = 0; i < 8; ++i) {
            bool bit { false };
            if (!readBit(bit)) return false;
            out = static_cast<uint8_t>((out << 1) | (bit ? 1 : 0));
        }
        // master ACK = pull SDA low on the 9th clock; NACK = leave high
        return writeBit(!ackP);
    }

  public:
    static Result write(uint8_t addr7, const uint8_t* data, uint8_t len,
                        bool stopP = true) {
        if (!startCondition()) return Result::TIMEOUT;
        Result r { writeByteAck(static_cast<uint8_t>(addr7 << 1)) };
        if (r != Result::OK) {
            stopCondition();
            return (r == Result::NACK_DATA) ? Result::NACK_ADDR : r;
        }
        for (uint8_t i = 0; i < len; ++i) {
            r = writeByteAck(data[i]);
            if (r != Result::OK) { stopCondition(); return r; }
        }
        if (stopP && !stopCondition()) return Result::TIMEOUT;
        return Result::OK;
    }

    static Result read(uint8_t addr7, uint8_t* data, uint8_t len,
                       bool stopP = true) {
        if (len == 0) return Result::OK;
        if (!startCondition()) return Result::TIMEOUT;
        Result r { writeByteAck(static_cast<uint8_t>((addr7 << 1) | 1)) };
        if (r != Result::OK) {
            stopCondition();
            return (r == Result::NACK_DATA) ? Result::NACK_ADDR : r;
        }
        for (uint8_t i = 0; i < len; ++i) {
            // ACK every byte except the last (the NACK ends the read)
            bool lastP { i == static_cast<uint8_t>(len - 1) };
            if (!readByteWithAck(data[i], !lastP)) {
                stopCondition();
                return Result::TIMEOUT;
            }
        }
        if (stopP && !stopCondition()) return Result::TIMEOUT;
        return Result::OK;
    }

    // see the 328P twin for the reasoning; identical semantics
    static Result writeRead(uint8_t addr7,
                            const uint8_t* wr, uint8_t wlen,
                            uint8_t* rd, uint8_t rlen) {
        Result r { write(addr7, wr, wlen, false) };
        if (r != Result::OK) return r;
        return read(addr7, rd, rlen, true);
    }

    static Result ping(uint8_t addr7) {
        if (!startCondition()) return Result::TIMEOUT;
        Result r { writeByteAck(static_cast<uint8_t>(addr7 << 1)) };
        stopCondition();
        return (r == Result::NACK_DATA) ? Result::NACK_ADDR : r;
    }

};

#endif

}
}
}
