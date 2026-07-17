#pragma once

#include "common.hpp"

#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "sleep.hpp"
#include "ticker.hpp"

/**
 * WATCHDOG
 *
 * The watchdog timer runs off its own on-chip 128 kHz RC oscillator,
 * which makes it two different tools:
 *
 *   1. A dead-man's switch (reset mode, WDE): if runaway code stops
 *      kicking it, the chip reboots. Not implemented here yet.
 *
 *   2. The only timer that keeps running in SLEEP_MODE_PWR_DOWN
 *      (interrupt mode, WDIE): a 16 ms - 8.2 s wake-up source at a few
 *      microamps. That is what this module implements -- see
 *      sleepOnePeriod() / sleepFor().
 *
 * Because the oscillator is an RC circuit, timeouts are nominal only:
 * expect +/-10%, drifting with voltage and temperature. Fine for
 * "auto-off after ten minutes"; useless for timekeeping.
 *
 * Register notes (WDTCR on the ATtiny85; WDTCSR, same layout, on the
 * ATtiny84 and ATmega328P):
 *
 *   - (1 << 7) - WDIF - Watchdog Timeout Interrupt Flag
 *   - (1 << 6) - WDIE - Watchdog Timeout Interrupt Enable
 *     Hardware CLEARS this bit when the WDT interrupt vector executes.
 *     In combined mode (WDIE|WDE) that is the bark-then-bite feature:
 *     the next timeout resets. Here (WDIE only) it doubles as a wake
 *     detector: WDIE still set after a sleep means something *else*
 *     woke us.
 *   - (1 << 4) - WDCE - Watchdog Change Enable
 *   - (1 << 3) - WDE  - Watchdog Enable (reset mode)
 *     Disabling WDE or changing the prescaler requires the timed
 *     sequence: set WDCE|WDE, then write the real value within 4 cycles.
 *   - WDP3..WDP0 (bits 5, 2, 1, 0): timeout = 2048 * 2^n cycles of
 *     128 kHz, i.e. 16 ms << n, for n in 0..9.
 *
 * MCUSR - MCU status register: WDRF / BORF / EXTRF / PORF record why
 * the chip last reset. THE TRAP: while WDRF is set, hardware forces
 * WDE on -- so after a watchdog reset the dog is already running, with
 * a 16 ms timeout, before main() begins. src/watchdog.cpp defuses this
 * from .init3 (snapshot MCUSR, clear it, disable the WDT) and the
 * snapshot is what resetCause() reports. If a build uses anything in
 * this namespace, link src/watchdog.cpp.
 */

namespace HAL {
namespace Watchdog {

// nominal milliseconds; the true period is 2048 * 2^n cycles of a
// +/-10% 128 kHz oscillator
enum class Timeout : uint8_t {
    MS16, MS32, MS64, MS128, MS256,
    MS512, MS1024, MS2048, MS4096, MS8192
};

enum class ResetCause : uint8_t {
    POWER_ON, EXTERNAL, BROWNOUT, WATCHDOG, UNKNOWN
};

// MCUSR as it was at reset, snapshotted from .init3 (src/watchdog.cpp)
extern uint8_t mcusrAtBoot;

// why did the chip last reset? Decoded from the .init3 snapshot, so
// it is valid no matter how late it is called
ResetCause resetCause();

constexpr uint8_t getWDPrescalerBits(Timeout timeout) {
    // WDP3 lives in bit 5, apart from WDP2..0 in bits 2..0
    uint8_t n { static_cast<uint8_t>(timeout) };
    return static_cast<uint8_t>(((n & 0x08) ? (1 << WDP3) : 0)
                                | (n & 0x07));
}

constexpr uint16_t getNominalMs(Timeout timeout) {
    return static_cast<uint16_t>(16U << static_cast<uint8_t>(timeout));
}

// arm interrupt mode (WDIE, no WDE): one timeout from now, WDT_vect
// fires. THE APP MUST DEFINE ISR(WDT_vect) -- EMPTY_INTERRUPT(WDT_vect)
// suffices for pure wake-ups; without one, avr-libc's __bad_interrupt
// jumps to the reset vector and the wake-up becomes a silent reboot.
// Remember hardware clears WDIE when the vector executes: re-arm for
// each period (sleepOnePeriod does)
template<Timeout timeout>
inline void enableInterrupt() {
    constexpr uint8_t prescalerBits { getWDPrescalerBits(timeout) };
    uint8_t sreg { SREG };
    cli();
    wdt_reset();  // start the period from now, not mid-count
    MCUSR &= static_cast<uint8_t>(~(1 << WDRF));
#if defined(__AVR_ATtiny85__)
    WDTCR |= (1 << WDCE) | (1 << WDE);  // timed sequence
    WDTCR = (1 << WDIE) | prescalerBits;
#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATmega328P__)
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | prescalerBits;
#endif
    SREG = sreg;
}

// noinline: `inline` is for linkage; a timed-sequence body called
// from every sleepOnePeriod instantiation should exist once (and
// -Winline agrees once the call sites accumulate)
__attribute__((noinline)) inline void disable() {
    uint8_t sreg { SREG };
    cli();
    MCUSR &= static_cast<uint8_t>(~(1 << WDRF));
#if defined(__AVR_ATtiny85__)
    WDTCR |= (1 << WDCE) | (1 << WDE);  // timed sequence
    WDTCR = 0x00;
#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATmega328P__)
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0x00;
#endif
    SREG = sreg;
}

// one watchdog period of SLEEP_MODE_PWR_DOWN. Returns true if the full
// period elapsed (the watchdog woke us); false if some other interrupt
// -- a pin change, say -- woke us first. The ticker is paused for the
// duration; on a watchdog wake it is credited the nominal period, and
// on a foreign wake the waking ISR's own Ticker::resume() call has
// already handled it (resume() on a running ticker is a no-op)
template<Timeout timeout>
bool sleepOnePeriod() {
    enableInterrupt<timeout>();
    HAL::Ticker::pause();
    HAL::Sleep::goToSleep(SLEEP_MODE_PWR_DOWN);
    // hardware cleared WDIE iff the WDT vector ran, so a still-set
    // WDIE means a foreign wake mid-period. (If the period elapses in
    // the microseconds between waking and this read, it is counted as
    // a watchdog wake -- and by then the period really has passed)
#if defined(__AVR_ATtiny85__)
    bool elapsedP { (WDTCR & (1 << WDIE)) == 0 };
#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATmega328P__)
    bool elapsedP { (WDTCSR & (1 << WDIE)) == 0 };
#endif
    disable();
    HAL::Ticker::resume(elapsedP ? getNominalMs(timeout) : 0);
    return elapsedP;
}

// chain periods for long stretches of PWR_DOWN (minutes in 8 s chunks).
// Stops early on a foreign wake and returns the number of *completed*
// periods, so the caller can tell "the timer ran out" (== nPeriods)
// from "someone touched something" (< nPeriods)
template<Timeout timeout>
uint16_t sleepFor(uint16_t nPeriods) {
    uint16_t completed { 0 };
    while (completed < nPeriods) {
        if (!sleepOnePeriod<timeout>()) break;
        ++completed;
    }
    return completed;
}

}
}
