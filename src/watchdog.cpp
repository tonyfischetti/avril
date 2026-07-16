
#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "watchdog.hpp"

namespace HAL {
namespace Watchdog {

// written from .init3 below, before static initialization runs -- so it
// must live in .noinit, or the C runtime's bss-clearing (.init4) would
// erase it moments later
uint8_t mcusrAtBoot __attribute__((section(".noinit")));

ResetCause resetCause() {
    if (mcusrAtBoot & (1 << WDRF))  return ResetCause::WATCHDOG;
    if (mcusrAtBoot & (1 << EXTRF)) return ResetCause::EXTERNAL;
    // PORF before BORF: a normal power-up can set both, and reporting
    // it as a brownout would cry wolf
    if (mcusrAtBoot & (1 << PORF))  return ResetCause::POWER_ON;
    if (mcusrAtBoot & (1 << BORF))  return ResetCause::BROWNOUT;
    return ResetCause::UNKNOWN;
}

}
}

// The early-boot trap this defuses: after a watchdog reset, WDRF is set
// in MCUSR, and while WDRF is set the hardware forces WDE on -- the dog
// is already running again, at its shortest (16 ms) timeout, before
// main() begins. Any startup slower than that resets again, forever.
// So, as early as possible: snapshot the cause, clear MCUSR (releasing
// WDE), and disable the watchdog.
//
// This is avr-libc's canonical pattern (see <avr/wdt.h>). It runs from
// .init3: after .init2 has set up the stack and zero register, before
// static initialization and main(). Init sections are fall-through
// code, not called functions, hence `naked` (no prologue, no ret);
// `used` keeps LTO from discarding a function nothing references.
__attribute__((naked, used, section(".init3")))
static void earlyWatchdogInit() {
    HAL::Watchdog::mcusrAtBoot = MCUSR;
    MCUSR = 0;
    wdt_disable();
}
