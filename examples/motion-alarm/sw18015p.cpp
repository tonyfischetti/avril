/**
 * motion-alarm, SW-18015P variant: a bike/tamper alarm that runs
 * until the battery dies of old age.
 *
 * Same circuit as the PIR variant (see pir.cpp for the full circuit
 * rationale: LDO, MOSFET, pulldown, flyback, divider) with the sensor
 * swapped: an SW-18015P spring switch (the KY-002 module's innards)
 * instead of a PIR. That one swap changes the appliance from a room
 * alarm to a TAMPER alarm -- it senses the device being moved, not
 * people moving near it -- and rewrites the power budget:
 *
 *   The spring switch draws ZERO current at rest. It is literally an
 *   open switch; the internal pullup only conducts during contact
 *   chatter. The armed floor drops from the PIR's ~58 uA to ~8 uA
 *   (AVR ~0.3 + LDO ~3 + divider ~4.5), which on a 9 V block's
 *   ~500 mAh is ~7 years of arithmetic -- past alkaline shelf life.
 *   The battery's own self-discharge, not the alarm, decides when
 *   it ends.
 *
 * Behavior (warn-then-escalate, the polite bike-alarm pattern):
 *   - boot: ~32 s exit window (lock the bike, walk away), one chirp
 *     when armed;
 *   - a jostle: short warning chirp, then LISTEN for ~10 s -- via
 *     sleepFor's early-exit semantics: the watchdog times the window
 *     asleep, and any new spring edge cuts the sleep short;
 *   - more disturbance inside the window: full siren (20 beeps).
 *     Quiet window: shrug, re-arm;
 *   - battery check with the distinctive triple-chirp at every
 *     (re)arm, same as the PIR variant.
 *
 * The spring's filthy contact-bounce bursts need no debouncing here:
 * ANY edge wakes the chip and sets one flag; a burst is one event.
 * The settle-naps after chirps/sirens let the spring stop chattering
 * before the flag is cleared.
 *
 * BIKE REALITY, required part: a physical power switch on the 9 V
 * lead. Riding IS sustained vibration -- arm it only when parked.
 *
 * Parts: as pir.cpp, minus the HC-SR501, plus an SW-18015P (or the
 * KY-002 module -- its onboard pullup is fine and redundant) and a
 * toggle switch for the battery lead.
 *
 * Wiring (physical DIP-8 pins):
 *   SW-18015P    -> PB2 (7) to GND   (internal pullup carries it;
 *                                     orientation affects sensitivity
 *                                     -- spring axis horizontal is
 *                                     most sensitive)
 *   MOSFET gate  -> PB1 (6), with the 100k pulldown
 *   divider tap  -> PB4 (3), with the 100n to GND
 *
 * Build: make VARIANT=sw18015p   (pir is the default variant)
 *
 * Exercises: GPIO (PCINT wake), Sleep (PWR_DOWN), Watchdog::sleepFor
 * (including its early-exit-on-foreign-wake semantics as the
 * escalation window), Analog (raw-battery divider).
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"

using Sensor = HAL::GPIO::GPIO<7>;   // PB2
using Gate   = HAL::GPIO::GPIO<6>;   // PB1
constexpr uint8_t BATT_PIN { 3 };    // PB4 = ADC2

// see pir.cpp for why this threshold isn't arbitrary (alkaline knee
// from above, reference-sag validity floor from below)
constexpr uint16_t LOW_BATT_MV { 6400 };

constexpr uint8_t SIREN_BEEPS { 20 };

volatile bool jostledP { false };

ISR(PCINT0_vect) {
    // every bounce of the spring lands here; one flag makes a burst
    // one event. The wake itself is the payload
    jostledP = true;
}

EMPTY_INTERRUPT(WDT_vect)

uint16_t batteryMillivolts() {
    HAL::Analog::setReference<HAL::Analog::Ref::VCC>();
    uint16_t raw { HAL::Analog::read<BATT_PIN>() };
    return static_cast<uint16_t>((static_cast<uint32_t>(raw)
                                  * 2UL * 5000UL) / 1024UL);
}

void chirp() {
    Gate::setHigh();
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS64>(1);
    Gate::setLow();
}

void lowBatteryChirp() {
    for (uint8_t i = 0; i < 3; ++i) {
        chirp();
        HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS128>(1);
    }
}

// settle (spring stops chattering), battery check, clear the flag so
// only NEW disturbance wakes us
void arm() {
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS512>(2);
    if (batteryMillivolts() < LOW_BATT_MV) lowBatteryChirp();
    jostledP = false;
}

int main() {
    HAL::Analog::begin();

    Gate::setLow();            // PORT first: never glitch the gate
    Gate::setOutput();
    Sensor::setInputPullup();  // open switch: zero standing current
    Sensor::enablePCINT();
    sei();

    // exit window: lock up and walk away (~32 s)
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS8192>(4);

    chirp();                   // "armed"
    arm();

    while (1) {
        // armed: PWR_DOWN until the spring says otherwise. ~8 uA
        HAL::Sleep::goToSleep(SLEEP_MODE_PWR_DOWN);

        if (!jostledP) continue;   // spurious wake
        jostledP = false;

        // warn first: a bump is not a theft
        chirp();

        // the escalation window, timed ASLEEP: sleepFor completes all
        // ten periods only if the spring stays quiet -- any new edge
        // is a foreign wake that cuts the sleep short
        uint16_t quiet {
            HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS1024>(10)
        };
        if (quiet < 10) {
            // still being handled: full siren
            for (uint8_t i = 0; i < SIREN_BEEPS; ++i) {
                Gate::setHigh();
                HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS512>(1);
                Gate::setLow();
                HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS512>(1);
            }
        }
        arm();
    }
}
