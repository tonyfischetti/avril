/**
 * motion-alarm: a PIR burglar alarm that lives ~a year on a 9 V block.
 *
 * What it does:
 *   - boots, waits ~64 s (PIR warm-up + your exit window), chirps
 *     once to say "armed";
 *   - sleeps in PWR_DOWN at ~55 uA until the PIR's pin change wakes
 *     it -- there is no polling and no Ticker anywhere;
 *   - on motion: sounds the buzzer in an on/off pattern (timed by the
 *     watchdog, asleep between beeps), then waits for the PIR to drop
 *     and re-arms;
 *   - checks the RAW battery through a divider at every (re)arm and
 *     signals a distinctive triple-chirp when the 9 V is dying.
 *
 * ---- The circuit, and WHY each part is there ----------------------
 *
 *     9V + ----+-------------------+------------[buzzer +]
 *              |                   |                 |
 *              |             1M (divider top)   [active buzzer,
 *          [MCP1702-5.0          |               rated 9-12 V]
 *           or HT7350]            +--> PB4 (3)       |
 *              |                  |    ADC       [1N4148 flyback
 *              |             1M (bottom)          across buzzer]
 *          5V rail -> tiny85      |                  |
 *              |    + PIR Vcc   100n                D  drain
 *              |                  |            +--G  2N7000
 *             GND ---------------GND           |     S  source
 *                                              |     |
 *                    PB1 (6) ------+-----------+    GND
 *                                  |
 *                                100k (gate pulldown)
 *                                  |
 *                                 GND
 *
 *   - MICROPOWER 5 V LDO (MCP1702-5.0 / HT7350, ~2-4 uA quiescent).
 *     NOT a 7805/78L05: its ~3 mA quiescent alone would kill the
 *     battery in weeks. Not a cheap buck module either, same reason.
 *     The 5 V rail also powers the PIR via its normal Vcc input and
 *     gives the 2N7000 a gate voltage that turns it fully on.
 *   - 2N7000 low-side switch (TO-92, flat face toward you: S-G-D --
 *     yes, source on the left; everyone guesses wrong once). From a
 *     5 V gate it is comfortably enhanced; its 200 mA limit is ~6x an
 *     active buzzer's draw.
 *   - 100k GATE PULLDOWN, non-negotiable: during reset, programming,
 *     and the microseconds before main() runs, PB1 floats -- without
 *     the pulldown the alarm screams every time you flash it.
 *   - 1N4148 FLYBACK DIODE across the buzzer (cathode/band to 9V+):
 *     magnetic buzzers are inductive, and the turn-off spike would go
 *     hunting for the MOSFET without it. (A pure piezo doesn't need
 *     it; fit it anyway -- you don't always know what's in the can.)
 *   - 1M/1M DIVIDER from the RAW 9 V: readVccMillivolts() is blind
 *     here (the LDO rail is constant by design), so the battery is
 *     measured directly, halved into ADC range. The 100n cap matters:
 *     1M-class sources violate the ADC's <=10k source-impedance
 *     guideline, and the cap is the charge reservoir that makes slow
 *     readings honest. The divider leaks 9V/2M ~= 4.5 uA, part of the
 *     budget below.
 *   - ACTIVE buzzer (built-in oscillator): gate high = noise, no PWM
 *     needed. Rated 9-12 V (or 3-24 V) -- a 5 V-only part distorts
 *     and dies young at 9 V. Louder at 9 V than 5 V by ~4-6 dB, which
 *     is the point of switching the raw battery.
 *
 * Power budget: AVR PWR_DOWN ~0.5 uA + PIR ~50 uA + LDO ~3 uA +
 * divider ~4.5 uA ~= 58 uA -> a ~500 mAh 9 V block lasts ~10 months
 * of armed silence. (Quieter/longer alternative: 3xAA direct, no
 * regulator at all -- but the buzzer at 4.5 V is half as loud.)
 *
 * Parts: ATtiny85 @ internal 8 MHz, HC-SR501 PIR, active buzzer
 * (9-12 V), 2N7000, MCP1702-5.0 or HT7350, 1N4148, 2x 1M, 100k,
 * 100 nF, 9 V battery.
 *
 * Wiring (physical DIP-8 pins):
 *   PIR OUT      -> PB2 (7)   (SR501 output is 3.3 V logic: above
 *                              the tiny's ~3.0 V VIH at Vcc=5 V)
 *   MOSFET gate  -> PB1 (6), with the 100k pulldown
 *   divider tap  -> PB4 (3), with the 100n to GND
 *
 * Exercises: GPIO (PCINT wake), Sleep (PWR_DOWN), Watchdog::sleepFor
 * (every delay in the program), Analog (raw-battery divider). No
 * Ticker, no UART: the appliance is silence, interrupted.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"

using Pir    = HAL::GPIO::GPIO<7>;   // PB2
using Gate   = HAL::GPIO::GPIO<6>;   // PB1
constexpr uint8_t BATT_PIN { 3 };    // PB4 = ADC2

// 9 V block: ~9.5 V fresh, useless below ~6 V. The divider halves it,
// the 5 V LDO rail is the ADC reference, so full scale reads 10.24 V
constexpr uint16_t LOW_BATT_MV { 6400 };

// alarm shape: 10 beeps of ~500 ms on / ~500 ms off
constexpr uint8_t ALARM_BEEPS { 10 };

volatile bool motionEdgeP { false };

ISR(PCINT0_vect) {
    // both edges land here; main() checks the PIR's actual level.
    // Nothing else to do -- the wake itself is the payload
    motionEdgeP = true;
}

EMPTY_INTERRUPT(WDT_vect)

uint16_t batteryMillivolts() {
    HAL::Analog::setReference<HAL::Analog::Ref::VCC>();
    uint16_t raw { HAL::Analog::read<BATT_PIN>() };
    // raw/1024 * 5000 mV, times 2 for the divider
    return static_cast<uint16_t>((static_cast<uint32_t>(raw)
                                  * 2UL * 5000UL) / 1024UL);
}

void beep(uint16_t periods512)  {   // watchdog-timed, asleep meanwhile
    Gate::setHigh();
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS512>(periods512);
    Gate::setLow();
}

void gap(uint16_t periods512) {
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS512>(periods512);
}

// three short chirps: the "battery dying" signature, distinct from
// both the single armed-chirp and the alarm pattern
void lowBatteryChirp() {
    for (uint8_t i = 0; i < 3; ++i) {
        Gate::setHigh();
        HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS64>(1);
        Gate::setLow();
        HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS128>(1);
    }
}

// every (re)arm: battery check, then clear any pending motion edge so
// only NEW motion wakes us
void arm() {
    if (batteryMillivolts() < LOW_BATT_MV) lowBatteryChirp();
    motionEdgeP = false;
}

int main() {
    HAL::Analog::begin();

    Gate::setLow();          // PORT first: never glitch the gate high
    Gate::setOutput();
    Pir::setInput();         // the SR501 drives push-pull: no pullup
    Pir::enablePCINT();
    sei();

    // PIR warm-up (~60 s of nonsense output) doubling as the exit
    // window: eight 8 s watchdog naps. Motion edges during this are
    // discarded by arm()
    HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS8192>(8);

    beep(1);                 // one chirp: "armed"
    arm();

    while (1) {
        // armed: sleep until SOMETHING happens. PWR_DOWN directly --
        // not sleepFor -- because only the PIR should wake us, and it
        // can take all night
        HAL::Sleep::goToSleep(SLEEP_MODE_PWR_DOWN);

        if (!motionEdgeP || !Pir::read()) {
            // falling edge or a spurious wake: not an intruder
            motionEdgeP = false;
            continue;
        }

        // intruder: make noise (asleep between beeps, even now)
        for (uint8_t i = 0; i < ALARM_BEEPS; ++i) {
            beep(1);
            gap(1);
        }

        // wait for the PIR to release (its hold time is set on the
        // module) so one intrusion is one alarm, then re-arm
        while (Pir::read()) {
            HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS1024>(1);
        }
        arm();
    }
}
