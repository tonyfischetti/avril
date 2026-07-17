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
 * ---- Sensor alternatives -----------------------------------------
 *
 * The sensor choice decides what this alarm IS. Two families:
 *
 *   PRESENCE detectors sense people moving in a space (this build:
 *   a room alarm). DISTURBANCE detectors sense the device itself
 *   being moved, tilted, or struck (a tamper alarm: bike, toolbox,
 *   door, package). Same firmware skeleton either way -- a wake
 *   source on PB2 -- but a different appliance.
 *
 * Presence family:
 *
 *   - HC-SR501 PIR (fitted here, ~$1): ~50 uA, 3-7 m range with the
 *     dome lens, adjustable hold time and sensitivity, push-pull
 *     3.3 V output, ~60 s power-on stabilization (why the firmware
 *     arms late). The default for a reason; also the budget's
 *     dominant term.
 *   - Panasonic EKMB "PaPIRs" (~$10): the luxury PIR. 1-6 uA
 *     depending on variant -- fitting one drops this alarm's floor
 *     from ~58 uA to ~12 uA and the 9 V lasts YEARS. Smaller,
 *     factory-characterized lens, near-instant startup, but no
 *     adjustments and mind the variant: some are open-drain outputs
 *     and need a (high-value) pull-up. The upgrade to buy when the
 *     battery life is the product.
 *   - RCWL-0516 microwave doppler (~$1): sees through plastic and
 *     thin walls, so it hides INSIDE an opaque enclosure -- no lens
 *     window to give the device away. The catches: ~3 mA
 *     continuously (battery-hostile: ~1 week on this 9 V), needs
 *     >4 V supply, and it also sees through the walls you didn't
 *     mean it to (false triggers from the next room). Wall-powered
 *     stealth builds only.
 *
 * Disturbance family:
 *
 *   - SW-18015P spring switch (the KY-002 module, ~$0.20) -- NOW
 *     IMPLEMENTED as this example's second variant: sw18015p.cpp,
 *     built with `make VARIANT=sw18015p`. A spring
 *     in a tube that chatters against a contact when jolted. ZERO
 *     quiescent current -- it is literally a switch, and with the
 *     PCINT pull-up it draws nothing while still. The output is a
 *     filthy burst of bounces, which for THIS architecture is
 *     perfect: any edge wakes the chip, and "debouncing" is just
 *     "treat a burst as one event, then a lockout nap". Firmware
 *     delta: enable the internal pullup on PB2 and alarm on any
 *     edge burst. Orientation-sensitive, misses slow tilt, no
 *     sensitivity adjustment -- but a tamper alarm with a ~7 uA
 *     floor (the divider and LDO become the budget).
 *   - 801S vibration module (~$1): the same spring idea plus an
 *     LM393 comparator and sensitivity pot for a clean digital
 *     output. The convenience costs the battery: the comparator
 *     chain idles at hundreds of uA -- worse than the raw switch it
 *     cleans up, on the wrong side of this design's budget. For
 *     wall power, or when adjustable sensitivity matters more than
 *     the battery.
 *   - Piezo ceramic disc (~$0.50): not a switch but a GENERATOR --
 *     flexing it makes voltage, so quiescent current is exactly
 *     zero and sensitivity is analog (a tap vs a thump are
 *     different amplitudes -- knock-pattern locks live here). The
 *     cost is interface fiddliness: it wants a ~1 M load resistor,
 *     and its spikes can exceed the rails (add a series resistor;
 *     the AVR's clamp diodes handle the rest at piezo-disc
 *     energies). Read it with Analog::read() while awake, or bias
 *     it so a sharp knock crosses the digital threshold for a
 *     PCINT wake. Most work, most interesting signal.
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

// The low-battery threshold, and why it isn't arbitrary. A 9 V block
// is six alkaline cells; 6.4 V (~1.07 V/cell) is the knee where the
// discharge curve stops plateauing and starts to cliff -- warning
// here leaves days of alarm function to swap the battery. The HARD
// constraint is from below: the measurement reads the divider against
// the 5 V rail as ADC reference, and once the battery sags to about
// 5 V + the LDO's dropout (~5.5-6.0 V) that reference itself sags --
// which makes the computed battery voltage OVERestimate exactly when
// the battery is dying. The warning must fire while the reference is
// still honest, i.e. comfortably above ~6 V. Tune within ~6.2-6.8 by
// taste; don't go below.
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
