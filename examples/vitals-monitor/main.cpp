/**
 * vitals-monitor: battery, temperature, and light on an LCD --
 * from an ATtiny84, mostly asleep.
 *
 * What it does, every ~2 seconds (sleeping in PWR_DOWN between):
 *   - battery: measures its own supply with NO external parts and
 *     shows volts plus a battery-icon gauge;
 *   - temperature: the AVR's on-die sensor, shown as approximate
 *     degrees C (honest tilde included -- see below);
 *   - light: a photoresistor divider, shown as a relative percent.
 *
 * HOW THE BATTERY GAUGE WORKS (the trick you've never used): the
 * chip cannot measure Vcc against Vcc -- that ratio is always 1. But
 * it has a fixed ~1.1 V bandgap reference it can route to the ADC as
 * an INPUT while Vcc stays the reference. The bandgap then reads as
 *     raw = 1.1 / Vcc * 1024
 * and solving backwards, Vcc = 1.1 * 1024 / raw. As the battery
 * sags, the fixed 1.1 V occupies a larger fraction of the range and
 * the raw reading RISES. Zero external parts; +/-10% chip-to-chip
 * until you calibrate the 1100 in analog.hpp's constant per board.
 *
 * Honesty notes: the on-die temperature sensor is ~1 LSB/degC with a
 * large uncalibrated per-chip offset -- the "~NN C" here assumes the
 * datasheet's typical (300 raw at 25 C) and is best read as "warmer/
 * cooler than before". The photoresistor percent is relative to the
 * divider, not lux.
 *
 * Parts: ATtiny84 @ 8 MHz internal, LCD1602 + PCF8574 backpack,
 * LDR + 10k fixed resistor as a divider. Power from 3xAA/4xAA or
 * USB -- the point is watching the voltage move. (Character LCDs
 * want ~5 V for good contrast; at 3.x V expect a faint display.)
 *
 * Wiring (physical DIP-14 pins):
 *   SDA -> PA6 (7),  SCL -> PA5 (8)    4.7k pull-ups (backpack has them)
 *   LDR divider midpoint -> PA0 (13)   (LDR to VCC, 10k to GND)
 *
 * Exercises: Analog (read/read8, readVccMillivolts, readTemperature,
 * reference switching), Watchdog::sleepFor (PWR_DOWN cadence),
 * Comms::I2C (bit-banged), Devices::LCD1602 (programmatic
 * createChar) -- and it never even starts the Ticker.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"
#include "devices/LCD1602.hpp"

using I2c = HAL::Comms::I2C::Master<7, 8, 100000>;   // SDA, SCL, ceiling
using Lcd = HAL::Devices::LCD1602<I2c>;

// tune to your pack: 3xAA alkaline sags ~4.7 V (fresh) to ~3.3 V (spent)
constexpr uint16_t BAT_FULL_MV  { 4700 };
constexpr uint16_t BAT_EMPTY_MV { 3300 };

// the watchdog wake-up needs a vector; nothing to do in it
EMPTY_INTERRUPT(WDT_vect)

// four battery glyphs (empty..full), drawn programmatically: a nub,
// bordered body, fill rising from the bottom
void makeBatteryGlyphs() {
    for (uint8_t lvl = 0; lvl < 4; ++lvl) {
        uint8_t g[8];
        g[0] = 0x0E;                       // the nub
        for (uint8_t r = 1; r < 7; ++r) {
            bool filledP { static_cast<uint8_t>(6 - r) < lvl * 2 };
            g[r] = static_cast<uint8_t>(0x11 | (filledP ? 0x0E : 0));
        }
        g[7] = 0x1F;                       // the base
        Lcd::createChar(lvl, g);
    }
}

void print2(uint8_t v) {
    char b[2];
    HAL::Utils::Fmt::fixed(b, v, 2);   // the shared formatter
    Lcd::write(b[0]);
    Lcd::write(b[1]);
}

int main() {
    HAL::Analog::begin();
    Lcd::begin();
    makeBatteryGlyphs();
    sei();   // sleepFor's watchdog wake-up needs interrupts on

    while (1) {
        // sample. Order matters a little: readTemperature() switches
        // to the 1.1 V reference, so the light reading re-selects VCC
        // first -- the Analog module auto-discards the settling
        // conversion on every reference change
        HAL::Analog::setReference<HAL::Analog::Ref::VCC>();
        uint16_t light { HAL::Analog::read<13>() };          // LDR divider
        uint16_t mv    { HAL::Analog::readVccMillivolts() }; // the trick
        int16_t  traw  { static_cast<int16_t>(
                             HAL::Analog::readTemperature()) };

        // battery gauge level 0..3 across the configured range
        uint8_t lvl { 0 };
        if (mv >= BAT_EMPTY_MV) {
            uint32_t span { static_cast<uint32_t>(BAT_FULL_MV)
                            - BAT_EMPTY_MV };
            uint32_t up   { static_cast<uint32_t>(mv) - BAT_EMPTY_MV };
            uint32_t l    { (up * 4) / span };
            lvl = l > 3 ? 3 : static_cast<uint8_t>(l);
        }

        // row 0: "bat 4.83V  [##]"
        Lcd::setCursor(0, 0);
        Lcd::print_P(PSTR("bat "));
        Lcd::write(static_cast<char>('0' + (mv / 1000) % 10));
        Lcd::write('.');
        print2(static_cast<uint8_t>((mv % 1000) / 10));
        Lcd::print_P(PSTR("V   "));
        Lcd::write(static_cast<char>(lvl));
        Lcd::print_P(PSTR("  "));

        // row 1: "temp ~24C  L 63%"  (datasheet-typical offset: 300
        // raw at 25 C, ~1 LSB per degree; per-chip offset applies)
        int16_t degC { static_cast<int16_t>(traw - 275) };
        if (degC < -9) degC = -9;
        if (degC > 99) degC = 99;
        Lcd::setCursor(0, 1);
        Lcd::print_P(PSTR("temp ~"));
        if (degC < 0) {
            Lcd::write('-');
            degC = static_cast<int16_t>(-degC);
        }
        print2(static_cast<uint8_t>(degC));
        Lcd::print_P(PSTR("C  L "));
        print2(static_cast<uint8_t>(
            (static_cast<uint32_t>(light) * 100) / 1023));
        Lcd::write('%');

        // lights out for ~2 s: PWR_DOWN, watchdog-timed. The LCD keeps
        // displaying (it has its own controller); only the AVR naps
        HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS2048>(1);
    }
}
