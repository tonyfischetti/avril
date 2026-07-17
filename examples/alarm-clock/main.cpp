/**
 * alarm-clock: a complete bedside alarm clock.
 *
 * What it does:
 *   - shows HH:MM:SS on the LCD, ticking off the DS3231 (so the time
 *     survives power loss on the coin cell, +/-2 ppm). The seconds
 *     tick on the RTC's OWN 1 Hz square wave, wired to a PCINT: each
 *     falling edge IS the second boundary, so the display never
 *     stutters the way an MCU-paced poll would (two almost-equal
 *     clocks beating against each other);
 *   - press the knob to step through set modes: time hours -> time
 *     minutes -> alarm hours -> alarm minutes -> back to clock.
 *     Rotate to adjust the highlighted field. Edits are written to
 *     the RTC when you leave the last field of each pair;
 *   - long-press to arm/disarm the alarm (bell glyph on the display);
 *   - when the alarm matches, the LED blinks until any press
 *     silences it. THE INTCN NUANCE: the DS3231's pin carries either
 *     the alarms or the square wave, never both -- but this design
 *     never needed the pin for alarms, because it polls the latched
 *     A1F flag (alarmFired()) on each second-tick. The flag sets on
 *     a match regardless of pin routing, so the pin is free to be
 *     the heartbeat;
 *   - if the RTC reports its oscillator stopped (battery died), the
 *     clock boots straight into set mode instead of showing fiction.
 *
 * Parts: ATmega328P @ 16 MHz, DS3231 module (ZS-042), LCD1602 with
 * PCF8574 backpack, KY-040-style rotary encoder with push button,
 * one LED + resistor. The I2C bus wants 4.7k pull-ups (the modules
 * usually carry them already).
 *
 * Wiring (physical DIP-28 pins):
 *   SDA           -> PC4 (27)  \  ONE two-wire bus: the DS3231 and
 *   SCL           -> PC5 (28)  /  the LCD backpack each connect BOTH
 *                                 lines; the address byte picks who
 *                                 answers
 *   RTC INT/SQW   -> PD5 (11)     open-drain: the MCU's internal
 *                                 pullup carries it
 *   encoder CLK   -> PD2 (4)
 *   encoder DT    -> PD3 (5)
 *   encoder SW    -> PD4 (6)
 *   alarm LED     -> PB0 (14), through ~330R to GND
 *   UART TX       -> PD1 (3), optional: 9600 baud boot/debug chatter
 *
 * Exercises: Comms::I2C, Devices::DS3231, Devices::LCD1602,
 * Devices::RotaryEncoderWithButton, Ticker, Sleep, Comms::UART --
 * and the canonical notify/process/sleep loop shape.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"
#include "devices/RotaryEncoderWithButton.hpp"
#include "devices/LCD1602.hpp"
#include "devices/DS3231.hpp"

namespace UART = HAL::Comms::UART;          // the README's alias tip
namespace DS   = HAL::Devices::DS3231;

using I2c      = HAL::Comms::I2C::Master<100000>;
using Lcd      = HAL::Devices::LCD1602<I2c>;
using Rtc      = DS::Clock<I2c>;
using AlarmLed = HAL::GPIO::GPIO<14>;       // PB0
using Sqw      = HAL::GPIO::GPIO<11>;       // PD5: the RTC's 1 Hz beat

HAL::Devices::RotaryEncoderWithButton<6, 30, 1000, HIGH, true,  // button
                                      4, 5, true> knob;         // clk, dt

volatile uint8_t previousPIND { 0xFF };
volatile bool    secondTickP  { true };   // true at boot: paint at once

ISR(PCINT2_vect) {
    uint32_t now { HAL::Ticker::getNumTicks() };
    uint8_t  cur { PIND };
    uint8_t  ch  { static_cast<uint8_t>(cur ^ previousPIND) };
    previousPIND = cur;
    knob.notifyInterruptOccurred(now, HAL::GPIO::Port::D, ch);
    // the RTC's falling edge IS the second boundary
    if ((ch & Sqw::mask) && !Sqw::read()) secondTickP = true;
}

enum class Mode : uint8_t {
    CLOCK, SET_TIME_H, SET_TIME_M, SET_ALARM_H, SET_ALARM_M
};

// app state at file scope: callbacks are plain function pointers, so
// this is where shared state lives (see the README's Callback note)
Mode         mode        { Mode::CLOCK };
DS::DateTime now         {};
uint8_t      alarmHour   { 7 };
uint8_t      alarmMinute { 0 };
bool         armedP      { true };
bool         ringingP    { false };
bool         dirtyP      { true };   // display wants a repaint

void programAlarm() {
    DS::DateTime a {};
    a.hour   = alarmHour;
    a.minute = alarmMinute;
    a.second = 0;
    Rtc::setAlarm1(a, DS::Alarm1Mode::HOUR_MIN_SEC_MATCH);
    Rtc::clearAlarmFlags();   // arm cleanly: a stale flag would ring now
}

// rotation adjusts whichever field the mode highlights
void adjust(int8_t d) {
    switch (mode) {
        case Mode::SET_TIME_H:
            now.hour = static_cast<uint8_t>((now.hour + 24 + d) % 24);
            break;
        case Mode::SET_TIME_M:
            now.minute = static_cast<uint8_t>((now.minute + 60 + d) % 60);
            break;
        case Mode::SET_ALARM_H:
            alarmHour = static_cast<uint8_t>((alarmHour + 24 + d) % 24);
            break;
        case Mode::SET_ALARM_M:
            alarmMinute = static_cast<uint8_t>((alarmMinute + 60 + d) % 60);
            break;
        case Mode::CLOCK:
            return;   // rotation does nothing on the clock face
    }
    dirtyP = true;
}

// press: silence a ringing alarm, otherwise step through the set
// modes -- committing to the RTC at each pair's exit
void onPress() {
    if (ringingP) {
        ringingP = false;
        Rtc::clearAlarmFlags();   // THE gotcha: INT/flag latches
        AlarmLed::setLow();
        dirtyP = true;
        return;
    }
    switch (mode) {
        case Mode::CLOCK:      mode = Mode::SET_TIME_H;  break;
        case Mode::SET_TIME_H: mode = Mode::SET_TIME_M;  break;
        case Mode::SET_TIME_M:
            now.second = 0;
            Rtc::setTime(now);            // also clears the stale flag
            mode = Mode::SET_ALARM_H;
            break;
        case Mode::SET_ALARM_H: mode = Mode::SET_ALARM_M; break;
        case Mode::SET_ALARM_M:
            programAlarm();
            mode = Mode::CLOCK;
            break;
    }
    dirtyP = true;
}

void onLongPress() {
    armedP = !armedP;
    if (!armedP && ringingP) {   // disarming also silences
        ringingP = false;
        Rtc::clearAlarmFlags();
        AlarmLed::setLow();
    }
    dirtyP = true;
}

void print2(uint8_t v) {
    char b[2];
    HAL::Utils::Fmt::fixed(b, v, 2);   // the shared formatter
    Lcd::write(b[0]);
    Lcd::write(b[1]);
}

void repaint() {
    Lcd::setCursor(0, 0);
    Lcd::print_P(PSTR("    "));
    print2(now.hour);
    Lcd::write(':');
    print2(now.minute);
    Lcd::write(':');
    print2(now.second);
    Lcd::print_P(PSTR("    "));

    Lcd::setCursor(0, 1);
    Lcd::write(armedP ? static_cast<char>(0) : ' ');   // bell glyph
    Lcd::write(' ');
    print2(alarmHour);
    Lcd::write(':');
    print2(alarmMinute);
    switch (mode) {   // which field the knob is editing
        case Mode::SET_TIME_H:  Lcd::print_P(PSTR("  set t.h")); break;
        case Mode::SET_TIME_M:  Lcd::print_P(PSTR("  set t.m")); break;
        case Mode::SET_ALARM_H: Lcd::print_P(PSTR("  set a.h")); break;
        case Mode::SET_ALARM_M: Lcd::print_P(PSTR("  set a.m")); break;
        case Mode::CLOCK:
            Lcd::print_P(ringingP ? PSTR("  WAKE UP") : PSTR("         "));
            break;
    }
}

int main() {
    HAL::Ticker::setupMSTimer();
    UART::init<9600>();
    AlarmLed::setOutput();
    knob.begin();
    sei();

    UART::println_P(PSTR("alarm-clock boot"));

    Sqw::setInputPullup();   // INT/SQW is open-drain
    Sqw::enablePCINT();

    Lcd::begin();
    static const uint8_t bell[8] { 0x04, 0x0E, 0x0E, 0x0E,
                                   0x1F, 0x00, 0x04, 0x00 };
    Lcd::createChar(0, bell);

    // trust the RTC only if it never lost its battery; otherwise
    // boot straight into set mode rather than displaying fiction
    bool staleP { true };
    if (Rtc::oscStopped(staleP) == HAL::Comms::I2C::Result::OK
            && !staleP) {
        Rtc::getTime(now);
    } else {
        UART::println_P(PSTR("RTC time invalid; entering set mode"));
        now.hour = 12; now.minute = 0; now.second = 0;
        now.date = 1;  now.month = 1;  now.year = 2026; now.dow = 1;
        mode = Mode::SET_TIME_H;
    }
    programAlarm();
    // the display's timebase: the RTC's own second boundary (see the
    // INTCN nuance in the header -- alarms stay on the polled flag)
    Rtc::enableSquareWave(DS::SqwFreq::HZ_1);

    knob.setOnCW([]()  { adjust(+1); });
    knob.setOnCCW([]() { adjust(-1); });
    knob.setOnPress(&onPress);
    knob.setOnLongPress(&onLongPress);

    while (1) {
        knob.process();

        // each RTC second-boundary: refresh the time and check the
        // alarm flag. No MCU-side pacing arithmetic at all -- the
        // wall clock itself says when a second has passed
        if (secondTickP && mode == Mode::CLOCK) {
            secondTickP = false;
            if (Rtc::getTime(now) == HAL::Comms::I2C::Result::OK) {
                dirtyP = true;   // seconds are showing: every tick
                bool a1 { false };
                bool a2 { false };
                if (armedP && !ringingP
                        && Rtc::alarmFired(a1, a2)
                               == HAL::Comms::I2C::Result::OK
                        && a1) {
                    ringingP = true;
                    UART::println_P(PSTR("ALARM"));
                }
            }
        }

        if (ringingP) {
            // ~4 Hz blink off the tick counter
            uint32_t ticks { HAL::Ticker::getNumTicks() };
            if (ticks & 0x80) AlarmLed::setHigh(); else AlarmLed::setLow();
        }

        if (dirtyP) {
            dirtyP = false;
            repaint();
        }

        // ticker wakes us within a millisecond; the canonical bottom
        // of an avril main loop
        HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
    }
}
