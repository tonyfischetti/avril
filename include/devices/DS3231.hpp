#pragma once

#include "common.hpp"

#include <stdint.h>

#include "../comms/i2c.hpp"

/**
 * DS3231 real-time clock: the temperature-compensated I2C RTC
 * (+/-2 ppm -- about a minute per YEAR, versus the watchdog
 * oscillator's minute per minute) with battery backup, two alarms,
 * and a factory-calibrated thermometer thrown in.
 *
 * Why it matters to a sleep-centric HAL: the alarm output (the
 * module's INT/SQW pin, open-drain, pull it up) goes LOW when an
 * alarm matches. Wire it to a PCINT and the AVR can sleep in
 * PWR_DOWN at microamps yet wake at a WALL-CLOCK time -- the
 * precision complement to Watchdog::sleepFor's "roughly N seconds".
 *
 * THE GOTCHA, worth reading twice: the alarm flags (A1F/A2F in the
 * status register) latch, and INT stays low until *you* clear them
 * with clearAlarmFlags(). Forget that and the failure is sneaky: the
 * alarm rings once, then never again -- the next match finds the flag
 * still set and INT already low, and a pin that is already low cannot
 * produce a falling edge to wake anything. Because the coin cell
 * keeps the latch alive across every MCU reboot and reflash, the only
 * event that resets it is the DS3231 losing ALL power. In other
 * words: your daily alarm fires exactly once per battery.
 *
 * Honesty feature: the oscillator-stop flag (OSF) is the chip
 * confessing "power AND battery were lost at some point; the time I
 * hold is fiction". Check oscStopped() at boot before trusting
 * getTime(); setTime() clears the flag, because setting the clock is
 * what makes time real again.
 *
 * All times are 24-hour and years 2000-2099 (the century bit is not
 * used). Fixed I2C address 0x68 -- the chip offers no alternative.
 */

namespace HAL {
namespace Devices {
namespace DS3231 {

using Result = HAL::Comms::I2C::Result;

struct DateTime {
    uint16_t year;     // full year: 2026
    uint8_t  month;    // 1..12
    uint8_t  date;     // 1..31
    uint8_t  dow;      // 1..7, whatever convention you keep
    uint8_t  hour;     // 0..23
    uint8_t  minute;   // 0..59
    uint8_t  second;   // 0..59
};

// alarm 1 has seconds resolution...
enum class Alarm1Mode : uint8_t {
    EVERY_SECOND,
    SECONDS_MATCH,        // once a minute, at :ss
    MIN_SEC_MATCH,        // once an hour, at mm:ss
    HOUR_MIN_SEC_MATCH,   // once a day, at hh:mm:ss
    DATE_TIME_MATCH       // once a month, at DD hh:mm:ss
};

// ...alarm 2 starts at minutes
enum class Alarm2Mode : uint8_t {
    EVERY_MINUTE,
    MINUTES_MATCH,        // once an hour, at mm
    HOUR_MIN_MATCH,       // once a day, at hh:mm
    DATE_TIME_MATCH       // once a month, at DD hh:mm
};

enum class SqwFreq : uint8_t { HZ_1, HZ_1024, HZ_4096, HZ_8192 };

template<typename I2cBus>
struct Clock {

    static constexpr uint8_t ADDR { 0x68 };

  private:
    // register map
    static constexpr uint8_t REG_TIME    { 0x00 };  // 7 BCD bytes
    static constexpr uint8_t REG_ALARM1  { 0x07 };
    static constexpr uint8_t REG_ALARM2  { 0x0B };
    static constexpr uint8_t REG_CONTROL { 0x0E };
    static constexpr uint8_t REG_STATUS  { 0x0F };
    static constexpr uint8_t REG_TEMP    { 0x11 };

    static constexpr uint8_t CTRL_INTCN { 0x04 };
    static constexpr uint8_t CTRL_A2IE  { 0x02 };
    static constexpr uint8_t CTRL_A1IE  { 0x01 };
    static constexpr uint8_t STAT_OSF   { 0x80 };

    static uint8_t bcd2bin(uint8_t b) {
        return static_cast<uint8_t>((b >> 4) * 10 + (b & 0x0F));
    }
    static uint8_t bin2bcd(uint8_t v) {
        return static_cast<uint8_t>(((v / 10) << 4) | (v % 10));
    }

    static Result readReg(uint8_t reg, uint8_t& val) {
        return I2cBus::writeRead(ADDR, &reg, 1, &val, 1);
    }

    static Result writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] { reg, val };
        return I2cBus::write(ADDR, buf, 2);
    }

    // read-modify-write with a mask; the status register especially
    // must be written surgically (other bits are flags and enables)
    static Result updateReg(uint8_t reg, uint8_t clear, uint8_t set) {
        uint8_t v;
        Result r { readReg(reg, v) };
        if (r != Result::OK) return r;
        v = static_cast<uint8_t>((v & static_cast<uint8_t>(~clear))
                                 | set);
        return writeReg(reg, v);
    }

  public:
    static Result getTime(DateTime& dt) {
        uint8_t reg { REG_TIME };
        uint8_t b[7];
        Result r { I2cBus::writeRead(ADDR, &reg, 1, b, 7) };
        if (r != Result::OK) return r;
        dt.second = bcd2bin(b[0]);
        dt.minute = bcd2bin(b[1]);
        dt.hour   = bcd2bin(static_cast<uint8_t>(b[2] & 0x3F)); // 24 h
        dt.dow    = b[3];
        dt.date   = bcd2bin(b[4]);
        dt.month  = bcd2bin(static_cast<uint8_t>(b[5] & 0x1F));
        dt.year   = static_cast<uint16_t>(2000 + bcd2bin(b[6]));
        return Result::OK;
    }

    // also clears the oscillator-stop flag: setting the clock is what
    // makes the time real again
    static Result setTime(const DateTime& dt) {
        uint8_t buf[8] {
            REG_TIME,
            bin2bcd(dt.second),
            bin2bcd(dt.minute),
            bin2bcd(dt.hour),          // 24-hour mode (bit 6 clear)
            dt.dow,
            bin2bcd(dt.date),
            bin2bcd(dt.month),
            bin2bcd(static_cast<uint8_t>(dt.year % 100))
        };
        Result r { I2cBus::write(ADDR, buf, 8) };
        if (r != Result::OK) return r;
        return updateReg(REG_STATUS, STAT_OSF, 0);
    }

    // "has the chip lost power and battery since the time was set?"
    // -- if true, getTime() returns fiction. Check at boot
    static Result oscStopped(bool& stoppedP) {
        uint8_t s;
        Result r { readReg(REG_STATUS, s) };
        if (r != Result::OK) return r;
        stoppedP = (s & STAT_OSF) != 0;
        return r;
    }

    // dt supplies the fields the mode actually matches on; the rest
    // are ignored. The alarm is configured but fires on INT only
    // after enableAlarmInterrupts()
    static Result setAlarm1(const DateTime& dt, Alarm1Mode mode) {
        // A1M4..A1M1 mask bits, one per register, 1 = "don't care":
        // each mode masks every field below its resolution
        uint8_t m { static_cast<uint8_t>(mode) };
        uint8_t buf[5] {
            REG_ALARM1,
            static_cast<uint8_t>(bin2bcd(dt.second)
                | ((mode == Alarm1Mode::EVERY_SECOND) ? 0x80 : 0)),
            static_cast<uint8_t>(bin2bcd(dt.minute)
                | ((m < static_cast<uint8_t>(Alarm1Mode::MIN_SEC_MATCH))
                       ? 0x80 : 0)),
            static_cast<uint8_t>(bin2bcd(dt.hour)
                | ((m < static_cast<uint8_t>(
                        Alarm1Mode::HOUR_MIN_SEC_MATCH)) ? 0x80 : 0)),
            static_cast<uint8_t>(bin2bcd(dt.date)
                | ((m < static_cast<uint8_t>(Alarm1Mode::DATE_TIME_MATCH))
                       ? 0x80 : 0))   // DY/DT=0: match date-of-month
        };
        return I2cBus::write(ADDR, buf, 5);
    }

    static Result setAlarm2(const DateTime& dt, Alarm2Mode mode) {
        uint8_t m { static_cast<uint8_t>(mode) };
        uint8_t buf[4] {
            REG_ALARM2,
            static_cast<uint8_t>(bin2bcd(dt.minute)
                | ((mode == Alarm2Mode::EVERY_MINUTE) ? 0x80 : 0)),
            static_cast<uint8_t>(bin2bcd(dt.hour)
                | ((m < static_cast<uint8_t>(Alarm2Mode::HOUR_MIN_MATCH))
                       ? 0x80 : 0)),
            static_cast<uint8_t>(bin2bcd(dt.date)
                | ((m < static_cast<uint8_t>(Alarm2Mode::DATE_TIME_MATCH))
                       ? 0x80 : 0))
        };
        return I2cBus::write(ADDR, buf, 4);
    }

    // route alarms to the INT pin (INTCN=1 kills the square wave) and
    // enable the chosen ones. INT is open-drain: pull it up, feed it
    // to a PCINT, and PWR_DOWN wake-ups happen at wall-clock times
    static Result enableAlarmInterrupts(bool a1, bool a2) {
        return updateReg(REG_CONTROL,
                         static_cast<uint8_t>(CTRL_A1IE | CTRL_A2IE),
                         static_cast<uint8_t>(CTRL_INTCN
                             | (a1 ? CTRL_A1IE : 0)
                             | (a2 ? CTRL_A2IE : 0)));
    }

    static Result alarmFired(bool& a1, bool& a2) {
        uint8_t s;
        Result r { readReg(REG_STATUS, s) };
        if (r != Result::OK) return r;
        a1 = (s & 0x01) != 0;
        a2 = (s & 0x02) != 0;
        return r;
    }

    // THE GOTCHA: A1F/A2F latch and hold INT low until cleared here.
    // Call this from process()/main after handling a wake-up, or the
    // alarm never fires again
    static Result clearAlarmFlags() {
        return updateReg(REG_STATUS, 0x03, 0);
    }

    // square wave on the same pin instead of alarms (INTCN=0):
    // 1 Hz on a PCINT is a wall-clock heartbeat that survives
    // PWR_DOWN. Mutually exclusive with alarm interrupts
    static Result enableSquareWave(SqwFreq f) {
        return updateReg(REG_CONTROL,
                         static_cast<uint8_t>(CTRL_INTCN | 0x18),
                         static_cast<uint8_t>(
                             static_cast<uint8_t>(f) << 3));
    }

    // factory-calibrated (+/-3 degC, 0.25 degC steps -- it exists to
    // trim the crystal), in quarter-degrees: 25.75 C -> 103.
    // Refreshed by the chip every 64 s
    static Result readTemperatureQuarters(int16_t& quarters) {
        uint8_t reg { REG_TEMP };
        uint8_t b[2];
        Result r { I2cBus::writeRead(ADDR, &reg, 1, b, 2) };
        if (r != Result::OK) return r;
        quarters = static_cast<int16_t>(
            (static_cast<int16_t>(static_cast<int8_t>(b[0])) * 4)
            + (b[1] >> 6));
        return Result::OK;
    }

};

}
}
}
