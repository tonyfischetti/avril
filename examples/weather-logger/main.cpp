/**
 * weather-logger: a datalogger with real timestamps, a live display,
 * and a log file your laptop opens directly -- despite avril's FAT32
 * being read-only.
 *
 * What it does:
 *   - every 2 s: samples the DS3231 (time + its calibrated
 *     thermometer), the LDR divider, and Vcc;
 *   - every 1 s: refreshes the LCD (clock + temperature on row 0,
 *     record count + supply voltage on row 1);
 *   - every 60 s: appends one fixed-width, human-readable record to
 *     LOG.CSV on the SD card;
 *   - every 500 ms: blinks a heartbeat LED;
 *   - between all of that: SLEEPS. Four independent cadences, one
 *     task table, and the loop bottom sleeps exactly as long as the
 *     scheduler says it may -- PWR_DOWN naps for long gaps (the
 *     ticker is paused and credited back), IDLE for short ones.
 *     This is Utils::Scheduler's demo: the tick-less loop.
 *
 * ---- THE LOGGING TRICK: raw writes into a read-only FAT -----------
 *
 * avril's fs/fat32.hpp cannot create or grow files -- writing FAT
 * metadata is deliberately unimplemented. The classic embedded
 * workaround: PREALLOCATE the file on your computer, and let the
 * read-only layer FIND it. A freshly formatted card's first file is
 * contiguous, so its blocks form one range that raw Sd::writeBlock()
 * calls can fill. File contents change; filesystem metadata doesn't;
 * the card still mounts anywhere and LOG.CSV opens in any editor.
 *
 * Card preparation (once, on the computer):
 *   1. format the card FAT32
 *   2. create a space-filled LOG.CSV as the FIRST file copied on:
 *        python3 -c "open('LOG.CSV','w').write(' ' * 16*1024*1024)"
 *      (or: dd if=/dev/zero bs=1m count=16 | tr '\\0' ' ' > LOG.CSV)
 *   3. copy it to the card, eject cleanly
 *
 * 16 MiB of 64-byte records = ~262,000 records = ~6 months at one
 * per minute. The firmware verifies the file is contiguous at boot
 * (Fat::contiguousBlockRange) and refuses politely if not.
 *
 * Resume after power loss is a binary search: blocks are filled in
 * order and the file was pre-filled with spaces, so "first block
 * whose first byte is a space" -- found in ~15 one-byte windowed
 * reads -- is where logging continues. Each record is persisted the
 * moment it's taken (the current block is rewritten per record), so
 * a power cut loses nothing but the partial minute.
 *
 * Division of clocks, worth noticing: the SCHEDULER's cadence is
 * approximate (watchdog naps are credited at nominal +/-10%), and
 * that's fine -- because every record is TIMESTAMPED by the DS3231
 * at +/-2 ppm. The scheduler decides roughly-when; the RTC records
 * exactly-when. Approximate scheduling, exact data.
 *
 * Parts: ATmega328P @ 16 MHz, microSD module (level-shifted, card
 * prepared as above), DS3231, LCD1602 + PCF8574 backpack, LDR + 10k
 * divider, heartbeat LED + resistor.
 *
 * Wiring (physical DIP-28 pins):
 *   SPI: MOSI PB3 (17), MISO PB4 (18), SCK PB5 (19); SD CS -> PB2 (16)
 *   SDA -> PC4 (27), SCL -> PC5 (28)     DS3231 + LCD, same bus
 *   LDR divider midpoint -> PC0 (23)     (LDR to 5V, 10k to GND)
 *   heartbeat LED -> PB0 (14), ~330R to GND
 *   UART TX -> PD1 (3), 9600 baud boot/status chatter
 *
 * Exercises: Utils::Scheduler (the tick-less loop), FS::FAT32
 * (contiguousBlockRange -- the read-only write path), Devices::SD
 * (raw writeBlock), DS3231, LCD1602, Analog, Watchdog::sleepFor,
 * SPI, I2C, UART, Ticker, Sleep. Nearly the whole HAL.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"
#include "utils/Scheduler.hpp"
#include "devices/SDCard.hpp"
#include "devices/DS3231.hpp"
#include "devices/LCD1602.hpp"
#include "fs/fat32.hpp"

namespace UART = HAL::Comms::UART;
namespace DS   = HAL::Devices::DS3231;

using I2c = HAL::Comms::I2C::Master<100000>;
using Lcd = HAL::Devices::LCD1602<I2c>;
using Rtc = DS::Clock<I2c>;
using Sd  = HAL::Devices::SD::Card<16, 8000000>;
using Fat = HAL::FS::FAT32<Sd>;
using Led = HAL::GPIO::GPIO<14>;     // PB0 heartbeat
constexpr uint8_t LDR_PIN { 23 };    // PC0

constexpr uint32_t SAMPLE_MS  {  2000 };
constexpr uint32_t DISPLAY_MS {  1000 };
constexpr uint32_t LOG_MS     { 60000 };
constexpr uint32_t BLINK_MS   {   500 };

constexpr uint8_t  REC_BYTES  { 64 };
constexpr uint8_t  RECS_PER_BLOCK { 512 / REC_BYTES };

EMPTY_INTERRUPT(WDT_vect)   // the tick-less loop's PWR_DOWN naps

// ---- latest samples (written by sampleTask, read by the others) ----
DS::DateTime now      {};
int16_t      tempQ    { 0 };      // quarter-degrees C, from the RTC
uint16_t     light    { 0 };      // LDR raw 0..1023
uint16_t     vccMv    { 0 };
uint32_t     recCount { 0 };

// ---- the log: one preallocated contiguous file ---------------------
uint8_t  block[512];
uint32_t logFirst { 0 };          // block range of LOG.CSV
uint32_t logCount { 0 };
uint32_t curBlock { 0 };
uint8_t  recInBlock { 0 };
bool     loggingP { false };      // false: card problem or log full

void blankBlock() {
    for (uint16_t i = 0; i < 512; ++i) block[i] = ' ';
}

// ---- tasks ---------------------------------------------------------

void sampleTask() {
    Rtc::getTime(now);
    Rtc::readTemperatureQuarters(tempQ);
    HAL::Analog::setReference<HAL::Analog::Ref::VCC>();
    light = HAL::Analog::read<LDR_PIN>();
    vccMv = HAL::Analog::readVccMillivolts();
}

void put2(char* p, uint8_t v) {
    p[0] = static_cast<char>('0' + (v / 10) % 10);
    p[1] = static_cast<char>('0' + v % 10);
}

// "2026-07-17 14:03:22  +23.75C  light 0512  vcc 4923mV" padded to
// 63 chars + '\n': fixed-width, so the file is both machine-parseable
// and pleasant in a text editor
void logTask() {
    if (!loggingP) return;
    char* r { reinterpret_cast<char*>(block)
              + static_cast<uint16_t>(recInBlock) * REC_BYTES };
    for (uint8_t i = 0; i < REC_BYTES - 1; ++i) r[i] = ' ';
    r[REC_BYTES - 1] = '\n';

    put2(r + 0, static_cast<uint8_t>(now.year / 100));
    put2(r + 2, static_cast<uint8_t>(now.year % 100));
    r[4] = '-';  put2(r + 5,  now.month);
    r[7] = '-';  put2(r + 8,  now.date);
    put2(r + 11, now.hour);
    r[13] = ':'; put2(r + 14, now.minute);
    r[16] = ':'; put2(r + 17, now.second);

    int16_t q { tempQ };
    r[21] = (q < 0) ? '-' : '+';
    if (q < 0) q = static_cast<int16_t>(-q);
    put2(r + 22, static_cast<uint8_t>((q / 4) % 100));
    r[24] = '.';
    put2(r + 25, static_cast<uint8_t>((q % 4) * 25));
    r[27] = 'C';

    r[30] = 'L'; r[31] = ' ';
    put2(r + 32, static_cast<uint8_t>(light / 100));
    put2(r + 34, static_cast<uint8_t>(light % 100));  // 4 digits

    r[38] = 'V'; r[39] = ' ';
    put2(r + 40, static_cast<uint8_t>(vccMv / 100));
    put2(r + 42, static_cast<uint8_t>(vccMv % 100));  // millivolts

    // persist NOW: rewrite the current block. A power cut costs at
    // most the partial minute, never a written record
    if (Sd::writeBlock(curBlock, block) != HAL::Devices::SD::Result::OK) {
        loggingP = false;
        UART::println_P(PSTR("log: write failed; logging stopped"));
        return;
    }
    recCount = recCount + 1;
    if (++recInBlock >= RECS_PER_BLOCK) {
        recInBlock = 0;
        blankBlock();
        curBlock = curBlock + 1;
        if (curBlock >= logFirst + logCount) {
            loggingP = false;
            UART::println_P(PSTR("log: file full; logging stopped"));
        }
    }
}

void put2Lcd(uint8_t v) {
    Lcd::write(static_cast<char>('0' + (v / 10) % 10));
    Lcd::write(static_cast<char>('0' + v % 10));
}

void displayTask() {
    // row 0: "14:03:22  +23.7C"
    Lcd::setCursor(0, 0);
    put2Lcd(now.hour);   Lcd::write(':');
    put2Lcd(now.minute); Lcd::write(':');
    put2Lcd(now.second);
    Lcd::print_P(PSTR("  "));
    int16_t q { tempQ };
    Lcd::write(q < 0 ? '-' : '+');
    if (q < 0) q = static_cast<int16_t>(-q);
    put2Lcd(static_cast<uint8_t>((q / 4) % 100));
    Lcd::write('.');
    Lcd::write(static_cast<char>('0' + ((q % 4) * 25) / 10));
    Lcd::write('C');

    // row 1: "rec 000123 4.92V"  (or the reason logging stopped)
    Lcd::setCursor(0, 1);
    if (!loggingP) {
        Lcd::print_P(PSTR("log stopped     "));
        return;
    }
    Lcd::print_P(PSTR("rec "));
    uint32_t c { recCount };
    char buf[7];
    for (int8_t i = 5; i >= 0; --i) {
        buf[i] = static_cast<char>('0' + c % 10);
        c /= 10;
    }
    buf[6] = '\0';
    Lcd::print(buf);
    Lcd::write(' ');
    Lcd::write(static_cast<char>('0' + (vccMv / 1000) % 10));
    Lcd::write('.');
    put2Lcd(static_cast<uint8_t>((vccMv % 1000) / 10));
    Lcd::write('V');
}

void blinkTask() { Led::toggle(); }

// ---- boot ----------------------------------------------------------

HAL::Utils::Scheduler<4> sched;

[[noreturn]] void die(const char* msg) {
    Lcd::clear();
    Lcd::print_P(PSTR("error:"));
    Lcd::setCursor(0, 1);
    Lcd::print(msg);
    UART::print("fatal: ");
    UART::println(msg);
    UART::flush();
    for (;;) HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
}

int main() {
    HAL::Ticker::setupMSTimer();
    HAL::Analog::begin();
    UART::init<9600>();
    Led::setOutput();
    sei();

    UART::println_P(PSTR("weather-logger boot"));
    Lcd::begin();
    Lcd::print_P(PSTR("starting..."));

    if (Sd::begin() != HAL::Devices::SD::Result::OK) die("no SD card");
    if (Fat::mount() != HAL::FS::Result::OK)         die("no FAT32");

    Fat::FileInfo info;
    if (Fat::find("LOG.CSV", info) != HAL::FS::Result::OK) {
        die("no LOG.CSV");     // see header: prepare the card first
    }
    switch (Fat::contiguousBlockRange(info, logFirst, logCount)) {
        case HAL::FS::Result::OK:         break;
        case HAL::FS::Result::FRAGMENTED: die("LOG fragmented");
        default:                          die("card i/o");
    }

    // resume: blocks fill in order and the file was pre-filled with
    // spaces, so binary-search the first still-blank block
    {
        uint32_t lo { logFirst };
        uint32_t hi { logFirst + logCount };
        uint8_t  b;
        while (lo < hi) {
            uint32_t mid { lo + (hi - lo) / 2 };
            if (Sd::readPartial(mid, 0, &b, 1)
                    != HAL::Devices::SD::Result::OK) {
                die("card i/o");
            }
            if (b == ' ') hi = mid; else lo = mid + 1;
        }
        if (lo >= logFirst + logCount) die("LOG.CSV full");
        curBlock = lo;
        recCount = (lo - logFirst) * RECS_PER_BLOCK;
    }
    blankBlock();
    loggingP = true;

    bool staleP { true };
    if (Rtc::oscStopped(staleP) != HAL::Comms::I2C::Result::OK) {
        die("no DS3231");
    }
    if (staleP) {
        // records without honest timestamps are worse than no
        // records: set the clock (e.g. with the alarm-clock example)
        die("RTC unset");
    }

    UART::print("resuming at record ");
    UART::println(recCount);

    sched.add(&sampleTask,  SAMPLE_MS);
    sched.add(&displayTask, DISPLAY_MS);
    sched.add(&logTask,     LOG_MS);
    sched.add(&blinkTask,   BLINK_MS);

    // THE TICK-LESS LOOP: sleep exactly as long as the schedule
    // allows. Long gaps become PWR_DOWN naps -- sleepFor pauses the
    // ticker and credits the nap back, so the timeline sails through
    while (1) {
        uint32_t wait { sched.run() };
        if (wait >= 300) {
            HAL::Watchdog::sleepFor<HAL::Watchdog::Timeout::MS256>(1);
        } else {
            HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
        }
    }
}
