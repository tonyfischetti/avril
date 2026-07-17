/**
 * mp3-player: a complete SD-card jukebox.
 *
 * What it does:
 *   - mounts a FAT32 card and plays every .MP3 in the root directory,
 *     auto-advancing at each track's end (wrapping at the last);
 *   - rotate the knob for previous/next track;
 *   - press to pause/resume (the codec just stops being fed; its FIFO
 *     picks up seamlessly on resume);
 *   - rotate WHILE pressed for volume;
 *   - LCD shows the filename and play/pause state on row 0; track
 *     number, decode time (from the codec itself), and volume on
 *     row 1;
 *   - every bring-up failure is a message on the LCD, not a blank
 *     stare: "no SD card", "no FAT32", "no VS1053", "no MP3 files".
 *
 * The architecture is the "AVR as data pump" story from the README:
 * FAT32 read()s land in a 512-byte RAM buffer (block-aligned, zero
 * wire overhead), and the codec is fed <=32-byte bites whenever DREQ
 * allows -- up to 16 per loop pass, which comfortably outruns 320 kbps
 * while the main loop stays responsive for the knob and display.
 *
 * Parts: ATmega328P @ 16 MHz, microSD module (level-shifted), VS1053b
 * breakout, LCD1602 + PCF8574 backpack, KY-040-style encoder with
 * push button. I2C wants 4.7k pull-ups (the backpack usually has
 * them). The card: FAT32, .MP3 files in the root, 8.3 names.
 *
 * Wiring (physical DIP-28 pins):
 *   SPI: MOSI PB3 (17), MISO PB4 (18), SCK PB5 (19)   shared bus
 *   SD CS         -> PB2 (16)   (the SS pin; begin() handles the trap)
 *   VS1053 XCS    -> PC1 (24)
 *   VS1053 XDCS   -> PC2 (25)
 *   VS1053 DREQ   -> PC3 (26)
 *   VS1053 XRESET -> PD4 (6)
 *   SDA           -> PC4 (27),  SCL -> PC5 (28)   (LCD backpack)
 *   encoder CLK   -> PD2 (4)
 *   encoder DT    -> PD3 (5)
 *   encoder SW    -> PD5 (11)
 *   UART TX       -> PD1 (3), optional: 9600 baud debug chatter
 *
 * Exercises: Comms::SPI, Comms::I2C, Comms::UART, Devices::SD,
 * FS::FAT32, Devices::VS1053, Devices::LCD1602,
 * Devices::RotaryEncoderWithButton, Ticker, Sleep.
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avril.hpp"
#include "devices/RotaryEncoderWithButton.hpp"
#include "devices/SDCard.hpp"
#include "devices/VS1053.hpp"
#include "devices/LCD1602.hpp"
#include "fs/fat32.hpp"

namespace UART = HAL::Comms::UART;
namespace VS   = HAL::Devices::VS1053;

using I2c = HAL::Comms::I2C::Master<100000>;
using Lcd = HAL::Devices::LCD1602<I2c>;
using Sd  = HAL::Devices::SD::Card<16, 8000000>;
using Fat = HAL::FS::FAT32<Sd>;
using Mp3 = VS::Codec<24, 25, 26, 6>;

HAL::Devices::RotaryEncoderWithButton<11, 30, 1000, HIGH, true,  // button
                                      4,  5,  true> knob;        // clk, dt

volatile uint8_t previousPIND { 0xFF };

ISR(PCINT2_vect) {
    uint32_t now { HAL::Ticker::getNumTicks() };
    uint8_t  cur { PIND };
    uint8_t  ch  { static_cast<uint8_t>(cur ^ previousPIND) };
    previousPIND = cur;
    knob.notifyInterruptOccurred(now, HAL::GPIO::Port::D, ch);
}

// ---- player state (file scope: callbacks are plain fn pointers) ----

uint8_t       blockBuf[512];      // the pump's block-aligned buffer
uint16_t      bufLen  { 0 };
uint16_t      bufUsed { 0 };
Fat::File     song;
Fat::FileInfo songInfo;
uint8_t       curTrack  { 0 };
uint8_t       numTracks { 0 };
bool          playingP  { false };
bool          pausedP   { false };
uint8_t       volAtt    { 0x30 };  // attenuation: smaller = louder

// requests from the knob callbacks, acted on in the main loop so
// track switching never happens in the middle of a pump pass
int8_t        trackDelta      { 0 };
bool          togglePauseReq  { false };
bool          dirtyP          { true };

void onCW()        { trackDelta     = static_cast<int8_t>(trackDelta + 1); }
void onCCW()       { trackDelta     = static_cast<int8_t>(trackDelta - 1); }
void onPress()     { togglePauseReq = true; }
void onPressedCW() {
    if (volAtt >= 4) volAtt = static_cast<uint8_t>(volAtt - 4);  // louder
    Mp3::setVolume(volAtt, volAtt);
    dirtyP = true;
}
void onPressedCCW() {
    if (volAtt <= 0x7A) volAtt = static_cast<uint8_t>(volAtt + 4);
    Mp3::setVolume(volAtt, volAtt);
    dirtyP = true;
}

// ---- playlist: the root directory itself, filtered to .MP3 --------

bool isMp3(const Fat::FileInfo& fi) {
    if (fi.dirP) return false;
    uint8_t len { 0 };
    while (fi.name[len] != '\0') ++len;
    return len > 4
        && fi.name[len - 4] == '.'
        && fi.name[len - 3] == 'M'
        && fi.name[len - 2] == 'P'
        && fi.name[len - 1] == '3';
}

// no track table in RAM: to find track N, walk the directory again.
// Iteration is a handful of windowed reads -- cheap enough per switch
bool trackByIndex(uint8_t idx, Fat::FileInfo& out) {
    Fat::DirIter it { Fat::root() };
    uint8_t n { 0 };
    while (Fat::next(it, out) == HAL::FS::Result::OK) {
        if (isMp3(out)) {
            if (n == idx) return true;
            ++n;
        }
    }
    return false;
}

uint8_t countTracks() {
    Fat::DirIter  it { Fat::root() };
    Fat::FileInfo fi;
    uint8_t n { 0 };
    while (Fat::next(it, fi) == HAL::FS::Result::OK) {
        if (isMp3(fi) && n < 255) ++n;
    }
    return n;
}

// ---- display ------------------------------------------------------

void print2(uint8_t v) {
    Lcd::write(static_cast<char>('0' + (v / 10) % 10));
    Lcd::write(static_cast<char>('0' + v % 10));
}

void repaint() {
    Lcd::setCursor(0, 0);
    Lcd::write(pausedP ? static_cast<char>(1) : static_cast<char>(0));
    Lcd::write(' ');
    uint8_t i { 0 };
    while (songInfo.name[i] != '\0') { Lcd::write(songInfo.name[i]); ++i; }
    while (i < 14) { Lcd::write(' '); ++i; }

    Lcd::setCursor(0, 1);
    print2(static_cast<uint8_t>(curTrack + 1));
    Lcd::write('/');
    print2(numTracks);
    Lcd::write(' ');
    uint16_t t { Mp3::decodeTime() };
    print2(static_cast<uint8_t>(t / 60));
    Lcd::write(':');
    print2(static_cast<uint8_t>(t % 60));
    Lcd::print_P(PSTR(" v"));
    print2(static_cast<uint8_t>((0x7E - volAtt) / 4));  // 0..31-ish
}

// bring-up failures become words, then a halt: an appliance should
// say what's wrong
void die(const char* msg) {
    Lcd::clear();
    Lcd::print_P(PSTR("error:"));
    Lcd::setCursor(0, 1);
    Lcd::print(msg);
    UART::print("fatal: ");
    UART::println(msg);
    UART::flush();
    for (;;) HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
}

// ---- playback -----------------------------------------------------

void startTrack(uint8_t idx) {
    if (!trackByIndex(idx, songInfo)) return;
    Fat::open(songInfo, song);
    bufLen = bufUsed = 0;
    playingP = true;
    pausedP  = false;
    dirtyP   = true;
    UART::print("playing: ");
    UART::println(songInfo.name);
}

// feed the codec up to 16 bites per pass; DREQ self-limits. Returns
// having set trackDelta on end-of-track so the switch happens in the
// main loop like every other one
void pumpAudio() {
    if (!playingP || pausedP) return;
    for (uint8_t burst = 0; burst < 16; ++burst) {
        if (bufUsed == bufLen) {
            uint16_t got { 0 };
            if (Fat::read(song, blockBuf, sizeof(blockBuf), got)
                    != HAL::FS::Result::OK || got == 0) {
                Mp3::stopTrack();      // clean end, no reset needed
                playingP   = false;
                trackDelta = static_cast<int8_t>(trackDelta + 1);
                return;                // auto-advance
            }
            bufLen  = got;
            bufUsed = 0;
        }
        if (!Mp3::readyForData()) return;
        uint16_t chunk { static_cast<uint16_t>(bufLen - bufUsed) };
        if (chunk > 32) chunk = 32;
        if (Mp3::sendData(blockBuf + bufUsed, chunk) != VS::Result::OK) {
            playingP = false;
            return;
        }
        bufUsed = static_cast<uint16_t>(bufUsed + chunk);
    }
}

int main() {
    HAL::Ticker::setupMSTimer();
    UART::init<9600>();
    knob.begin();
    sei();

    UART::println_P(PSTR("mp3-player boot"));

    if (Lcd::begin() != HAL::Comms::I2C::Result::OK) {
        // no display to complain on; the UART is all we have
        UART::println_P(PSTR("fatal: no LCD (addr 0x27? try 0x3F)"));
    }
    static const uint8_t play[8]  { 0x08, 0x0C, 0x0E, 0x0F,
                                    0x0E, 0x0C, 0x08, 0x00 };
    static const uint8_t pause[8] { 0x1B, 0x1B, 0x1B, 0x1B,
                                    0x1B, 0x1B, 0x1B, 0x00 };
    Lcd::createChar(0, play);
    Lcd::createChar(1, pause);
    Lcd::clear();
    Lcd::print_P(PSTR("starting..."));

    if (Sd::begin()  != HAL::Devices::SD::Result::OK) die("no SD card");
    if (Fat::mount() != HAL::FS::Result::OK)          die("no FAT32");
    if (Mp3::begin() != VS::Result::OK)               die("no VS1053");
    Mp3::setVolume(volAtt, volAtt);

    numTracks = countTracks();
    if (numTracks == 0) die("no MP3 files");
    UART::print("tracks: ");
    UART::println(static_cast<uint32_t>(numTracks));

    knob.setOnCW(&onCW);
    knob.setOnCCW(&onCCW);
    knob.setOnPress(&onPress);
    knob.setOnPressedCW(&onPressedCW);
    knob.setOnPressedCCW(&onPressedCCW);

    startTrack(0);

    uint32_t lastClock { 0 };

    while (1) {
        knob.process();

        // act on knob requests here, never mid-pump
        if (trackDelta != 0) {
            if (playingP) Mp3::stopTrack();
            int16_t t { static_cast<int16_t>(curTrack + trackDelta) };
            trackDelta = 0;
            while (t < 0) t = static_cast<int16_t>(t + numTracks);
            curTrack = static_cast<uint8_t>(t % numTracks);
            startTrack(curTrack);
        }
        if (togglePauseReq) {
            togglePauseReq = false;
            if (playingP) {
                pausedP = !pausedP;
                dirtyP  = true;
            }
        }

        pumpAudio();

        // once a second: refresh the decode-time display. A few ms of
        // LCD traffic is nothing against the codec's ~50 ms of FIFO
        uint32_t ticks { HAL::Ticker::getNumTicks() };
        if (playingP && !pausedP && ticks - lastClock >= 1000) {
            lastClock = ticks;
            dirtyP = true;
        }

        if (dirtyP) {
            dirtyP = false;
            repaint();
        }

        HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
    }
}
