#pragma once

#include "common.hpp"

#include <stdint.h>

#include "../gpio.hpp"
#include "../comms/spi.hpp"

/**
 * VS1053b audio codec: the chip that decodes MP3 (and Ogg, AAC, WAV,
 * MIDI...) so the AVR doesn't have to -- because it can't. The AVR's
 * job is data pump: read file bytes, shove them at the codec whenever
 * DREQ says "feed me". The codec has its own FIFO, so there is no hard
 * real-time anywhere in this driver.
 *
 * Wiring (all SPI pins shared with the rest of the bus):
 *   - XCS  (template `xcsPin`):  chip select for SCI, the 16-bit
 *     control-register channel (mode, clock, volume, status)
 *   - XDCS (template `xdcsPin`): chip select for SDI, the raw data
 *     channel the encoded audio streams into
 *   - DREQ (template `dreqPin`): the codec's "I can take 32 more
 *     bytes / one more command" output. Every operation here waits on
 *     it (bounded -- a dead chip is a Result, not a hang)
 *   - XRESET (template `rstPin`): hardware reset, pulsed by begin()
 *
 * Two speeds, one bus, same pattern as the SD driver: after reset the
 * codec runs directly off its 12.288 MHz crystal and SCI tops out
 * around XTALI/7 (~1.75 MHz), so begin() starts on a slow bus, writes
 * SCI_CLOCKF to multiply the internal clock to 3.0x XTALI, and only
 * then shifts to `fastHz` (default 4 MHz, safely under the multiplied
 * limits). Because other devices (the SD card) share the bus at other
 * speeds, every public operation re-asserts its own SPI configuration
 * first -- two register writes of insurance.
 *
 * Bring-up ritual: sineTestStart() makes a tone with NO card, NO file
 * and NO decoding -- if the sine plays, the wiring and SCI/SDI/DREQ
 * plumbing are all proven. Do that before debugging anything else.
 */

namespace HAL {
namespace Devices {
namespace VS1053 {

enum class Result : uint8_t {
    OK,
    TIMEOUT,          // DREQ never rose: dead chip, bad wiring, no power
    NO_CHIP,          // SCI_STATUS version field isn't a VS1053's
    NOT_INITIALIZED   // begin() has not succeeded
};

// SCI register map (datasheet section 9.6)
namespace reg {
constexpr uint8_t MODE        { 0x0 };
constexpr uint8_t STATUS      { 0x1 };
constexpr uint8_t CLOCKF      { 0x3 };
constexpr uint8_t DECODE_TIME { 0x4 };
constexpr uint8_t WRAM        { 0x6 };
constexpr uint8_t WRAMADDR    { 0x7 };
constexpr uint8_t HDAT0       { 0x8 };
constexpr uint8_t HDAT1      { 0x9 };
constexpr uint8_t VOL         { 0xB };
}

template<uint8_t  xcsPin,
         uint8_t  xdcsPin,
         uint8_t  dreqPin,
         uint8_t  rstPin,
         uint32_t fastHz = 4000000>
struct Codec {

    using XCS  = HAL::GPIO::GPIO<xcsPin>;
    using XDCS = HAL::GPIO::GPIO<xdcsPin>;
    using DREQ = HAL::GPIO::GPIO<dreqPin>;
    using RST  = HAL::GPIO::GPIO<rstPin>;

    // SCI at reset must stay under XTALI/7; after CLOCKF the ceiling
    // rises to CLKI/7 (~5.2 MHz at 3.0x), so 4 MHz default is safe
    using SlowSpi = HAL::Comms::SPI::Master<1000000>;
    using FastSpi = HAL::Comms::SPI::Master<fastHz>;

  private:
    static constexpr uint16_t SM_SDINEW { 1U << 11 };
    static constexpr uint16_t SM_TESTS  { 1U << 5 };
    static constexpr uint16_t SM_CANCEL { 1U << 3 };

    static inline bool readyP { false };
    static inline bool fastP  { false };

    // ~tens of ms of spins: DREQ excursions are normally well under a
    // millisecond, with end-of-track cancellation the slow outlier
    static constexpr uint32_t DREQ_SPINS { F_CPU / 50 };

    // the bus is shared: whoever ran last (the SD card, say) left its
    // own speed configured, so re-assert ours before touching the chip
    static void busUp() {
        if (fastP) FastSpi::begin(); else SlowSpi::begin();
    }

    static bool waitDREQ() {
        uint32_t spins { DREQ_SPINS };
        while (!DREQ::read()) {
            if (--spins == 0) return false;
        }
        return true;
    }

    template<typename Spi>
    static bool sciWriteRaw(uint8_t addr, uint16_t val) {
        if (!waitDREQ()) return false;
        XCS::setLow();
        Spi::write(0x02);   // SCI write opcode
        Spi::write(addr);
        Spi::write(static_cast<uint8_t>(val >> 8));
        Spi::write(static_cast<uint8_t>(val));
        XCS::setHigh();
        return true;
    }

    template<typename Spi>
    static bool sciReadRaw(uint8_t addr, uint16_t& val) {
        if (!waitDREQ()) return false;
        XCS::setLow();
        Spi::write(0x03);   // SCI read opcode
        Spi::write(addr);
        uint16_t hi { Spi::read() };
        val = static_cast<uint16_t>((hi << 8) | Spi::read());
        XCS::setHigh();
        return true;
    }

    // one SDI chunk: at most 32 bytes per DREQ-high, per the datasheet
    static bool sendChunk(const uint8_t* buf, uint8_t len) {
        if (!waitDREQ()) return false;
        XDCS::setLow();
        for (uint8_t i = 0; i < len; ++i) FastSpi::write(buf[i]);
        XDCS::setHigh();
        return true;
    }

    static bool sendSame(uint8_t b, uint16_t count) {
        while (count > 0) {
            uint8_t chunk { count > 32 ? static_cast<uint8_t>(32)
                                       : static_cast<uint8_t>(count) };
            if (!waitDREQ()) return false;
            XDCS::setLow();
            for (uint8_t i = 0; i < chunk; ++i) FastSpi::write(b);
            XDCS::setHigh();
            count = static_cast<uint16_t>(count - chunk);
        }
        return true;
    }

  public:
    static Result begin() {
        readyP = false;
        fastP  = false;

        XCS::setHigh();   XCS::setOutput();
        XDCS::setHigh();  XDCS::setOutput();
        DREQ::setInput();
        RST::setLow();    RST::setOutput();   // hold in reset...
        SlowSpi::begin();
        __builtin_avr_delay_cycles(F_CPU / 1000);   // ...for ~1 ms
        RST::setHigh();

        if (!waitDREQ()) return Result::TIMEOUT;   // chip never woke

        // SS_VER lives in STATUS bits 7:4; the VS1053 reports 4.
        // (Relatives report their own: VS1003 -> 3, VS1063 -> 6.)
        uint16_t status;
        if (!sciReadRaw<SlowSpi>(reg::STATUS, status)) {
            return Result::TIMEOUT;
        }
        if (((status >> 4) & 0x0F) != 4) return Result::NO_CHIP;

        // 3.0x clock multiplier; every SCI/SDI ceiling scales with it
        if (!sciWriteRaw<SlowSpi>(reg::CLOCKF, 0x6000)) {
            return Result::TIMEOUT;
        }
        if (!waitDREQ()) return Result::TIMEOUT;   // multiplier settling

        fastP = true;
        FastSpi::begin();
        readyP = true;
        setVolume(0x28, 0x28);   // -20 dB: audible, not alarming
        return Result::OK;
    }

    // attenuation in 0.5 dB steps per channel: 0x00 = full blast,
    // 0xFE = practically silent
    static Result setVolume(uint8_t leftAtt, uint8_t rightAtt) {
        if (!readyP) return Result::NOT_INITIALIZED;
        busUp();
        return sciWriteRaw<FastSpi>(
                   reg::VOL,
                   static_cast<uint16_t>((static_cast<uint16_t>(leftAtt) << 8)
                                         | rightAtt))
            ? Result::OK : Result::TIMEOUT;
    }

    // the data pump's two halves: poll readyForData() from the main
    // loop, and when it's true, sendData() the next file bytes. Chunks
    // of 32 pair naturally with FAT32 read()s into a small buffer
    static bool readyForData() { return DREQ::read(); }

    static Result sendData(const uint8_t* buf, uint16_t len) {
        if (!readyP) return Result::NOT_INITIALIZED;
        busUp();
        while (len > 0) {
            uint8_t chunk { len > 32 ? static_cast<uint8_t>(32)
                                     : static_cast<uint8_t>(len) };
            if (!sendChunk(buf, chunk)) return Result::TIMEOUT;
            buf += chunk;
            len  = static_cast<uint16_t>(len - chunk);
        }
        return Result::OK;
    }

    // clean end-of-track, per datasheet 10.5.1: flush the decoder with
    // 2052 of its designated end-fill byte, request cancel, and keep
    // feeding fill until the chip acknowledges. After this the codec
    // is ready for the next track with no soft reset
    static Result stopTrack() {
        if (!readyP) return Result::NOT_INITIALIZED;
        busUp();
        uint16_t ef;
        if (!sciWriteRaw<FastSpi>(reg::WRAMADDR, 0x1E06) ||
            !sciReadRaw<FastSpi>(reg::WRAM, ef)) {
            return Result::TIMEOUT;
        }
        uint8_t fill { static_cast<uint8_t>(ef) };
        if (!sendSame(fill, 2052)) return Result::TIMEOUT;
        if (!sciWriteRaw<FastSpi>(reg::MODE, SM_SDINEW | SM_CANCEL)) {
            return Result::TIMEOUT;
        }
        for (uint8_t tries = 0; tries < 64; ++tries) {   // <= 2048 bytes
            if (!sendSame(fill, 32)) return Result::TIMEOUT;
            uint16_t mode;
            if (!sciReadRaw<FastSpi>(reg::MODE, mode)) {
                return Result::TIMEOUT;
            }
            if (!(mode & SM_CANCEL)) return Result::OK;
        }
        return Result::TIMEOUT;
    }

    // seconds since decoding started -- cheap fuel for a UI
    static uint16_t decodeTime() {
        if (!readyP) return 0;
        busUp();
        uint16_t t { 0 };
        sciReadRaw<FastSpi>(reg::DECODE_TIME, t);
        return t;
    }

    // format detection: 0 = nothing decoding; otherwise a magic value
    // per format (0xFFEx = MP3). Useful to confirm the pump works
    static uint16_t hdat1() {
        if (!readyP) return 0;
        busUp();
        uint16_t h { 0 };
        sciReadRaw<FastSpi>(reg::HDAT1, h);
        return h;
    }

    // THE BRING-UP TOOL: a pure tone with no card, no file and no
    // decoding involved. If this plays, XCS/XDCS/DREQ/SPI wiring is
    // all proven. `n` picks the pitch (datasheet 10.12.1; 0x44 is a
    // pleasant ~1 kHz at 44.1 kHz output)
    static Result sineTestStart(uint8_t n = 0x44) {
        if (!readyP) return Result::NOT_INITIALIZED;
        busUp();
        if (!sciWriteRaw<FastSpi>(reg::MODE, SM_SDINEW | SM_TESTS)) {
            return Result::TIMEOUT;
        }
        const uint8_t seq[8] { 0x53, 0xEF, 0x6E, n, 0, 0, 0, 0 };
        return sendChunk(seq, 8) ? Result::OK : Result::TIMEOUT;
    }

    static Result sineTestStop() {
        if (!readyP) return Result::NOT_INITIALIZED;
        busUp();
        const uint8_t seq[8] { 0x45, 0x78, 0x69, 0x74, 0, 0, 0, 0 };
        if (!sendChunk(seq, 8)) return Result::TIMEOUT;
        return sciWriteRaw<FastSpi>(reg::MODE, SM_SDINEW)
            ? Result::OK : Result::TIMEOUT;
    }

};

}
}
}
