#pragma once

#include "common.hpp"

#include <stdint.h>

#include "../gpio.hpp"
#include "../comms/spi.hpp"

/**
 * SD card BLOCK driver -- deliberately no filesystem.
 *
 * At this layer an SD card is a huge array of numbered 512-byte blocks:
 * a multi-gigabyte EEPROM. That is enough for logging (write records
 * into sequential blocks, read them back with a card reader and a few
 * lines of host-side script) and it is the substrate a FAT library
 * would sit on if computer-mountable files are ever wanted. FAT is a
 * separate, later decision -- notably because a real FAT needs a
 * 512-byte sector buffer, which the tinies (512 B of RAM, total)
 * cannot ever provide. This driver needs NO block-sized buffer:
 * reads can address a window within a block, and writes can stream
 * from a byte-source callback.
 *
 * Protocol notes (SD Physical Layer spec, SPI mode):
 *   - init runs at <= 400 kHz: CMD0 (software reset, valid CRC
 *     required), CMD8 (voltage check, distinguishes v2 from v1 cards),
 *     ACMD41 in a loop until the card leaves idle, CMD58 to learn
 *     whether addressing is by block (SDHC/SDXC) or by byte (SDSC).
 *     After init the bus shifts to `fastHz`.
 *   - reads/writes move 512-byte blocks framed by a 0xFE token; writes
 *     end with the card holding MISO low while it burns the flash
 *     ("busy"). CRC is off in SPI mode after init, so data CRCs are
 *     dummy bytes.
 *   - every wait is bounded (token, busy, ACMD41), so a missing or
 *     wedged card returns a Result instead of hanging the main loop --
 *     same philosophy as the I2C module.
 *
 * Hardware notes: SD cards are strictly 3.3 V parts -- from a 5 V AVR,
 * use a module with a level shifter (or run the whole system at
 * 3.3 V). Writes draw bursts of 30-100 mA; budget for it. Cheap
 * no-name cards are notoriously loose about the init handshake, so
 * bring up with a name-brand card first.
 *
 * Not implemented (yet, deliberately): multi-block transfers
 * (CMD18/CMD25) and MMC-era cards (CMD1 init).
 */

namespace HAL {
namespace Devices {
namespace SD {

enum class Result : uint8_t {
    OK,
    NO_CARD,          // CMD0 never answered: not inserted / not wired
    UNSUPPORTED,      // voltage-check echo failed, or a pre-SD MMC card
    TIMEOUT,          // a bounded wait ran out (init, token, or busy)
    CMD_ERROR,        // a command returned an unexpected R1
    READ_ERROR,       // the card sent an error token instead of data
    WRITE_ERROR,      // the card rejected the written block
    BAD_PARAMS,       // offset+count would run past the block
    NOT_INITIALIZED   // begin() has not succeeded
};

template<uint8_t csPin, uint32_t fastHz = 8000000>
struct Card {

    using CS      = HAL::GPIO::GPIO<csPin>;
    // two configurations of the same bus: the spec caps the init
    // handshake at 400 kHz; after it, fastHz applies (a ceiling, per
    // the SPI module's semantics)
    using SlowSpi = HAL::Comms::SPI::Master<400000>;
    using FastSpi = HAL::Comms::SPI::Master<fastHz>;

    // a write's byte source: called once per index 0..511, in order.
    // This is how a 512-byte block leaves a chip with 512 bytes of RAM
    using ByteSource = uint8_t (*)(uint16_t idx);

  private:
    static inline bool sdhcP  { false };
    static inline bool readyP { false };

    // wait bounds, in polled SPI bytes. One poll is 8/fastHz seconds,
    // so these are ~100 ms and ~500 ms at the ceiling -- and longer in
    // wall time if the bus runs slower, which errs the right way
    static constexpr uint32_t TOKEN_TRIES { fastHz / 80 };
    static constexpr uint32_t BUSY_TRIES  { fastHz / 16 };

    // SDHC/SDXC address by block number, SDSC by byte offset
    static uint32_t blockToAddr(uint32_t block) {
        return sdhcP ? block : (block << 9);
    }

    template<typename Spi>
    static void deselect() {
        CS::setHigh();
        Spi::write(0xFF);   // 8 more clocks so the card releases MISO
    }

    // R1 arrives within 8 bytes of a command (NCR); bit 7 of a
    // response is always 0, idle 0xFF has it set
    template<typename Spi>
    static uint8_t readR1() {
        for (uint8_t i = 0; i < 10; ++i) {
            uint8_t r { Spi::read() };
            if (!(r & 0x80)) return r;
        }
        return 0xFF;
    }

    // CRC matters only for CMD0 (0x95) and CMD8 (0x87): after those,
    // SPI mode runs CRC-off and 0x01 is a well-formed dummy
    template<typename Spi>
    static uint8_t command(uint8_t cmd, uint32_t arg, uint8_t crc) {
        Spi::write(0xFF);   // sync gap; some cards need it
        Spi::write(static_cast<uint8_t>(0x40 | cmd));
        Spi::write(static_cast<uint8_t>(arg >> 24));
        Spi::write(static_cast<uint8_t>(arg >> 16));
        Spi::write(static_cast<uint8_t>(arg >> 8));
        Spi::write(static_cast<uint8_t>(arg));
        Spi::write(crc);
        return readR1<Spi>();
    }

    // CMD24 preamble: address the block and open the data phase
    static Result writeStart(uint32_t block) {
        if (!readyP) return Result::NOT_INITIALIZED;
        CS::setLow();
        if (command<FastSpi>(24, blockToAddr(block), 0x01) != 0x00) {
            deselect<FastSpi>();
            return Result::CMD_ERROR;
        }
        FastSpi::write(0xFF);   // >= one byte gap before the token
        FastSpi::write(0xFE);   // single-block data token
        return Result::OK;
    }

    // dummy CRC, data-response check, then wait out the card's
    // internal flash programming (MISO held low while busy)
    static Result writeFinish() {
        FastSpi::write(0xFF);
        FastSpi::write(0xFF);
        if ((FastSpi::read() & 0x1F) != 0x05) {   // xxx00101 = accepted
            deselect<FastSpi>();
            return Result::WRITE_ERROR;
        }
        uint32_t tries { BUSY_TRIES };
        while (FastSpi::read() != 0xFF) {
            if (--tries == 0) {
                deselect<FastSpi>();
                return Result::TIMEOUT;
            }
        }
        deselect<FastSpi>();
        return Result::OK;
    }

  public:
    static Result begin() {
        readyP = false;
        CS::setHigh();
        CS::setOutput();
        SlowSpi::begin();

        // >= 74 clocks with CS high: the card's SPI-mode wake-up ritual
        for (uint8_t i = 0; i < 10; ++i) SlowSpi::write(0xFF);

        CS::setLow();

        // CMD0: software reset into idle state (R1 = 0x01)
        uint8_t r1 { 0xFF };
        for (uint8_t i = 0; i < 10 && r1 != 0x01; ++i) {
            r1 = command<SlowSpi>(0, 0, 0x95);
        }
        if (r1 != 0x01) {
            deselect<SlowSpi>();
            return Result::NO_CARD;
        }

        // CMD8: v2 cards echo the check pattern; v1 cards call it
        // an illegal command (R1 bit 2)
        bool v2 { false };
        r1 = command<SlowSpi>(8, 0x000001AA, 0x87);
        if (!(r1 & 0x04)) {
            uint8_t r7[4];
            for (uint8_t i = 0; i < 4; ++i) r7[i] = SlowSpi::read();
            if (r7[2] != 0x01 || r7[3] != 0xAA) {
                deselect<SlowSpi>();
                return Result::UNSUPPORTED;   // voltage range mismatch
            }
            v2 = true;
        }

        // ACMD41 until the card finishes its own power-up (R1 0x01 ->
        // 0x00). HCS bit advertises SDHC support to v2 cards. ~1 s
        // worth of attempts at 400 kHz
        uint16_t attempts { 5000 };
        for (;;) {
            command<SlowSpi>(55, 0, 0x01);   // "next command is app-specific"
            r1 = command<SlowSpi>(41, v2 ? 0x40000000UL : 0, 0x01);
            if (r1 == 0x00) break;
            if (r1 & 0x04) {
                deselect<SlowSpi>();
                return Result::UNSUPPORTED;   // ACMD41 illegal: MMC era
            }
            if (--attempts == 0) {
                deselect<SlowSpi>();
                return Result::TIMEOUT;
            }
        }

        // CMD58: the OCR's CCS bit decides block vs byte addressing
        sdhcP = false;
        if (v2) {
            if (command<SlowSpi>(58, 0, 0x01) != 0x00) {
                deselect<SlowSpi>();
                return Result::CMD_ERROR;
            }
            uint8_t ocr[4];
            for (uint8_t i = 0; i < 4; ++i) ocr[i] = SlowSpi::read();
            sdhcP = (ocr[0] & 0x40) != 0;
        }

        // byte-addressed cards: pin the block size to 512 (the default,
        // but belt and braces)
        if (!sdhcP) {
            if (command<SlowSpi>(16, 512, 0x01) != 0x00) {
                deselect<SlowSpi>();
                return Result::CMD_ERROR;
            }
        }

        deselect<SlowSpi>();
        FastSpi::begin();   // handshake done: shift the bus up
        readyP = true;
        return Result::OK;
    }

    static bool isSDHC() { return sdhcP; }

    // read `count` bytes starting `offset` into a block -- the whole
    // block still crosses the wire (the protocol insists), but only
    // the window lands in RAM. readBlock() is the offset=0/count=512
    // special case
    static Result readPartial(uint32_t block, uint16_t offset,
                              uint8_t* dst, uint16_t count) {
        if (!readyP) return Result::NOT_INITIALIZED;
        if (static_cast<uint32_t>(offset) + count > 512) {
            return Result::BAD_PARAMS;
        }
        CS::setLow();
        if (command<FastSpi>(17, blockToAddr(block), 0x01) != 0x00) {
            deselect<FastSpi>();
            return Result::CMD_ERROR;
        }
        // the card answers with 0xFF until the data token (0xFE);
        // anything else is an error token
        uint32_t tries { TOKEN_TRIES };
        uint8_t  tok;
        while ((tok = FastSpi::read()) == 0xFF) {
            if (--tries == 0) {
                deselect<FastSpi>();
                return Result::TIMEOUT;
            }
        }
        if (tok != 0xFE) {
            deselect<FastSpi>();
            return Result::READ_ERROR;
        }
        uint16_t i { 0 };
        for (; i < offset; ++i)          FastSpi::write(0xFF);  // skip
        for (uint16_t c = 0; c < count; ++c) dst[c] = FastSpi::read();
        i = static_cast<uint16_t>(offset + count);
        for (; i < 514; ++i)             FastSpi::write(0xFF);  // rest + CRC
        deselect<FastSpi>();
        return Result::OK;
    }

    static Result readBlock(uint32_t block, uint8_t* dst512) {
        return readPartial(block, 0, dst512, 512);
    }

    static Result writeBlock(uint32_t block, const uint8_t* src512) {
        Result r { writeStart(block) };
        if (r != Result::OK) return r;
        for (uint16_t i = 0; i < 512; ++i) FastSpi::write(src512[i]);
        return writeFinish();
    }

    // the RAM-poor path: the source callback is asked for bytes
    // 0..511 in order, and nothing block-sized ever exists in memory
    static Result writeBlockStream(uint32_t block, ByteSource source) {
        Result r { writeStart(block) };
        if (r != Result::OK) return r;
        for (uint16_t i = 0; i < 512; ++i) FastSpi::write(source(i));
        return writeFinish();
    }

    // capacity in 512-byte blocks, from the CSD register (CMD9);
    // 0 on any failure. Handles both CSD layouts (v2 = SDHC/SDXC,
    // v1 = byte-addressed cards)
    static uint32_t numBlocks() {
        if (!readyP) return 0;
        CS::setLow();
        if (command<FastSpi>(9, 0, 0x01) != 0x00) {
            deselect<FastSpi>();
            return 0;
        }
        uint32_t tries { TOKEN_TRIES };
        uint8_t  tok;
        while ((tok = FastSpi::read()) == 0xFF) {
            if (--tries == 0) { deselect<FastSpi>(); return 0; }
        }
        if (tok != 0xFE) { deselect<FastSpi>(); return 0; }
        uint8_t csd[16];
        for (uint8_t i = 0; i < 16; ++i) csd[i] = FastSpi::read();
        FastSpi::write(0xFF);   // CRC
        FastSpi::write(0xFF);
        deselect<FastSpi>();

        if ((csd[0] >> 6) == 1) {
            // CSD v2: capacity = (C_SIZE + 1) * 512 KiB
            uint32_t cSize { (static_cast<uint32_t>(csd[7] & 0x3F) << 16)
                           | (static_cast<uint32_t>(csd[8]) << 8)
                           |  static_cast<uint32_t>(csd[9]) };
            return (cSize + 1) * 1024UL;
        }
        // CSD v1: capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^READ_BL_LEN
        uint32_t cSize { (static_cast<uint32_t>(csd[6] & 0x03) << 10)
                       | (static_cast<uint32_t>(csd[7]) << 2)
                       | (static_cast<uint32_t>(csd[8]) >> 6) };
        uint8_t cMult { static_cast<uint8_t>(
            ((csd[9] & 0x03) << 1) | (csd[10] >> 7)) };
        uint8_t readBlLen { static_cast<uint8_t>(csd[5] & 0x0F) };
        return (cSize + 1) << (cMult + 2 + readBlLen - 9);
    }

};

}
}
}
