#pragma once

#include <stdint.h>

/**
 * ID3 tag reading -- enough metadata to put a title on a 16-char LCD,
 * and not one frame more.
 *
 * readTags() fills the caller's title/artist buffers and leaves the
 * file positioned at the FIRST AUDIO BYTE, so it replaces both the
 * "what's this track called" question and the tag-skip that makes
 * tracks start instantly (modern rips carry 50-500 KB of embedded
 * album art up front; at a ~40 KB/s pump that's seconds of silence
 * unless you hop it). One call, both jobs.
 *
 * The lookup order is v2 first, v1 fallback, PER FIELD: an ID3v2 tag
 * carrying only a title still gets its artist from the ID3v1 record
 * at the end of the file, if one exists. The caller's fallback for
 * "neither" is the filename -- readTags returns false and both
 * buffers hold "".
 *
 * What it reads, and what it deliberately does not:
 *   - ID3v2.3 and v2.4: the TIT2 (title) and TPE1 (artist) text
 *     frames only. v2.3 frame sizes are plain big-endian 32-bit;
 *     v2.4's are syncsafe (7 bits per byte) -- the version byte in
 *     the header picks the decoder. Frames are walked by absolute
 *     seek, so a 500 KB APIC (album art) frame sitting before the
 *     text frames costs one seek, not a half-megabyte read.
 *   - Text encodings: ISO-8859-1 and UTF-8 pass through byte-for-
 *     byte (ASCII is clean everywhere; other bytes are the display's
 *     problem, which for an HD44780 means mojibake for accents --
 *     truncation and honesty over transliteration tables). UTF-16
 *     (BOM'd LE/BE, and the BOM-less BE of encoding 2) keeps the
 *     low byte of Latin-1 code points and shows '?' for the rest.
 *   - ID3v1: the fixed 128-byte record at file end ("TAG" magic,
 *     title at +3, artist at +33, 30 bytes each, space- or
 *     NUL-padded).
 *   - NOT read: v2.2 (ancient, 3-byte frame IDs), unsynchronised
 *     tags, compressed/encrypted frames. All are detected and
 *     SKIPPED CORRECTLY -- the audio-start position is still right,
 *     and the v1 fallback still runs -- they just contribute no
 *     text. Rare enough that decoding them isn't worth the flash.
 *
 * `Fs` is any filesystem type shaped like HAL::FS::FAT32: a File
 * with a public `size`, plus static read(File&, uint8_t*, uint16_t,
 * uint16_t&) and seek(File&, uint32_t), each returning an enum with
 * an OK member. That shape (and <stdint.h> as the only include)
 * keeps this host-testable against an in-memory mock filesystem --
 * see tests/id3_test.cpp, which feeds it synthetic tags of every
 * flavor. If you touch this file, run that.
 */

namespace HAL {
namespace Utils {

template<typename Fs>
struct ID3 {

    using File = typename Fs::File;

  private:
    static bool okSeek(File& f, uint32_t pos) {
        auto r { Fs::seek(f, pos) };
        return r == decltype(r)::OK;
    }

    static bool readN(File& f, uint8_t* dst, uint16_t n) {
        uint16_t got;
        auto r { Fs::read(f, dst, n, got) };
        return r == decltype(r)::OK && got == n;
    }

    // 28 effective bits, 7 per byte, high bits zero -- sized so the
    // byte stream can never contain a false MPEG frame-sync
    static uint32_t syncsafe(const uint8_t* p) {
        return (static_cast<uint32_t>(p[0] & 0x7F) << 21)
             | (static_cast<uint32_t>(p[1] & 0x7F) << 14)
             | (static_cast<uint32_t>(p[2] & 0x7F) << 7)
             |  static_cast<uint32_t>(p[3] & 0x7F);
    }

    static uint32_t be32(const uint8_t* p) {
        return (static_cast<uint32_t>(p[0]) << 24)
             | (static_cast<uint32_t>(p[1]) << 16)
             | (static_cast<uint32_t>(p[2]) << 8)
             |  static_cast<uint32_t>(p[3]);
    }

    static bool frameIs(const uint8_t* fh,
                        char a, char b, char c, char d) {
        return fh[0] == a && fh[1] == b && fh[2] == c && fh[3] == d;
    }

    // decode one text frame's payload (encoding byte + text) into
    // out[cap]. One bulk read into a stack buffer: 80 bytes covers
    // 79 ISO/UTF-8 chars or ~38 UTF-16 code units -- far past any
    // cap a character LCD implies. Frames larger than that are
    // simply truncated; the caller re-seeks by absolute position,
    // so where this leaves the file pointer doesn't matter
    static void readText(File& f, uint32_t dataSize,
                         char* out, uint8_t cap) {
        out[0] = '\0';
        if (cap < 2 || dataSize < 2) return;
        uint8_t  raw[80];
        uint16_t want { dataSize < sizeof(raw)
                            ? static_cast<uint16_t>(dataSize)
                            : static_cast<uint16_t>(sizeof(raw)) };
        uint16_t got;
        auto r { Fs::read(f, raw, want, got) };
        if (r != decltype(r)::OK || got < 2) return;

        uint8_t        enc  { raw[0] };
        const uint8_t* p    { raw + 1 };
        uint16_t       left { static_cast<uint16_t>(got - 1) };
        uint8_t        o    { 0 };

        if (enc == 0 || enc == 3) {          // ISO-8859-1 / UTF-8
            while (left > 0 && o + 1U < cap) {
                uint8_t c { *p++ };
                --left;
                if (c == '\0') break;
                out[o++] = static_cast<char>(c);
            }
        } else if (enc == 1 || enc == 2) {   // UTF-16 (1: BOM, 2: BE)
            bool beP { enc == 2 };
            if (enc == 1) {
                if (left < 2) return;
                beP = (p[0] == 0xFE && p[1] == 0xFF);
                p    += 2;
                left  = static_cast<uint16_t>(left - 2);
            }
            while (left >= 2 && o + 1U < cap) {
                uint8_t hi { beP ? p[0] : p[1] };
                uint8_t lo { beP ? p[1] : p[0] };
                p    += 2;
                left  = static_cast<uint16_t>(left - 2);
                if (hi == 0 && lo == 0) break;
                out[o++] = (hi == 0) ? static_cast<char>(lo) : '?';
            }
        }
        // taggers pad with trailing spaces often enough to trim
        while (o > 0 && out[o - 1] == ' ') --o;
        out[o] = '\0';
    }

    // one 30-byte ID3v1 field: trim the space/NUL padding from the
    // right, then copy what fits
    static void v1Field(const uint8_t* src, char* out, uint8_t cap) {
        uint8_t n { 30 };
        while (n > 0 && (src[n - 1] == ' ' || src[n - 1] == '\0')) {
            --n;
        }
        uint8_t o { 0 };
        while (o < n && o + 1U < cap) {
            out[o] = static_cast<char>(src[o]);
            ++o;
        }
        out[o] = '\0';
    }

  public:
    // Fills title/artist (NUL-terminated; "" where no tag supplied
    // one) and leaves `f` positioned at the first audio byte -- call
    // it right after open(), then pump. `cap` is each buffer's full
    // size including the NUL. Returns true if either field was found
    static bool readTags(File& f, char* title, char* artist,
                         uint8_t cap) {
        if (cap == 0) return false;
        title[0]  = '\0';
        artist[0] = '\0';

        uint32_t audioStart { 0 };

        uint8_t h[10];
        if (okSeek(f, 0) && readN(f, h, 10)
                && h[0] == 'I' && h[1] == 'D' && h[2] == '3') {
            uint32_t size { syncsafe(h + 6) };
            audioStart = 10 + size + ((h[5] & 0x10) ? 10UL : 0UL);

            uint8_t ver      { h[3] };
            bool    unsyncP  { (h[5] & 0x80) != 0 };
            if ((ver == 3 || ver == 4) && !unsyncP) {
                uint32_t tagEnd { 10 + size };
                uint32_t pos    { 10 };
                if (h[5] & 0x40) {           // extended header: hop it.
                    uint8_t e[4];            // v2.4's size counts itself
                    if (readN(f, e, 4)) {    // (syncsafe); v2.3's doesn't
                        pos = (ver == 4) ? 10 + syncsafe(e)
                                         : 10 + 4 + be32(e);
                    } else {
                        pos = tagEnd;
                    }
                }
                // walk frames by absolute position; the count bound
                // keeps a corrupt tag from becoming an long stall
                uint8_t frames { 0 };
                while (pos + 10 <= tagEnd && frames < 64
                        && (title[0] == '\0' || artist[0] == '\0')) {
                    if (!okSeek(f, pos)) break;
                    uint8_t fh[10];
                    if (!readN(f, fh, 10)) break;
                    if (fh[0] == 0) break;   // padding: no more frames
                    uint32_t fsize { (ver == 4) ? syncsafe(fh + 4)
                                                : be32(fh + 4) };
                    if (fsize == 0 || fsize > tagEnd - pos - 10) break;
                    // fh[9] is the format flags byte: nonzero means
                    // compressed/encrypted/grouped/unsynced -- opaque
                    // to us, so such a frame is skipped, not decoded
                    if (fh[9] == 0) {
                        if (title[0] == '\0'
                                && frameIs(fh, 'T', 'I', 'T', '2')) {
                            readText(f, fsize, title, cap);
                        } else if (artist[0] == '\0'
                                && frameIs(fh, 'T', 'P', 'E', '1')) {
                            readText(f, fsize, artist, cap);
                        }
                    }
                    pos += 10 + fsize;
                    frames = static_cast<uint8_t>(frames + 1);
                }
            }
        }

        // v1 fallback, per field, for whatever v2 didn't supply.
        // "TAG" + 30-byte title + 30-byte artist = the record's
        // first 63 bytes, one read
        if ((title[0] == '\0' || artist[0] == '\0') && f.size >= 128
                && okSeek(f, f.size - 128)) {
            uint8_t t[63];
            if (readN(f, t, 63)
                    && t[0] == 'T' && t[1] == 'A' && t[2] == 'G') {
                if (title[0]  == '\0') v1Field(t + 3,  title,  cap);
                if (artist[0] == '\0') v1Field(t + 33, artist, cap);
            }
        }

        // a tag that claims more bytes than the file has loses the
        // argument: play from 0 rather than refuse
        if (!okSeek(f, audioStart)) okSeek(f, 0);
        return title[0] != '\0' || artist[0] != '\0';
    }

};

}
}
