// Host-side test for include/utils/ID3.hpp -- the parser is templated
// on a filesystem SHAPE, not on the real FAT32 driver, so a ~20-line
// in-memory mock lets the host build synthetic tags of every flavor
// and check the parse. Build and run:
//
//     c++ -std=c++17 -Wall -Wextra -I../include -o id3_test \
//         id3_test.cpp && ./id3_test

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "utils/ID3.hpp"

// ---- the mock filesystem: a byte vector with a read/seek face ------

struct MemFs {
    struct File {
        const uint8_t* data;
        uint32_t       size;
        uint32_t       pos;
    };
    enum class Result : uint8_t { OK, BAD_PARAMS };

    static Result read(File& f, uint8_t* dst, uint16_t want,
                       uint16_t& got) {
        got = 0;
        while (want > 0 && f.pos < f.size) {
            dst[got++] = f.data[f.pos++];
            --want;
        }
        return Result::OK;
    }
    static Result seek(File& f, uint32_t target) {
        if (target > f.size) return Result::BAD_PARAMS;
        f.pos = target;
        return Result::OK;
    }
};

using Id3 = HAL::Utils::ID3<MemFs>;
using Bytes = std::vector<uint8_t>;

// ---- synthetic-tag builders ----------------------------------------

static void putSyncsafe(Bytes& v, uint32_t n) {
    v.push_back(static_cast<uint8_t>((n >> 21) & 0x7F));
    v.push_back(static_cast<uint8_t>((n >> 14) & 0x7F));
    v.push_back(static_cast<uint8_t>((n >> 7)  & 0x7F));
    v.push_back(static_cast<uint8_t>(n & 0x7F));
}

static void putBe32(Bytes& v, uint32_t n) {
    v.push_back(static_cast<uint8_t>(n >> 24));
    v.push_back(static_cast<uint8_t>(n >> 16));
    v.push_back(static_cast<uint8_t>(n >> 8));
    v.push_back(static_cast<uint8_t>(n));
}

// one frame: 4-char id, size (encoding picked by version), two zero
// flag bytes (unless overridden), then the payload
static void addFrame(Bytes& v, uint8_t ver, const char* id,
                     const Bytes& payload, uint8_t fmtFlags = 0) {
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<uint8_t>(id[i]));
    if (ver == 4) putSyncsafe(v, static_cast<uint32_t>(payload.size()));
    else          putBe32(v, static_cast<uint32_t>(payload.size()));
    v.push_back(0);
    v.push_back(fmtFlags);
    v.insert(v.end(), payload.begin(), payload.end());
}

static Bytes textPayload(uint8_t enc, const char* s) {
    Bytes p { enc };
    while (*s) p.push_back(static_cast<uint8_t>(*s++));
    return p;
}

static Bytes utf16Payload(bool bomP, bool beP, const char* s) {
    Bytes p { static_cast<uint8_t>(bomP ? 1 : 2) };
    if (bomP) {
        if (beP) { p.push_back(0xFE); p.push_back(0xFF); }
        else     { p.push_back(0xFF); p.push_back(0xFE); }
    }
    while (*s) {
        uint8_t c { static_cast<uint8_t>(*s++) };
        if (beP) { p.push_back(0); p.push_back(c); }
        else     { p.push_back(c); p.push_back(0); }
    }
    return p;
}

// wrap frames in a v2 header (+ optional padding, footer flag);
// returns the complete tag. flags bit 4 = footer, bit 7 = unsync
static Bytes v2Tag(uint8_t ver, const Bytes& frames,
                   uint32_t padding = 0, uint8_t flags = 0) {
    Bytes v { 'I', 'D', '3', ver, 0, flags };
    putSyncsafe(v, static_cast<uint32_t>(frames.size()) + padding);
    v.insert(v.end(), frames.begin(), frames.end());
    v.insert(v.end(), padding, 0);
    if (flags & 0x10) {
        Bytes f { '3', 'D', 'I', ver, 0, flags };
        putSyncsafe(f, static_cast<uint32_t>(frames.size()) + padding);
        v.insert(v.end(), f.begin(), f.end());
    }
    return v;
}

static Bytes v1Tag(const char* title, const char* artist) {
    Bytes v { 'T', 'A', 'G' };
    auto field = [&v](const char* s) {
        for (int i = 0; i < 30; ++i) {
            v.push_back(*s ? static_cast<uint8_t>(*s++) : ' ');
        }
    };
    field(title);
    field(artist);
    v.insert(v.end(), 128 - 63, 0);       // album/year/comment/genre
    return v;
}

static Bytes audio(uint32_t n) { return Bytes(n, 0xAB); }

static Bytes cat(std::initializer_list<Bytes> parts) {
    Bytes v;
    for (const Bytes& p : parts) v.insert(v.end(), p.begin(), p.end());
    return v;
}

// ---- harness -------------------------------------------------------

static int fails = 0;

static void check(const Bytes& file, bool wantFound,
                  const char* wantTitle, const char* wantArtist,
                  uint32_t wantPos, const char* what,
                  uint8_t cap = 15) {
    char title[32], artist[32];
    MemFs::File f { file.data(),
                    static_cast<uint32_t>(file.size()), 0 };
    bool found { Id3::readTags(f, title, artist, cap) };
    if (found != wantFound) {
        std::printf("FAIL %s: found=%d want %d\n", what, found,
                    wantFound);
        ++fails;
    }
    if (std::strcmp(title, wantTitle) != 0) {
        std::printf("FAIL %s: title '%s' want '%s'\n", what, title,
                    wantTitle);
        ++fails;
    }
    if (std::strcmp(artist, wantArtist) != 0) {
        std::printf("FAIL %s: artist '%s' want '%s'\n", what, artist,
                    wantArtist);
        ++fails;
    }
    if (f.pos != wantPos) {
        std::printf("FAIL %s: pos %u want %u\n", what, f.pos, wantPos);
        ++fails;
    }
}

int main() {
    // v2.3, ISO-8859-1, both frames
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "Kind Of Blue"));
        addFrame(fr, 3, "TPE1", textPayload(0, "Miles Davis"));
        Bytes tag { v2Tag(3, fr) };
        uint32_t start { static_cast<uint32_t>(tag.size()) };
        check(cat({ tag, audio(64) }), true,
              "Kind Of Blue", "Miles Davis", start, "v2.3 ISO");
    }
    // v2.4, UTF-8, syncsafe frame sizes (payload > 127 bytes forces
    // the syncsafe/plain distinction to matter)
    {
        Bytes big { textPayload(3, "Title") };
        big.insert(big.end(), 200, ' ');   // long trailing pad, trimmed
        Bytes fr;
        addFrame(fr, 4, "TIT2", big);
        addFrame(fr, 4, "TPE1", textPayload(3, "Artist"));
        Bytes tag { v2Tag(4, fr) };
        uint32_t start { static_cast<uint32_t>(tag.size()) };
        check(cat({ tag, audio(64) }), true,
              "Title", "Artist", start, "v2.4 UTF-8 syncsafe");
    }
    // UTF-16: LE with BOM, BE with BOM, and encoding 2 (BE, no BOM)
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", utf16Payload(true, false, "Waltz"));
        addFrame(fr, 3, "TPE1", utf16Payload(true, true, "Evans"));
        Bytes tag { v2Tag(3, fr) };
        check(cat({ tag, audio(8) }), true, "Waltz", "Evans",
              static_cast<uint32_t>(tag.size()), "utf16 BOM LE/BE");
    }
    {
        Bytes fr;
        addFrame(fr, 4, "TIT2", utf16Payload(false, true, "NoBom"));
        Bytes tag { v2Tag(4, fr) };
        check(cat({ tag, audio(8) }), true, "NoBom", "",
              static_cast<uint32_t>(tag.size()), "utf16 enc2");
    }
    // a big APIC frame BEFORE the text frames: skipped by seek
    {
        Bytes art(5000, 0xEE);
        Bytes fr;
        addFrame(fr, 3, "APIC", art);
        addFrame(fr, 3, "TIT2", textPayload(0, "Buried"));
        Bytes tag { v2Tag(3, fr) };
        check(cat({ tag, audio(8) }), true, "Buried", "",
              static_cast<uint32_t>(tag.size()), "APIC first");
    }
    // padding after the frames: walk stops at the zero frame id, the
    // audio-start still honors the full advertised tag size
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "Padded"));
        Bytes tag { v2Tag(3, fr, 300) };
        check(cat({ tag, audio(8) }), true, "Padded", "",
              static_cast<uint32_t>(tag.size()), "padding");
    }
    // footer flag: audio starts 10 bytes later
    {
        Bytes fr;
        addFrame(fr, 4, "TIT2", textPayload(0, "Footed"));
        Bytes tag { v2Tag(4, fr, 0, 0x10) };
        check(cat({ tag, audio(8) }), true, "Footed", "",
              static_cast<uint32_t>(tag.size()), "footer flag");
    }
    // v1 only: space-padded fields, audio starts at 0
    {
        check(cat({ audio(500), v1Tag("So What", "Miles") }), true,
              "So What", "Miles", 0, "v1 only");
    }
    // per-field merge: v2 has the title, v1 supplies the artist
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "FromV2"));
        Bytes tag { v2Tag(3, fr) };
        check(cat({ tag, audio(300), v1Tag("Ignored", "FromV1") }),
              true, "FromV2", "FromV1",
              static_cast<uint32_t>(tag.size()), "v2+v1 merge");
    }
    // no tags at all
    {
        check(audio(500), false, "", "", 0, "untagged");
    }
    // and an untagged file too small for a v1 record to even fit
    {
        check(audio(64), false, "", "", 0, "tiny untagged");
    }
    // truncation: cap 8 keeps 7 chars + NUL
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2",
                 textPayload(0, "A Love Supreme Pt 1"));
        Bytes tag { v2Tag(3, fr) };
        check(cat({ tag, audio(8) }), true, "A Love", "",
              static_cast<uint32_t>(tag.size()), "cap 8", 8);
    }
    // tag header claims more than the file holds: fall back to pos 0
    {
        Bytes v { 'I', 'D', '3', 3, 0, 0 };
        putSyncsafe(v, 100000);
        check(cat({ v, audio(64) }), false, "", "", 0, "liar tag");
    }
    // v2.2: not parsed, but skipped correctly; v1 supplies the text
    {
        Bytes v { 'I', 'D', '3', 2, 0, 0 };
        putSyncsafe(v, 40);
        v.insert(v.end(), 40, 0x11);
        uint32_t start { static_cast<uint32_t>(v.size()) };
        check(cat({ v, audio(200), v1Tag("OldTag", "OldArtist") }),
              true, "OldTag", "OldArtist", start, "v2.2 skip + v1");
    }
    // unsynchronised tag: same story -- skip, don't parse
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "Mangled"));
        Bytes tag { v2Tag(3, fr, 0, 0x80) };
        check(cat({ tag, audio(200), v1Tag("Clean", "") }), true,
              "Clean", "", static_cast<uint32_t>(tag.size()),
              "unsync skip + v1");
    }
    // compressed frame (nonzero format flags): skipped, v1 fills in
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "Zipped"), 0x80);
        Bytes tag { v2Tag(3, fr) };
        check(cat({ tag, audio(200), v1Tag("Plain", "") }), true,
              "Plain", "", static_cast<uint32_t>(tag.size()),
              "compressed frame skip");
    }
    // v2.4 extended header: hopped, frames after it still found
    {
        Bytes fr;
        addFrame(fr, 4, "TIT2", textPayload(0, "PastExt"));
        Bytes body;
        putSyncsafe(body, 6);              // ext size counts itself
        body.push_back(1); body.push_back(0);
        body.insert(body.end(), fr.begin(), fr.end());
        Bytes tag { v2Tag(4, body, 0, 0x40) };
        check(cat({ tag, audio(8) }), true, "PastExt", "",
              static_cast<uint32_t>(tag.size()), "v2.4 ext header");
    }
    // v2.3 extended header: size excludes itself
    {
        Bytes fr;
        addFrame(fr, 3, "TIT2", textPayload(0, "PastExt3"));
        Bytes body;
        putBe32(body, 6);                  // 6 more bytes follow
        body.insert(body.end(), 6, 0);
        body.insert(body.end(), fr.begin(), fr.end());
        Bytes tag { v2Tag(3, body, 0, 0x40) };
        check(cat({ tag, audio(8) }), true, "PastExt3", "",
              static_cast<uint32_t>(tag.size()), "v2.3 ext header");
    }
    // an empty v1 title field stays "" (all-space field is padding)
    {
        check(cat({ audio(300), v1Tag("", "OnlyArtist") }), true,
              "", "OnlyArtist", 0, "v1 empty title");
    }

    if (fails == 0) std::printf("id3_test: all tests passed\n");
    else            std::printf("id3_test: %d FAILURES\n", fails);
    return fails == 0 ? 0 : 1;
}
