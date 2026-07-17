#pragma once

#include "../common.hpp"

#include <stdint.h>

#include "../devices/SDCard.hpp"

/**
 * Read-only FAT32 -- enough filesystem to find files a computer put on
 * the card, and not one line more.
 *
 * The use case: drag MP3s (or logs, or config) onto a card from a
 * computer, then open and stream them from the AVR. So: mount, iterate
 * directories, find by 8.3 name, sequential read, seek. No write, no
 * create, no delete, no long filenames (an LFN-named file still
 * appears, under its auto-generated SHORTN~1 alias).
 *
 * The trick that makes this fit the tinies: NO SECTOR BUFFER. FAT32
 * work decomposes into small windowed reads -- directory entries are
 * 32-byte records, FAT chain links are 4-byte little-endian words --
 * which is exactly what the SD driver's readPartial() provides. Total
 * RAM: ~20 bytes of static mount state plus ~16 per open File.
 * (The wire still carries a full 512-byte block per window -- the
 * protocol insists -- so for BULK data reads, pass read() large,
 * block-aligned buffers where RAM allows: at 512 bytes per call the
 * wire overhead is zero.)
 *
 * Seeking: FAT is a linked list of clusters, so seek() walks the chain
 * from the start -- one 4-byte read per cluster (4-32 KiB each).
 * Jumping into the middle of a 5 MB file costs a few hundred windowed
 * reads (tens of ms). Fine for "skip to 1:30"; do not call per byte.
 *
 * Cards up to 2 GB often come formatted FAT16, which this does not
 * speak -- reformat as FAT32 (SDHC cards come FAT32 out of the box).
 */

namespace HAL {
namespace FS {

enum class Result : uint8_t {
    OK,
    NOT_MOUNTED,      // mount() hasn't succeeded
    NO_FILESYSTEM,    // no boot signature where one was expected
    UNSUPPORTED,      // not FAT32 (FAT12/16 card?), or sector size != 512
    IO_ERROR,         // the block device reported an error
    NOT_FOUND,        // find() missed / directory iteration ended
    BAD_PARAMS        // e.g. seek past end of file
};

template<typename BlockDev>   // an instantiation of HAL::Devices::SD::Card
struct FAT32 {

    struct FileInfo {
        char     name[13];      // "NAME.EXT", NUL-terminated, uppercase
        uint32_t size;
        uint32_t firstCluster;
        bool     dirP;
    };

    struct File {
        uint32_t firstCluster;
        uint32_t size;
        uint32_t pos;
        uint32_t cluster;       // the cluster containing `pos`
    };

    struct DirIter {
        uint32_t cluster;
        uint16_t idx;           // entry index within that cluster
    };

  private:
    using BR = HAL::Devices::SD::Result;

    static inline bool     mountedP     { false };
    static inline uint32_t fatStart     { 0 };  // block of first FAT
    static inline uint32_t dataStart    { 0 };  // block of cluster #2
    static inline uint32_t rootCluster  { 0 };
    static inline uint8_t  spcLog2      { 0 };  // log2(blocks per cluster)
    static inline uint32_t clusterMask  { 0 };  // bytesPerCluster - 1

    static constexpr uint32_t CHAIN_END { 0x0FFFFFF8UL };

    static uint16_t le16(const uint8_t* p) {
        return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
    }
    static uint32_t le32(const uint8_t* p) {
        return static_cast<uint32_t>(p[0])
             | (static_cast<uint32_t>(p[1]) << 8)
             | (static_cast<uint32_t>(p[2]) << 16)
             | (static_cast<uint32_t>(p[3]) << 24);
    }

    static uint32_t clusterToBlock(uint32_t c) {
        return dataStart + ((c - 2) << spcLog2);
    }

    // one FAT chain link: a 4-byte window into the FAT (128 links per
    // block); the top 4 bits of an entry are reserved, hence the mask
    static Result nextCluster(uint32_t c, uint32_t& next) {
        uint8_t e[4];
        if (BlockDev::readPartial(fatStart + (c >> 7),
                                  static_cast<uint16_t>((c & 0x7F) * 4),
                                  e, 4) != BR::OK) {
            return Result::IO_ERROR;
        }
        next = le32(e) & 0x0FFFFFFFUL;
        return Result::OK;
    }

    // "FOO     MP3" (the on-disk 11-byte form) -> "FOO.MP3"
    static void formatName(const uint8_t* raw, char* out) {
        uint8_t o { 0 };
        for (uint8_t i = 0; i < 8 && raw[i] != ' '; ++i) {
            out[o++] = static_cast<char>(raw[i]);
        }
        if (raw[8] != ' ') {
            out[o++] = '.';
            for (uint8_t i = 8; i < 11 && raw[i] != ' '; ++i) {
                out[o++] = static_cast<char>(raw[i]);
            }
        }
        out[o] = '\0';
    }

    static char upper(char c) {
        return (c >= 'a' && c <= 'z')
            ? static_cast<char>(c - ('a' - 'A'))
            : c;
    }

    static bool nameEq(const char* a, const char* b) {
        while (*a && *b) {
            if (upper(*a++) != upper(*b++)) return false;
        }
        return *a == *b;
    }

  public:
    // parse the card's bookkeeping: works on cards with an MBR
    // partition table and on "superfloppy" cards formatted as one
    // bare volume
    static Result mount() {
        mountedP = false;

        uint8_t sig[2];
        if (BlockDev::readPartial(0, 510, sig, 2) != BR::OK) {
            return Result::IO_ERROR;
        }
        if (sig[0] != 0x55 || sig[1] != 0xAA) return Result::NO_FILESYSTEM;

        // block 0 is either the volume itself (BPB) or an MBR whose
        // first partition points at it. A BPB announces itself with
        // bytesPerSector at offset 11; an MBR has a partition entry
        // at offset 446
        uint8_t  probe[2];
        uint32_t volStart { 0 };
        if (BlockDev::readPartial(0, 11, probe, 2) != BR::OK) {
            return Result::IO_ERROR;
        }
        if (le16(probe) != 512) {
            uint8_t part[8];   // partition 0: type at +4, LBA at +8..11
            if (BlockDev::readPartial(0, 446 + 4, part, 8) != BR::OK) {
                return Result::IO_ERROR;
            }
            volStart = le32(part + 4);
            if (volStart == 0) return Result::NO_FILESYSTEM;
        }

        // the BPB fields this driver needs all sit in the first 48
        // bytes of the volume block
        uint8_t bpb[48];
        if (BlockDev::readPartial(volStart, 0, bpb, 48) != BR::OK) {
            return Result::IO_ERROR;
        }
        if (le16(bpb + 11) != 512) return Result::UNSUPPORTED;

        uint8_t spc { bpb[13] };
        if (spc == 0 || (spc & (spc - 1)) != 0) return Result::UNSUPPORTED;
        spcLog2 = 0;
        while ((spc >> spcLog2) != 1) ++spcLog2;
        clusterMask = (static_cast<uint32_t>(spc) << 9) - 1;

        // FAT32's tell: the FAT12/16 root-entry-count and FAT-size
        // fields are zero, and the real values live in the FAT32 area
        if (le16(bpb + 17) != 0 || le16(bpb + 22) != 0) {
            return Result::UNSUPPORTED;
        }

        uint16_t reserved { le16(bpb + 14) };
        uint8_t  numFATs  { bpb[16] };
        uint32_t fatSize  { le32(bpb + 36) };
        rootCluster       = le32(bpb + 44);

        fatStart  = volStart + reserved;
        dataStart = fatStart + static_cast<uint32_t>(numFATs) * fatSize;

        mountedP = true;
        return Result::OK;
    }

    static DirIter root() { return { rootCluster, 0 }; }

    // descend into a subdirectory found by next()/find()
    static DirIter openDir(const FileInfo& info) {
        return { info.firstCluster, 0 };
    }

    // the next real file/directory entry, skipping deleted entries,
    // long-filename records, and the volume label. Returns NOT_FOUND
    // at the end of the directory
    static Result next(DirIter& it, FileInfo& out) {
        if (!mountedP) return Result::NOT_MOUNTED;
        for (;;) {
            if (it.cluster < 2 || it.cluster >= CHAIN_END) {
                return Result::NOT_FOUND;
            }
            // 16 32-byte entries per block, spc blocks per cluster
            if (it.idx >= (16U << spcLog2)) {
                uint32_t nxt;
                Result r { nextCluster(it.cluster, nxt) };
                if (r != Result::OK) return r;
                it.cluster = nxt;
                it.idx     = 0;
                continue;
            }
            uint8_t e[32];
            uint32_t block { clusterToBlock(it.cluster) + (it.idx >> 4) };
            if (BlockDev::readPartial(block,
                                      static_cast<uint16_t>((it.idx & 15) * 32),
                                      e, 32) != BR::OK) {
                return Result::IO_ERROR;
            }
            ++it.idx;
            if (e[0] == 0x00) return Result::NOT_FOUND;   // end marker
            if (e[0] == 0xE5) continue;                   // deleted
            uint8_t attr { e[11] };
            if ((attr & 0x0F) == 0x0F) continue;          // LFN record
            if (attr & 0x08) continue;                    // volume label
            formatName(e, out.name);
            out.size         = le32(e + 28);
            out.firstCluster = (static_cast<uint32_t>(le16(e + 20)) << 16)
                             | le16(e + 26);
            out.dirP         = (attr & 0x10) != 0;
            return Result::OK;
        }
    }

    // case-insensitive 8.3 lookup in the root directory: "SONG.MP3"
    // and "song.mp3" both match. (For subdirectories, iterate with
    // next() over openDir() yourself)
    static Result find(const char* name, FileInfo& out) {
        DirIter it { root() };
        Result r;
        while ((r = next(it, out)) == Result::OK) {
            if (nameEq(out.name, name)) return Result::OK;
        }
        return r;
    }

    static void open(const FileInfo& info, File& f) {
        f.firstCluster = info.firstCluster;
        f.size         = info.size;
        f.pos          = 0;
        f.cluster      = info.firstCluster;
    }

    // sequential read from the current position; `got` reports how
    // many bytes actually arrived (short only at end of file). For
    // bulk streaming, larger and block-aligned is faster -- each
    // windowed read costs a full block on the wire
    static Result read(File& f, uint8_t* dst, uint16_t want,
                       uint16_t& got) {
        got = 0;
        if (!mountedP) return Result::NOT_MOUNTED;
        while (want > 0 && f.pos < f.size) {
            uint32_t left  { f.size - f.pos };
            uint16_t off   { static_cast<uint16_t>(f.pos & 511) };
            uint16_t chunk { static_cast<uint16_t>(512 - off) };
            if (chunk > want) chunk = want;
            if (left < chunk) chunk = static_cast<uint16_t>(left);
            uint32_t block { clusterToBlock(f.cluster)
                             + ((f.pos >> 9) & ((1U << spcLog2) - 1)) };
            if (BlockDev::readPartial(block, off, dst + got, chunk)
                    != BR::OK) {
                return Result::IO_ERROR;
            }
            f.pos += chunk;
            got    = static_cast<uint16_t>(got + chunk);
            want   = static_cast<uint16_t>(want - chunk);
            // stepped onto a cluster boundary with more file ahead:
            // follow the chain
            if (f.pos < f.size && (f.pos & clusterMask) == 0) {
                uint32_t nxt;
                Result r { nextCluster(f.cluster, nxt) };
                if (r != Result::OK) return r;
                if (nxt >= CHAIN_END) return Result::IO_ERROR;  // truncated chain
                f.cluster = nxt;
            }
        }
        return Result::OK;
    }

    // absolute reposition. FAT is a linked list, so this walks the
    // chain from the file's first cluster: cheap for small files,
    // tens of ms into a multi-MB one. Seeking to exactly `size` is
    // legal (subsequent reads return 0 bytes)
    static Result seek(File& f, uint32_t target) {
        if (!mountedP) return Result::NOT_MOUNTED;
        if (target > f.size) return Result::BAD_PARAMS;
        // when target sits exactly on a cluster boundary at EOF, the
        // chain may legitimately end just before it -- walk to the
        // cluster containing the last real byte instead
        uint32_t effective { (target == f.size && target != 0)
                                 ? target - 1 : target };
        uint32_t hops { effective >> (spcLog2 + 9) };
        uint32_t c    { f.firstCluster };
        while (hops--) {
            uint32_t nxt;
            Result r { nextCluster(c, nxt) };
            if (r != Result::OK) return r;
            if (nxt >= CHAIN_END) return Result::IO_ERROR;
            c = nxt;
        }
        f.cluster = c;
        f.pos     = target;
        return Result::OK;
    }

};

}
}
