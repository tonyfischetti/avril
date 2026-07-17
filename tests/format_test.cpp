// Host-side test for include/utils/format.hpp -- the formatters are
// pure computation, so the host compiler can check every edge against
// its own snprintf. Build and run:
//
//     c++ -std=c++17 -Wall -Wextra -I../include -o format_test \
//         format_test.cpp && ./format_test
//
// (The __attribute__((noinline)) annotations are GCC/Clang-portable,
// so this compiles anywhere the examples' toolchain philosophy does.)

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "utils/format.hpp"

namespace Fmt = HAL::Utils::Fmt;

static int fails = 0;

static void expectStr(const char* got, const char* want,
                      const char* what) {
    if (std::strcmp(got, want) != 0) {
        std::printf("FAIL %s: got '%s' want '%s'\n", what, got, want);
        ++fails;
    }
}

static void expectField(const char* got, size_t n, const char* want,
                        const char* what) {
    if (std::strncmp(got, want, n) != 0) {
        std::printf("FAIL %s: got '%.*s' want '%s'\n", what,
                    static_cast<int>(n), got, want);
        ++fails;
    }
}

int main() {
    // u32: digit-count boundaries and extremes
    {
        const uint32_t cases[] = { 0, 1, 9, 10, 99, 100, 999, 1000,
                                   65535, 65536, 999999999,
                                   1000000000, 4294967295u };
        for (uint32_t v : cases) {
            char buf[11], want[16];
            std::snprintf(want, sizeof want, "%u", v);
            expectStr(Fmt::u32(buf, v), want, "u32");
        }
    }
    // s32: signs and the INT32_MIN trick
    {
        const int32_t cases[] = { 0, 1, -1, 9, -9, 10, -10,
                                  2147483647, -2147483647 - 1,
                                  -1000000 };
        for (int32_t v : cases) {
            char buf[12], want[16];
            std::snprintf(want, sizeof want, "%d", v);
            expectStr(Fmt::s32(buf, v), want, "s32");
        }
    }
    // fixed: padding and low-digit truncation
    {
        char b[8];
        Fmt::fixed(b, 7, 2);        expectField(b, 2, "07", "fixed 07");
        Fmt::fixed(b, 42, 2);       expectField(b, 2, "42", "fixed 42");
        Fmt::fixed(b, 123, 2);      expectField(b, 2, "23", "fixed trunc");
        Fmt::fixed(b, 0, 3);        expectField(b, 3, "000", "fixed 000");
        Fmt::fixed(b, 123456, 6);   expectField(b, 6, "123456", "fixed 6");
        Fmt::fixed(b, 4294967295u, 5);
        expectField(b, 5, "67295", "fixed trunc32");
    }
    // hex: all 256 bytes, plus 16/32-bit composition
    {
        for (int i = 0; i < 256; ++i) {
            char b[2], want[8];
            std::snprintf(want, sizeof want, "%02X", i);
            Fmt::hex8(b, static_cast<uint8_t>(i));
            expectField(b, 2, want, "hex8");
        }
        char b16[4], b32[8], want[16];
        std::snprintf(want, sizeof want, "%04X", 0xABCD);
        Fmt::hex16(b16, 0xABCD); expectField(b16, 4, want, "hex16");
        std::snprintf(want, sizeof want, "%08X", 0xDEADBEEFu);
        Fmt::hex32(b32, 0xDEADBEEFu);
        expectField(b32, 8, want, "hex32");
    }
    // bin8
    {
        char b[8];
        Fmt::bin8(b, 0x00); expectField(b, 8, "00000000", "bin8 00");
        Fmt::bin8(b, 0xFF); expectField(b, 8, "11111111", "bin8 FF");
        Fmt::bin8(b, 0xA5); expectField(b, 8, "10100101", "bin8 A5");
        Fmt::bin8(b, 0x01); expectField(b, 8, "00000001", "bin8 01");
    }

    if (fails == 0) std::printf("all format tests passed\n");
    return fails;
}
