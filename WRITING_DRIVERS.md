# Writing avril device drivers

This is the amnesia document: everything about how avril drivers get
written, from opening the datasheet to committing the module, written
so that a past author with a head injury could reconstruct the whole
craft from it. The general method comes first; then three concrete
walkthroughs — HT16K33 (easy), RDA5807 (medium), BMP581 (a modern
register machine) — that apply it to real parts, in order.

("Driver" is the right term, incidentally. A driver is code that owns
a piece of hardware and presents semantics instead of registers —
`tune(1013)` instead of "write 0x3D02 to register 0x03." That's
exactly what `devices/` modules are. The fact that there's no
operating system underneath doesn't change the job description.)

---

# Part I — The general method

## 1. What an avril driver is

### Where things live

- `include/comms/` — bus masters (UART, SPI, I²C). They move bytes;
  they know nothing about any particular chip.
- `include/devices/` — chip drivers. They own one part's register
  map and semantics, and *use* a comms module to reach it.
- `include/fs/`, `include/utils/` — software layers above and beside.

A new chip driver is almost always `include/devices/<Part>.hpp`.

### The anatomy

Every driver is a header-only template. The house pattern, visible in
any existing device:

```cpp
#pragma once

#include <stdint.h>
// ... the minimum else

namespace HAL {
namespace Devices {

template<typename I2c>            // the bus, INJECTED as a type
struct SomeChip {

    enum class Result : uint8_t { OK, NACK_ADDR, TIMEOUT, WRONG_CHIP };

  private:
    static constexpr uint8_t ADDR { 0x42 };
    // register primitives first ...

  public:
    static Result begin();        // called before sei()
    // ... semantic API on top
};

}
}
```

The load-bearing decisions, and why:

- **The bus is a template parameter**, not a hardcoded call. This is
  what lets one driver serve the 328P's hardware TWI and the tinies'
  bit-banged I²C without an `#ifdef` — and what makes host-testing
  possible (see §4): a mock bus type slots in where the real one was.
  `LCD1602<I2c>`, `FAT32<BlockDev>`, `ID3<Fs>` all follow this.
- **Compile-time configuration goes in template parameters** (pins,
  clock ceilings, addresses when they're solder-jumper-fixed).
  Runtime state is `static inline` members. Most drivers are fully
  static — no instances — because the chip itself is a singleton on
  the bus. (Drivers with per-instance state, like Button, are actual
  objects.)
- **Every wait is bounded.** Any loop that waits on hardware has a
  timeout and returns a `Result`, never spins forever. A missing
  chip must produce an error string on the LCD/UART, not a hang.
  Look at any `begin()`: the pattern is a down-counter around a
  status poll.
- **`Result` enums, not bools.** `NACK_ADDR` vs `TIMEOUT` vs
  `WRONG_CHIP` is exactly the information you need at the bench, and
  the examples' `die()` pattern turns it into words on a display.
- **Lifecycle**: construct statically → `begin()` before `sei()` →
  if the part produces asynchronous events, notify from the ISR and
  decode in `process()` from the main loop. ISRs only notify; the
  two sanctioned exceptions (RotaryEncoder, IRReceiverNEC) decode in
  the ISR because their protocol timing demands it, keep all state
  ISR-side, and hand results across one atomic claim. Read the
  concurrency section of the README before adding a third.
- **Booleans end in `P`** (`playingP`, `mountedP`) — predicate style.
- **The header comment is the documentation**: what the part is, the
  one-paragraph theory of operation, the wiring table (physical DIP
  pin numbers, not just port names), the gotchas that cost an
  afternoon. Write it as the letter to your future amnesiac self —
  which, given that you're reading this file, worked.

### The warning ratchet

Everything builds under the full strict set (see any example
Makefile: `-Wall -Wextra -Wpedantic -Weffc++ -Wold-style-cast
-Wsign-conversion -Winline` …and the rest). Zero warnings is the
bar, and two flags have house idioms:

- `-Winline`: header functions need `inline` for linkage, but GCC
  treats `inline` as a *request* and warns when it declines. For
  busy-wait bodies and anything called from several sites, the fix
  is `__attribute__((noinline)) inline` — linkage yes, inlining no.
  Single-instruction GPIO accessors get `always_inline` instead.
- `-Wold-style-cast` / `-Wsign-conversion`: `static_cast`
  everywhere, and arithmetic on `uint8_t` promotes to `int` — the
  `static_cast<uint8_t>(x + 1)` dance is everywhere for a reason.

Class-template member functions defined in-class don't trigger the
`-Winline` ratchet in practice (FAT32 is the existence proof), so
inside a `template struct` you mostly don't think about it.

## 2. Reading the datasheet

The datasheet is the driver, pre-written in the wrong language. The
method:

**Skip to the back half.** The front is marketing and pinouts. What
you need, in the order you'll need it:

1. **Electrical characteristics.** Supply range first — most modern
   parts are 1.8–3.6 V and *not* 5 V-tolerant. On a 5 V 328P that
   means level shifting or running the MCU at 3.3 V/8 MHz. (Rule of
   thumb for reading a 3.3 V output with a 5 V AVR: V_IH at 5 V is
   0.6·Vcc = 3.0 V, so 3.3 V highs are *marginal-legal* — it often
   works on a breadboard and is still the wrong design. For I²C
   specifically, pulling the bus up to 3.3 V only is the usual
   compromise; for SPI you want a real shifter, like the SD module's.)
   Also note max bus clock — it becomes the driver's documented
   ceiling (`Master<400000>`, `Card<16, 8000000>` style).
2. **The interface chapter.** Bus address (see the 7-bit trap below),
   register width, **endianness** of multi-byte values, whether
   reads auto-increment the register pointer, whether reads need a
   repeated start. Write these five facts at the top of your crib
   sheet; four of the five will bite if assumed.
3. **The register map.** Don't read it linearly. Find: the chip-ID
   register (your first-contact handshake), the reset mechanism
   (dedicated register, magic byte, or pin), the mode/power-control
   register, the status register, and the data-out registers. That's
   the skeleton; the other forty registers are features.
4. **Timing.** Reset-to-ready delays, conversion times, minimum
   pulse widths. Translate to cycles at your F_CPU; short precise
   waits use `__builtin_avr_delay_cycles(n)` (compile-time-constant,
   cycle-exact), long or repeated waits on battery use
   `Watchdog::sleepFor`.
5. **The recommended init sequence.** Buried in an "operation" or
   "quick start" section, and it is *gospel* — chips frequently
   require an exact order (oscillator before display; charge pump
   before anything; status-poll before trusting data). Transcribe it
   verbatim into the crib sheet.

**Write the crib sheet.** One page: address, the five interface
facts, the six skeleton registers with their magic values, the init
sequence, the timing numbers, quirks. The HT16K33 crib in
NEXT_STEPS.md is the model. The discipline matters because the
datasheet is 60 pages and the driver needs 15 facts — extracting
them *once, on paper* beats re-finding them at every compile.

**Search the errata separately.** Datasheets don't advertise their
bugs; search "<part> errata" and skim existing drivers' issue
trackers. The DPS310's temperature bug lives in a community wiki,
not the datasheet.

**Read one existing driver — for wire truths, not style.** An
Adafruit/Arduino driver is a record of what *actually* had to be
sent, including undocumented magic. Where their code contradicts the
datasheet, the code usually wins, and that contradiction is exactly
the note to carry into your crib sheet. Then close their repo and
write avril's driver in avril's shape.

**The 7-bit address trap**, in full because it costs everyone one
evening exactly once: I²C addresses are 7 bits followed by a R/W̄
bit. Datasheets randomly quote either the 7-bit address (0x70) or
the "8-bit write address" (0xE0 = 0x70<<1). avril's I²C master takes
7-bit. If a part is deaf at its advertised address, try
`addr >> 1` before touching a wire.

## 3. First contact at the bench

Before any driver code exists, prove the part is alive and reachable.
This stage uses a throwaway smoke test (§4) plus the meter and the
analyzer.

### Multimeter first

- Rails: is the module actually getting 3.3/5 V *at its pins*? A
  breadboard power rail with a break in the middle is a classic.
- Continuity: beep out every jumper, MCU pin to module pin. Cheap
  jumper wires have ~5% mortality.
- Module archaeology: does the breakout have an onboard regulator
  and level shifter (SD modules usually; RDA5807 boards usually
  not)? Follow the traces; look for a 5-pin SOT-23 (regulator) and
  a small array chip or MOSFET pairs (shifter).
- Current draw as a vital sign, if your supply shows it: a chip
  drawing ~0 µA when it should draw milliamps isn't in reset — it's
  not powered or not soldered.

### I²C: the ACK scan

The single most valuable fifteen lines in bring-up. Every I²C chip
answers its address with an ACK even when totally unconfigured, so
scanning all 128 addresses and printing who ACKs tells you the
wiring, the pullups, the address, and the chip's pulse — all before
a single register is touched:

```cpp
for (uint8_t a = 1; a < 0x78; ++a) {
    if (I2c::ping(a) == HAL::Comms::I2C::Result::OK) {   // start+addr+stop
        UART::print("ACK 0x");
        char h[2]; HAL::Utils::Fmt::hex8(h, a);
        UART::printByte(static_cast<uint8_t>(h[0]));
        UART::printByte(static_cast<uint8_t>(h[1]));
        UART::newline();
    }
}
```

Read the scan like a doctor: **nothing at all** → pullups missing or
SDA/SCL swapped (the bus idles low on the analyzer — see below).
**Everything ACKs (all 119)** → SDA shorted to ground or a pullup to
the wrong rail; a stuck-low SDA reads as universal ACK. **The wrong
single address** → 7-bit trap, or solder jumpers, or you have the
sibling part. **The right address, sometimes** → marginal pullups or
a breadboard contact; wiggle test.

### SPI: chip-ID, because there is no scan

SPI has no ACK — a wrong-wired SPI bus reads as a perfectly
confident stream of garbage. So first contact is always **read the
chip-ID/WHO_AM_I register** and compare against the datasheet
constant. Two pre-checks when even that fails:

- Loopback: jumper MISO to MOSI, no chip involved; transfer 0xA5 and
  expect 0xA5 back. Proves the master and the wiring to the header.
- The all-0xFF / all-0x00 oracle: 0xFF means MISO never moves (not
  connected, or CS not actually asserted — check the CS pin with the
  meter during a transfer); 0x00 means MISO grounded or the chip is
  held in reset.

And remember the 328P's **SS trap**: pin PB2 must be an output (any
output) or the hardware demotes itself from master to slave
mid-flight. `SPI::begin()` handles it; a hand-rolled smoke test that
bypasses `begin()` will rediscover it the hard way.

### The logic analyzer

The analyzer (any cheap 8-channel fx2-class clone + sigrok/PulseView)
is the primary instrument for everything protocol-shaped. Workflow:

- **Sample rate ≥ 4× the bus clock**, and more is free: 100 kHz I²C
  → 1 MS/s minimum, 8 MHz SPI → 24 MS/s (drop the SPI clock during
  bring-up instead; nothing about bring-up needs speed).
- Wire GND first, always — no shared ground, no capture.
- Add the protocol decoder (I²C or SPI) in PulseView, point it at
  the channels, and read *transactions*, not edges.
- Capture around the interesting moment: trigger on the falling edge
  of SDA (I²C start) or CS (SPI).

What healthy looks like, and the three diseases:

- **Healthy I²C write**: START, address byte, a *low* 9th bit (ACK),
  data bytes each ACKed, STOP. The decoder labels all of it.
- **Address NAK** (9th bit high after the address): the chip isn't
  at that address — wiring, 7-bit trap, or power. This is the wire
  view of a failed ACK scan.
- **Data NAK** (address ACKed, some later byte refused): you reached
  the chip and it rejected the *content* — usually an invalid
  register number, sometimes "busy, go away," occasionally a
  write-protected register. The distinction between address-NAK and
  data-NAK is the most diagnostic single bit in I²C debugging, which
  is why the I²C master's `Result` distinguishes them.
- **Bus never idle-high**: missing pullups (both lines float near
  ground and the analyzer shows noise, not protocol) or a stuck
  slave holding SDA low (see the lockup recovery note in §6).

For SPI, the analyzer answers the question the protocol can't: is
CS actually dropping? Is data changing on the edge the chip samples
(CPOL/CPHA — compare the capture against the datasheet's timing
diagram; mode 0 means data valid on rising edge, clock idle low)?
Is MISO moving at all?

Multimeter vs analyzer, the division of labor: the meter answers
"is it powered and connected," the analyzer answers "what was
actually said." When a driver misbehaves, resist single-stepping
code — capture the bus instead. The wire does not lie, and one
capture usually replaces an hour of printf archaeology.

## 4. The development loop

### Smoke test first, in the scratchpad, not in the repo

Driver development happens against a **throwaway main.cpp** in a
scratch directory, compiled with an example Makefile's flag set
(copy any example's Makefile and gut it). The smoke test is
ephemeral and never committed; when it dies, it's rebuilt in five
minutes from an example. Structure:

```
scratch/
  Makefile        <- copied from examples/anything/, HAL_LOC fixed up
  main.cpp        <- UART init, the experiment, results printed
```

UART at 9600 is the console of truth for the entire process. First
line of every smoke test: print a banner. If the banner itself is
garbage, stop — that's the F_CPU/fuse mismatch (the makefile says
16 MHz, the chip's fuses still say internal 8 MHz, every baud rate
is off by 2×). `make fuses` once per chip; garbled banner = fuses.

### Increments, each one printed

Never write the whole driver and then test it. The ladder, one rung
per flash cycle:

1. **Reach it**: ACK scan / chip-ID read. Prints the address or ID.
2. **Reset it**: issue the soft reset, wait the datasheet delay,
   re-read the ID or status. Proves writes land and timing is right.
3. **One register, write + readback**: pick a harmless config
   register, write a distinctive value, read it back, print both.
   Proves the full register plumbing including endianness. (A value
   like 0x55AA exposes byte swaps immediately; 0x0000 proves
   nothing.)
4. **The feature**: one measurement / one glyph / one station. From
   here it's the chip's semantics, and the plumbing below is trusted.

Each rung is ~10 lines added to the smoke test. When a rung fails,
the fault is in the 10 new lines or the wire — capture the bus.

### Driver shape follows the same ladder

Inside the eventual driver file, write the **register primitives
first** and the semantics on top:

```cpp
  private:
    static Result writeReg(uint8_t reg, uint16_t val);
    static Result readReg(uint8_t reg, uint16_t& val);
    // ^ these two encapsulate: address, endianness, repeated-start.
    //   Get them right once; every feature above is then just
    //   "which register, which bits"
```

The smoke-test rungs 1–3 effectively *are* these primitives being
born. When rung 4 works in the smoke test, the code is usually
minutes from being a real driver: move it into
`include/devices/<Part>.hpp`, shape it into the house anatomy (§1),
and re-point the smoke test at the driver to confirm nothing changed.

### Host-test the pure computation

If the driver contains real math — compensation polynomials, BCD,
packing/unpacking, anything that transforms bytes without touching
hardware — pull it into a shape a host test can reach, following
`tests/format_test.cpp` and `tests/id3_test.cpp`. The ID3 module
shows the trick for I/O-adjacent logic: template on the *shape* of
the dependency, feed a 20-line in-memory mock on the host. Every
compensation formula transcribed from a datasheet deserves one test
against the datasheet's own worked example (they almost always
include one — search the chapter for "for example").

### Promotion checklist

When the smoke test does the real thing on real hardware:

- `include/devices/<Part>.hpp` in the house anatomy, header comment
  written (wiring table! gotchas!).
- Zero warnings under the full flag set — build an example that
  includes it, not just the smoke test.
- Read the `avr-size` line and sanity-check it against what the
  driver *should* cost (§5); if the change was supposed to be
  invisible to existing consumers, prove it byte-identical (§5).
- An example (new or extended) if the part is interesting enough to
  demonstrate — examples are the living documentation.
- README: a Devices section entry, and the examples table row.
- NEXT_STEPS.md: cross the item off.
- Commit inside avril; pointer bump in the parent repo.

## 5. Reading the artifact: size, disassembly, byte-identity

The compiler's output is evidence, and three habits read it. None of
them need hardware, which makes them the *cheapest* verification in
the whole workflow — they run on every build, not every flash.

### `avr-size`: text / data / bss, and why you read it every build

Every example Makefile prints this after linking:

```
   text    data     bss   total filename
  10418     152     674   11244 build/main.elf
```

What the segments *are*, because the names stopped being mnemonic
in 1975:

- **text** — code plus read-only data that lives in flash: your
  functions, PROGMEM tables, `PSTR()` strings. Flash-only.
- **data** — initialized globals/statics (`uint8_t volAtt {0x30}`).
  This segment is the expensive one: it costs **flash AND RAM** —
  the initial values are stored in flash and copied into RAM by the
  C runtime before `main()`. A string literal used the plain way
  (`UART::print("hello")`) lands here and occupies RAM forever;
  that's the entire reason the `_P` function variants and `PSTR()`
  exist. When data creeps up unexpectedly, look for a string that
  should have been PROGMEM.
- **bss** — zero-initialized globals (buffers, most state). RAM
  only; costs nothing in flash (the runtime just zeroes the range).

The two budgets: **flash used = text + data** (against 32 KB on the
328P, 8 KB on the tinies); **static RAM used = data + bss** (against
2 KB / 512 B). Everything left over is the stack's, and there is no
gauge on the stack — the only warning you get is corruption when it
collides with bss. Rule of thumb on these parts: keep data + bss
under ~75% of RAM unless you've reasoned about worst-case stack
depth (deepest call chain plus the worst-case ISR stacking from the
README's deadline ledger).

Watch the *deltas*, not the absolutes. A driver should cost roughly
what it looks like it costs. The classic ambushes: one `float`
sneaking into arithmetic drags in ~1–2 KB of libgcc (text jumps);
one `%`-formatting call pulls the printf machinery (kilobytes); one
un-PSTR'd string moves flash bytes into RAM (data jumps). When a
number jumps and you don't know why:

```
avr-nm -C --size-sort build/main.elf | tail -20
```

names the twenty biggest symbols — the culprit is usually on the
list wearing a name you don't recognize (`__mulsf3`: that's the
float multiply you didn't mean to have).

### Disassembly: `avr-objdump`

```
avr-objdump -d build/main.elf | less        # plain disassembly
avr-objdump -S build/main.elf | less        # interleaved with source
                                            #   (needs -g; play nice
                                            #    with LTO by adding -g
                                            #    to the flags, it costs
                                            #    nothing in the .hex)
```

Disassemble the **.elf**, not the .hex (the .hex has no symbols).
When to reach for it:

- **Auditing the inlining decisions.** `always_inline` accessors
  should have vanished into their callers; `noinline` helpers should
  appear exactly once. Search for the symbol name — if a
  single-instruction GPIO accessor exists as an actual `call`
  target, the attribute isn't where you think it is.
- **Cycle-counting bit-banged timing.** Before deciding a waveform
  needs hand-written assembly (the NeoPixel driver's origin story),
  disassemble what the compiler produced and count: most AVR
  instructions are 1 cycle, taken branches 2, `lds`/`sts` 2,
  `call`+`ret` ~7–8 together — the instruction-set summary in any
  AVR datasheet has the full table. A loop body's cycle count ×
  iterations against the deadline tells you whether C is fast
  enough *before* you burn an evening on asm. This is also how the
  cycle-counted asm gets *verified* after it's written.
- **Auditing an ISR's true cost.** The prologue/epilogue (the
  `push`/`pop` wall) is invisible in C but real in the deadline
  ledger: an ISR that calls a non-inlined function forces the
  compiler to save every call-clobbered register — ~18 pushes +
  ~18 pops ≈ 72 cycles of pure overhead before any work happens.
  Disassemble the vector (search `__vector_`) and count the wall;
  if it's tall, the fix is usually making the ISR body smaller or
  fully inlined, and the proof it worked is the same disassembly.
- **Settling "did the compiler really do that?"** Hoisted volatile
  reads, surprise 32-bit arithmetic on things you thought were
  8-bit, a `switch` that became a jump table in RAM — when behavior
  and source disagree, the disassembly is the tiebreaker. The wire
  doesn't lie (§3), and neither does this.

### Byte-identity: the refactor acceptance test

A refactor that *shouldn't* change behavior — comment edits, doc
moves, renaming, restructuring headers, adding a driver nothing
uses yet — shouldn't change the generated code either. That claim
is checkable, and checking it converts "I'm pretty sure this is
safe" into a proof:

```
make clean && make && md5 build/main.hex     # before
# ...apply the refactor...
make clean && make && md5 build/main.hex     # after: same hash = same
                                             # flash image, bit for bit
```

This is the standing acceptance test for anything that touches
headers lamp-box consumes ("lamp-box rebuilds byte-identical" has
closed the argument in a dozen commits), and it's the promotion
criterion for the NeoPixel-relocation item in NEXT_STEPS.md.

The fine print that makes it work: compare the **.hex** (the flash
image), not the .elf (which carries debug paths and can differ
harmlessly); build clean both times with identical flags; and never
let `__DATE__`/`__TIME__` into the source — one timestamp macro and
every build is unique forever. avr-gcc is deterministic given the
same inputs, so a hash mismatch is never noise: it means the change
*did* reach the compiler's output, and the next move is
`avr-objdump -d` on both .elfs and a diff to see exactly where.

When the hashes match, say so in the commit message — it's one
line, and it's the strongest sentence a refactor commit can contain.

## 6. The debugging playbook

Symptom → ordered suspects. The order is by base rate, learned the
usual way.

**No ACK / can't reach the chip**
1. Wiring/continuity (meter).
2. The 7-bit address trap (`addr >> 1`).
3. Pullups missing (analyzer: bus not idle-high). 4.7 kΩ to the
   *chip's* rail.
4. Wrong rail (3.3 V part, 5 V bus).
5. Chip held in reset or sleep by a strapping pin (check every CE/
   EN/RST pin's required level in the datasheet — floating enable
   pins float differently on Tuesdays).
6. Bus lockup: a slave abandoned mid-read holds SDA low forever
   (its next data bit is 0). Recognize it on the analyzer (SDA low
   at idle); recover by clocking SCL manually up to 9 times until
   SDA releases, then STOP. Power-cycling the slave also works at
   the bench; the clock-out is what a robust `begin()` does.

**Reads return garbage (but the transaction completes)**
1. Endianness (write 0x55AA, read it back).
2. Missing repeated start — many chips require
   write-register-number → REPEATED START → read, and abandon the
   pointer on a STOP. `I2c::writeRead()` is that idiom, ready-made;
   a hand-rolled write-then-read pair is not the same thing on the
   wire. Check what the read primitive actually emits (analyzer).
3. Reading before data is ready (status/DRDY bit not polled;
   conversion time not waited).
4. Auto-increment assumed but absent, or present with a twist (some
   chips need a flag bit OR'd into the register number to enable
   it — classic on older Bosch parts and some radios).
5. Sign extension: a 24-bit two's-complement value in a `uint32_t`
   isn't negative until you make it so.

**SPI reads all-0xFF or all-0x00** — see §3's oracle. Add: wrong
SPI mode (capture and compare edges against the datasheet timing
diagram), and MISO/MOSI naming confusion at the module's silkscreen
(SDI/SDO/DIN/DOUT — beep it out to the chip pin if in doubt).

**Works, then stops after seconds/minutes**
1. Watchdog you forgot was armed (`.init3` guard exists precisely
   because of the reset loop this causes — check `resetCause()`).
2. A shared SPI bus with an unmanaged CS: some *other* device's CS
   floated low and it's talking over your chip. CS discipline: every
   device's CS driven, always, even devices you're not using yet.
3. Bus lockup mid-session (see above) after an ISR interrupted a
   transaction — bus transactions and ISRs that touch the same bus
   don't mix.
4. Brownout during a transmit burst (check with the meter on the
   rail, or resetCause() again).

**Works on the bench, flaky on the breadboard / at speed**
1. Decoupling: 100 nF ceramic *at the module's power pins*. The
   number of "software" bugs that are this is not funny.
2. Pullup strength vs wire capacitance: 400 kHz I²C through a
   breadboard rats-nest wants 2.2 kΩ, or wants to be 100 kHz.
   (Analyzer view: SDA/SCL rising edges visibly slow/curved.)
3. Long SPI jumpers at 8 MHz: shorten or slow down.

**The Heisenbug rule**: UART printing inside timing-sensitive code
changes the timing (a 9600-baud byte is ~1 ms of, at minimum,
interrupt traffic). When a bug vanishes under instrumentation,
switch to the analyzer, a scope pin (`PORTB |= _BV(0)` around the
suspect region), or a captured counter printed *later*.

---

# Part II — Three walkthroughs

Three parts, in escalating difficulty, each applying Part I
end-to-end: recon → crib sheet → first contact → skeleton → the
tricky fragments → verification arc. These are walkthroughs, not
implementations — the point is that after each one, the remaining
typing is mechanical.

## Walkthrough 1: HT16K33 (4-digit 7-segment over I²C)

The gentlest possible first driver: write-only in practice, no
endianness questions, no status polling, 5 V-happy (no level
shifting on a 5 V 328P — the only part in this document with that
courtesy), and failure is *visible* — the display either lights or
doesn't.

### Recon

Holtek HT16K33 datasheet. It's a RAM-mapped LED controller: you
write a 16-byte display RAM and it multiplexes the LEDs forever on
its own. Commands are **single bytes** where the high nibble is the
command class and the low nibble is the argument — there is no
register map in the usual sense. The crib sheet, in full:

- Address: **0x70** + 3 solder jumpers (A0–A2) → 0x70–0x77. 7-bit,
  conveniently quoted as such.
- Init, in order: `0x21` (system oscillator ON — the chip is
  comatose until this), `0x81` (display on, blink off; low bits
  select blink rates nobody wants), `0xE0 | dim` (brightness,
  dim = 0–15; note 0 is *dimmest*, not off).
- Display RAM: a write transaction of `{0x00, then up to 16 data
  bytes}` — first byte is the RAM start address, data auto-
  increments. Even bytes drive row A (the digits you have), odd
  bytes drive a second row that a 4-digit backpack doesn't wire up.
- On the common (Adafruit-style) 4-digit backpack, the five RAM
  positions are: digit, digit, **colon**, digit, digit — i.e. RAM
  bytes 0, 2, 4, 6, 8 with the colon at byte 4, bit 1.
- Segment font: standard 7-segment truth table, `0b0gfedcba`.
  Belongs in PROGMEM.

### First contact

Wire it (VCC, GND, SDA→PC4/27, SCL→PC5/28 on the 328P; the backpack
has pullups), run the ACK scan. Expect exactly one line: `ACK 0x70`.
Anything else, back to §3's scan diagnosis. Capture one scan pass
with the analyzer just to see a NAK-wall with one ACK in it — a
useful calibration for your eyes.

### Skeleton

```cpp
namespace HAL {
namespace Devices {

template<typename I2c, uint8_t addr = 0x70>
struct HT16K33Quad7 {

    enum class Result : uint8_t { OK, NACK };

    static Result begin();                    // 0x21, 0x81, 0xE0|15
    static Result setBrightness(uint8_t d);   // 0..15
    static Result showDigits(uint8_t d0, uint8_t d1,
                             uint8_t d2, uint8_t d3, bool colonP);
    static Result showNumber(uint16_t n);     // Fmt::fixed + font
    static Result clear();

  private:
    static Result cmd(uint8_t c);             // one-byte write
    static const uint8_t FONT[16] PROGMEM;    // 0-9 A-F
};

}
}
```

### The fragments that matter

The single-byte command — everything in `begin()` is three of these:

```cpp
static Result cmd(uint8_t c) {
    return I2c::write(addr, &c, 1) == HAL::Comms::I2C::Result::OK
               ? Result::OK : Result::NACK;
}
```

And the RAM write with the backpack's interleaved layout — this is
the entire "hard part" of the driver:

```cpp
static Result showDigits(uint8_t d0, uint8_t d1,
                         uint8_t d2, uint8_t d3, bool colonP) {
    uint8_t buf[11] {
        0x00,                                  // RAM start address
        seg(d0), 0,                            // odd bytes: unwired row
        seg(d1), 0,
        static_cast<uint8_t>(colonP ? 0x02 : 0x00), 0,   // colon @ byte 4
        seg(d2), 0,
        seg(d3), 0,
    };
    return I2c::write(addr, buf, sizeof(buf)) == ...;
}
```

(`seg()` is a PROGMEM lookup: `pgm_read_byte(&FONT[v & 0x0F])`.)

`showNumber` is then `Fmt::fixed` into four chars → four font
lookups — the formatting layer already exists; don't re-peel digits.

### Verification arc

1. `begin()` alone → nothing visible (RAM is zero) but three ACKed
   transactions on the analyzer.
2. Lamp test: all segments on (`showDigits(8,8,8,8,true)` with a
   font where 8 = 0x7F). Any dead segment is hardware, and you found
   it before writing display logic.
3. `showNumber(1234)`. If digits are scrambled *in a pattern* —
   e.g. everything shifted one position — recount the interleaved
   layout; if a single digit renders wrong *shapes*, the font table
   has a bad row.
4. Brightness sweep 0→15, blink off confirmed.
5. Done-state: wire it as the alarm-clock's face (it's a clock
   display with a colon; the DS3231 example is sitting right there),
   or hand it a `Scheduler` cadence in a new example.

Total driver: ~80 lines. The walkthrough exists because it exercises
every Part I habit — crib sheet, scan, analyzer, increments — with
nothing else to go wrong.

## Walkthrough 2: RDA5807 (FM receiver over I²C)

Medium difficulty, and the fun one: 16-bit big-endian registers, a
chip with **two I²C personalities**, a status-polling idiom
(tune-then-wait), a stream of asynchronous data (RDS) that wants the
process()-pattern, and your ears as a debugging instrument.

### Recon

The RDA5807M datasheet is thin and awkwardly translated — this is
the walkthrough where "read an existing driver for wire truths"
earns its keep (several Arduino drivers exist; read one against the
register map and note where they disagree with the paper). Crib
sheet highlights:

- **3.3 V part**, and the common RRD-102 module has *no* regulator
  and *no* shifter. On a 5 V 328P: pull the bus up to 3.3 V only
  (marginal-legal, breadboard-fine) or run the whole system at
  3.3 V/8 MHz. Decide before wiring, not after.
- **Two addresses, same chip**: `0x10` is "sequential mode" — reads
  and writes always start at register 0x02 and auto-increment,
  TEA5767-style, no register numbers on the wire. `0x11` is "index
  mode" — write the register number, then the data; random access.
  **Use 0x11 and pretend 0x10 doesn't exist**; sequential mode
  exists for retrofitting TEA5767 products.
- Registers are **16-bit, big-endian on the wire** (high byte
  first). Chip ID: register 0x00 high byte = 0x58.
- The registers that matter: `0x02` (DHIZ audio-enable, DMUTE,
  SEEK/SEEKUP, soft-reset bit, ENABLE), `0x03` (channel + TUNE
  strobe + band + spacing), `0x05` (volume, low nibble), `0x0A`
  (status: STC seek/tune-complete, RDSR ready, current channel),
  `0x0B` (RSSI), `0x0C`–`0x0F` (RDS blocks A–D).
- Tune math, 100 kHz spacing, band 87–108: `chan = freq_in_100kHz
  − 870`, e.g. 98.1 MHz → 981 − 870 = 111. Write chan into 0x03's
  top ten bits with TUNE set; poll STC in 0x0A.
- Module wants a 32.768 kHz crystal — the RRD-102 has it onboard.

### First contact

ACK scan. The chip answers **both 0x10 and 0x11** (some clones also
answer 0x60, the TEA5767 compatibility address) — a two-or-three-ACK
scan from one chip, which is a nice sanity check that the scan and
your mental model agree. Then rung 2/3 in one move: read register
0x00 via 0x11 and check the high byte is 0x58. This proves indexed
reads *and* endianness at once.

### Skeleton

```cpp
template<typename I2c>
struct RDA5807 {

    enum class Result : uint8_t { OK, NACK, WRONG_CHIP, TUNE_TIMEOUT };

    static Result  begin();                  // reset, enable, unmute
    static Result  tune(uint16_t freq100k);  // 981 = 98.1 MHz
    static Result  seekUp();                 // hardware seek + STC wait
    static Result  setVolume(uint8_t v);     // 0..15
    static uint8_t rssi();
    static bool    processRDS();             // call from the main loop;
    static const char* stationName();        //   8 chars, NUL-terminated

  private:
    static constexpr uint8_t IDX { 0x11 };   // the indexed personality
    static Result writeReg(uint8_t reg, uint16_t v);
    static Result readReg(uint8_t reg, uint16_t& v);
    static Result waitSTC();                 // bounded poll on 0x0A
    // RDS accumulation state: psName[9], seen-mask, etc.
};
```

### The fragments that matter

The register primitives — get the endianness right here and the
rest of the chip is just bit maps:

```cpp
static Result writeReg(uint8_t reg, uint16_t v) {
    uint8_t buf[3] { reg,
                     static_cast<uint8_t>(v >> 8),     // BIG-endian:
                     static_cast<uint8_t>(v) };        // high byte first
    return I2c::write(IDX, buf, 3) == ... ;
}

static Result readReg(uint8_t reg, uint16_t& v) {
    // write the register number, REPEATED START, read two bytes --
    // a STOP between them resets the chip's pointer. This is exactly
    // what I2c::writeRead() exists for (verify on the analyzer: ONE
    // transaction with an Sr in the middle, not two transactions)
    uint8_t d[2];
    if (I2c::writeRead(IDX, &reg, 1, d, 2) != ...) return Result::NACK;
    v = static_cast<uint16_t>((static_cast<uint16_t>(d[0]) << 8) | d[1]);
    return Result::OK;
}
```

Tune with a bounded wait — the house wait idiom applied:

```cpp
static Result tune(uint16_t freq100k) {
    uint16_t chan { static_cast<uint16_t>(freq100k - 870) };
    // 0x03: [15:6] channel, [4] TUNE, [3:2] band 00 (87-108),
    //       [1:0] spacing 00 (100 kHz)
    Result r { writeReg(0x03,
                   static_cast<uint16_t>((chan << 6) | 0x0010)) };
    if (r != Result::OK) return r;
    return waitSTC();          // ~200 tries x 1 ms -> TUNE_TIMEOUT
}
```

RDS is the interesting design decision. Groups arrive twice a
second-ish; each group carries 2 characters of the 8-character
station name (group type 0A/0B) in block B's low bits + block D.
This is a *slow accumulation*, so the driver follows the
notify/process split in spirit: `processRDS()` is called from the
main loop, polls RDSR in 0x0A (cheap read), and when a group is
ready, reads 0x0C–0x0F and slots two chars into `psName` at the
position block B dictates. A seen-mask of 4 bits tells you when all
four pairs have landed at least once; that's when `stationName()`
starts returning something. No ISR, no ticker — the main loop's
natural cadence is plenty. Clear the accumulation on every `tune()`.

### Verification arc

The radio's gift to bring-up: **static is a signal too.**

1. Chip-ID read (0x58) — plumbing proven.
2. `begin()` (soft reset via 0x02's reset bit, then ENABLE + DMUTE
   + DHIZ set) → headphones on the module jack → *you hear hiss*.
   That hiss is the entire analog path, power, and clock proven in
   one sensory bit. No hiss: check DHIZ (output high-impedance bit
   — its sense is inverted from what you'd guess; another wire
   truth to crib from existing drivers).
3. `tune(981)` (or whatever transmits strong locally) → music.
   Wrong station → recheck the −870 math and band bits. Nothing but
   hiss → antenna (a 60 cm jumper wire on the ANT pad works in a
   city).
4. `setVolume` sweep; `seekUp()` hopping stations.
5. RDS: print `stationName()` once the mask fills — expect the
   call-sign scrolling in on a strong station within ~2 s. Garbage
   characters → you've got block B's position bits misaligned;
   they're the low 2 bits, and each position writes chars 2n,2n+1.
6. Done-state: this + LCD1602 + the encoder is obviously a radio
   appliance example (`examples/fm-radio/`), and pairs naturally
   with the HT16K33 as a frequency display.

## Walkthrough 3: BMP581 (barometric pressure over I²C)

The modern-register-machine walkthrough: a part where the datasheet
is excellent and the discipline is *sequencing* — check ID, reset,
poll status, configure, then read data that's only valid when the
chip says so. Everything learned here transfers to the whole class
of current Bosch/ST/TDK sensors. (A BMP580 runs the identical code
— it's the superseded silicon revision of the same part; if one's
in the drawer, nothing below changes.)

### Recon

Bosch BMP581 datasheet — and this generation shares *nothing* with
the BMP180/280 lineage: new register map, new protocol, and
mercifully, **compensation runs on-chip**. No calibration
coefficients to read, no 40-line polynomial to transcribe: pressure
comes out in pascals. Crib sheet:

- **1.8–3.6 V.** Same rail decision as the RDA5807. Not optional,
  not marginal — decide the system voltage first.
- Address **0x46** (SDO low) / **0x47** (SDO high) — a floating SDO
  gives a coin-flip address; strap it.
- `CHIP_ID` @ 0x01 = **0x50**.
- Soft reset: write **0xB6** to `CMD` @ 0x7E, wait ~2 ms, then
  check `INT_STATUS` @ 0x27 bit 4 (POR complete) and re-read ID.
- `STATUS` @ 0x28: NVM ready/error bits — check once at begin().
- Config: `OSR_CONFIG` @ 0x36 (oversampling for T and P, plus
  **press_en bit 6 — pressure measurement is OFF by default**, the
  one genuine gotcha in the map), `ODR_CONFIG` @ 0x37 (mode in bits
  1:0 — standby/normal/forced/non-stop — and output rate).
- Data: **six registers 0x1D–0x22**, temperature then pressure,
  each a 24-bit value, **little-endian** (LSB first — opposite of
  the RDA5807; this pairing is why §2 says write endianness down
  per-chip and never assume). Temp in °C = raw/2¹⁶; pressure in
  Pa = raw/2⁶.
- `INT_STATUS` bit 0 = data ready; readable by polling (an INT pin
  exists when a future design wants wake-on-sample).

### First contact

Rail check with the meter *before* power-on (a 5 V accident here is
not recoverable). ACK scan → 0x46. Read 0x01 → 0x50, or
`WRONG_CHIP` — and that Result exists because SPI-strapped boards,
sibling parts, and clones all ACK happily and then aren't the chip
you think they are.

### Skeleton

```cpp
template<typename I2c, uint8_t addr = 0x46>
struct BMP581 {

    enum class Result : uint8_t {
        OK, NACK, WRONG_CHIP, RESET_TIMEOUT, DATA_TIMEOUT
    };

    static Result begin();          // id -> reset -> POR ok -> config
    static Result readForced(int32_t& milliCelsius, uint32_t& pascals);
    // (a normal-mode/ODR streaming variant can come later; forced
    //  one-shot is the battery-friendly shape and the simpler start)

  private:
    static Result writeReg(uint8_t reg, uint8_t v);
    static Result readRegs(uint8_t reg, uint8_t* dst, uint8_t n);
    static Result waitDataReady();  // bounded poll on INT_STATUS
};
```

### The fragments that matter

`begin()` is the sequencing lesson in code form:

```cpp
static Result begin() {
    uint8_t id;
    if (readRegs(0x01, &id, 1) != Result::OK) return Result::NACK;
    if (id != 0x50)                           return Result::WRONG_CHIP;

    writeReg(0x7E, 0xB6);                     // soft reset
    // datasheet: t_soft_res ~2 ms -- wait, then require POR bit
    __builtin_avr_delay_cycles(F_CPU / 400);  // 2.5 ms
    uint8_t s;
    for (uint8_t tries = 10; ; --tries) {
        if (tries == 0) return Result::RESET_TIMEOUT;
        if (readRegs(0x27, &s, 1) == Result::OK && (s & 0x10)) break;
        __builtin_avr_delay_cycles(F_CPU / 1000);
    }

    // osr_config: press_en (bit 6) | OSR choices. Pressure stays
    // disabled without press_en, and the chip will happily hand you
    // temperature-only forever while you debug the wrong thing
    return writeReg(0x36, 0x40 | OSR_BITS);
}
```

The burst read with the little-endian 24-bit unpack — the second
place bugs live:

```cpp
static Result readForced(int32_t& mC, uint32_t& pa) {
    // ODR_CONFIG: forced mode strobe (mode bits = 0b10)
    ...
    Result r { waitDataReady() };             // INT_STATUS bit 0,
    if (r != Result::OK) return r;            //   bounded, ~50 ms cap

    uint8_t d[6];                             // one burst: 0x1D..0x22
    if (readRegs(0x1D, d, 6) != Result::OK) return Result::NACK;

    // LSB FIRST. temp is signed 24-bit: assemble in uint32_t, shift
    // up to bit 31, arithmetic-shift back down to sign-extend
    int32_t traw { static_cast<int32_t>(
        (static_cast<uint32_t>(d[2]) << 24)
      | (static_cast<uint32_t>(d[1]) << 16)
      | (static_cast<uint32_t>(d[0]) << 8)) >> 8 };
    uint32_t praw { static_cast<uint32_t>(d[3])
                  | (static_cast<uint32_t>(d[4]) << 8)
                  | (static_cast<uint32_t>(d[5]) << 16) };

    mC = ...;   // traw * 1000 / 65536, kept in integer math --
                //   (traw * 125) >> 13 gets you there without overflow
    pa = praw >> 6;                           // Pa exactly
    return Result::OK;
}
```

Both conversions deserve a host test (§4): feed the assembled-bytes
functions a datasheet worked example and a few hand-built negative
temperatures — the sign-extension shift is precisely the kind of
line that reads right and isn't.

### Verification arc

1. ID handshake. 2. Reset + POR bit observed on the analyzer as a
   real sequence (write 0x7E, gap, reads of 0x27).
3. First `readForced()` at the desk: expect **~95,000–103,000 Pa**
   (weather and altitude move it; 101,325 is sea-level-standard,
   not a law) and a plausible room temperature. Pressure exactly 0
   or wildly off → press_en forgotten, or the LE unpack is
   byte-swapped (0x55AA test on a scratch register settles it).
4. Breathe on it: temperature jumps a degree, pressure spikes.
   Physical causality is the best integration test.
5. The party trick that proves the noise floor: log pressure once a
   second and *lift the board one meter*. ~12 Pa is unmistakable on
   this part. That's the moment this chip stops being abstract.
6. Done-state: a `pressure` column in the weather-logger's record
   (it's one more field in an existing example), or altitude trend
   on the LCD. Altitude math is a caller concern — log pascals, and
   keep the barometric formula out of the driver.

---

## Coda: the pattern under all three

Every walkthrough was the same five moves wearing different
registers: **crib the datasheet** down to one page, **prove reach**
(ACK/ID) before anything else, **build the register primitives**
and verify them with a readback, **sequence the init** exactly as
the datasheet orders it with every wait bounded, and **climb to the
feature** one printed rung at a time with the analyzer as the
referee. When a fourth part lands on the desk — a CAN controller,
an OLED, a DAC — it will be the same five moves. Write the crib
sheet first.
