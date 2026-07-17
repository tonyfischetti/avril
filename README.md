# avril

A HAL for AVR microcontrollers that has to go and make things so complicated

[![forthebadge](https://forthebadge.com/images/badges/no-ragrets.svg)](http://forthebadge.com)

This is a personal project. I'm mostly focusing on the ATTiny85 and
ATMega328P.

---

## Table of contents

- [Philosophy](#philosophy)
- [Requirements and setup](#requirements-and-setup)
- [The concurrency contract](#the-concurrency-contract)
- [Core modules](#core-modules)
  - [`gpio.hpp` — pin abstraction](#gpiohpp--pin-abstraction)
  - [`ticker.hpp` — 1 kHz millisecond timebase](#tickerhpp--1-khz-millisecond-timebase)
  - [`sleep.hpp` — sleep-mode helper](#sleephpp--sleep-mode-helper)
  - [`watchdog.hpp` — timed wake-ups from power-down](#watchdoghpp--timed-wake-ups-from-power-down)
- [Comms](#comms)
  - [`comms/i2c.hpp` — blocking I²C master](#commsi2chpp--blocking-ic-master)
  - [`comms/spi.hpp` — blocking SPI master](#commsspihpp--blocking-spi-master)
  - [`comms/uart.hpp` — debug serial (ATmega328P only)](#commsuarthpp--debug-serial-atmega328p-only)
- [FS](#fs)
  - [`fs/fat32.hpp` — read-only FAT32](#fsfat32hpp--read-only-fat32)
- [Utils](#utils)
  - [`IntTransitionDebouncer` — interrupt-driven debouncing](#inttransitiondebouncer--interrupt-driven-debouncing)
  - [`LFSR` — fast 8-bit pseudorandomness](#lfsr--fast-8-bit-pseudorandomness)
- [Devices](#devices)
  - [`Button`](#button)
  - [`RotaryEncoder`](#rotaryencoder)
  - [`RotaryEncoderWithButton`](#rotaryencoderwithbutton)
  - [`LCD1602` — 16×2 text over I²C](#lcd1602--162-text-over-ic)
  - [`SD::Card` — raw-block SD storage](#sdcard--raw-block-sd-storage)
  - [`VS1053::Codec` — MP3 (and more) playback](#vs1053codec--mp3-and-more-playback)
- [Putting it together: the canonical main loop](#putting-it-together-the-canonical-main-loop)
- [Known limitations](#known-limitations)

---

## Philosophy

avril makes a few opinionated bets, and everything else follows from them:

**1. Configuration belongs in template parameters, not runtime state.**
A pin number, a debounce window, or a baud rate never changes while the
program runs, so it should never occupy RAM or cost an indirection. A
`Button<3, 30, 1000, HIGH, true>` bakes its pin mask and timing into the
generated code; the optimizer constant-folds the lot and dead-code-eliminates
the polarity branches you didn't take. On a chip with 512 bytes of SRAM
(ATtiny85), this matters. The cost is C++ template syntax; the benefit is
that an entire device driver often compiles down to what you'd have written
by hand in C, but with the wiring mistakes caught by `static_assert` at
compile time.

**2. Header-only where possible.**
Everything except the Ticker (which owns an ISR and global state) and the
LFSR (hand-written assembly) lives entirely in headers. Client projects add
one `-I` flag and compile `avril/src/*.cpp` and `avril/src/**/*.S` alongside
their own sources. With LTO enabled there is no penalty for this, and it
keeps the build system trivial.

**3. Interrupts notify; the main loop processes.**
The single most important design rule in the library. Pin-change ISRs do the
minimum work that genuinely must happen at interrupt time — timestamp the
edge and record it — and *all* state-machine logic, including user callbacks,
runs in main-loop context. See [The concurrency contract](#the-concurrency-contract).

**4. Sleep is a first-class concern.**
The Ticker can be paused and resumed with tick compensation, debouncers
report whether they have in-flight work that must keep the MCU awake, and
the whole input stack is designed so a project can drop into
`SLEEP_MODE_PWR_DOWN` between events and `SLEEP_MODE_IDLE` between animation
frames. Battery projects are the intended audience.

**5. Only three MCUs, checked at compile time.**
`common.hpp` refuses to compile for anything but the ATtiny84, ATtiny85,
and ATmega328P. Supporting "all AVRs" is how HALs become the complicated
thing this library's tagline is making fun of. Where the chips differ
(register names, ISR vectors, pin tables), the difference is handled in one
`#if` per module, not an abstraction layer.

## Requirements and setup

- avr-gcc with C++20 support (`-std=gnu++2a`; developed against avr-gcc 12–14)
- `-mmcu=attiny85`, `-mmcu=attiny84`, or `-mmcu=atmega328p` (an ATtiny84A
  builds as `-mmcu=attiny84`; same registers, different compiler macro)
- `F_CPU` defined (the Ticker `static_assert`s that an exact 1 ms divisor exists for it)
- Recommended: `-Os -flto -ffunction-sections -fdata-sections -Wl,--gc-sections`

```make
HAL_LOC   := ../avril
SRC       += $(wildcard $(HAL_LOC)/src/*.cpp)
SRC       += $(wildcard $(HAL_LOC)/src/**/*.cpp)
SRC       += $(wildcard $(HAL_LOC)/src/**/*.S)
CXXFLAGS  += -I$(HAL_LOC)/include
```

Then either include the umbrella header or individual modules:

```cpp
#include "avril.hpp"                          // everything
#include "devices/RotaryEncoderWithButton.hpp" // devices are opt-in
```

`common.hpp` (pulled in by everything) also defines `HIGH`/`LOW` as
`constexpr bool` and the `Callback` type (`void (*)()`) — plain function
pointers, not `std::function`, because there is no heap here.

## The concurrency contract

Every shared-state bug this library has ever had came from breaking one of
these rules, so they are now the rules:

1. **ISRs call `notifyInterruptOccurred(now, changed)` and nothing else** on
   avril objects. Notification is cheap (a masked compare and up to two
   volatile stores) and safe by construction.
2. **`process()` / `processAnyInterrupts()` run only in main-loop context.**
   User callbacks therefore also run only in main-loop context, so ordinary
   application state touched by callbacks needs no `volatile` and no atomics.
3. **Objects that live on the ISR/main boundary use `ATOMIC_BLOCK` claim
   sections for every cross-context handoff** — the debouncer and the
   rotary encoder's step accumulator are the residents. If you find
   yourself wanting to share anything else across that boundary, either
   route it through a claim of the same shape or reconsider.
4. **Anything checked before sleeping must be re-checked with interrupts
   disabled.** See [the canonical main loop](#putting-it-together-the-canonical-main-loop).

**The sanctioned exception: ISR-side decoders.** Rule 2 pushes state
machines into the main loop, and for buttons that is strictly better. But
some signals carry their information in the *instant* of the edge — the
phase relationship between a quadrature encoder's two channels, the width
of a protocol pulse — and that information is already stale by the time
the main loop runs. For those, the decode itself may run in the ISR,
provided three invariants hold:

1. the decoder's working state is touched from **ISR context only**;
2. results cross to the main loop through a single **atomic claim** (a
   counter or a latch, consumed under `ATOMIC_BLOCK` in `process()`);
3. callbacks still fire **only from `process()`** — never from the ISR.

The contract's actual purpose — one writer per datum, no two-context
mutation — survives intact; only the *location* of the state machine
moves. `RotaryEncoder` is the resident example (its Gray-code table folds
each edge into a signed quarter-step accumulator in the ISR), and any
future pulse-timing decoder (IR remotes, for instance) should take the
same shape.

The canonical pin-change ISR looks like this (one per port; `PCINT0_vect`
shown):

```cpp
volatile uint8_t previousPINB { 0xFF };

ISR(PCINT0_vect) {
    HAL::Ticker::resume(1);                  // no-op unless we were sleeping
    uint32_t now     = HAL::Ticker::getNumTicks();
    uint8_t  current = PINB;
    uint8_t  changed = current ^ previousPINB;
    previousPINB     = current;

    device.notifyInterruptOccurred(now, HAL::GPIO::Port::B, changed);
}                                       // ^ notify. don't process.
```

Why `resume(1)` first: if the MCU was in `PWR_DOWN`, Timer0 was stopped and
`getNumTicks()` would return a stale value; `resume` restarts the clock and
adds a 1-tick compensation so timestamps taken in the wake-up ISR are sane.
If the ticker wasn't paused, `resume` does nothing.

The `Port` argument says which port the `changed` mask belongs to, and each
device drops notifications for ports it doesn't live on. On the ATtiny85
this is trivia (there is only PORTB). On the ATtiny84 (PA/PB, with
`PCINT0_vect`/`PCINT1_vect` respectively) and the ATmega328P (three ports,
three vectors) it is load-bearing: bit positions collide across ports — so
write one ISR per port you use (each with its own `previousPINx`), pass the
right `Port`, and you can then spread devices across ports freely and
notify every device from every ISR without cross-talk. The port comparison is against a `constexpr`, so it costs
nothing when the argument is a compile-time constant.

## Core modules

### `gpio.hpp` — pin abstraction

Pins are named by **physical DIP pin number**, not port/bit. Rationale: when
you're probing a breadboard at 1am, the number silkscreened next to the leg
is the one you can see. A compile-time table maps physical pins to
port/bit/PCINT info, and invalid pins (power pins, XTAL, out-of-range) fail
at compile time.

```cpp
using LED = HAL::GPIO::GPIO<6>;   // ATtiny85 physical pin 6 == PB1

LED::setOutput();
LED::setHigh();
LED::toggle();
bool v = LED::read();

using BTN = HAL::GPIO::GPIO<3>;   // PB4
BTN::setInputPullup();
BTN::enablePCINT();               // sets GIMSK/PCICR bit + PCMSK mask
```

Everything is `static`; a `GPIO<N>` is a namespace with a type's syntax.
Register selection happens entirely at compile time on all three MCUs: SFR
addresses aren't C++ constant expressions, so instead of a pointer table the
register *lvalue* is chosen by `if constexpr` on the pin's (constexpr) port.
The result is zero RAM per pin and each operation compiling to a single
`sbi`/`cbi`/`sbic` where the register is in low I/O space. Power, crystal,
and other non-GPIO physical pins are rejected by `static_assert`.

### `ticker.hpp` — 1 kHz millisecond timebase

Timer0 in CTC mode, interrupting at exactly 1 kHz. "Exactly" is enforced:
prescaler selection happens in `constexpr` code that searches
{8, 64, 256, 1024} for a prescaler where `F_CPU / (prescaler * 1000)` is an
integer that fits OCR0A, and **fails the build** if none exists — a wrong
clock silently drifting is much worse than a compile error.

```cpp
HAL::Ticker::setupMSTimer();       // configure + enable compare interrupt
sei();

uint32_t now = HAL::Ticker::getNumTicks();   // atomic 32-bit read
```

`getNumTicks()` wraps after ~49.7 days; consumers must compare timestamps
with unsigned subtraction (`now - then >= window`), which is wrap-safe.

For measurements finer than a millisecond there is `getMicros()`, which
combines the tick count with the live `TCNT0` value — sub-millisecond
information the timer hardware generates anyway:

```cpp
uint32_t nowUs = HAL::Ticker::getMicros();   // 8 us resolution at 8 MHz
```

Resolution is `prescaler / MHz` microseconds per count (8 µs at 8 MHz / 64,
4 µs at 16 MHz / 64). It handles the classic `micros()` race — a compare
match that fires after interrupts are disabled (or while executing inside an
ISR, where the tick ISR cannot run) leaves `ticks` stale by one; the pending
`OCF0A` flag is checked and the missing millisecond credited manually — so
it is safe to call from ISR context. Three caveats: it wraps every ~71.6
minutes (same wrap-safe subtraction rule); it is *resolution*, not
*accuracy* (the timebase is still your clock source — an internal RC
oscillator is a few percent off nominal, which cancels out when comparing
intervals against each other but makes this no wall clock); and it is
meaningless while the ticker is paused. If interrupts stay disabled for more
than one full millisecond, the excess beyond the first pending tick cannot
be detected and the result under-reports — don't do that.

The pause/resume pair exists for `SLEEP_MODE_PWR_DOWN`, which stops Timer0's
clock:

```cpp
HAL::Ticker::pause();              // saves TCCR0B, stops the timer
// ... sleep ...
HAL::Ticker::resume(1);            // restores TCCR0B, adds 1 tick compensation
```

Both are atomic and idempotent: `pause()` on an already-paused timer will not
clobber the saved prescaler bits, and `resume()` on a running timer is a
no-op. (These properties are load-bearing — an earlier version had a race
here that could permanently freeze the timebase.)

The module owns `ISR(TIM0_COMPA_vect)` / `ISR(TIMER0_COMPA_vect)`, so Timer0
belongs to avril in any project that uses the Ticker.

### `sleep.hpp` — sleep-mode helper

```cpp
HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);      // or SLEEP_MODE_PWR_DOWN, etc.
```

One function, but the instruction ordering inside it is the entire point:

```cpp
cli();
set_sleep_mode(mode);
sleep_enable();
sei();          // AVR guarantees the *next* instruction executes
sleep_cpu();    // before any pending interrupt is serviced
sleep_disable();
```

The `sei(); sleep_cpu();` pair is the standard AVR idiom that makes
"sleep unless something is already pending" atomic: an interrupt that fires
at any point either runs before `sleep_enable()` matters or wakes the chip
immediately out of `sleep_cpu()`. It cannot slip into a gap and leave you
sleeping through an event you already received.

Note that `goToSleep` returns with interrupts **enabled**, and note what it
does *not* do: decide whether sleeping is a good idea. Gating conditions
(pending debounces, dirty flags) must be checked by the caller **with
interrupts disabled** — see the canonical loop below. For `SLEEP_MODE_IDLE`
with the Ticker running, no gate is needed at all: the next tick wakes you
within a millisecond regardless.

### `watchdog.hpp` — timed wake-ups from power-down

The watchdog timer runs off its own on-chip 128 kHz oscillator, which
makes it **the only timer on the chip that keeps running in
`SLEEP_MODE_PWR_DOWN`**. That is what this module exploits: a wake-up
source from deep sleep, at a few microamps, in nominal periods of
16 ms – 8.2 s. (The classic dead-man's-switch reset mode is a possible
future addition; the register documentation in the header covers it.)

```cpp
EMPTY_INTERRUPT(WDT_vect)   // REQUIRED -- see below

// sleep ~1 minute in 8 s chunks; a pin-change wake cuts it short
using namespace HAL::Watchdog;
uint16_t completed { sleepFor<Timeout::MS8192>(7) };
if (completed == 7) { /* the full time elapsed: nobody touched anything */ }
else                { /* a foreign interrupt woke us early */ }
```

The pieces, individually:

- **`sleepOnePeriod<Timeout>()`** — one watchdog period of `PWR_DOWN`.
  It arms interrupt mode (WDIE), pauses the Ticker, sleeps, and on wake
  answers the question "who woke me?" by reading WDIE back: hardware
  clears WDIE when the WDT vector executes, so a still-set bit means a
  *foreign* interrupt (a pin change, say) woke the chip mid-period. On a
  watchdog wake the Ticker is credited the nominal period; on a foreign
  wake the waking ISR's own `Ticker::resume()` already handled it
  (`resume()` on a running ticker is a no-op, so the two never
  double-credit).
- **`sleepFor<Timeout>(n)`** — chains `n` periods and stops early on a
  foreign wake, returning the number of *completed* periods so the
  caller can tell "the timer ran out" (`== n`) from "someone touched
  something" (`< n`). This is the building block for idle auto-off:
  sleep in 8 s chunks, and only act when the full count completes.
- **`resetCause()`** — why did the chip last reset? `POWER_ON`,
  `EXTERNAL`, `BROWNOUT`, or `WATCHDOG`, decoded from a snapshot of
  MCUSR taken before `main()` (see below). A firmware that flashes an
  error color when it finds itself watchdog-rebooted is a firmware whose
  bugs announce themselves.
- **`enableInterrupt<Timeout>()` / `disable()`** — the raw arm/disarm,
  with the WDCE/WDE timed sequences done correctly and `SREG` restored
  rather than blindly re-enabling interrupts.

**You must define `ISR(WDT_vect)`** (the app owns ISRs, as always —
`EMPTY_INTERRUPT(WDT_vect)` suffices for pure wake-ups). Without one,
avr-libc's `__bad_interrupt` jumps to the reset vector, and every
watchdog wake-up becomes a silent reboot — a genuinely confusing way to
fail.

**The `.init3` foundation** (`src/watchdog.cpp` — linked automatically
if your Makefile globs `src/*.cpp` per the setup section): after a
watchdog reset, WDRF is set in MCUSR, and while WDRF is set the hardware
**forces WDE on** with the prescaler reset to its shortest timeout —
the dog is already running again, at 16 ms, before `main()` begins, and
any slower startup resets forever. So a `naked` function in section
`.init3` (after the stack exists, before static initialization) snapshots
MCUSR into a `.noinit` variable, clears it, and disables the watchdog.
This is avr-libc's canonical pattern, and the snapshot is what
`resetCause()` reports.

Accuracy caveat, worth repeating from the header: the 128 kHz oscillator
is an RC circuit — expect ±10%, drifting with voltage and temperature.
`Timeout::MS8192` means "about eight seconds," which is exactly right
for "auto-off after ten minutes" and exactly wrong for timekeeping. The
Ticker credit keeps `getNumTicks()` *roughly* monotonic across sleeps,
not accurate.

## Comms

Buses for talking to *other chips*, as distinct from the top-level modules
(the MCU's own facilities), `devices/` (things wired to pins), `fs/`
(filesystems over block devices), and `utils/` (software helpers).
Everything here lives under `HAL::Comms::`, mirroring how `devices/` maps
to `HAL::Devices`. Note that drivers for chips that *sit on* a bus (an SD
card, an MCP2515) belong in `devices/`, consuming a comms module — bus
versus thing-on-the-bus is the boundary — and anything that interprets a
block device's *contents* belongs in `fs/`.

The extra namespace level costs a few characters at every debug print;
the idiomatic relief is an alias at the top of an app file:

```cpp
namespace UART = HAL::Comms::UART;   // then UART::println(...) as before
```

### `comms/i2c.hpp` — blocking I²C master

Transaction-shaped, error-honest, and available on **all three MCUs** —
the 328P through its hardware TWI peripheral, the tinies through a
bit-banged open-drain master on **any two pins**:

```cpp
// 328P (fixed pins SDA=PC4/27, SCL=PC5/28):
using I2c = HAL::Comms::I2C::Master<100000>;          // ceiling in Hz
// tinies (pins are template params, then the ceiling):
using I2c = HAL::Comms::I2C::Master<5, 7, 100000>;    // SDA, SCL

I2c::begin();
uint8_t reg { 0x0F };
uint8_t id;
if (I2c::writeRead(0x48, &reg, 1, &id, 1) == HAL::Comms::I2C::Result::OK) {
    // wrote the register pointer, repeated-START, read one byte back
}
```

The API is `write(addr7, buf, len, stopP)` / `read(...)` /
`writeRead(...)` / `ping(addr7)`, and **every call returns a `Result`**:
`OK`, `NACK_ADDR` (nobody home), `NACK_DATA` (device rejected a byte),
`BUS_ERROR`, or `TIMEOUT`. Unlike SPI, I²C can fail — a missing device,
a wedged one holding SCL low, absent pull-ups — so **every wait in the
module is bounded** and a dead bus becomes an error code, never a hung
main loop. On any error the module sends a STOP to free the bus before
returning.

Things worth knowing:

- **Addresses are 7-bit** (0x00–0x7F); the R/W bit is the module's
  business. A datasheet that says "write address 0x78, read address
  0x79" means the 7-bit address 0x3C.
- **Pull-ups are mandatory physics.** The lines are open-drain and only
  ever driven low. On the 328P the internal ~35k pull-ups are enabled by
  default (template param) and can carry a short bench wire at 100 kHz —
  marginal beyond that; fit 4.7k resistors for anything real. On the
  tinies **external pull-ups are required, full stop**: the open-drain
  emulation keeps PORT at 0 so DDR alone flips between drive-low and
  release, which makes the internal pull-ups unusable by construction.
- **`writeRead` is the sensor idiom** — register pointer, then repeated
  START (no STOP in between: a STOP would let another master barge in,
  and many parts reset their register pointer on it), then read. The
  `stopP=false` argument on `write`/`read` exposes the same mechanism
  for hand-rolled transaction shapes.
- **`ping(addr7)`** is an address probe: loop it over 0x08–0x77 and
  that's a bus scanner.
- **Clock stretching is honored in both backends** — in hardware on the
  328P, and in the bit-banged master by waiting (bounded) for SCL to
  actually rise after releasing it.
- **`maxHz` is a ceiling**, as in the SPI module. The 328P picks the
  largest TWBR rate at or below it (build fails outside TWBR's range:
  ~30–444 kHz at 16 MHz); the bit-banged backend times half-periods to
  the ceiling and its loop overhead only slows it further.
- **Master only, blocking, no ISR** — one transaction at a time, errors
  where you can see them. An I²C *slave* is a planned separate module;
  it is ISR-driven by nature and a different beast entirely.

### `comms/spi.hpp` — blocking SPI master

SPI is barely a protocol — a shift register with a clock — which makes the
master side the easiest bus there is, and the gateway to SD cards, displays,
and CAN (via MCP2515). All three MCUs are supported, each through its own
silicon:

```cpp
using Spi = HAL::Comms::SPI::Master<400000>;  // ceiling in Hz; optional Mode
                                              // and BitOrder params follow
using CS  = HAL::GPIO::GPIO<16>;        // chip select is YOUR problem

Spi::begin();
CS::setOutput(); CS::setHigh();

CS::setLow();
uint8_t id { Spi::transfer(0x9F) };     // full duplex: byte out, byte in
Spi::write(0x42);                       // don't care what comes back
uint8_t b  { Spi::read() };             // clocks out 0xFF, returns the reply
CS::setHigh();
```

Design notes, in the usual spirit:

- **`maxHz` is a ceiling, not a promise** — the fastest achievable rate at
  or below it is chosen at compile time. On the 328P that's the smallest
  hardware divider that fits (`/2` … `/128`, SPI2X included); asking for
  slower than `F_CPU/128` fails the build, UART-style. On the tinies the
  strobe loop's natural rate is ~`F_CPU/12`, and compile-time delay padding
  (`__builtin_avr_delay_cycles`) is inserted only when the ceiling demands
  slower — e.g. an SD card's 400 kHz initialization phase.
- **`transfer()` cannot hang.** The master generates the clock, so unlike
  I²C or SD-card waits there is nothing to time out on. Blocking is safe
  here, not just simple.
- **CS is not managed** — chip select is a per-*device* concern, not a
  per-bus one; drive it with a `GPIO<pin>` (low = selected on virtually
  everything).
- **Backends.** The 328P uses the hardware SPI peripheral: all four modes,
  both bit orders, fixed pins MOSI=PB3(17)/MISO=PB4(18)/SCK=PB5(19). Beware
  **the SS trap**: in master mode, if SS (PB2, pin 16) is an *input* and
  anything pulls it low, the hardware silently demotes the peripheral to
  slave. `begin()` therefore sets SS as an output; it remains usable as an
  ordinary GPIO — a device's CS, even. The tinies use the **USI in
  three-wire mode** with a software-strobed clock (the datasheet's own SPI
  master recipe): modes 0/1 only, MSB-first only — that's all the USI can
  do, and asking for more is a `static_assert` failure, not a silent
  reinterpretation. USI pins are fixed: tiny85 DO=PB1(6)/DI=PB0(5)/
  USCK=PB2(7); tiny84 DO=PA5(8)/DI=PA6(7)/USCK=PA4(9). Note DO/DI are the
  USI's names: DO is this chip's output (→ slave's MOSI), DI its input
  (← slave's MISO).
- **Master only.** SPI slave has no flow control — the master clocks when
  it pleases and an unserviced byte is simply lost — so an AVR-as-SPI-slave
  needs an interrupt-driven design that this deliberately isn't. (Contrast
  I²C slave, where clock stretching makes the hardware wait for you.)

### `comms/uart.hpp` — debug serial (ATmega328P only)

Blocking, transmit-oriented, deliberately primitive — it exists so a 328P
project can print numbers at a human during bring-up, not to be a serial
framework. Compiles to nothing on the ATtiny84 and ATtiny85 (no USART).
Blocking is a *feature* at this altitude: no ISR, no buffer, no message
lost when the chip resets two instructions later, and a known bounded
cost (~87 µs per byte at 115200).

```cpp
namespace UART = HAL::Comms::UART;

UART::init<115200>();            // baud is a template parameter
UART::println_P(PSTR("booted")); // string lives in flash ONLY
UART::print("ticks: ");          // this one costs flash AND RAM
UART::println(someUint32);       // ~40 bytes of code, no snprintf
UART::println(-42);              // signed overloads do the right thing
UART::printlnHex(PINB);          // fixed-width uppercase hex: "2C"
UART::flush();                   // REQUIRED before sleeping/rebooting
```

The baud divisor is computed at compile time with proper rounding, and
double-speed mode (U2X) is selected automatically when it halves the rate
error — which is what makes 115200 @ 16 MHz actually work (naive truncated
math lands 8.5% off; rounded U2X lands within 2.1%). A rate that can't be
achieved within 2.5%, or that overflows the 12-bit UBRR register, is a
**compile error**, in the same spirit as the Ticker refusing an inexact
millisecond: a baud rate that's silently wrong is worse than one that
doesn't build.

Things worth knowing:

- **Use `print_P`/`println_P` with `PSTR()` for string literals.** A plain
  `print("...")` literal lands in `.data` — flash *and* a RAM shadow copied
  at startup. On a 2 KB-RAM part, debug strings are the classic silent RAM
  eater; `print_P(PSTR("..."))` keeps them in flash only.
- **Integer printing is hand-rolled** (digits peeled into a stack buffer),
  not `snprintf`-backed: the old `%lu` call dragged ~1.4 KB of avr-libc's
  formatted-print machinery into any build that printed a number. The full
  overload set — `uint32_t`/`int32_t`/`uint16_t`/`int16_t` — exists so
  plain `int` arguments (`int` == `int16_t` on AVR) resolve unambiguously;
  8-bit values promote to the `int16_t` overload and print numerically.
  `printByte()` sends a raw character.
- **`printHex` / `printlnHex`** (8/16/32-bit) print fixed-width uppercase
  hex with no prefix — two digits per byte, so a register dump reads like
  the datasheet.
- **Call `flush()` before sleeping.** `printByte` waits for the *buffer*
  (UDRE0), not the *shift register* (TXC0), so the final frame is still on
  the wire when the last `print` returns; a `goToSleep` (or watchdog
  reboot) right behind it mangles the last character mid-frame. `flush()`
  blocks until the transmitter is truly idle — `printByte` clears the
  sticky TXC0 flag as each byte is queued so `flush()` measures *this*
  transmission, and a one-byte "anything sent yet?" flag keeps a
  pre-first-print `flush()` from spinning forever on a flag that never
  latches.

Still deliberately absent: interrupt-driven (buffered) transmit — blocking
is the point — and any receive API, though the receiver hardware is
enabled (`RXEN0`) awaiting one.

## FS

Filesystem layers over block devices — one more floor of the same
building: `comms/spi.hpp` moves bytes, `devices/SDCard.hpp` turns them
into numbered blocks, and `fs/` interprets what a *computer* wrote into
those blocks. Lives under `HAL::FS::`.

### `fs/fat32.hpp` — read-only FAT32

Enough filesystem to find files a computer put on the card, and not one
line more: mount, iterate directories, find by name, sequential read,
seek. No write, no create, no long filenames (LFN-named files appear
under their `SHORTN~1` aliases). The motivating use case is streaming —
drag MP3s onto a card, walk the root directory, pump the bytes at a
decoder chip.

```cpp
#include "fs/fat32.hpp"
using Sd  = HAL::Devices::SD::Card<16>;
using Fat = HAL::FS::FAT32<Sd>;          // templated on the block device

Fat::mount();                            // MBR or superfloppy, FAT32 only
Fat::FileInfo info;
for (Fat::DirIter it { Fat::root() };
     Fat::next(it, info) == HAL::FS::Result::OK; ) {
    // info.name ("SONG.MP3"), info.size, info.dirP, ...
}
Fat::find("TRACK01.MP3", info);          // case-insensitive 8.3 lookup
Fat::File f;
Fat::open(info, f);
uint16_t got;
Fat::read(f, buf, sizeof(buf), got);     // sequential; got < len at EOF
Fat::seek(f, info.size / 2);             // random access within the file
```

Design notes:

- **No sector buffer, anywhere.** FAT32 work decomposes into small
  windowed reads — directory entries are 32-byte records, FAT chain
  links 4-byte words — which is exactly the SD driver's `readPartial()`.
  Total RAM: ~18 bytes of mount state plus 16 per open `File`, so the
  full SPI + SD + FAT32 stack runs on a tiny.
- **Random access, two flavors.** *Across* files: directory iteration
  hands you every file's location, so "track 7" or shuffle is just
  opening a different `FileInfo`. *Within* a file: `seek()` works, but
  FAT is a linked list of clusters, so it walks the chain from the
  start — one 4-byte read per 4–32 KiB cluster, tens of ms into a
  multi-MB file. Fine per track-skip, not per byte.
- **Wire cost intuition**: every windowed read still moves a full
  512-byte block over SPI (the SD protocol insists), so bulk streaming
  should call `read()` with the largest buffer RAM allows — at 512
  bytes per call the overhead is zero, and a tiny reading 16-byte
  windows pays 32×. Budget accordingly against your data rate.
- **Errors are `HAL::FS::Result`** (`NOT_MOUNTED`, `NO_FILESYSTEM`,
  `UNSUPPORTED`, `IO_ERROR`, `NOT_FOUND`, `BAD_PARAMS`) — same
  philosophy as the layers below it.
- **FAT32 only**: cards ≤ 2 GB often ship FAT16, which this deliberately
  doesn't speak — reformat them; SDHC cards come FAT32 out of the box.

## Utils

### `IntTransitionDebouncer` — interrupt-driven debouncing

The foundation of the whole input stack. One debouncer watches one pin.

```cpp
HAL::Utils::IntTransitionDebouncer<3,      // physical pin
                                   30,     // lockout window, ms
                                   HIGH,   // initial/idle state
                                   true>   // enable internal pullup
    sw;

sw.begin();                 // input (+pullup), enables PCINT for the pin
// in ISR: sw.notifyInterruptOccurred(now, changed);
// in main loop:
switch (sw.processAnyInterrupts()) {
    case HAL::Transition::FALLING:  /* pressed  */ break;
    case HAL::Transition::RISING:   /* released */ break;
    case HAL::Transition::NONE:     break;
}
```

**Design: immediate report + lockout window + end-of-window true-up.**
The reasoning, because it's the least obvious part of the library:

1. *The first edge of an event is always genuine.* Bounce happens after
   contact, not before it. So the first edge is reported immediately — its
   direction is necessarily away from the current stable state, no pin read
   required — giving zero-latency response.
2. *The window is purely a lockout.* Edges arriving within
   `debounceWaitTime` of the first are bounce and are ignored. Classic.
3. *At the end of the window, sample once and true-up.* If the pin doesn't
   match the recorded stable state — a tap that was pressed *and released*
   inside the window — the missing opposite transition is reported. This is
   what makes very short taps register as press+release instead of
   disappearing.

The alternative design (sample only at window end, report the transition
then) is more noise-immune on paper, but has two failure modes this design
was specifically built to kill: sub-window taps vanish entirely, and a
"no net change" outcome tempts you not to clear the pending state, which
blocks sleep forever. The lockout here clears *unconditionally* at window
end. Trade-off, stated honestly: an electrical noise spike on the pin will
register as a tap. On a pulled-up pin with a real switch this is rare; if
your environment is noisy, widen the window or add hardware filtering.

Cross-context safety: `notifyInterruptOccurred` (ISR side) records the edge
only if no lockout is active; both the edge claim and the window-end clear
in `processAnyInterrupts` (main side) are `ATOMIC_BLOCK` claim sections, so
the pair is safe under the ISR-notify/main-process contract without any
further locking. `pendingDebounceTimeout()` returns true while a lockout is
active — the signal that the MCU must not enter `PWR_DOWN` yet.

A tick value of 0 is used as the "idle" sentinel; an edge landing on the
actual tick 0 is recorded as tick 1.

### `LFSR` — fast 8-bit pseudorandomness

An 8-bit maximal-period (taps `0b00011101`) Fibonacci LFSR, implemented in
assembly, wrapped in a static interface:

```cpp
HAL::Utils::Random::LFSR::init(seed);        // seed 0 is replaced internally
uint8_t r = HAL::Utils::Random::LFSR::nextByte();
```

This is *statistical texture*, not randomness with any security property:
255-state period, fully deterministic from the seed. It exists because
candle-flicker-grade noise shouldn't cost more than a few dozen cycles per
byte. Callers wanting less obvious short-period artifacts can re-seed
periodically (see lamp-box's CandlePattern, which increments its seed every
frame — a "jumping" trick that breaks up the 255-frame cycle for pleasingly
non-repeating flicker).

## Devices

Device drivers compose debouncers and add semantics. All follow the same
lifecycle: construct (statically), `begin()` before `sei()`, notify from the
ISR, `process()` from the main loop, and register plain-function callbacks.

### `Button`

```cpp
HAL::Devices::Button<3,       // physical pin
                     30,      // debounce window, ms
                     1000,    // long-press threshold, ms
                     HIGH,    // passive (unpressed) state
                     true>    // use internal pullup
    btn;

btn.begin();
btn.setOnPress([]     { /* edge: pressed  */ });
btn.setOnRelease([]   { /* edge: released */ });
btn.setOnLongPress([] { /* held past threshold */ });
// ISR: btn.notifyInterruptOccurred(now, changed);
// loop: ButtonAction a = btn.process();   // also fires the callbacks
```

Semantics worth knowing:

- **Long-press** fires while the button is still held (checked by polling
  `process()` against the tick count), not on release — so it can drive
  press-and-hold UI.
- By default a long-press **suppresses the following release event**
  (`supressReleaseAfterLongPress`), so one physical gesture doesn't fire two
  logical actions. Disable via template parameter if you want both. The
  parameter governs only whether a long-press *arms* the suppression;
  external arming (next bullet) works regardless.
- `suppressNextReleaseEvent()` arms the same suppression from outside.
  `RotaryEncoderWithButton` calls it when rotation happens while pressed,
  so the release that ends a chorded gesture doesn't also register as a
  click. The Button is deliberately the *single owner* of this flag — see
  the composite's section for the bug that motivated that.
- `allowConsecutiveLongPresses` (default off) controls whether continuing to
  hold fires repeated long-presses.
- Works with either polarity (`passiveState` HIGH for pull-up wiring, LOW
  for pull-down); the unused polarity's code paths are compile-time dead.
- `pendingDebounceTimeout()` is true while a debounce is in flight **or the
  button is held** — the latter so a sleep gate can't power down mid-hold
  and kill the long-press timer.

### `RotaryEncoder`

Quadrature decoding for detented mechanical encoders (KY-040 style):

```cpp
HAL::Devices::RotaryEncoder<7,      // CLK pin
                            2,      // DT pin
                            true>   // pullups
    re;                             // + reverseP and stepsPerDetent
                                    //   template params, defaulted

re.setOnCW([]  { /* clockwise detent */ });
re.setOnCCW([] { /* counterclockwise detent */ });
```

Decoding strategy: a Gray-code transition table, run **in the ISR**, per
the sanctioned-exception rules above. Both channels' PCINTs are enabled;
on every edge of either channel, `notifyInterruptOccurred` reads the live
(CLK, DT) pair, looks up the transition `(previous → current)` in a
16-entry table, and adds the result — +1, −1, or 0 quarter-steps — to a
signed accumulator. `process()` atomically claims whole detents
(`stepsPerDetent` quarter-steps, default 4 = one full electrical cycle,
correct for KY-040-style parts) and fires `onCW`/`onCCW` in main-loop
context.

This replaced a simpler decoder (sample DT in `process()` when CLK
falls) that had a real failure mode: **direction lives in the
instantaneous phase relationship of the two channels**, and at fast
rotation the quadrature quarter-period shrinks to about the main loop's
latency — so by the time `process()` sampled DT, it read the *next*
phase and stepped backwards. The second failure was subtler: the old
path derived edge polarity by toggling a remembered state, so one
dropped CLK edge inverted its worldview until the *next* dropped edge
flipped it back. The table decoder is immune to both by construction:

- it reads **live pin levels** on every edge, never a toggled guess, so
  it resynchronizes to reality at every single transition;
- a missed intermediate state shows up as a two-bit jump, which the
  table maps to 0 — one quarter-step lost, never a reversed one;
- **contact bounce needs no debouncing at all**: a bounce retraces
  adjacent table transitions (+1 then −1) and cancels arithmetically in
  the accumulator before anything reaches a callback. This is why there
  is no debounce-window parameter (and no passive-state one either —
  quadrature has no passive level);
- direction reversals mid-cycle likewise cancel; only completed detents
  ever surface.

The accumulator saturates at ±120 quarter-steps rather than wrapping
(an int8_t wrap would replay as ~32 phantom detents in the wrong
direction). `pendingDebounceTimeout()` is kept for sleep-gate
compatibility but is no longer timed — nothing in the decoder reads the
Ticker. It reports true while a *complete* detent is banked and
unclaimed; a partial cycle is not "pending," because its remaining edges
wake the MCU by themselves. A `reverseP` template parameter flips the
direction convention rather than making you swap wires; the table's sign
convention matches the old decoder, so existing wiring keeps its
meaning.

### `RotaryEncoderWithButton`

Composition of the two above, adding the *chorded* gestures a clickable
encoder invites:

```cpp
HAL::Devices::RotaryEncoderWithButton<
    3, 30, 1000, HIGH, true,     // button:  pin, debounce, long-press, passive, pullup
    7, 2, true                   // encoder: clk, dt, pullup
> knob;

knob.begin();
knob.setOnCW(...);          knob.setOnCCW(...);         // plain rotation
knob.setOnPressedCW(...);   knob.setOnPressedCCW(...);  // rotation while held
knob.setOnPress(...);       knob.setOnRelease(...);
knob.setOnLongPress(...);
```

The one piece of real logic it owns: **rotating while pressed suppresses the
next release**. Without this, "press + rotate + let go" would fire the
release action (whatever "click" means in your UI) after every chorded
gesture. The suppression flag lives in the *Button* (armed via
`suppressNextReleaseEvent()`), not here — the Button already owns an
identical flag for its suppress-release-after-long-press rule, and the two
must be one flag. When the composite kept its own copy, a chorded gesture
held past the long-press threshold armed **both**; the one physical release
cleared only the Button's, and the composite's stale copy silently ate the
release of the *next* ordinary press. The user-visible symptom: after a
press-and-turn lasting more than the long-press threshold, the next click
does nothing and it "takes two presses."

Ordering inside `process()` also matters: **button events are reported
before a rotary detent is claimed**. The encoder banks its detents, so one
deferred to the next loop iteration (they run at kHz rates) loses nothing —
but a button event is consumed from the debouncer the moment
`btn.process()` returns it. The old code preferred the rotary action and
simply dropped a PRESS or RELEASE that coincided with a banked detent; a
dropped RELEASE left suppression state stale the same way.

`notifyInterruptOccurred` fans out to both children, so the ISR
only deals with one object; `pendingDebounceTimeout()` ORs the children, so
the sleep gate does too.

### `LCD1602` — 16×2 text over I²C

The classic HD44780 character LCD behind the ubiquitous PCF8574 I²C
"backpack": text output on two wires, on **all three MCUs** (the tinies
via the bit-banged I²C master — a tiny85 driving a text display is a
genuinely pleasing sight).

```cpp
#include "devices/LCD1602.hpp"
using I2c = HAL::Comms::I2C::Master<100000>;   // or the tiny pin-template form
using Lcd = HAL::Devices::LCD1602<I2c>;        // optional addr7 param; 0x27
                                               // default, A-variant chips 0x3F
Lcd::begin();
Lcd::print_P(PSTR("now playing:"));            // PROGMEM, like the UART
Lcd::setCursor(0, 1);
Lcd::print("TRACK01.MP3");
Lcd::print(someUint32);                        // and int32_t
Lcd::backlight(false);
Lcd::createChar(0, glyph8);                    // custom 5×8 glyphs, slots 0-7
Lcd::write(static_cast<char>(0));              // ...printed like characters
```

Worth knowing:

- **How the backpack works**: the PCF8574 is an 8-bit I²C GPIO expander
  wired to the LCD's 4-bit mode (RS/RW/EN/backlight on the low nibble,
  D4–D7 on the high). Every LCD byte becomes one 4-byte I²C transaction
  (two nibbles, each with an EN pulse) — ~450 µs per character at
  100 kHz, which hides the HD44780's ~37 µs instruction time completely,
  so only `clear()`/`home()` (~1.6 ms inside the controller) carry
  explicit delays.
- **Errors are `HAL::Comms::I2C::Result` passed straight through** —
  the only thing that can fail is the bus. `begin()` returning
  `NACK_ADDR` almost always means the other backpack address; the I²C
  module's `ping()` scanner settles it in seconds.
- Write-only by design (RW is held low); the busy flag is never read —
  bus timing already exceeds instruction timing.

### `SD::Card` — raw-block SD storage

The first thing-on-a-bus device: an SD card driven over `comms/spi.hpp`,
speaking the SD SPI-mode protocol. **Deliberately no filesystem** — at
this layer the card is a huge array of numbered 512-byte blocks, a
multi-gigabyte EEPROM. That's the whole substrate a datalogger needs
(write records into sequential blocks; read them back over a USB card
reader with `dd` and a few lines of host-side script), and it's what a
FAT library would sit on if computer-mountable files are ever wanted.
FAT is a separate later decision, and on the tinies a non-decision: a
real FAT needs a 512-byte sector buffer, and the tinies have 512 bytes
of RAM, *total*.

```cpp
#include "devices/SDCard.hpp"          // opt-in, like all devices
using Sd = HAL::Devices::SD::Card<16>; // CS pin; optional fastHz ceiling

if (Sd::begin() != HAL::Devices::SD::Result::OK) { /* no card / dead bus */ }
Sd::numBlocks();                       // capacity, from the CSD register
Sd::readBlock(0, buf512);              // classic full-block read (328P-ish)
Sd::readPartial(0, 510, sig, 2);       // a *window* of a block: tiny-friendly
Sd::writeBlock(n, buf512);             // full-block write
Sd::writeBlockStream(n, &makeByte);    // byte-source callback: no buffer ever
```

Design notes:

- **No block-sized buffer anywhere in the driver.** Reads can address a
  window within a block (the whole block still crosses the wire — the
  protocol insists — but only the window lands in RAM), and writes can
  stream from a `uint8_t (*)(uint16_t idx)` callback asked for bytes
  0–511 in order. This is how a 512-byte block leaves a chip with 512
  bytes of RAM.
- **Every call returns a `Result`** (`OK`, `NO_CARD`, `UNSUPPORTED`,
  `TIMEOUT`, `CMD_ERROR`, `READ_ERROR`, `WRITE_ERROR`, `BAD_PARAMS`,
  `NOT_INITIALIZED`) and every wait — init handshake, data token, write
  busy — is bounded, in the I²C module's spirit: a missing or wedged
  card is an error code, not a hung lamp.
- **Two speeds, one bus**: the SD spec caps the init handshake at
  400 kHz, so `begin()` runs on a `SPI::Master<400000>` and shifts to
  the `fastHz` ceiling (default 8 MHz) once the card is up. `begin()`
  handles the whole ritual — 74 wake-up clocks, CMD0/CMD8/ACMD41/CMD58 —
  and detects SDHC/SDXC vs byte-addressed cards (`isSDHC()`).
- **Hardware realities**: SD cards are strictly 3.3 V parts — from a 5 V
  AVR use a module with a level shifter. Writes draw 30–100 mA bursts.
  Cheap no-name cards are notoriously loose about the init handshake;
  bring up with a name-brand card.
- **Not implemented, deliberately**: multi-block transfers (CMD18/25)
  and MMC-era cards.

### `VS1053::Codec` — MP3 (and more) playback

No AVR can decode MP3 — that takes ~20+ MIPS of DSP and tens of KB of
RAM — so the VS1053b does it in silicon, and the AVR's job collapses to
**data pump**: read file bytes, shove them at the codec whenever its
DREQ pin says "feed me". The codec has its own FIFO, so nothing here is
real-time.

```cpp
#include "devices/VS1053.hpp"
using Mp3 = HAL::Devices::VS1053::Codec<24, 25, 26, 6>;
//                       XCS, XDCS, DREQ, XRESET  (+ optional fastHz)

Mp3::begin();                    // reset pulse, chip-version check, clock up
Mp3::sineTestStart();            // BRING-UP: a tone with no card, no file
Mp3::setVolume(0x20, 0x20);      // attenuation, 0.5 dB steps per channel

// the pump (see the smoke test for the full loop):
if (Mp3::readyForData()) Mp3::sendData(buf, 32);
Mp3::stopTrack();                // clean end-of-track; next song needs no reset
```

Design notes:

- **Two chip selects, one bus**: XCS gates SCI (the 16-bit control
  registers), XDCS gates SDI (the raw byte stream). Both ride the shared
  SPI bus alongside the SD card — and since neighbors leave their own
  speeds configured, every public operation re-asserts its own SPI
  config first (two register writes of insurance).
- **Two speeds, same story as SD**: after reset the codec runs straight
  off its 12.288 MHz crystal (SCI tops out ~1.75 MHz), so `begin()`
  starts slow, multiplies the internal clock to 3.0× via `SCI_CLOCKF`,
  then shifts to `fastHz` (default 4 MHz).
- **`begin()` verifies the chip** (SS_VER == 4 in `SCI_STATUS`, i.e. a
  real VS1053) and every DREQ wait is bounded — dead chip, `Result`,
  no hang.
- **`sineTestStart()` before anything else on new hardware**: it proves
  XCS/XDCS/DREQ and the bus with zero moving parts. If the sine plays,
  every later bug is in software.
- **`stopTrack()`** does the datasheet's clean ending (2052 end-fill
  bytes, `SM_CANCEL`, fill until acknowledged) so the next track starts
  without a reset. `decodeTime()` and `hdat1()` give a UI seconds-played
  and format confirmation (`0xFFEx` = MP3 decoding right now).
- **The pump pattern** (proven in the 328P smoke test): `Fat::read()`
  512-byte chunks into a RAM buffer — block-aligned, so zero wire
  overhead — then 32-byte `sendData()` feedings whenever
  `readyForData()`. At 320 kbps that's ~40 KB/s against an SPI budget
  many times larger; the main loop stays responsive throughout.

## Putting it together: the canonical main loop

This is the sleep-integrated skeleton avril is designed around (it is, not
coincidentally, the shape of lamp-box's `main.cpp`):

```cpp
int main() {
    HAL::Ticker::setupMSTimer();
    knob.begin();                 // before sei(): pins + PCINT masks
    sei();

    while (1) {
        knob.process();           // all callbacks fire here, main context

        uint16_t frameDelay = currentThing();   // 0 = nothing animating

        if (frameDelay) {
            // animation pacing: IDLE sleep, ticker keeps running.
            // no atomic gate needed -- the next tick wakes us within 1 ms,
            // so a race costs at most one tick of oversleep.
            uint32_t start = HAL::Ticker::getNumTicks();
            while (HAL::Ticker::getNumTicks() - start < frameDelay) {
                knob.process();
                HAL::Sleep::goToSleep(SLEEP_MODE_IDLE);
            }
        } else {
            // deep sleep: gate + commit must be atomic. re-check the
            // conditions under cli(); goToSleep's sei();sleep_cpu() pair
            // finishes the job. wake path: PCINT ISR calls resume(1).
            cli();
            if (!knob.pendingDebounceTimeout()) {
                HAL::Ticker::pause();
                HAL::Sleep::goToSleep(SLEEP_MODE_PWR_DOWN);
            } else {
                sei();
            }
        }
    }
}
```

The division of labor: `IDLE` between animation frames (Timer0 still runs,
1 ms heartbeat, ~4x less MCU draw than spinning), `PWR_DOWN` when nothing is
happening at all (µA territory; only a pin change can wake you, which is why
the ISR must `Ticker::resume()` before reading time).

## Known limitations

- **Three MCUs only**, by design. Porting to another AVR means extending
  the pin table, the ISR vector `#if`s, and the watchdog/UART register
  names. (The ATtiny84 port was exactly this list and ~60 lines.)
- **One debounced event in flight per pin** (buttons). The debouncer holds
  a single edge; edges during the lockout are treated as bounce, period.
  This is correct for human-speed presses and wrong for burst signals —
  don't use it as a general edge-capture facility. (The rotary encoder
  used to route through it and suffered for exactly this reason; it now
  decodes ISR-side and doesn't use the debouncer at all.)
- **Timer0 belongs to the Ticker.** If your project needs Timer0 for PWM,
  you don't get a millisecond timebase (or you port the Ticker to another
  timer).
- **Callbacks are bare function pointers.** No context argument, no
  closures with captures. State goes in file-scope objects; captureless
  lambdas work.
- **`getNumTicks()` wraps at ~49.7 days.** All library-internal comparisons
  are wrap-safe; keep yours wrap-safe too (`now - then >= window`, never
  `now >= then + window`).
- **UART is TX-oriented and blocking**; RX is enabled but nothing reads it.
- **SPI is master-only and blocking** (deliberately — see its section);
  slave mode would need an interrupt-driven design the USI and SPI
  peripherals support but this module doesn't attempt.
- **I²C is master-only and blocking** too; the slave — ISR-driven by
  nature — is planned as its own module.
