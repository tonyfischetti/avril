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
  - [`watchdog.hpp` — watchdog as dead-man's switch](#watchdoghpp--watchdog-as-dead-mans-switch)
  - [`uart.hpp` — debug serial (ATmega328P only)](#uarthpp--debug-serial-atmega328p-only)
- [Utils](#utils)
  - [`IntTransitionDebouncer` — interrupt-driven debouncing](#inttransitiondebouncer--interrupt-driven-debouncing)
  - [`LFSR` — fast 8-bit pseudorandomness](#lfsr--fast-8-bit-pseudorandomness)
- [Devices](#devices)
  - [`Button`](#button)
  - [`RotaryEncoder`](#rotaryencoder)
  - [`RotaryEncoderWithButton`](#rotaryencoderwithbutton)
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

### `watchdog.hpp` — watchdog as dead-man's switch

A thin, heavily-commented wrapper (the register documentation in the header
is half the value of the module). The timeout is a compile-time power of two
of 128 kHz cycles, from 2^11 (16 ms) to 2^20 (8.2 s):

```cpp
using WDT = HAL::Watchdog::Watchdog<18>;   // 2^18 cycles ~= 2.04 s

WDT::reset();     // (re)arm in *interrupt* mode: WDIE set
WDT::disable();   // timed-sequence disable

ISR(WDT_vect) {
    // you own what happens on timeout
}
```

`reset()` arms the watchdog in **interrupt mode** (WDIE), not reset mode, so
you must provide `ISR(WDT_vect)`; per the datasheet, WDIE must be re-set
after each interrupt or the *next* timeout hard-resets the chip — which
makes the default behavior a one-warning-then-reboot dead-man's switch,
useful in itself. Both `reset()` and `disable()` perform the required timed
sequences (WDCE/WDE) with interrupts disabled, and both clear WDRF first,
because a set WDRF forcibly re-enables WDE and would make the disable
silently fail.

### `uart.hpp` — debug serial (ATmega328P only)

Blocking, transmit-oriented, deliberately primitive — it exists so a 328P
project can print numbers at a human during bring-up, not to be a serial
framework. Compiles to nothing on the ATtiny84 and ATtiny85 (no USART).

```cpp
HAL::UART::init<115200>();        // baud is a template parameter
HAL::UART::println("booted");     // println appends \r\n
HAL::UART::print("ticks: ");
HAL::UART::println(someUint32);   // snprintf-backed; pulls in stdio, ~1-2 KB
```

The baud divisor is computed at compile time with proper rounding, and
double-speed mode (U2X) is selected automatically when it halves the rate
error — which is what makes 115200 @ 16 MHz actually work (naive truncated
math lands 8.5% off; rounded U2X lands within 2.1%). A rate that can't be
achieved within 2.5%, or that overflows the 12-bit UBRR register, is a
**compile error**, in the same spirit as the Ticker refusing an inexact
millisecond: a baud rate that's silently wrong is worse than one that
doesn't build.

Mind that `print(uint32_t)` drags `snprintf` into flash; on a tight build,
that's the first thing to go.

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
                            0,      // vestigial (was: debounce window)
                            HIGH,   // vestigial (was: passive state)
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
  the accumulator before anything reaches a callback. This is why the
  `debounceWaitTime` and `passiveState` parameters are vestigial — they
  are kept only so existing instantiations keep compiling;
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
    7, 2, 0, HIGH, true          // encoder: clk, dt, (vestigial ×2), pullup
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
