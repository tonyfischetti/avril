# Next steps

The shelf: things designed, discussed, or endorsed but deliberately
not built yet, roughly in priority order. Each entry carries enough
context to pick it up cold — what it is, why it's shaped the way it
is, and what was already decided in the design conversations. Cross
off (or delete) as they land.

## Hardware bench testing — the gate everything else waits behind

Nothing in the recent build-out has touched real hardware: the entire
MP3 stack (SPI → SD → FAT32 → VS1053), UART RX, watchdog naps, the
LCD, the ADC, the DS3231, the IR receiver — all verified only by the
compiler. The bring-up order that de-risks fastest:

1. **Watchdog naps + UART RX echo** — no new parts needed.
2. **SPI + SD init + FAT32 mount** — a level-shifted microSD module
   and a FAT32 card. First file contiguous → `contiguousBlockRange`
   testable too.
3. **VS1053: sine test FIRST** — the built-in sine sweep proves SPI,
   wiring, and the analog path before any file is involved.
4. **LCD1602** — if `begin()` returns `NACK_ADDR`, try 0x3F (the
   backpack ships with either).
5. **The mp3-player example end to end** — the payoff.

Shopping list: VS1053b breakout, microSD module w/ level shifter,
LCD1602 + PCF8574 backpack, a name-brand card formatted FAT32 with a
`TRACK01.MP3` on it.

## HT16K33 4-digit 7-segment driver — **Tony's project**

Claimed, crib sheet delivered. The short version: it's genuine I²C
(unlike the TM1637 — see below). Init is three one-byte commands:
`0x21` (oscillator on), `0x81` (display on, no blink), `0xE0|dim`
(brightness 0–15). Display RAM is written as
`{0x00, d0, 0, d1, 0, colon, 0, d2, 0, d3, 0}` — every other byte is
for the unused second row. Segment font goes in PROGMEM; on the
Adafruit backpack the colon is RAM position 2, bit 1. Review offered
when ready.

Companion for later: **TM1637** looks like I²C but isn't — LSB-first,
no address byte, its own start/stop timing. It needs a small
dedicated bit-bang driver, not the I²C module.

## lamp-box IR remote integration

The driver (`IRReceiverNEC`) and its design doc (`TSOP4838.md`) are
done; §11 of the doc is the integration plan for lamp-box itself:
remote control of pattern/brightness alongside the encoder. The
interesting part is the deadline ledger — SK6812's ~80 µs inter-byte
latch tolerance vs the IR ISR's runtime — which §11 already works
through. lamp-box is an external consumer of avril, so this lands in
the lamp-box repo, not here.

## I²C slave (328P TWI)

"Could be cool" — makes a 328P a peripheral another MCU (or a Pi)
can poll. Weekend-scale: an ISR state machine over ~10 TWI status
codes, with a register-file API (master writes a register number,
reads values back). Two properties make it friendlier than it
sounds: clock stretching means the hardware waits for us when we're
slow, and address-match can wake the chip from power-down — a
battery sensor node that sleeps until asked. Fits the sanctioned
ISR-side-decoder contract (state machine in the ISR, results cross
via one atomic claim). USI slave on the tinies is substantially
harder; bit-banged slave: no.

## SSD1306 OLED driver — bufferless text mode

Design agreed: no framebuffer (a 128×64 buffer is 1 KB — the whole
tiny85). Instead, stream 5×7 PROGMEM glyphs straight into
page/column address windows, giving an LCD1602-equivalent text API
on all three MCUs. Dimensions are a `template <w, h>`: they drive
the MUX ratio (0x3F/0x1F), the COM-pins config (0x12/0x02 — get
this wrong and you get the classic interlaced-garbage bug), and the
page clamp. Charge pump `0x8D 0x14` or the screen stays black. I²C
address 0x3C; on the 328P use `Master<400000>`. Full-graphics /
page-buffer variants deferred until something needs pixels.

## LCD1602RGB driver (native-I²C RGB modules)

The Grove/Waveshare-style combo boards: an AiP31068 LCD controller
at 0x3E driven by plain `{0x80, cmd}` / `{0x40, data}` writes — no
nibble/EN dance, so it's *simpler* than the PCF8574 backpack — plus
a separate PCA9633/SGM31323 LED driver at its own address for
`setColor(r, g, b)`. ~100 lines, `devices/LCD1602RGB.hpp`. Color is
backlight-only (the glass is monochrome; negative panels read as
"FG color", positive as "BG"). Demo idea: alarm clock face — red
when ringing, amber on low battery.

## NeoPixel driver → avril relocation

Agreed worthwhile: move lamp-box's cycle-counted SK6812 asm into
avril `devices/` **as-is**. The 10-cycle-per-bit assembly can't be
templated, so it keeps the `-DNEO_PORT`/`-DNEO_BIT` compile-time
config as a documented exception to the template-parameter house
style. Needs an `#ifdef NEO_PORT` guard around the `.S` file (the
examples' src glob would otherwise assemble it everywhere) and an
`F_CPU == 8MHz` gate (the cycle counts assume it). Acceptance test:
lamp-box rebuilds byte-identical.

## PWM / analogWrite

Agreed worth building *when a consumer lands* (servo, passive-buzzer
melody, LED dimming). No DAC on these chips, so it's hardware PWM on
the non-Ticker timers (Timer0 belongs to the Ticker). Pin inventory:
328P Timer1 PB1(15)/PB2(16), Timer2 PB3(17)/PD3(5) — note the SPI
collisions; t84 Timer1 PA6/PA5; t85 Timer1 PB1(6)/PB4(3) — PB1 is
lamp-box's idle LED pin, so free dimming there. The tiny85's Timer1
is the weird one: its own prescaler chain to /16384 and the PLL.
A Servo driver would be its own `devices/` module on Timer1.

## UART RX tier 3 — ISR ring buffer

Only when something talks to the 328P faster than the main loop
polls: `RXCIE0` + `USART_RX_vect` filling a power-of-two ring,
ISR-side store, main-loop drain — the standard notify-don't-process
shape. The polled `tryRead()` covers everything current examples do;
build this when a burst producer (GPS, ESP link) shows up.

## MCP2515 (CAN bus)

An SPI register driver — structurally like the VS1053 work, so the
comms substrate is ready. The practical blocker is test hardware:
CAN needs two nodes (or the chip's loopback mode, which proves the
SPI driver but not the bus). Shelved until there are two boards that
want to talk.

## Entropy seed — `Watchdog::harvestEntropyByte()`

Endorsed 2026-07-16. Poll `WDIF` with the I-bit cleared (no ISR),
sample TCNT0's LSB at each ~16 ms watchdog timeout, XOR+rotate-fold
32–64 samples into one byte (~0.5–1 s at boot). Purpose: seed
lamp-box's 8-bit LFSR so the candle flicker differs each boot —
today it's the constant `jumpingSeed{93}`. Guard against a harvested
0 (LFSR lockup), ideally inside `LFSR::init` itself. Prerequisites
(watchdog interrupt mode, `.init3` guard) are already done.

## External DAC (if true analog out is ever wanted)

MCP4725 (I²C) or MCP4921 (SPI) — both are trivial single-register
drivers over the existing comms modules. Listed here mostly so the
"is analogWrite real analog?" question has its recorded answer: no,
and this is the actual answer if one is needed.

## examples/toy-kernel — **explicitly low priority**

A learning artifact, not infrastructure, and framed that way in its
docs when built: a minimal preemptive kernel on the 328P. Two or
three tasks with their own stacks (~100 B each — AVR has no separate
interrupt stack, so every task stack needs ISR headroom), a Timer1
tick, and the crown jewel: the ~35-byte context switch in asm
(32 registers + SREG + PC). Round-robin first, priorities after.
Framing note from the design discussion: avril is *already* a
two-tier foreground/background system (ISRs are the preemptive
tier), and the tick-less `Scheduler::run()` is degenerate EDF — the
embedded canon is RMS (Liu & Layland's ~69.3% bound) and EDF (100%),
not CFS, which solves a contention problem that ~0%-utilization
firmware doesn't have. Cheap upgrades if ever wanted instead of a
kernel: document add()-order-as-priority; sort due tasks by deadline
for true EDF (~12 lines).
