# avril examples

Full, real, practical programs — not snippets. Every example is a
complete appliance: a `main.cpp` you can read top to bottom, a
`Makefile` that builds it against the avril tree it lives in
(`HAL_LOC := ../..`), and a wiring table in the header comment. If an
example doesn't build with `make`, that's a bug.

Conventions:

- One directory per example: `examples/<name>/{main.cpp, Makefile}`.
- The header comment of `main.cpp` is the documentation: what it does,
  the parts list, the wiring table, and which avril modules it
  exercises.
- Examples follow the canonical main-loop shape from the main README
  (notify-from-ISR, process-in-main, sleep at the bottom) — they are
  the living demonstrations of the concurrency contract, not
  exceptions to it.
- `make` produces `build/main.hex`; `make flash` programs it
  (usbtiny by default — `make flash PROGRAMMER=usbasp` to override).
- `make fuses` sets the chip's fuses to match the example's clock
  configuration — **once per chip**, since fuses persist. The active
  line in each Makefile documents what every fuse byte means, and
  commented-out alternatives (factory restore, internal-RC, BOD
  variants) sit beside it, each with its tradeoff. The warnings about
  RSTDISBL/SPIEN are not decorative: those two brick ISP access.

Current examples:

| example | MCU | modules exercised |
|---|---|---|
| `alarm-clock` | ATmega328P | I²C, DS3231, LCD1602, RotaryEncoderWithButton, Ticker, Sleep, UART |
| `mp3-player` | ATmega328P | SPI, I²C, UART, SD, FAT32, VS1053, LCD1602, RotaryEncoderWithButton, Ticker, Sleep |
| `vitals-monitor` | ATtiny84 | Analog (Vcc gauge, on-die temp, LDR), Watchdog (PWR_DOWN cadence), I²C (bit-banged), LCD1602 |
| `motion-alarm` | ATtiny85 | GPIO (PCINT wake), Sleep (PWR_DOWN), Watchdog (all timing), Analog (raw-battery divider) |
| `ir-probe` | ATmega328P | IRReceiverNEC (timestamp-first ISR), LCD1602, UART, Ticker, Sleep |
