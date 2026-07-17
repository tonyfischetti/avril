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
- Flash them with your usual programmer; `make` produces
  `build/main.hex`.

Current examples:

| example | MCU | modules exercised |
|---|---|---|
| `alarm-clock` | ATmega328P | I²C, DS3231, LCD1602, RotaryEncoderWithButton, Ticker, Sleep, UART |
