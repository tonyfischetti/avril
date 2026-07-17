
The README mentions the power usage of the TSOP. It mentions that there is
a design doc's power section. Do we have that?

I'm seeing a lot of functions for converting to decimal, bin, hex, etc...
Should that live in utils?

In `include/devices/DS3231.hpp`
"Forget that, and your daily alarm fires exactly once per battery."
I don't get it.

Pertaining to the jukebox...
Line 120-122 compare the file extension to "MP3". Is it case sensitive?

Pertaining to the jukebox (again)
Is it a dumb idea to try to parse ID3 tags?

Pertaining to the jukebox (again)
Can I have a folder for each playlist? The playlist name can be the
directory name

Pertaining to the jukebox (again)
I see it uses `getNumTicks()`. Why that and not `getMicros()`.
Is there a benefit to the former when microsecond resolution is unecessary?

I see the clock example uses `getNumTicks()` to wait a second. Should it not
use the DS3231's square wave function, instead?

I see that the alarm clock uses a shared SDA. 
I thought both SDA and CLK is shared

The comms and devices sometimes uses `__builtin_avr_delay_cycles`. What's that?
What's the benefit over using `sleepFor`?

Pertaining to the motion alarm, and the line `constexpr uint16_t LOW_BATT_MV { 6400 };`
Is that just an arbitrary cutoff?

Pertaining to the motion alarm.
Can we make main.cpp called pir.cpp and also make an implementation
that uses SW-18015P, too? That'd be super useful for bikes. What's your
estimate of the power saving using that approach?

Isn't `busy()` an integral part of the IRReceiverNEC driver?
The ir-probe example never uses it.

The I2C implementation of the ATMega is _not_ bit-banged, right?

Both the IRReceiver and the RotaryEncoder necessarily decodes
in the ISR. I was always under the impression that it is __critical__
that the ISR is as short as possible. Does the extended processing in
the ISR block pose any issues? Under what circumstances would it cause
an issue? And, similarly, what can you actually get away with?

In the IR probe example, a mention is made to "Vishay's supply filter".
What is that? Should that be added to the docs?

The next four questions have to do with displays...
First, is the LCD driver compatible with 20x4 displays?

Second, I've used LCD1602 that allow you to change both the
BG and FG colors. What's that about?

Third, I've used 4-digit 7-segment I2C Displays before. I think they
use the HT16K33 (TM1637, too?). Can we write a driver for that?
Or maybe that's an easy one that I can take

Fourth, is a driver for the SSD1306 easy to write? How are
different pixel dimensions handled?

(off of display concerns)
Can we design and implement an example of a UART receive. Very simple

I'm very interested in scheduling. CFS, elevator, priority
  3. The theory you named — and why it's the wrong theory here, which is itself the lesson. CFS solves
  fairness under contention: many long-running tasks fighting over a saturated CPU, "who gets the next
  slice." Our CPU is idle 99.9% and every task is a short event handler — there is no contention to
  arbitrate, so fairness is meaningless. The resource that's actually scarce in embedded isn't CPU time,
  it's deadlines — which makes the relevant literature RMS and EDF, not CFS. Rate-Monotonic Scheduling:
  static priorities by frequency (fastest task = highest priority), with the famous Liu & Layland result
  that it's guaranteed schedulable only up to ~69.3% CPU utilization. EDF: dynamic earliest-deadline-first,
  schedulable to 100% — and pleasingly, our tick-less run() is a degenerate EDF: it computes the earliest
  deadline to know how long to sleep, and only skips the deadline-ordering of execution because at ~0%
  utilization the order among simultaneously-due tasks can't matter. (The elevator algorithm is a different
  beast entirely — an I/O scheduler minimizing disk-head travel — whose closest local analog would be
  ordering SD block operations, and our logger's strictly-sequential writes are the case where elevator
  scheduling degenerates to "just go forward.")

  So the verdict: as infrastructure, skip it — the ISR/cooperative split plus the tick-less table covers
  every real need this hardware has, and the honest upgrades if one ever pinches are cheap: priority = run
  order in add() (already true, just undocumented as such), or sorting due-tasks by deadline for true EDF (a
  dozen lines). But as a learning artifact? A minimal preemptive kernel on the 328P — two tasks, two
  stacks, a Timer1 tick, ~100 lines of which the context-switch asm is the crown jewel — is one of the great
  systems-programming exercises, the kind where the Linux code you've read suddenly becomes muscle memory.
  If that itch is real, examples/toy-kernel would be a delightfully weird resident of the examples
  directory: not practical, explicitly labeled so, and more educational than the practical ones. Want it on
  the shelf under that framing?




it's deadlines — which makes the relevant literature RMS and EDF, not CFS.


LINKS TO EXAMPLES, READMEs, docs, etc...


E-Ink displays


WHAT IS A SMOKE TEST??!!?!!?

test harnesses



IN-FLIGHT!!!

brew upgrade claude-code
