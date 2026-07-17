
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

I see that the alarm clock uses a shared SDA. I though it was the CLK that could
be shared.

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
What is that?

The next few questions have to do with displays...
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




E-Ink displays



