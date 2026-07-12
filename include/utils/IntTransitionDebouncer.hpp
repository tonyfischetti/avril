#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "ticker.hpp"

    //  TODO  BEEF UP DOCUMENTATION
    //  TODO  mention .enable()

/**
 * So here's how this works...
 *
 * From the client code you do something like
            HAL::Utils::TransitionDebouncer<3> sw(HIGH, 30, true);
            HAL::Utils::IntTransitionDebouncer<3, 30, HIGH, true> sw;
 * where 3 is the physical pin, 30 is the debounce window, 
 * HIGH is its starting state, and `true` denotes that it has a
 * pull up. 
 *
 * _In order for it to work_ you have to do
            sw.begin();
 * somewhere in main before interrupts are enabled.
 * Then the pin is set for input (possibly with a pull-up)
 * and PCINT on that port is enabled.
 *
 * In the ISR that corresponds to that PORT, you would do something like
 *
        ISR(PCINT0_vect) {
            uint32_t now = HAL::Ticker::getNumTicks();
            uint8_t current = PINB;
            uint8_t changed = current ^ previousPINB;
            previousPINB = current;

            sw.notifyInterruptOccurred(now, HAL::GPIO::Port::B, changed);
        }

 * where `previousPINB` is `volatile uint8_t previousPINB { 0xFF  };`
 * or whatever. The Port argument says which port the `changed` mask
 * belongs to; the debouncer drops notifications for other ports (bit
 * positions collide across ports, so this matters on the ATmega328P
 * where each port has its own PCINT vector -- write one ISR per port
 * and pass the right Port, and devices can be spread across ports
 * freely).
 *
 * This notifies the Debouncer that a change happened in the PORT. It
 * supplies the number of ticks in Timer0 (see Ticker.hpp) and a mask
 * of which pins changed.
 *
 * The `notifyInterruptOccurred(...)` member function checks if there was
 * a pin change for its specific GPIO register. If so, and no lockout is
 * currently active, it records the edge (`lockoutStart` = time of the
 * interrupt, `edgePending` = true) and the lockout begins. Edges that
 * arrive during an active lockout (i.e. bounce) are ignored.
 *
 * Events get processed by the main loop, in code like

        switch (sw.processAnyInterrupts()) {
            case HAL::Transition::RISING:
                break;
            case HAL::Transition::FALLING:
                changeMode();
                break;
            case HAL::Transition::NONE:
                break;
            default:
                break;
        }
 *
 * The `processAnyInterrupts` member function reports in two phases:
 *
 *   1. The recorded edge is reported _immediately_ (the first edge of a
 *      press or release is always a genuine transition; its direction is
 *      simply away from the current stable state). This gives zero-latency
 *      response and means even very short taps register.
 *
 *   2. Once `debounceWaitTime` has elapsed since the edge, the pin is
 *      sampled once more. If it doesn't match the stable state (e.g. a
 *      tap that was released within the window), the missing opposite
 *      transition is reported. Then the lockout is cleared.
 *
 * So, basically, the window serves as a lock-out: the bounce following an
 * edge is ignored, and the end-of-window sample trues everything up.
 *
 * As mentioned above, it used HAL::Ticker
 * This means that if you put the MCU to sleep, the timer stops. To cope
 * with this (if you want to use SLEEP_MODE_PWR_DOWN) do the following
 * everytime you want to go to sleep
 *
        if (!sw.pendingDebounceTimeout()) {
            HAL::Ticker::pause();
            HAL::Sleep::goToSleep(SLEEP_MODE_PWR_DOWN);
        }
 *
 * This will prevent it from pausing before the latest interrupt notification
 * is resolved
 * _AND_
 * resume the ticker first thing in the ISR
        ...
        HAL::Ticker::resume();
        ...
 * before HAL::Ticker::getNumTicks() is called
 *
 * Beware: an unprocessed interrupt will prevent the MCU from sleeping
 *
 */

//  TODO  member function to cancel and pending interrupt notices

namespace HAL {

enum class Transition : uint8_t { RISING, FALLING, NONE };

namespace Utils {

template<uint8_t physicalPin,
         uint32_t debounceWaitTime,
         bool initialState,
         bool usePullupP>
class IntTransitionDebouncer {

    using Callback = void (*)();

    HAL::GPIO::GPIO<physicalPin> gpio;
    volatile uint32_t            lockoutStart; // time of first edge; 0 = idle
    volatile bool                edgePending;  // edge seen but not yet reported
    bool                         stableState;
    Callback                     onFalling;
    Callback                     onRising;

  public:

    IntTransitionDebouncer()
        : gpio         { },
          lockoutStart { 0 },
          edgePending  { false },
          stableState  { initialState },
          onFalling    { nullptr },
          onRising     { nullptr } {
    }

    void begin() {
        if constexpr (usePullupP)
            gpio.setInputPullup();
        else
            gpio.setInput();
        gpio.enablePCINT();
    }

    void notifyInterruptOccurred(uint32_t now, HAL::GPIO::Port port,
                                 uint8_t changed) {
        // `changed` is a port-relative mask; bit positions collide across
        // ports, so a notification for the wrong port must be dropped
        // here. The comparison is against a constexpr and folds away
        // when the caller passes a compile-time port (the usual case)
        if (port != gpio.info.port) return;
        if (changed & gpio.mask) {
            if (!lockoutStart) {
                lockoutStart = now ? now : 1; // 0 means "idle"
                edgePending  = true;
            }
        }
    }

    void setOnFalling(Callback fnptr) { onFalling = fnptr; }
    void setOnRising(Callback fnptr)  {  onRising = fnptr; }

    Transition processAnyInterrupts() {
        Transition transition           { Transition::NONE };
        uint32_t   snapshotOfLockout    { 0 };

        READ_VOLATILE_U32(lockoutStart, snapshotOfLockout);

        if (snapshotOfLockout == 0) return Transition::NONE;

        // phase 1: report the recorded edge immediately (the claim is
        // atomic; process() may run from both ISR and main contexts)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (edgePending) {
                edgePending = false;
                stableState = !stableState;
                transition  = stableState
                    ? Transition::RISING
                    : Transition::FALLING;
            }
        }
        if (transition != Transition::NONE) {
            if (transition == Transition::RISING  &&  onRising)  onRising();
            if (transition == Transition::FALLING && onFalling) onFalling();
            return transition;
        }

        // phase 2: at the end of the lockout window, sample the pin and
        // report the opposite transition if the edge's state didn't hold
        // (e.g. a tap released within the window); then clear the lockout
        uint32_t now = HAL::Ticker::getNumTicks();
        if ((now - snapshotOfLockout) >= debounceWaitTime) {
            bool nowState  { gpio.read() };
            bool wonTheRaceP { false };
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (lockoutStart == snapshotOfLockout && !edgePending) {
                    lockoutStart = 0;
                    if (nowState != stableState) {
                        stableState = nowState;
                        transition = nowState
                            ? Transition::RISING
                            : Transition::FALLING;
                    }
                    wonTheRaceP = true;
                }
            }
            if (wonTheRaceP) {
                if (transition == Transition::RISING  &&  onRising)  onRising();
                if (transition == Transition::FALLING && onFalling) onFalling();
            }
        }
        return transition;
    }

    bool getStableState() {
        return stableState;
    }

    bool pendingDebounceTimeout() {
        return lockoutStart > 0;
    }

};


}
}

