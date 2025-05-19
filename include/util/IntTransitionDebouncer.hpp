#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "ticker.hpp"

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

            sw.notifyInterruptOccurred(now, changed);
        }

 * where `previousPINB` is `volatile uint8_t previousPINB { 0xFF  };`
 * or whatever.
 *
 * This notifies the Debouncer that a change happened in the PORT. It
 * supplies the number of ticks in Timer0 (see Ticker.hpp) and a mask
 * of which pins changed.
 *
 * The `notifyInterruptOccurred(...)` member function checks if there was
 * a pin change for its specific GPIO register. If so, it checks if there
 * is an "unprocessed" interrupt. If so, the notification is ignored. If
 * not, `lastUnprocessedPrimeInterrupt` gets set with the time the interrupt
 * occurred.
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
 * The `processAnyInterrupts` member function checks if there's an 
 * unprocessed event. If there is, it gets the number of ticks at the 
 * time the member function is called. If a time (since the interrupt)
 * exceeds the `debounceWaitTime`, the pin is read and that is taken to be
 * the new `stableState`. Since we know the last stable state, we know if
 * the Transition is (enum) Transition::RISING or Transition::FALLING
 * (or Transition::NONE). 
 *
 * So, basically, this serves as a lock-out. Any spurious changes (within
 * the debounce window) are ignored. Once the dust settles, the pin is read
 * for real
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

template<uint8_t physicalPin, uint32_t debounceWaitTime,
         bool initialState, bool usePullupP>
class IntTransitionDebouncer {

    using Callback = void (*)();

    HAL::GPIO::GPIO<physicalPin> gpio;
    bool stableState;
    volatile uint32_t lastUnprocessedPrimeInterrupt;
    Callback onFalling;
    Callback onRising;

  public:

    IntTransitionDebouncer() 
        : lastUnprocessedPrimeInterrupt { 0 },
          stableState                   { initialState },
          onFalling                     { nullptr },
          onRising                      { nullptr } {
    }

    void begin() {
        if constexpr (usePullupP)
            gpio.setInputPullup();
        else
            gpio.setInput();
        gpio.enablePCINT();
    }

    void notifyInterruptOccurred(uint32_t now, uint8_t changed) {
        if (changed & gpio.mask) {
            if (!lastUnprocessedPrimeInterrupt) {
                lastUnprocessedPrimeInterrupt = now;
            }
        }
    }

    void setOnFalling(Callback fnptr) { onFalling = fnptr; }
    void setOnRising(Callback fnptr)  { onRising  = fnptr; }

    Transition processAnyInterrupts() {
        Transition transition { Transition::NONE };
        uint32_t snapshotOfPrimeInterreuptTime { 0 };
        READ_VOLATILE_U32(lastUnprocessedPrimeInterrupt, snapshotOfPrimeInterreuptTime);
        if (snapshotOfPrimeInterreuptTime > 0) {
            uint32_t now = HAL::Ticker::getNumTicks();
            if ((static_cast<uint32_t>((now - snapshotOfPrimeInterreuptTime))) >= debounceWaitTime) {
                bool nowState = gpio.read();

                if (nowState && !stableState)
                    transition = Transition::RISING;
                else if (!nowState && stableState)
                    transition = Transition::FALLING;
                stableState = nowState;
                WRITE_VOLATILE_U32(lastUnprocessedPrimeInterrupt, 0);
            }
        }
        switch (transition) {
            case HAL::Transition::RISING:
                if (onRising) onRising();
                break;
            case HAL::Transition::FALLING:
                if (onFalling) onFalling();
                break;
            default:
                break;
        }
        return transition;
    }

    bool pendingDebounceTimeout() {
        if (lastUnprocessedPrimeInterrupt > 0)
            return true;
        return false;
    }

};


}
}

