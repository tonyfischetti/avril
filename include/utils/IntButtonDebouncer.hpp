#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "ticker.hpp"


namespace HAL {

enum class ButtonAction : uint8_t { NONE, RELEASE, PRESS, LONG_PRESS };

namespace Utils {

    //  TODO  try to template _that_ for uint32 T
template<uint8_t  physicalPin,
         uint32_t debounceWaitTime,
         uint32_t longPressWaitTime,
         bool     passiveState,
         bool     usePullupP>
class IntButtonDebouncer {

    using Callback = void (*)();

    HAL::GPIO::GPIO<physicalPin> gpio;
    volatile uint32_t            lastUnprocessedPrimeInterrupt;
    bool                         stableState;
    Callback                     onRelease;
    Callback                     onPress;
    Callback                     onLongPress;
    uint32_t                     lastPressed;

  public:

    IntButtonDebouncer() 
        : gpio                          { },
          lastUnprocessedPrimeInterrupt { 0 },
          stableState                   { passiveState }, // assume released
          onRelease                     { nullptr },
          onPress                       { nullptr },
          onLongPress                   { nullptr },
          lastPressed                   { 0 } {
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

    void setOnPress(Callback fnptr)      { onPress     = fnptr; }
    void setOnLongPress(Callback fnptr)  { onLongPress = fnptr; }
    void setOnRelease(Callback fnptr)    { onRelease   = fnptr; }


    ButtonAction process() {
        ButtonAction buttonAction                 { ButtonAction::NONE };
        uint32_t     snapshotOfPrimeInterruptTime { 0 };
        uint32_t     now                          { HAL::Ticker::getNumTicks() };
        bool         nowState                     { gpio.read() };
        ButtonAction tentative                    { ButtonAction::NONE };

        READ_VOLATILE_U32(lastUnprocessedPrimeInterrupt, snapshotOfPrimeInterruptTime);

        if (snapshotOfPrimeInterruptTime == 0) {
            if (nowState == stableState && nowState!=passiveState) {
                if (((now - lastPressed) >= longPressWaitTime)) {
                    lastPressed = now; // HERE?!
                    if (onLongPress) onLongPress();
                    return ButtonAction::LONG_PRESS;
                }
            }
        }
        else { // snapshotOfPrimeInterruptTime > 0
            if ((now - snapshotOfPrimeInterruptTime) >= debounceWaitTime) {
                if (nowState != stableState) {
                    tentative = (nowState == passiveState)
                        ? ButtonAction::RELEASE
                        : ButtonAction::PRESS;
                }

                bool wonTheRaceP { false };
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (lastUnprocessedPrimeInterrupt == snapshotOfPrimeInterruptTime) {
                        lastUnprocessedPrimeInterrupt = 0;
                        stableState = nowState;
                        buttonAction = tentative;
                        wonTheRaceP = true;
                    }
                }

                if (wonTheRaceP) {
                    //  TODO  redo this
                    if (buttonAction == ButtonAction::PRESS) lastPressed = now;
                    if (buttonAction == ButtonAction::RELEASE && onRelease)      onRelease();
                    if (buttonAction == ButtonAction::PRESS && onPress)          onPress();
                }
            }
        }
        return buttonAction;
    }

    bool pendingActionP() {
        //                                          could be a long press
        return lastUnprocessedPrimeInterrupt > 0 && !stableState;
    }

};

}
}

