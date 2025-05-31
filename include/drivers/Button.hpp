#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "../utils/IntTransitionDebouncer.hpp"
#include "ticker.hpp"

//  TODO  DO I NEED TO INCLUDE TICKER?!

/*
 *  TODO  add documentation
 */

/*
 *  TODO  double click? how
 */

/*
 *  TODO  make a VERY long press
 */

namespace HAL {

enum class ButtonAction : uint8_t { NONE, RELEASE, PRESS, LONG_PRESS };

namespace Drivers {

template<uint8_t  physicalPin,
         uint32_t debounceWaitTime,
         uint32_t longPressWaitTime,
         bool     passiveState,
         bool     usePullupP,
         bool     supressReleaseAfterLongPress=true,
         bool     allowConsecutiveLongPresses=false>
class Button {

    using Callback = void (*)();

    HAL::GPIO::GPIO<physicalPin> gpio;
    HAL::Utils::IntTransitionDebouncer<physicalPin,
                                       debounceWaitTime,
                                       passiveState,
                                       usePullupP> debouncer;
    uint32_t lastPressed;
    bool     suppressNextRelease;
    bool     longPressLockoutP;
    Callback onRelease;
    Callback onPress;
    Callback onLongPress;

  public:
    Button()
        : gpio                { },
          debouncer           { },
          lastPressed         { 0 },
          suppressNextRelease { false },
          longPressLockoutP   { false },
          onRelease           { nullptr },
          onPress             { nullptr },
          onLongPress         { nullptr } {
    }

    void begin() {
        debouncer.begin();
    }

    void notifyInterruptOccurred(uint32_t now, uint8_t changed) {
        debouncer.notifyInterruptOccurred(now, changed);
    }

    void setOnRelease(Callback fnptr)   {   onRelease = fnptr; }
    void setOnPress(Callback fnptr)     {     onPress = fnptr; }
    void setOnLongPress(Callback fnptr) { onLongPress = fnptr; }


    ButtonAction process() {
        Transition btnTransition { debouncer.processAnyInterrupts() };
        ButtonAction btnAction   { ButtonAction::NONE };
        uint32_t     now         { HAL::Ticker::getNumTicks() };
        bool         nowState    { gpio.read() };
        bool         stableState { debouncer.getStableState() };

        if (btnTransition == Transition::NONE) {
            if (nowState == stableState && nowState!=passiveState) {
                if (((now - lastPressed) >= longPressWaitTime)) {
                    if (!allowConsecutiveLongPresses && longPressLockoutP) {
                        return ButtonAction::NONE;
                    }
                    lastPressed = now; // HERE?!
                    suppressNextRelease = true;
                    longPressLockoutP = true;
                    if (onLongPress) onLongPress();
                    return ButtonAction::LONG_PRESS;
                }
            }
        }

        //  TODO  refactor this bs
        else if (btnTransition == Transition::FALLING) {
            if (passiveState) {
                lastPressed = now;
                if (onPress) onPress();
                btnAction = ButtonAction::PRESS;
            } else {
                longPressLockoutP = false;
                if (supressReleaseAfterLongPress && suppressNextRelease) {
                    suppressNextRelease = false;
                } else {
                    if (onRelease) onRelease();
                    btnAction =  ButtonAction::RELEASE;
                }

            }
        } else if (btnTransition == Transition::RISING) {
            if (passiveState) {
                longPressLockoutP = false;
                if (supressReleaseAfterLongPress && suppressNextRelease) {
                    suppressNextRelease = false;
                } else {
                    if (onRelease) onRelease();
                    btnAction =  ButtonAction::RELEASE;
                }
            } else {
                if (onPress) onPress();
                btnAction = ButtonAction::PRESS;
            }
        }

        return btnAction;
    }


    bool pendingDebounceTimeout() {
        //  TODO  AND IF HELD DOWN!
        return debouncer.pendingDebounceTimeout() || !debouncer.getStableState();
    }

};


}
}
