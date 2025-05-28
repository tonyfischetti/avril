#pragma once

#include "common.hpp"

#include <stdint.h>
#include "gpio.hpp"
#include "../utils/IntTransitionDebouncer.hpp"

/*
 *  TODO  add documentation
 */

/*
 *  TODO  double click? how
 */

namespace HAL {

enum class ButtonAction : uint8_t { NONE, RELEASE, PRESS, LONG_PRESS };

namespace Drivers {

template<uint8_t  physicalPin,
         uint32_t debounceWaitTime,
         uint32_t longPressWaitTime,
         bool     passiveState,
         bool     usePullupP>
class Button {

    using Callback = void (*)();

    HAL::Utils::IntTransitionDebouncer<physicalPin,
                                       debounceWaitTime,
                                       passiveState,
                                       usePullupP> debouncer;
    Callback onRelease;
    Callback onPress;
    Callback onLongPress;

  public:
    Button()
        : debouncer   { },
          onRelease   { nullptr },
          onPress     { nullptr },
          onLongPress { nullptr } {
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

        if (btnTransition == Transition::FALLING) {
            if (passiveState) {
                if (onPress) onPress();
                btnAction = ButtonAction::PRESS;
            } else {
                if (onRelease) onRelease();
                btnAction =  ButtonAction::RELEASE;
            }
        } else if (btnTransition == Transition::RISING) {
            if (passiveState) {
                if (onRelease) onRelease();
                btnAction =  ButtonAction::RELEASE;
            } else {
                if (onPress) onPress();
                btnAction = ButtonAction::PRESS;
            }
        }

        return btnAction;
    }


    bool pendingDebounceTimeout() {
        //  TODO  AND IF HELD DOWN!
        return debouncer.pendingDebounceTimeout();
    }

};


}
}
