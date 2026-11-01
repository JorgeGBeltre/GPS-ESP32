#include "Infrastructure/Button/GpioButton.h"
#include <Arduino.h>

namespace relay::infra {

void GpioButton::init() {
    pinMode(pin_, INPUT_PULLUP);
    debouncedPressed_ = false;
    longPressFired_ = false;
    consecutiveMatchCount_ = 0;
    lastSampleTimeMs_ = millis();
}

domain::ButtonEvent GpioButton::update() {
    const unsigned long now = millis();
    if (now - lastSampleTimeMs_ < SAMPLE_PERIOD_MS) {
        return domain::ButtonEvent::None;
    }
    lastSampleTimeMs_ = now;

    const bool rawPressed = (digitalRead(pin_) == LOW);
    const uint8_t requiredSamples = (debounceMs_ >= SAMPLE_PERIOD_MS)
                                        ? static_cast<uint8_t>(debounceMs_ / SAMPLE_PERIOD_MS)
                                        : 1;

    if (rawPressed != debouncedPressed_) {
        ++consecutiveMatchCount_;
        if (consecutiveMatchCount_ >= requiredSamples) {
            debouncedPressed_ = rawPressed;
            consecutiveMatchCount_ = 0;

            if (debouncedPressed_) {
                pressStartTimeMs_ = now;
                longPressFired_ = false;
            } else {
                const unsigned long heldMs = now - pressStartTimeMs_;
                if (!longPressFired_ && heldMs >= debounceMs_ && heldMs < longPressMs_) {
                    return domain::ButtonEvent::ShortPress;
                }
            }
        }
    } else {
        consecutiveMatchCount_ = 0;
        if (debouncedPressed_ && !longPressFired_) {
            const unsigned long heldMs = now - pressStartTimeMs_;
            if (heldMs >= longPressMs_) {
                longPressFired_ = true;
                return domain::ButtonEvent::LongPress;
            }
        }
    }

    return domain::ButtonEvent::None;
}

}
