#pragma once

#include <cstdint>
#include "Domain/Interfaces/IButtonHandler.h"

namespace relay::infra {

/**
 * @brief Muestreador de botón físico activo en bajo (INPUT_PULLUP) con:
 * - Periodo de muestreo de 25 ms
 * - Antirrebote (debounce) por contador de muestras estables (50 ms = 2 muestras)
 * - Disparo inmediato de LongPress al cumplir 5,000 ms (sin esperar a soltar el botón)
 * - Disparo de ShortPress al soltar entre 50 ms y < 5,000 ms
 */
class GpioButton : public domain::IButtonHandler {
public:
    static constexpr uint16_t SAMPLE_PERIOD_MS = 25;

    GpioButton(uint8_t pin, uint16_t debounceMs = 50, uint16_t longPressMs = 5000)
        : pin_(pin), debounceMs_(debounceMs), longPressMs_(longPressMs) {}

    void init() override;
    domain::ButtonEvent update() override;
    uint8_t getPin() const override { return pin_; }

private:
    uint8_t pin_;
    uint16_t debounceMs_;
    uint16_t longPressMs_;

    unsigned long lastSampleTimeMs_ = 0;
    unsigned long pressStartTimeMs_ = 0;
    uint8_t consecutiveMatchCount_ = 0;
    bool debouncedPressed_ = false;
    bool longPressFired_ = false;
};

}
