#pragma once

#include <Arduino.h>
#include "Domain/Interfaces/IIndicator.h"

namespace relay::infra {

/**
 * @brief Generador de patrones no bloqueante para el LED de estado (SLED) con base de 100 ms:
 * - Disconnected:  1000 ms ON / 1000 ms OFF (Buscando Wi-Fi)
 * - Connecting:    200 ms ON  / 1800 ms OFF (1 destello corto cada 2 s al conectar al broker MQTTv5)
 * - QuickPairing:  100 ms ON, 100 ms OFF, 100 ms ON, 700 ms OFF (2 destellos rápidos + pausa)
 * - CompatibleAp:  100 ms ON  / 100 ms OFF continuo (5 Hz en modo SoftAP GPS-<ChipID>)
 * - OtaProgress:   100 ms ON, 100 ms OFF x3 + 400 ms OFF (3 destellos rápidos + pausa)
 * - Operational:   Encendido fijo si sledOnline=true, Apagado si sledOnline=false
 */
class GpioIndicator : public domain::IIndicator {
public:
    static constexpr unsigned long BASE_TICK_MS = 100;

    explicit GpioIndicator(uint8_t pin, int fastBlinkMs = 100, int slowBlinkMs = 400)
        : pin_(pin), fastBlinkMs_(fastBlinkMs), slowBlinkMs_(slowBlinkMs) {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
    }

    void setSledOnline(bool enabled) override {
        sledOnline_ = enabled;
        if (status_ == domain::IndicatorStatus::Operational) {
            writePin(sledOnline_);
        }
    }

    bool isSledOnline() const override {
        return sledOnline_;
    }

    void show(domain::IndicatorStatus status) override {
        if (status_ != status) {
            status_ = status;
            tickStep_ = 0;
            lastTickMs_ = 0;
        }
        if (status_ == domain::IndicatorStatus::Operational) {
            writePin(sledOnline_);
        }
    }

    void update() override {
        if (status_ == domain::IndicatorStatus::Operational) {
            writePin(sledOnline_);
            return;
        }

        const unsigned long now = millis();
        if (lastTickMs_ != 0 && (now - lastTickMs_) < BASE_TICK_MS) {
            return;
        }
        lastTickMs_ = now;

        bool turnOn = false;
        switch (status_) {
            case domain::IndicatorStatus::Disconnected:
                turnOn = (tickStep_ % 20U) < 10U;
                break;

            case domain::IndicatorStatus::Connecting:
                turnOn = (tickStep_ % 20U) < 2U;
                break;

            case domain::IndicatorStatus::QuickPairing: {
                const uint32_t phase = tickStep_ % 10U;
                turnOn = (phase == 0U || phase == 2U);
                break;
            }

            case domain::IndicatorStatus::CompatibleAp:
                turnOn = (tickStep_ % 2U) == 0U;
                break;

            case domain::IndicatorStatus::OtaProgress:
            case domain::IndicatorStatus::Error: {
                const uint32_t phase = tickStep_ % 10U;
                turnOn = (phase == 0U || phase == 2U || phase == 4U);
                break;
            }

            case domain::IndicatorStatus::Operational:
            default:
                turnOn = sledOnline_;
                break;
        }

        ++tickStep_;
        writePin(turnOn);
    }

private:
    void writePin(bool on) {
        ledState_ = on;
        digitalWrite(pin_, on ? HIGH : LOW);
    }

    uint8_t pin_;
    int fastBlinkMs_;
    int slowBlinkMs_;
    domain::IndicatorStatus status_ = domain::IndicatorStatus::Disconnected;
    unsigned long lastTickMs_ = 0;
    uint32_t tickStep_ = 0;
    bool ledState_ = false;
    bool sledOnline_ = true;
};

}
