#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "Domain/Interfaces/IGpsProvider.h"

namespace relay::infra {

class TinyGpsPlusProvider : public domain::IGpsProvider {
public:
    TinyGpsPlusProvider(HardwareSerial& serial = Serial2,
                        uint8_t rxPin = 16,
                        uint8_t txPin = 17,
                        uint32_t baud = 9600);

    void begin() override;
    void update() override;
    void readData(domain::GpsLocation& out) override;
    float currentSpeedKmph() override;
    int getSatellites() override;
    bool hasFix() override;
    void diagnose() override;

    TinyGPSPlus& rawGps() { return gps_; }

private:
    TinyGPSPlus     gps_;
    HardwareSerial& gpsSerial_;
    uint8_t         rxPin_;
    uint8_t         txPin_;
    uint32_t        baud_;
};

} // namespace relay::infra
