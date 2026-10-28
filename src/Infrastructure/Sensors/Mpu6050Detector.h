#pragma once

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include "Application/AccidentAlgorithm.h"
#include "Domain/Interfaces/IAccidentDetector.h"

namespace relay::infra {

class Mpu6050Detector : public domain::IAccidentDetector {
public:
    Mpu6050Detector(uint8_t sdaPin = 21,
                    uint8_t sclPin = 22,
                    int sensitivity = 2000,
                    int rolloverAngleDeg = 60,
                    float harshAccelG = 0.45f,
                    float harshBrakeG = -0.55f,
                    unsigned long checkIntervalMs = 100,
                    unsigned long cooldownMs = 60000);

    void begin() override;
    bool check(domain::AccidentEvent& out, float currentSpeedKmph) override;

    bool isConnected() const { return isConnected_; }
    void setSensitivity(int sens) { sensitivity_ = sens; }
    void setRolloverAngle(int deg) { rolloverAngleDeg_ = deg; }
    void setHarshDynamicsThresholds(float accelG, float brakeG) {
        harshAccelG_ = accelG;
        harshBrakeG_ = brakeG;
    }

private:
    uint8_t               sdaPin_;
    uint8_t               sclPin_;
    int                   sensitivity_;
    int                   rolloverAngleDeg_;
    float                 harshAccelG_;
    float                 harshBrakeG_;
    unsigned long         checkIntervalMs_;
    unsigned long         cooldownMs_;

    MPU6050               mpu_;
    app::AccidentAlgorithm algorithm_;
    bool                  isConnected_ = false;
    unsigned long         lastCheckMs_ = 0;
    unsigned long         lastEventMs_ = 0;
};

} // namespace relay::infra
