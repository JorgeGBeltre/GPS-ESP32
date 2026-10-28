#pragma once

#include <cstdint>

namespace relay::app {

struct AccidentSample {
    bool evaluated = false;
    bool impact_detected = false;
    bool rollover_detected = false;
    bool harsh_accel_detected = false;
    bool harsh_brake_detected = false;

    float impact_magnitude = 0.0f;
    float roll_angle = 0.0f;
    float pitch_angle = 0.0f;
    float accel_g = 0.0f;
};

class AccidentAlgorithm {
public:
    static constexpr int FILTER_SIZE = 5;

    AccidentAlgorithm();

    AccidentSample evaluate(int16_t ax, int16_t ay, int16_t az,
                            int sensitivity, int rolloverAngleDeg,
                            float speedKmph,
                            float harshAccelG = 0.45f,
                            float harshBrakeG = -0.55f);

    void reset();

private:
    int16_t oldx, oldy, oldz;
    int     vibration;
    int     devibrate;
    float   acc_history[FILTER_SIZE];
    int     acc_idx;
};

} // namespace relay::app
