#include "Application/AccidentAlgorithm.h"
#include <cmath>

namespace relay::app {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kStaLtaRatio = 3.0f;       // STA/LTA anomaly ratio
constexpr float kHighSpeedKmph = 50.0f;    // False positive suppression speed
constexpr float kAccelLsbPerG = 16384.0f;  // MPU6050 default scale (+/-2g)
}

AccidentAlgorithm::AccidentAlgorithm() {
    reset();
}

void AccidentAlgorithm::reset() {
    oldx = 0;
    oldy = 0;
    oldz = 0;
    vibration = 2;
    devibrate = 75;
    acc_idx = 0;
    for (int i = 0; i < FILTER_SIZE; ++i) {
        acc_history[i] = 0.0f;
    }
}

AccidentSample AccidentAlgorithm::evaluate(int16_t ax, int16_t ay, int16_t az,
                                           int sensitivity, int rolloverAngleDeg,
                                           float speedKmph,
                                           float harshAccelG,
                                           float harshBrakeG) {
    AccidentSample sample;

    const int deltx = ax - oldx;
    const int delty = ay - oldy;
    const int deltz = az - oldz;

    oldx = ax;
    oldy = ay;
    oldz = az;

    // Vibration damping: swallow high-frequency rebound after a burst
    vibration--;
    if (vibration < 0) vibration = 0;
    if (vibration > 0) return sample;

    sample.evaluated = true;

    const float rawMag = std::sqrt(static_cast<float>(deltx) * deltx +
                                   static_cast<float>(delty) * delty +
                                   static_cast<float>(deltz) * deltz);

    // Moving average (LTA)
    acc_history[acc_idx % FILTER_SIZE] = rawMag;
    acc_idx++;
    float avgMag = 0.0f;
    for (int i = 0; i < FILTER_SIZE; ++i) {
        avgMag += acc_history[i];
    }
    avgMag /= FILTER_SIZE;

    const float STA = rawMag;
    const float LTA = avgMag > 0 ? avgMag : 1.0f;

    const bool high_g_impact = (rawMag >= static_cast<float>(sensitivity));
    const bool anomalia_detectada = (STA / LTA) > kStaLtaRatio;

    bool validacion_gps = true;
    if (high_g_impact && speedKmph > kHighSpeedKmph) {
        // High-G at sustained high speed is frequently rough terrain/potholes, not a collision
        validacion_gps = false;
    }

    if (high_g_impact && anomalia_detectada && validacion_gps) {
        sample.impact_detected = true;
        sample.impact_magnitude = rawMag;
        vibration = devibrate;
    }

    // Accelerations in G
    const float axf = ax / kAccelLsbPerG;
    const float ayf = ay / kAccelLsbPerG;
    const float azf = az / kAccelLsbPerG;

    // Roll and Pitch angles in degrees
    const float roll = std::atan2(ayf, std::sqrt(axf * axf + azf * azf)) * 180.0f / kPi;
    const float pitch = std::atan2(-axf, std::sqrt(ayf * ayf + azf * azf)) * 180.0f / kPi;

    sample.roll_angle = roll;
    sample.pitch_angle = pitch;

    if (std::fabs(roll) > rolloverAngleDeg || std::fabs(pitch) > rolloverAngleDeg) {
        sample.rollover_detected = true;
    }

    // Longitudinal driving dynamics (assuming X-axis is vehicle travel direction)
    sample.accel_g = axf;
    if (axf >= harshAccelG) {
        sample.harsh_accel_detected = true;
    } else if (axf <= harshBrakeG) {
        sample.harsh_brake_detected = true;
    }

    return sample;
}

} // namespace relay::app
