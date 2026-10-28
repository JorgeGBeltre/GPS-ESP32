#pragma once

#include <cstdint>
#include <string>
#include "Domain/Entities/GpsLocation.h"

namespace relay::domain {

struct AccidentEvent {
    bool        impact_detected = false;
    bool        rollover_detected = false;
    bool        harsh_accel_detected = false;
    bool        harsh_brake_detected = false;

    float       impact_magnitude = 0.0f;
    float       roll_angle = 0.0f;
    float       pitch_angle = 0.0f;
    float       accel_g = 0.0f;
    unsigned long detection_time = 0;

    GpsLocation location;

    bool hasAlert() const {
        return impact_detected || rollover_detected || harsh_accel_detected || harsh_brake_detected;
    }

    std::string alertType() const {
        if (impact_detected) return "IMPACT";
        if (rollover_detected) return "ROLLOVER";
        if (harsh_brake_detected) return "HARSH_BRAKE";
        if (harsh_accel_detected) return "HARSH_ACCEL";
        return "NONE";
    }
};

} // namespace relay::domain
