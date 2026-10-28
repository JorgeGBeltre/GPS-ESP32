#pragma once

#include <string>
#include <cstdint>

namespace relay::domain {

struct GpsLocation {
    double      latitude = 0.0;
    double      longitude = 0.0;
    float       altitude = 0.0f;
    float       speed = 0.0f;       // km/h
    float       course = 0.0f;      // degrees
    int         satellites = 0;
    bool        isValid = false;
    std::string timestamp;
};

} // namespace relay::domain
