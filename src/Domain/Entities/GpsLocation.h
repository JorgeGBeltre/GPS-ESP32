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
    float       hdop = 99.9f;
    int         satellites = 0;
    bool        isValid = false;
    bool        driftFiltered = false;
    bool        towDetected = false;
    bool        virtualIgnition = false;
    bool        jammingDetected = false;
    bool        roaming = false;
    std::string timestamp;
};

} // namespace relay::domain
