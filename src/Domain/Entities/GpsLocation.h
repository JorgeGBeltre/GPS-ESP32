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

    // LBS (Location-Based Service / Cell ID positioning)
    bool        lbsValid = false;
    bool        isLbsFallback = false;
    int         mcc = 0;
    int         mnc = 0;
    uint32_t    lac = 0;
    uint32_t    cellId = 0;
    int         lbsAccuracyM = 0;
};

} // namespace relay::domain
