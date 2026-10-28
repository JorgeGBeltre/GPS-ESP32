#pragma once

#include "Domain/Entities/GpsLocation.h"

namespace relay::domain {

class IGpsProvider {
public:
    virtual ~IGpsProvider() = default;

    virtual void begin() = 0;
    virtual void update() = 0;
    virtual void readData(GpsLocation& out) = 0;
    virtual float currentSpeedKmph() = 0;
    virtual int getSatellites() = 0;
    virtual bool hasFix() = 0;
    virtual void diagnose() = 0;
};

} // namespace relay::domain
