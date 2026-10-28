#pragma once

#include "Domain/Entities/AccidentEvent.h"

namespace relay::domain {

class IAccidentDetector {
public:
    virtual ~IAccidentDetector() = default;

    virtual void begin() = 0;
    virtual bool check(AccidentEvent& out, float currentSpeedKmph) = 0;
};

} // namespace relay::domain
