#pragma once

#include <string>
#include "Domain/Entities/AccidentEvent.h"
#include "Domain/Entities/GpsLocation.h"

namespace relay::domain {

class IGsmAlertSender {
public:
    virtual ~IGsmAlertSender() = default;

    virtual void begin(const std::string& authCode = "") = 0;
    virtual void update() = 0;
    virtual bool sendAlert(const AccidentEvent& accident,
                           const GpsLocation& gps,
                           const std::string& timestamp,
                           const std::string& chipId) = 0;
    virtual bool sendSms(const std::string& phoneNumber, const std::string& message) = 0;
    virtual bool isReady() = 0;
};

} // namespace relay::domain
