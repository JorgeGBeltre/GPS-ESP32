#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string>
#include "Domain/Interfaces/IGsmAlertSender.h"

namespace relay::infra {

class Sim7000AlertSender : public domain::IGsmAlertSender {
public:
    Sim7000AlertSender(HardwareSerial& serial = Serial1,
                       uint8_t rxPin = 26,
                       uint8_t txPin = 27,
                       uint32_t baud = 115200,
                       const std::string& emergencyPhone = "+18296741199");

    void begin(const std::string& authCode = "") override;
    void update() override;
    bool sendAlert(const domain::AccidentEvent& accident,
                   const domain::GpsLocation& gps,
                   const std::string& timestamp,
                   const std::string& chipId) override;
    bool sendSms(const std::string& phoneNumber, const std::string& message) override;
    bool isReady() override;

    void setEmergencyPhone(const std::string& phone) { emergencyPhone_ = phone; }
    void setCaCert(const char* caCertPem);

private:
    bool sendATCommand(const String& command, const String& expectedResponse, unsigned long timeoutMs = 2000);

    HardwareSerial& serial_;
    uint8_t         rxPin_;
    uint8_t         txPin_;
    uint32_t        baud_;
    std::string     emergencyPhone_;
    bool            isReady_ = false;
    unsigned long   lastCheckMs_ = 0;
};

} // namespace relay::infra
