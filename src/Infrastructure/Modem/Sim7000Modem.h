#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <string>
#include <vector>
#include "Domain/Interfaces/IGpsProvider.h"
#include "Domain/Interfaces/IGsmAlertSender.h"
#include "Domain/Entities/GpsLocation.h"
#include "Domain/Entities/AccidentEvent.h"

namespace relay::infra {

class Sim7000Modem : public domain::IGpsProvider, public domain::IGsmAlertSender {
public:
    Sim7000Modem(HardwareSerial& serial = Serial2,
                 uint8_t rxPin = 16,
                 uint8_t txPin = 17,
                 uint8_t pwrKeyPin = 23,
                 uint32_t baud = 115200,
                 const std::string& emergencyPhone = "+18090000000");

    // Electronic Power Control (PWRKEY)
    void powerOn();
    void powerOff();
    void restartModem();

    // IGpsProvider overrides
    void begin() override;
    void update() override;
    void readData(domain::GpsLocation& out) override;
    float currentSpeedKmph() override;
    int getSatellites() override;
    bool hasFix() override;
    void diagnose() override;

    // IGsmAlertSender overrides
    void begin(const std::string& authCode) override;
    bool sendAlert(const domain::AccidentEvent& accident,
                   const domain::GpsLocation& gps,
                   const std::string& timestamp,
                   const std::string& chipId) override;
    bool sendSms(const std::string& phoneNumber, const std::string& message) override;
    bool isReady() override;

    void setEmergencyPhone(const std::string& phone) { emergencyPhone_ = phone; }
    void setCaCert(const char* caCertPem);

private:
    bool sendATCommand(const String& cmd, const String& expectedResponse, unsigned long timeoutMs = 2000);
    String sendATCommandWithReply(const String& cmd, unsigned long timeoutMs = 2000);
    bool probeAtOrNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, int attempts, unsigned long timeoutMs);
    bool listenForNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, unsigned long durationMs);
    void configureGnssAt();
    void queryGnss();
    void parseGnssInf(const String& line);
    void processIncomingByte(char c);
    void parseNmeaLine(const String& line);
    void syncFromNmea();

    HardwareSerial& serial_;
    uint8_t         rxPin_;
    uint8_t         txPin_;
    uint8_t         pwrKeyPin_;
    uint32_t        baud_;
    uint32_t        activeBaud_;
    std::string     emergencyPhone_;

    TinyGPSPlus     nmeaGps_;
    String          nmeaLineBuf_;

    bool            initialized_ = false;
    bool            isReady_ = false;
    bool            nmeaMode_ = false;
    bool            gnssPowered_ = false;
    uint32_t        rxBytesCount_ = 0;
    unsigned long   lastGnssQueryMs_ = 0;
    unsigned long   lastReadyCheckMs_ = 0;
    unsigned long   lastWarningMs_ = 0;

    domain::GpsLocation cachedLocation_;
    int                 satellitesInView_ = 0;
    int                 satellitesUsed_ = 0;
    int                 gsvSatsGps_ = 0;
    int                 gsvSatsGlonass_ = 0;
    int                 maxC_N0_ = 0;
    String              lastCgnsInfRaw_;
};

} // namespace relay::infra
