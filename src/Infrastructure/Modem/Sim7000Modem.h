#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <string>
#include <vector>
#include "Domain/Interfaces/IGpsProvider.h"
#include "Domain/Interfaces/IGsmAlertSender.h"
#include "Domain/Interfaces/ILogger.h"
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

    Sim7000Modem(HardwareSerial& serial,
                 uint8_t rxPin,
                 uint8_t txPin,
                 uint8_t pwrKeyPin,
                 uint32_t baud,
                 domain::ILogger& /*logger*/,
                 const std::string& emergencyPhone = "+18090000000")
        : Sim7000Modem(serial, rxPin, txPin, pwrKeyPin, baud, emergencyPhone) {}

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
    uint16_t modemRecoveries() const override { return modemRecoveries_; }
    void setOtaActive(bool active) { otaActive_ = active; }
    void recoverUnresponsiveModem();

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
    void setApn(const std::string& apn, const std::string& user = "", const std::string& pass = "") {
        apn_     = apn;
        apnUser_ = user;
        apnPass_ = pass;
    }
    void setRawLogging(bool enabled) { rawLogging_ = enabled; }
    void enableGnss(bool enable) {
        if (enable) configureGnssAt();
    }

    void probeCellularProvisioningState();
    bool isSimPresent() const { return simPresent_; }
    bool isApnAutoDetected() const { return apnAutoDetected_; }
    bool isCellularConnected() const { return cellularConnected_; }
    const std::string& resolvedApn() const { return resolvedApn_; }
    int mcc() const { return mcc_; }
    int mnc() const { return mnc_; }
    uint32_t lac() const { return lac_; }
    uint32_t cellId() const { return cellId_; }

private:
    bool sendATCommand(const String& cmd, const String& expectedResponse, unsigned long timeoutMs = 2000);
    String sendATCommandWithReply(const String& cmd, unsigned long timeoutMs = 2000);
    bool probeAtOrNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, int attempts, unsigned long timeoutMs);
    bool listenForNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, unsigned long durationMs);
    std::string autoResolveCarrierApn();
    void applyDriftAndTowFilter(double rawLat, double rawLon, float speedKmh, float hdop);
    void pollCellularHealth();
    void pollCellularLbs(bool requestCoordinatesIfNoGnss);
    void applyLbsToCachedLocation(bool hasSatelliteFix);
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
    std::string     apn_;
    std::string     apnUser_;
    std::string     apnPass_;
    bool            rawLogging_ = false;

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
    unsigned long   lastCellCheckMs_ = 0;
    unsigned long   lastLbsQueryMs_ = 0;

    domain::GpsLocation cachedLocation_;
    bool                hasAnchor_ = false;
    double              anchorLat_ = 0.0;
    double              anchorLon_ = 0.0;
    int                 lastCsq_ = 0;
    bool                jammingDetected_ = false;
    bool                roaming_ = false;
    int                 mcc_ = 0;
    int                 mnc_ = 0;
    uint32_t            lac_ = 0;
    uint32_t            cellId_ = 0;
    bool                hasLbsCoords_ = false;
    double              lbsLat_ = 0.0;
    double              lbsLon_ = 0.0;
    int                 lbsAccuracyM_ = 0;
    int                 satellitesInView_ = 0;
    int                 satellitesUsed_ = 0;
    int                 gsvSatsGps_ = 0;
    int                 gsvSatsGlonass_ = 0;
    int                 maxC_N0_ = 0;
    String              lastCgnsInfRaw_;
    bool                simPresent_ = false;
    bool                apnAutoDetected_ = false;
    bool                cellularConnected_ = false;
    std::string         resolvedApn_;
    bool                towLatched_ = false;
    bool                otaActive_ = false;
    uint8_t             consecutiveAtFailures_ = 0;
    uint8_t             unreadyProbeCount_ = 0;
    uint16_t            modemRecoveries_ = 0;
    unsigned long       lastModemRecoveryMs_ = 0;
};

} // namespace relay::infra
