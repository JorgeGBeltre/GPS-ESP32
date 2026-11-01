#pragma once

#include "Application/Messaging/MqttTopics.h"
#include "Application/Serialization/IPayloadSerializer.h"
#include "Application/Services/ConnectivityService.h"
#include "Domain/Entities/AccidentEvent.h"
#include "Domain/Entities/GpsLocation.h"
#include "Domain/Interfaces/IAccidentDetector.h"
#include "Domain/Interfaces/IClock.h"
#include "Domain/Interfaces/IDeviceRepository.h"
#include "Domain/Interfaces/IGpsProvider.h"
#include "Domain/Interfaces/IGsmAlertSender.h"
#include "Domain/Interfaces/ILogger.h"
#include "Domain/Interfaces/ITimeProvider.h"
#include "Application/Services/TelemetryService.h"

namespace relay::app {

class AccidentService {
public:
    AccidentService(domain::IAccidentDetector& detector,
                    domain::IGpsProvider& gps,
                    ConnectivityService& connectivity,
                    domain::IDeviceRepository& deviceRepo,
                    domain::ITimeProvider& timeProvider,
                    IPayloadSerializer& serializer,
                    const MqttTopics& topics,
                    domain::ILogger& logger,
                    domain::IGsmAlertSender* gsmSender = nullptr)
        : detector_(detector),
          gps_(gps),
          connectivity_(&connectivity),
          deviceRepo_(&deviceRepo),
          timeProvider_(&timeProvider),
          serializer_(&serializer),
          topics_(topics),
          log_(logger),
          gsmSender_(gsmSender) {}

    AccidentService(domain::IAccidentDetector& detector,
                    domain::IGpsProvider& gps,
                    domain::IGsmAlertSender& gsmSender,
                    TelemetryService& telemetry,
                    domain::IClock& /*clock*/,
                    domain::ILogger& logger,
                    const std::string& /*emergencyPhone*/,
                    bool /*smsAlertsEnabled*/,
                    unsigned long /*cooldownMs*/,
                    unsigned long /*checkIntervalMs*/)
        : detector_(detector),
          gps_(gps),
          telemetry_(&telemetry),
          log_(logger),
          gsmSender_(&gsmSender) {}

    void process();
    void setGsmSender(domain::IGsmAlertSender* gsm) { gsmSender_ = gsm; }

private:
    domain::IAccidentDetector& detector_;
    domain::IGpsProvider&      gps_;
    ConnectivityService*       connectivity_ = nullptr;
    domain::IDeviceRepository* deviceRepo_   = nullptr;
    domain::ITimeProvider*     timeProvider_ = nullptr;
    IPayloadSerializer*        serializer_   = nullptr;
    TelemetryService*          telemetry_    = nullptr;
    MqttTopics                 topics_{};
    domain::ILogger&           log_;
    domain::IGsmAlertSender*   gsmSender_    = nullptr;
};

} // namespace relay::app
