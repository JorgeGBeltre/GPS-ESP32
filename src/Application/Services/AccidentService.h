#pragma once

#include "Application/Messaging/MqttTopics.h"
#include "Application/Serialization/IPayloadSerializer.h"
#include "Application/Services/ConnectivityService.h"
#include "Domain/Entities/AccidentEvent.h"
#include "Domain/Entities/GpsLocation.h"
#include "Domain/Interfaces/IAccidentDetector.h"
#include "Domain/Interfaces/IDeviceRepository.h"
#include "Domain/Interfaces/IGpsProvider.h"
#include "Domain/Interfaces/IGsmAlertSender.h"
#include "Domain/Interfaces/ILogger.h"
#include "Domain/Interfaces/ITimeProvider.h"

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
          connectivity_(connectivity),
          deviceRepo_(deviceRepo),
          timeProvider_(timeProvider),
          serializer_(serializer),
          topics_(topics),
          log_(logger),
          gsmSender_(gsmSender) {}

    void process();
    void setGsmSender(domain::IGsmAlertSender* gsm) { gsmSender_ = gsm; }

private:
    domain::IAccidentDetector& detector_;
    domain::IGpsProvider&      gps_;
    ConnectivityService&       connectivity_;
    domain::IDeviceRepository& deviceRepo_;
    domain::ITimeProvider&     timeProvider_;
    IPayloadSerializer&        serializer_;
    const MqttTopics           topics_;
    domain::ILogger&           log_;
    domain::IGsmAlertSender*   gsmSender_;
};

} // namespace relay::app
