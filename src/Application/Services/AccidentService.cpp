#include "Application/Services/AccidentService.h"

namespace relay::app {

void AccidentService::process() {
    domain::AccidentEvent event;

    if (!detector_.check(event, gps_.currentSpeedKmph())) {
        return;
    }

    // Attach latest GPS fix
    gps_.readData(event.location);

    auto id = deviceRepo_.identity();
    std::string timestamp = timeProvider_.isoTimestamp();

    log_.warn("[ALERT] " + event.alertType() + " detectado! Lat: " +
              std::to_string(event.location.latitude) + ", Lon: " +
              std::to_string(event.location.longitude));

    // 1. Publish immediately to MQTT / MQTTS v5
    if (connectivity_.transport().isConnected()) {
        std::string payload = serializer_.formatAccidentAlert(id, event, timestamp);
        std::string topic = topics_.deviceEvents; // event/<chipId>
        connectivity_.transport().publish(topic, payload);
        log_.info("[ALERT] Alerta publicada en " + topic);
    }

    // 2. Dispatch SMS alert if modem sender is present and ready
    if (gsmSender_ && gsmSender_->isReady()) {
        gsmSender_->sendAlert(event, event.location, timestamp, id.chipId);
        log_.info("[ALERT] Alerta SMS enviada al modem celular");
    }
}

} // namespace relay::app
