#include "Infrastructure/Messaging/Mqttv5Transport.h"
#include "Infrastructure/Messaging/mqtt_client.h"
#include "Infrastructure/Ota/ota_manager.h"

namespace relay::infra {

void Mqttv5Transport::enqueue(const std::string& topic, const std::string& payload) {
    if (topic.empty() || payload.empty() || payload.size() > kMaxPayloadBytes) return;
    const size_t slot = (head_ + count_) % kQueueCapacity;
    if (count_ == kQueueCapacity) {
        head_ = (head_ + 1) % kQueueCapacity;
    } else {
        ++count_;
    }
    queue_[slot].topic = topic;
    queue_[slot].payload = payload;
}

size_t Mqttv5Transport::flushQueue(size_t maxPerTick) {
    if (!isMQTTConnected() || otaInProgress() || count_ == 0) return 0;
    size_t sent = 0;
    while (count_ > 0 && sent < maxPerTick && isMQTTConnected() && !otaInProgress()) {
        const QueuedFrame& item = queue_[head_];
        if (!publishMQTT(String(item.topic.c_str()), String(item.payload.c_str()), 1, false)) {
            break;
        }
        queue_[head_].topic.clear();
        queue_[head_].payload.clear();
        head_ = (head_ + 1) % kQueueCapacity;
        --count_;
        ++sent;
    }
    return sent;
}

bool Mqttv5Transport::connect() {
    return connectMQTT();
}

void Mqttv5Transport::disconnect() {
    disconnectMQTT();
}

bool Mqttv5Transport::isConnected() const {
    return isMQTTConnected();
}

bool Mqttv5Transport::publish(const std::string& topic, const std::string& payload) {
    if (!isMQTTConnected() || otaInProgress()) {
        // Store-and-forward para eventos críticos (events/*, event/*) y transiciones de relé (*/report)
        if (!otaInProgress() &&
            (topic.rfind("events/", 0) == 0 ||
             topic.rfind("event/", 0) == 0 ||
             topic.find("/report") != std::string::npos)) {
            enqueue(topic, payload);
        }
        return false;
    }
    if (!publishMQTT(String(topic.c_str()), String(payload.c_str()), 1, false)) {
        if (topic.rfind("events/", 0) == 0 ||
            topic.rfind("event/", 0) == 0 ||
            topic.find("/report") != std::string::npos) {
            enqueue(topic, payload);
        }
        return false;
    }
    return true;
}

void Mqttv5Transport::subscribe(const std::string& topic) {
    subscribeMQTT(String(topic.c_str()), 1);
}

void Mqttv5Transport::onMessage(MessageHandler handler) {
    handler_ = std::move(handler);
    setMqttMessageSink([this](const String& topic, const String& payload) {
        if (handler_) {
            handler_(topic.c_str(), payload.c_str());
        }
    });
}

void Mqttv5Transport::onConnect(std::function<void()> cb) {
    onConnectCb_ = std::move(cb);
    setMqttConnectCallback([this]() {
        if (onConnectCb_) {
            onConnectCb_();
        }
    });
}

void Mqttv5Transport::loop() {
    mqttLoop();
    flushQueue(2);
}

}
