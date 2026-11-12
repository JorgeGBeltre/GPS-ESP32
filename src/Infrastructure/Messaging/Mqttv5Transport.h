#pragma once

#include <array>
#include <cstddef>
#include <string>
#include "Domain/Interfaces/ITelemetryTransport.h"

namespace relay::infra {

class Mqttv5Transport : public domain::ITelemetryTransport {
public:
    static constexpr size_t kQueueCapacity = 10;
    static constexpr size_t kMaxPayloadBytes = 1536;

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;
    bool publish(const std::string& topic, const std::string& payload) override;
    void subscribe(const std::string& topic) override;
    void onMessage(MessageHandler handler) override;
    void onConnect(std::function<void()> cb) override;
    size_t queuedCount() const override { return count_; }
    void loop() override;

private:
    struct QueuedFrame {
        std::string topic;
        std::string payload;
    };

    void enqueue(const std::string& topic, const std::string& payload);
    size_t flushQueue(size_t maxPerTick = 2);

    MessageHandler handler_;
    std::function<void()> onConnectCb_;
    std::array<QueuedFrame, kQueueCapacity> queue_{};
    size_t head_ = 0;
    size_t count_ = 0;
};

}
