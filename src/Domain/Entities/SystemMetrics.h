#pragma once

#include <cstdint>
#include <string>

namespace relay::domain {

struct SystemMetrics {
    int           rssi              = 0;
    uint32_t      freeHeap          = 0;
    unsigned long uptime            = 0;
    bool          wifiConnected     = false;
    bool          mqttConnected     = false;
    uint32_t      bootCount         = 0;
    uint32_t      unexpectedReboots = 0;
    uint16_t      wdtOffences       = 0;
    uint16_t      modemRecoveries   = 0;
    uint16_t      queuedMessages    = 0;
    std::string   resetReason;
};

}
