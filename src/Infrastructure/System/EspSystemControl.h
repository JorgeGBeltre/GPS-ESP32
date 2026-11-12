#pragma once

#include <Arduino.h>
#include <cstdint>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "Domain/Interfaces/ISystemControl.h"

namespace relay::infra {

namespace detail {
struct BootWatchdogRtcState {
    uint32_t magic;
    uint32_t bootCount;
    uint32_t unexpectedReboots;
    uint16_t wdtOffences;
    uint8_t  gracefulFlag;
    uint8_t  wdtStage2Flag;
};

inline RTC_NOINIT_ATTR BootWatchdogRtcState g_rtcState;
inline const char* g_resetReasonStr = "UNKNOWN";
inline esp_timer_handle_t g_wdtTimer = nullptr;
inline volatile int64_t g_lastHeartbeatUs = 0;
inline volatile uint8_t g_wdtStage = 0;
inline volatile bool g_wdtPaused = false;
inline volatile bool g_stage1WarnPending = false;

inline const char* mapResetReason(esp_reset_reason_t r, bool graceful, bool wdtStage2) {
    if (wdtStage2) return "MIT_WDT_STAGE2";
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT_PIN";
        case ESP_RST_SW:        return graceful ? "GRACEFUL_SW" : "UNEXPECTED_SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

inline void wdtTimerCallback(void* /*arg*/) {
    const int64_t nowUs = esp_timer_get_time();
    if (g_wdtPaused) {
        g_lastHeartbeatUs = nowUs;
        g_wdtStage = 0;
        return;
    }
    constexpr int64_t kStallWindowUs = 25000000LL; // 25s por etapa
    if (nowUs - g_lastHeartbeatUs < kStallWindowUs) return;
    if (g_wdtStage == 0) {
        g_wdtStage = 1;
        g_rtcState.wdtOffences++;
        g_lastHeartbeatUs = nowUs;
        g_stage1WarnPending = true;
    } else {
        g_rtcState.gracefulFlag = 0;
        g_rtcState.wdtStage2Flag = 1;
        g_rtcState.wdtOffences++;
        esp_restart();
    }
}
} // namespace detail

class EspSystemControl : public domain::ISystemControl {
public:
    static void initBootAndWatchdog() {
        constexpr uint32_t kMagic = 0x54524752UL;
        const esp_reset_reason_t reason = esp_reset_reason();
        if (detail::g_rtcState.magic != kMagic || reason == ESP_RST_POWERON) {
            detail::g_rtcState.magic = kMagic;
            detail::g_rtcState.bootCount = 1;
            detail::g_rtcState.unexpectedReboots = 0;
            detail::g_rtcState.wdtOffences = 0;
            detail::g_rtcState.gracefulFlag = 0;
            detail::g_rtcState.wdtStage2Flag = 0;
            detail::g_resetReasonStr = "POWERON";
        } else {
            detail::g_rtcState.bootCount++;
            const bool graceful = (detail::g_rtcState.gracefulFlag != 0);
            const bool wdtStage2 = (detail::g_rtcState.wdtStage2Flag != 0);
            detail::g_resetReasonStr = detail::mapResetReason(reason, graceful, wdtStage2);
            if (!graceful && reason != ESP_RST_EXT && reason != ESP_RST_DEEPSLEEP) {
                detail::g_rtcState.unexpectedReboots++;
            }
            detail::g_rtcState.gracefulFlag = 0;
            detail::g_rtcState.wdtStage2Flag = 0;
        }
        detail::g_lastHeartbeatUs = esp_timer_get_time();
        detail::g_wdtStage = 0;
        detail::g_wdtPaused = false;
        if (detail::g_wdtTimer == nullptr) {
            esp_timer_create_args_t args{};
            args.callback = &detail::wdtTimerCallback;
            args.name = "gps_wdt_2st";
            if (esp_timer_create(&args, &detail::g_wdtTimer) == ESP_OK) {
                esp_timer_start_periodic(detail::g_wdtTimer, 5000000ULL);
            }
        }
    }

    static void heartbeat() {
        detail::g_lastHeartbeatUs = esp_timer_get_time();
        detail::g_wdtStage = 0;
        if (detail::g_stage1WarnPending) {
            detail::g_stage1WarnPending = false;
            Serial.printf("[WDT] Etapa-1 recuperada (ofensas=%u)\n",
                          (unsigned)detail::g_rtcState.wdtOffences);
        }
    }

    static void setWatchdogPaused(bool paused) {
        detail::g_wdtPaused = paused;
        detail::g_lastHeartbeatUs = esp_timer_get_time();
        detail::g_wdtStage = 0;
    }

    static void markGracefulReboot() {
        detail::g_rtcState.gracefulFlag = 1;
        detail::g_rtcState.wdtStage2Flag = 0;
        setWatchdogPaused(true);
    }

    static uint32_t bootCount() { return detail::g_rtcState.bootCount; }
    static uint32_t unexpectedReboots() { return detail::g_rtcState.unexpectedReboots; }
    static uint16_t wdtOffences() { return detail::g_rtcState.wdtOffences; }
    static const char* resetReason() { return detail::g_resetReasonStr; }

    void reboot() override {
        Serial.println("[SYSTEM] Reiniciando ESP32...");
        markGracefulReboot();
        delay(100);
        ESP.restart();
    }
};

}
