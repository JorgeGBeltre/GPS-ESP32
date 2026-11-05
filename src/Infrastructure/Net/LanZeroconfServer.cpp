#include "Infrastructure/Net/LanZeroconfServer.h"
#include "Application/Services/CryptoUtils.h"
#include "Infrastructure/Config/config.h"
#include "Infrastructure/Config/nvs_config.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <cstdio>
#include <cstring>

namespace relay::infra {

static constexpr uint16_t kLanDiscoveryUdpPort = 48899;

LanZeroconfServer::LanZeroconfServer(app::RelayService& relays,
                                     app::TelemetryService& telemetry,
                                     app::TimerSchedulerService& timerScheduler,
                                     domain::IIndicator& indicator,
                                     domain::IConfigRepository& configRepo,
                                     const std::string& activeDeviceId,
                                     const std::string& relayDeviceId,
                                     uint16_t port)
    : relays_(relays),
      telemetry_(telemetry),
      timerScheduler_(timerScheduler),
      indicator_(indicator),
      configRepo_(configRepo),
      activeDeviceId_(activeDeviceId),
      relayDeviceId_(relayDeviceId),
      port_(port),
      server_(port) {}

void LanZeroconfServer::registerEndpoint(const char* uri) {
    server_.on(uri, HTTP_POST, [this, uri]() {
        String body = server_.arg("plain");
        char responseBuf[896] = {0};
        int httpStatus = 200;
        dispatchZeroconf(uri, body, responseBuf, sizeof(responseBuf), httpStatus);
        server_.send(httpStatus, "application/json", responseBuf);
    });
}

void LanZeroconfServer::startServer() {
    if (running_) return;

    char hostName[48];
    std::snprintf(hostName, sizeof(hostName), "iot_%s", activeDeviceId_.c_str());
    if (MDNS.begin(hostName)) {
        MDNS.addService("_iot", "_tcp", port_);
        MDNS.addServiceTxt("_iot", "_tcp", "txtvers", "1");
        MDNS.addServiceTxt("_iot", "_tcp", "id", activeDeviceId_.c_str());
        MDNS.addServiceTxt("_iot", "_tcp", "type", "gps_relay");
        MDNS.addServiceTxt("_iot", "_tcp", "family", "gps_esp32");
        MDNS.addServiceTxt("_iot", "_tcp", "apivers", "1");
    }

    registerEndpoint("/zeroconf/switch");
    registerEndpoint("/zeroconf/switches");
    registerEndpoint("/zeroconf/startup");
    registerEndpoint("/zeroconf/pulse");
    registerEndpoint("/zeroconf/sled");
    registerEndpoint("/zeroconf/info");
    registerEndpoint("/zeroconf/signal_strength");
    registerEndpoint("/zeroconf/wifi");
    registerEndpoint("/zeroconf/config");
    registerEndpoint("/zeroconf/timers");
    registerEndpoint("/zeroconf/ota_unlock");
    registerEndpoint("/zeroconf/unlink");

    server_.on("/zeroconf/info", HTTP_GET, [this]() {
        char responseBuf[896] = {0};
        int httpStatus = 200;
        dispatchZeroconf("/zeroconf/info", "{}", responseBuf, sizeof(responseBuf), httpStatus);
        server_.send(httpStatus, "application/json", responseBuf);
    });

    server_.begin();
    udpRunning_ = discoveryUdp_.begin(kLanDiscoveryUdpPort) != 0;
    running_ = true;
    notifyMdnsStateChange();
    Serial.printf("[LAN] Servidor HTTP Zeroconf iniciado en puerto %u (mDNS: %s.local)\n", port_, hostName);
}

void LanZeroconfServer::stopServer() {
    if (!running_) return;
    if (udpRunning_) {
        discoveryUdp_.stop();
        udpRunning_ = false;
    }
    server_.stop();
    MDNS.end();
    running_ = false;
}

void LanZeroconfServer::handleDiscoveryUdp() {
    if (!udpRunning_) return;
    int pktLen = discoveryUdp_.parsePacket();
    if (pktLen <= 0 || pktLen > 256) return;

    char reqBuf[257] = {0};
    int readBytes = discoveryUdp_.read(reqBuf, 256);
    if (readBytes <= 0) return;
    reqBuf[readBytes] = '\0';

    if (std::strstr(reqBuf, "IOT_DISCOVER") == nullptr) return;

    const auto* st = relays_.getState(relayDeviceId_);
    const bool relayOn = (st && st->currentState);
    const char* credMode = resolveDeviceCredentialMode(true);
    const bool adoptReady = (std::strcmp(credMode, "adopt_only") == 0);
    const String ipStr = WiFi.localIP().toString();

    char resp[512];
    std::snprintf(
        resp, sizeof(resp),
        "{\"deviceid\":\"%s\",\"chip_id\":\"%s\",\"ip\":\"%s\",\"port\":%u,"
        "\"device_family\":\"gps_esp32\",\"device_code\":2,"
        "\"model\":\"VESIS-GPS-ESP32\",\"type\":\"gps_relay\",\"fwVersion\":\"%s\","
        "\"credential_mode\":\"%s\",\"adopt_ready\":%s,\"apn_auto_detected\":%s,"
        "\"account_bound\":%s,\"switch\":\"%s\"}",
        activeDeviceId_.c_str(),
        activeDeviceId_.c_str(),
        ipStr.c_str(),
        port_,
        FIRMWARE_VERSION,
        credMode,
        adoptReady ? "true" : "false",
        isCellularApnAutoDetected() ? "true" : "false",
        isAccountBound() ? "true" : "false",
        relayOn ? "on" : "off");

    discoveryUdp_.beginPacket(discoveryUdp_.remoteIP(), discoveryUdp_.remotePort());
    discoveryUdp_.write(reinterpret_cast<const uint8_t*>(resp), std::strlen(resp));
    discoveryUdp_.endPacket();
}

void LanZeroconfServer::loop(bool wifiConnected) {
    if (!wifiConnected) {
        if (running_) stopServer();
        return;
    }
    if (!running_) {
        startServer();
    }
    server_.handleClient();
    handleDiscoveryUdp();
}

void LanZeroconfServer::notifyMdnsStateChange() {
    if (!running_) return;

    ++sequence_;
    char seqBuf[16];
    std::snprintf(seqBuf, sizeof(seqBuf), "%u", sequence_);

    const auto* st = relays_.getState(relayDeviceId_);
    const bool relayOn = (st && st->currentState);

    char plainData[220];
    std::snprintf(plainData, sizeof(plainData),
                  "{\"switch\":\"%s\",\"startup\":\"%s\",\"pulse\":\"%s\",\"pulseWidth\":%u,\"sledOnline\":\"%s\"}",
                  relayOn ? "on" : "off",
                  domain::startupModeToString(relays_.getStartupMode()),
                  relays_.isPulseEnabled() ? "on" : "off",
                  relays_.getPulseWidthMs(),
                  indicator_.isSledOnline() ? "on" : "off");

    MDNS.addServiceTxt("_iot", "_tcp", "seq", static_cast<const char*>(seqBuf));

    domain::DeviceConfig devCfg;
    configRepo_.load(devCfg);
    if (!devCfg.deviceKey.empty()) {
        uint8_t rawIv[16] = {0};
        std::snprintf(reinterpret_cast<char*>(rawIv), sizeof(rawIv), "%015u", sequence_);
        char ivB64[32] = {0};
        app::CryptoUtils::base64Encode(rawIv, 16, ivB64, sizeof(ivB64));
        char dataB64[360] = {0};
        if (app::CryptoUtils::encryptZeroconfPayload(
                devCfg.deviceKey.c_str(), ivB64, plainData, dataB64, sizeof(dataB64))) {
            MDNS.addServiceTxt("_iot", "_tcp", "encrypt", "true");
            MDNS.addServiceTxt("_iot", "_tcp", "iv", static_cast<const char*>(ivB64));
            MDNS.addServiceTxt("_iot", "_tcp", "data1", static_cast<const char*>(dataB64));
            return;
        }
    }

    MDNS.addServiceTxt("_iot", "_tcp", "data1", static_cast<const char*>(plainData));
}

void LanZeroconfServer::dispatchZeroconf(const char* uri, const String& rawBody, char* respBuf, size_t respBufSize, int& httpStatusOut) {
    httpStatusOut = 200;
    auto writeError = [&](int errCode, int httpCode = 200) {
        httpStatusOut = httpCode;
        std::snprintf(respBuf, respBufSize,
                      "{\"sequence\":\"%u\",\"seq\":%u,\"error\":%d}",
                      sequence_, sequence_, errCode);
    };

    if (rawBody.length() == 0 || rawBody.length() > 1024) {
        writeError(400);
        return;
    }

    StaticJsonDocument<768> outerDoc;
    if (deserializeJson(outerDoc, rawBody) != DeserializationError::Ok) {
        writeError(400);
        return;
    }

    bool isEncrypted = outerDoc["encrypt"].as<bool>();
    char decryptedJson[512] = {0};
    StaticJsonDocument<512> innerDoc;
    JsonVariant dataObj;

    domain::DeviceConfig devCfg;
    configRepo_.load(devCfg);

    if (isEncrypted) {
        const char* ivB64 = outerDoc["iv"];
        const char* dataB64 = outerDoc["data"];
        if (!ivB64 || !dataB64 || devCfg.deviceKey.empty()) {
            writeError(400);
            return;
        }
        if (!app::CryptoUtils::decryptZeroconfPayload(devCfg.deviceKey.c_str(), ivB64, dataB64, decryptedJson, sizeof(decryptedJson))) {
            writeError(400);
            return;
        }
        if (deserializeJson(innerDoc, decryptedJson) != DeserializationError::Ok) {
            writeError(400);
            return;
        }
        dataObj = innerDoc.as<JsonVariant>();
    } else if (outerDoc["data"].is<JsonObject>()) {
        dataObj = outerDoc["data"];
    } else if (outerDoc["data"].is<const char*>()) {
        if (deserializeJson(innerDoc, outerDoc["data"].as<const char*>()) != DeserializationError::Ok) {
            writeError(400);
            return;
        }
        dataObj = innerDoc.as<JsonVariant>();
    } else {
        dataObj = outerDoc.as<JsonVariant>();
    }

    const unsigned long now = millis();

    if (std::strcmp(uri, "/zeroconf/unlink") == 0) {
        const char* proof = dataObj["unlink_proof"] | outerDoc["unlink_proof"];
        if (!proof || !verifyUnlinkProof(devCfg.deviceKey.c_str(), activeDeviceId_.c_str(), proof)) {
            writeError(403, 403);
            return;
        }
        writeError(0, 200);
        if (unlinkCallback_) {
            server_.send(200, "application/json", respBuf);
            delay(150);
            unlinkCallback_();
        }
        return;
    }

    if (std::strcmp(uri, "/zeroconf/switch") == 0 || std::strcmp(uri, "/zeroconf/switches") == 0) {
        const char* sw = dataObj["switch"];
        if (!sw && dataObj["switches"].is<JsonArray>()) {
            sw = dataObj["switches"][0]["switch"];
        }
        if (!sw) {
            writeError(400);
            return;
        }
        const bool turnOn = (std::strcmp(sw, "on") == 0);
        const bool turnOff = (std::strcmp(sw, "off") == 0);
        if (!turnOn && !turnOff) {
            writeError(400);
            return;
        }
        relays_.setDesiredState(relayDeviceId_, turnOn);
        relays_.applyDesiredState(relayDeviceId_, now);
        telemetry_.publishEvent("LAN_ZEROCONF_SWITCH", std::string("Cambio por HTTP LAN: ") + sw, sw);
        telemetry_.publishRelayState(relayDeviceId_);
        notifyMdnsStateChange();
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/startup") == 0) {
        const char* st = dataObj["startup"];
        if (!st) {
            writeError(400);
            return;
        }
        relays_.setStartupMode(domain::startupModeFromString(st));
        notifyMdnsStateChange();
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/pulse") == 0) {
        const char* pl = dataObj["pulse"];
        if (!pl) {
            writeError(400);
            return;
        }
        const bool en = (std::strcmp(pl, "on") == 0);
        const uint32_t widthMs = dataObj["pulseWidth"] | relays_.getPulseWidthMs();
        if (!relays_.configurePulse(en, widthMs, now)) {
            writeError(400);
            return;
        }
        notifyMdnsStateChange();
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/sled") == 0) {
        const char* sl = dataObj["sledOnline"];
        if (!sl) {
            writeError(400);
            return;
        }
        const bool en = (std::strcmp(sl, "on") == 0);
        indicator_.setSledOnline(en);
        configRepo_.saveSledOnline(en);
        notifyMdnsStateChange();
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/signal_strength") == 0) {
        ++sequence_;
        std::snprintf(respBuf, respBufSize,
                      "{\"sequence\":\"%u\",\"seq\":%u,\"error\":0,\"data\":{\"signalStrength\":%d}}",
                      sequence_, sequence_, (int)WiFi.RSSI());
        return;
    }

    if (std::strcmp(uri, "/zeroconf/info") == 0) {
        ++sequence_;
        const auto* st = relays_.getState(relayDeviceId_);
        const bool relayOn = (st && st->currentState);
        const char* credMode = resolveDeviceCredentialMode(true);
        const bool adoptReady = (std::strcmp(credMode, "adopt_only") == 0);
        const char* effectiveApn = !devCfg.apn.empty() ? devCfg.apn.c_str() : getCellularResolvedApn();

        std::snprintf(
            respBuf, respBufSize,
            "{\"sequence\":\"%u\",\"seq\":%u,\"error\":0,\"data\":{"
            "\"deviceid\":\"%s\",\"chip_id\":\"%s\","
            "\"device_family\":\"gps_esp32\",\"device_code\":2,"
            "\"device_type\":\"GPS-Relay\",\"model\":\"VESIS-GPS-ESP32\","
            "\"credential_mode\":\"%s\",\"adopt_ready\":%s,"
            "\"apn_auto_detected\":%s,\"cellular_connected\":%s,"
            "\"account_bound\":%s,"
            "\"switch\":\"%s\",\"startup\":\"%s\","
            "\"pulse\":\"%s\",\"pulseWidth\":%u,\"sledOnline\":\"%s\","
            "\"apn\":\"%s\",\"emergencyPhone\":\"%s\","
            "\"fwVersion\":\"%s\",\"rssi\":%d}}",
            sequence_, sequence_,
            activeDeviceId_.c_str(),
            activeDeviceId_.c_str(),
            credMode,
            adoptReady ? "true" : "false",
            isCellularApnAutoDetected() ? "true" : "false",
            isCellularDataConnected() ? "true" : "false",
            isAccountBound() ? "true" : "false",
            relayOn ? "on" : "off",
            domain::startupModeToString(relays_.getStartupMode()),
            relays_.isPulseEnabled() ? "on" : "off",
            relays_.getPulseWidthMs(),
            indicator_.isSledOnline() ? "on" : "off",
            effectiveApn ? effectiveApn : "",
            devCfg.emergencyPhone.c_str(),
            FIRMWARE_VERSION,
            (int)WiFi.RSSI());
        return;
    }

    if (std::strcmp(uri, "/zeroconf/wifi") == 0 || std::strcmp(uri, "/zeroconf/config") == 0) {
        const char* ssid = dataObj["ssid"] | dataObj["wifi_ssid"];
        if (std::strcmp(uri, "/zeroconf/wifi") == 0 && (!ssid || std::strlen(ssid) == 0 || std::strlen(ssid) > 32)) {
            writeError(400);
            return;
        }
        if (ssid && std::strlen(ssid) > 0 && std::strlen(ssid) <= 32) {
            devCfg.wifiSsid = ssid;
            devCfg.wifiPass = dataObj["password"] | (dataObj["wifi_pass"] | "");
        }
        if (const char* apn = dataObj["apn"]) {
            if (std::strlen(apn) > 0) devCfg.apn = apn;
        }
        if (const char* au = dataObj["apn_user"]) {
            devCfg.apnUser = au;
        }
        if (const char* ap = dataObj["apn_pass"]) {
            devCfg.apnPass = ap;
        }
        if (const char* ph = dataObj["emergency_phone"] | dataObj["phone"]) {
            if (std::strlen(ph) > 0) devCfg.emergencyPhone = ph;
        }
        if (dataObj.containsKey("sensitivity")) {
            devCfg.accidentSensitivity = dataObj["sensitivity"].as<int>();
        }
        if (dataObj.containsKey("rollover_angle")) {
            devCfg.rolloverAngleDeg = dataObj["rollover_angle"].as<int>();
        }
        if (const char* brk = dataObj["mqtt_broker"] | (dataObj["serverName"] | dataObj["server"])) {
            if (std::strlen(brk) > 0) devCfg.mqttBroker = brk;
        }
        if (dataObj.containsKey("mqtt_port") || dataObj.containsKey("port")) {
            int p = dataObj["mqtt_port"] | (dataObj["port"] | 0);
            if (p > 0 && p <= 65535) devCfg.mqttPort = p;
        }
        if (const char* usr = dataObj["mqtt_user"] | (dataObj["apikey"] | dataObj["account"])) {
            if (std::strlen(usr) > 0) {
                devCfg.mqttUser = usr;
                devCfg.accountBound = true;
            }
        }
        if (const char* mp = dataObj["mqtt_pass"]) {
            devCfg.mqttPass = mp;
        }
        if (const char* tok = dataObj["bind_token"] | dataObj["token"]) {
            devCfg.bindToken = tok;
            if (std::strlen(tok) > 0) devCfg.accountBound = true;
        }
        configRepo_.save(devCfg);
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/timers") == 0) {
        if (!dataObj["timers"].is<JsonArray>()) {
            writeError(400);
            return;
        }
        timerScheduler_.clearAllTimers();
        uint8_t nextId = 1;
        for (JsonVariant item : dataObj["timers"].as<JsonArray>()) {
            domain::ScheduledTimer t{};
            t.id = item["id"] | nextId++;
            t.enabled = (item["enabled"] | 1) != 0;
            const char* tp = item["type"] | "once";
            if (std::strcmp(tp, "repeat") == 0) t.type = domain::TimerType::Repeat;
            else if (std::strcmp(tp, "duration") == 0) t.type = domain::TimerType::Duration;
            else t.type = domain::TimerType::Once;

            const char* doSw = item["do"]["switch"] | "off";
            t.targetAction = (std::strcmp(doSw, "on") == 0);
            const char* startSw = item["startDo"]["switch"] | "on";
            const char* endSw = item["endDo"]["switch"] | "off";
            t.startAction = (std::strcmp(startSw, "on") == 0);
            t.endAction = (std::strcmp(endSw, "on") == 0);
            t.targetEpochSec = item["at"] | 0U;
            t.weekdaysMask = item["weekdays"] | 0x7FU;
            t.hourUtc = item["hour"] | 0U;
            t.minuteUtc = item["minute"] | 0U;
            t.cycleStartEpochSec = item["startAt"] | 0U;
            t.cyclePeriodSec = item["periodSec"] | 0U;
            t.cycleOnDurationSec = item["onDurationSec"] | 0U;
            timerScheduler_.upsertTimer(t);
        }
        writeError(0);
        return;
    }

    if (std::strcmp(uri, "/zeroconf/ota_unlock") == 0) {
        otaUnlocked_ = true;
        writeError(0);
        return;
    }

    writeError(404);
}

} // namespace relay::infra

