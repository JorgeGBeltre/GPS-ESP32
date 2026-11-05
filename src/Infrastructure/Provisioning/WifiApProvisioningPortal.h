#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <cstdlib>
#include <cstring>
#include "Domain/Interfaces/IButtonHandler.h"
#include "Domain/Interfaces/IIndicator.h"
#include "Infrastructure/Config/config.h"
#include "Infrastructure/Config/nvs_config.h"

namespace relay::infra {

class WifiApProvisioningPortal {
public:
    static bool startPortal(uint32_t timeoutSec = 180,
                            domain::IIndicator* indicator = nullptr,
                            domain::IButtonHandler* button = nullptr) {
        loadConfig();

        WiFiManager wifiManager;
        uint64_t chipID = ESP.getEfuseMac();
        char chipHex[16];
        snprintf(chipHex, sizeof(chipHex), "%04X%08X", (uint16_t)(chipID >> 32), (uint32_t)chipID);
        String apSSID = String("GPS-") + (config.device_id.length() > 0 ? config.device_id : String(chipHex));
        const char* apPassword = "12345678";

        Serial.printf("[PROVISION] Iniciando Compatible SoftAP: %s (Clave: %s)\n",
                      apSSID.c_str(), apPassword);

        if (indicator) {
            indicator->show(domain::IndicatorStatus::CompatibleAp);
        }

        WiFi.setAutoReconnect(false);
        WiFi.disconnect(false, false);
        delay(100);

        char portBuf[12];
        snprintf(portBuf, sizeof(portBuf), "%d", config.mqtt_port > 0 ? config.mqtt_port : 8883);

        WiFiManagerParameter customApn("apn", "APN Celular", config.apn.c_str(), 64);
        WiFiManagerParameter customApnUser("apn_user", "Usuario APN (Opcional)", config.apn_user.c_str(), 48);
        WiFiManagerParameter customApnPass("apn_pass", "Clave APN (Opcional)", config.apn_pass.c_str(), 48);
        WiFiManagerParameter customPhone("phone", "Teléfono Emergencia SMS", config.emergency_phone.c_str(), 24);
        WiFiManagerParameter customMqttServer("server", "Servidor MQTT 5.0", config.mqtt_broker.c_str(), 64);
        WiFiManagerParameter customMqttPort("port", "Puerto MQTT (8883 TLS)", portBuf, 6);
        WiFiManagerParameter customMqttUser("apikey", "Usuario / API Key", config.mqtt_user.c_str(), 64);
        WiFiManagerParameter customMqttPass("mqtt_pass", "Clave MQTT (Opcional)", config.mqtt_pass.c_str(), 64);
        WiFiManagerParameter customBindToken("token", "Bind Token (Opcional)", config.bind_token.c_str(), 64);

        wifiManager.addParameter(&customApn);
        wifiManager.addParameter(&customApnUser);
        wifiManager.addParameter(&customApnPass);
        wifiManager.addParameter(&customPhone);
        wifiManager.addParameter(&customMqttServer);
        wifiManager.addParameter(&customMqttPort);
        wifiManager.addParameter(&customMqttUser);
        wifiManager.addParameter(&customMqttPass);
        wifiManager.addParameter(&customBindToken);

        bool portalSaved = false;
        bool jsonApSaved = false;
        wifiManager.setSaveConfigCallback([&]() {
            portalSaved = true;
        });

        // Endpoints JSON de Compatible Pairing Mode (GET /device, GET /api/info, POST /ap, POST /api/wifi)
        wifiManager.setWebServerCallback([&]() {
            if (!wifiManager.server) return;

            auto sendDeviceInfo = [&]() {
                const bool wifiConn = (WiFi.status() == WL_CONNECTED);
                const bool cellConn = isCellularDataConnected();
                const bool autoApn  = isCellularApnAutoDetected();
                const char* credMode = resolveDeviceCredentialMode(wifiConn);
                const bool adoptReady = (std::strcmp(credMode, "adopt_only") == 0);
                const String effectiveApn = config.apn.length() > 0 ? config.apn : String(getCellularResolvedApn());

                StaticJsonDocument<640> doc;
                doc["deviceid"]              = config.device_id.length() > 0 ? config.device_id.c_str() : chipHex;
                doc["device_id"]             = config.device_id.length() > 0 ? config.device_id.c_str() : chipHex;
                doc["chipid"]                = chipHex;
                doc["apikey"]                = config.device_key.c_str();
                doc["device_key"]            = config.device_key.c_str();
                doc["device_family"]         = "gps_esp32";
                doc["device_code"]           = 2;
                doc["device_type"]           = "GPS-Relay";
                doc["model"]                 = "VESIS-GPS-ESP32";
                doc["channel_count"]         = 1;
                doc["version"]               = FIRMWARE_VERSION;
                doc["config_state"]          = resolveDeviceConfigState(wifiConn, cellConn, false);
                doc["credential_mode"]       = credMode;
                doc["requires_wifi"]         = (std::strcmp(credMode, "wifi") == 0 || std::strcmp(credMode, "wifi_and_apn") == 0);
                doc["requires_apn"]          = (std::strcmp(credMode, "apn") == 0 || std::strcmp(credMode, "wifi_and_apn") == 0);
                doc["supports_wifi"]         = true;
                doc["supports_apn"]          = true;
                doc["sim_present"]           = isCellularSimPresent();
                doc["apn_auto_detected"]     = autoApn;
                doc["cellular_connected"]    = cellConn;
                doc["adopt_ready"]           = adoptReady;
                doc["apn"]                   = effectiveApn;
                doc["emergency_phone"]       = config.emergency_phone;
                doc["account_bound"]         = isAccountBound();
                doc["has_wifi_credentials"]  = hasValidWifiCredentials();
                doc["has_broker_credentials"]= hasValidBrokerCredentials();
                doc["accept"]                = "post";
                String out;
                serializeJson(doc, out);
                wifiManager.server->sendHeader("Connection", "close");
                wifiManager.server->send(200, "application/json", out);
            };

            auto handleApPost = [&]() {
                String body = wifiManager.server->arg("plain");
                StaticJsonDocument<640> doc;
                if (deserializeJson(doc, body) != DeserializationError::Ok) {
                    wifiManager.server->sendHeader("Connection", "close");
                    wifiManager.server->send(400, "application/json", "{\"error\":400}");
                    return;
                }

                String nextSsid = doc["ssid"] | (doc["wifi_ssid"] | "");
                String nextApn  = doc["apn"] | "";
                const bool hasBrokerBundle =
                    doc.containsKey("server") || doc.containsKey("serverName") ||
                    doc.containsKey("mqtt_broker") || doc.containsKey("apikey") ||
                    doc.containsKey("mqtt_user") || doc.containsKey("bind_token") ||
                    doc.containsKey("token");
                const bool canAdoptWithoutCreds =
                    hasBrokerBundle &&
                    (isCellularApnAutoDetected() || isCellularDataConnected() ||
                     config.apn.length() > 0 || hasValidWifiCredentials());

                if (nextSsid.length() == 0 && nextApn.length() == 0 && !canAdoptWithoutCreds) {
                    wifiManager.server->sendHeader("Connection", "close");
                    wifiManager.server->send(400, "application/json", "{\"error\":400}");
                    return;
                }

                if (nextSsid.length() > 0) {
                    config.wifi_ssid = nextSsid;
                    if (doc.containsKey("password")) config.wifi_pass = doc["password"].as<String>();
                    else if (doc.containsKey("pass")) config.wifi_pass = doc["pass"].as<String>();
                    else if (doc.containsKey("wifi_pass")) config.wifi_pass = doc["wifi_pass"].as<String>();
                }

                if (nextApn.length() > 0) {
                    config.apn = nextApn;
                }
                if (doc.containsKey("apn_user")) config.apn_user = doc["apn_user"].as<String>();
                if (doc.containsKey("apn_pass")) config.apn_pass = doc["apn_pass"].as<String>();

                if (doc.containsKey("emergency_phone")) {
                    String ph = doc["emergency_phone"].as<String>();
                    if (ph.length() > 0) config.emergency_phone = ph;
                } else if (doc.containsKey("phone")) {
                    String ph = doc["phone"].as<String>();
                    if (ph.length() > 0) config.emergency_phone = ph;
                }

                if (doc.containsKey("sms_enabled")) config.sms_enabled = doc["sms_enabled"].as<bool>();
                if (doc.containsKey("sensitivity")) config.sensitivity = doc["sensitivity"].as<int>();
                if (doc.containsKey("rollover_angle")) config.rollover_angle = doc["rollover_angle"].as<int>();

                if (doc.containsKey("serverName")) config.mqtt_broker = doc["serverName"].as<String>();
                else if (doc.containsKey("mqtt_broker")) config.mqtt_broker = doc["mqtt_broker"].as<String>();
                else if (doc.containsKey("server")) config.mqtt_broker = doc["server"].as<String>();

                if (doc.containsKey("port")) config.mqtt_port = doc["port"].as<int>();
                else if (doc.containsKey("mqtt_port")) config.mqtt_port = doc["mqtt_port"].as<int>();

                if (doc.containsKey("apikey")) config.mqtt_user = doc["apikey"].as<String>();
                else if (doc.containsKey("mqtt_user")) config.mqtt_user = doc["mqtt_user"].as<String>();
                else if (doc.containsKey("account")) config.mqtt_user = doc["account"].as<String>();

                if (doc.containsKey("mqtt_pass")) config.mqtt_pass = doc["mqtt_pass"].as<String>();
                if (doc.containsKey("devicekey")) config.device_key = doc["devicekey"].as<String>();
                if (doc.containsKey("bind_token")) config.bind_token = doc["bind_token"].as<String>();
                else if (doc.containsKey("token")) config.bind_token = doc["token"].as<String>();

                if (hasBrokerBundle) {
                    config.account_bound = true;
                }

                if (doc.containsKey("startup")) {
                    String st = doc["startup"].as<String>();
                    st.toLowerCase();
                    config.startup_mode = (st == "on") ? 1 : ((st == "stay") ? 2 : 0);
                }
                if (doc.containsKey("pulse")) {
                    String pl = doc["pulse"].as<String>();
                    pl.toLowerCase();
                    config.pulse_enabled = (pl == "on" || pl == "1" || pl == "true");
                }
                if (doc.containsKey("pulseWidth")) {
                    uint32_t pw = doc["pulseWidth"].as<uint32_t>();
                    if (pw >= 500 && pw <= 36000000 && (pw % 500) == 0) {
                        config.pulse_width_ms = pw;
                    }
                }
                if (doc.containsKey("sledOnline")) {
                    String sl = doc["sledOnline"].as<String>();
                    sl.toLowerCase();
                    config.sled_online = (sl != "off" && sl != "0" && sl != "false");
                }

                wifiManager.server->sendHeader("Connection", "close");
                wifiManager.server->send(200, "application/json", "{\"error\":0}");
                jsonApSaved = true;
                portalSaved = true;
            };

            wifiManager.server->on("/device", HTTP_GET, sendDeviceInfo);
            wifiManager.server->on("/api/info", HTTP_GET, sendDeviceInfo);
            wifiManager.server->on("/ap", HTTP_POST, handleApPost);
            wifiManager.server->on("/api/wifi", HTTP_POST, handleApPost);
        });

        wifiManager.setConfigPortalBlocking(false);
        wifiManager.setConfigPortalTimeout(timeoutSec);

        if (button) {
            unsigned long releaseStart = millis();
            while (digitalRead(button->getPin()) == LOW && (millis() - releaseStart) < 3000) {
                if (indicator) indicator->update();
                delay(20);
            }
            button->init();
        }

        wifiManager.startConfigPortal(apSSID.c_str(), apPassword);

        const unsigned long startMs = millis();
        const unsigned long timeoutMs = timeoutSec * 1000UL;
        bool cancelledByButton = false;

        while ((millis() - startMs) < timeoutMs) {
            if (indicator) {
                indicator->update();
            }

            if (wifiManager.process() || portalSaved) {
                break;
            }

            if (button) {
                auto ev = button->update();
                if (ev == domain::ButtonEvent::ShortPress) {
                    Serial.println("[PROVISION] Pulsación corta detectada -> Saliendo de portal SoftAP GPS.");
                    cancelledByButton = true;
                    wifiManager.stopConfigPortal();
                    break;
                }
            }

            delay(20);
        }

        if (!cancelledByButton && (portalSaved || WiFi.status() == WL_CONNECTED || WiFi.SSID().length() > 0)) {
            if (!jsonApSaved) {
                if (WiFi.SSID().length() > 0) {
                    config.wifi_ssid = WiFi.SSID();
                }
                if (WiFi.psk().length() > 0) {
                    config.wifi_pass = WiFi.psk();
                }
                if (strlen(customApn.getValue()) > 0) {
                    config.apn = customApn.getValue();
                }
                if (strlen(customApnUser.getValue()) > 0) {
                    config.apn_user = customApnUser.getValue();
                }
                if (strlen(customApnPass.getValue()) > 0) {
                    config.apn_pass = customApnPass.getValue();
                }
                if (strlen(customPhone.getValue()) > 0) {
                    config.emergency_phone = customPhone.getValue();
                }
                if (strlen(customMqttServer.getValue()) > 0) {
                    config.mqtt_broker = customMqttServer.getValue();
                }
                if (strlen(customMqttPort.getValue()) > 0) {
                    int p = atoi(customMqttPort.getValue());
                    if (p > 0 && p <= 65535) config.mqtt_port = p;
                }
                if (strlen(customMqttUser.getValue()) > 0) {
                    config.mqtt_user = customMqttUser.getValue();
                }
                if (strlen(customMqttPass.getValue()) > 0) {
                    config.mqtt_pass = customMqttPass.getValue();
                }
                if (strlen(customBindToken.getValue()) > 0) {
                    config.bind_token = customBindToken.getValue();
                }
            }

            saveConfig();
            Serial.printf("[PROVISION] Credenciales guardadas en NVS (SSID: %s, APN: %s). Reiniciando...\n",
                          config.wifi_ssid.c_str(), config.apn.c_str());
            delay(200);
            ESP.restart();
            return true;
        }

        Serial.println("[PROVISION] Portal SoftAP finalizado sin cambios.");
        WiFi.mode(WIFI_STA);
        return false;
    }
};

} // namespace relay::infra
