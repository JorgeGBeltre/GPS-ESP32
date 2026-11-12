#include "Infrastructure/Messaging/mqtt_client.h"
#include "Infrastructure/Config/nvs_config.h"
#include "Infrastructure/Net/NetworkClient.h"
#include "Infrastructure/Ota/ota_manager.h"
#include <ArduinoJson.h>

ESP32_MQTTv5 mqtt;

namespace relay::infra {

static String g_server;
static uint16_t g_port = 8883;
static String g_clientId;
static String g_apiKey;
static String g_firmwareVersion;

static std::function<void(const String&, const String&)> g_messageSink;
static std::function<void()> g_onConnectCallback;
static std::function<void()> g_onUnlinkVerifiedCallback;

static void tryVerifyReasonStringUnlink(const String& reason) {
    int pos = reason.indexOf("ACCOUNT_UNLINKED:");
    if (pos == -1) return;
    String proof = reason.substring(pos + 17);
    proof.trim();
    if (proof.length() > 32) {
        proof = proof.substring(0, 32);
    }
    if (verifyUnlinkProof(config.device_key.c_str(), g_clientId.c_str(), proof.c_str())) {
        Serial.println("[MQTT5] Desvinculación de cuenta verificada criptográficamente (CONNACK/DISCONNECT).");
        if (g_onUnlinkVerifiedCallback) {
            g_onUnlinkVerifiedCallback();
        }
    } else {
        Serial.println("[MQTT5] AVISO: Prueba ACCOUNT_UNLINKED inválida rechazada.");
    }
}

void setMqttMessageSink(std::function<void(const String&, const String&)> sink) {
    g_messageSink = std::move(sink);
}

void setMqttConnectCallback(std::function<void()> cb) {
    g_onConnectCallback = std::move(cb);
}

void setMqttUnlinkVerifiedCallback(std::function<void()> cb) {
    g_onUnlinkVerifiedCallback = std::move(cb);
}

void initMQTT(const char* server, uint16_t port,
              const char* clientId, const char* apiKey,
              const char* rootCa, const char* firmwareVersion) {
    g_server = server;
    g_port = port;
    g_clientId = clientId;
    g_apiKey = apiKey;
    g_firmwareVersion = firmwareVersion;

    configureNetworkClient(rootCa, port);

    Serial.printf("[MQTT5] Inicializando broker %s:%d con cliente %s...\n",
                  server, port, clientId);

    mqtt.begin(getActiveNetworkClient(), server, port);
    mqtt.setBufferSize(4096);
    mqtt.setKeepAlive(60);
    mqtt.setCleanStart(false);
    mqtt.setAutoReconnect(false); // Reconexión exponencial centralizada en RelayFirmwareController
    mqtt.setSessionExpiry(3600);

    MQTTProperties willProps;
    willProps.setContentType("application/json");
    willProps.setWillDelay(5);
    String willTopic = "status/" + g_clientId;
    String willPayload = "{\"status\":\"offline\",\"device_id\":\"" + g_clientId + "\"}";
    mqtt.setWill(willTopic, willPayload, 1, true, &willProps);

    mqtt.onConnect([]() {
        Serial.println("[MQTT5] ¡Conectado al broker exitosamente!");

        initOTA(g_clientId, g_firmwareVersion);
        mqtt.subscribe("devices/" + g_clientId + "/unlink/command", 1);

        MQTTProperties reconnectWillProps;
        reconnectWillProps.setContentType("application/json");
        reconnectWillProps.setWillDelay(5);
        mqtt.setWill("status/" + g_clientId,
                     "{\"status\":\"offline\",\"device_id\":\"" + g_clientId + "\"}",
                     1, true, &reconnectWillProps);

        if (g_onConnectCallback) {
            g_onConnectCallback();
        }
    });

    mqtt.onDisconnect([](MQTTReasonCode rc) {
        Serial.printf("[MQTT5] Desconectado (Código: 0x%02X)\n", (uint8_t)rc);
        if ((uint8_t)rc == 0x87) {
            tryVerifyReasonStringUnlink(mqtt.lastReasonString());
        }
    });

    mqtt.onError([](MQTTReasonCode rc, const String& msg) {
        Serial.printf("[MQTT5] Error 0x%02X: %s\n", (uint8_t)rc, msg.c_str());
        if ((uint8_t)rc == 0x87) {
            tryVerifyReasonStringUnlink(msg);
            tryVerifyReasonStringUnlink(mqtt.lastReasonString());
        }
    });

    mqtt.onMessage([](const MQTTMessage& msg) {
        Serial.printf("[MQTT5] Mensaje recibido en [%s]\n", msg.topic.c_str());
        if (msg.topic == ("devices/" + g_clientId + "/unlink/command")) {
            StaticJsonDocument<320> doc;
            if (deserializeJson(doc, msg.payload) == DeserializationError::Ok) {
                const char* proof = doc["unlink_proof"] | "";
                if (verifyUnlinkProof(config.device_key.c_str(), g_clientId.c_str(), proof)) {
                    Serial.println("[MQTT5] Comando de desvinculación verificado. Restaurando a Quick Pairing...");
                    if (g_onUnlinkVerifiedCallback) {
                        g_onUnlinkVerifiedCallback();
                    }
                } else {
                    Serial.println("[MQTT5] AVISO: Comando unlink rechazado por unlink_proof inválido.");
                }
            }
            return;
        }
        if (g_messageSink) {
            g_messageSink(msg.topic, msg.payload);
        }
    });
}

bool connectMQTT() {
    if (mqtt.connected()) return true;
    if (!hasValidBrokerCredentials()) return false;

    mqtt.begin(getActiveNetworkClient(), g_server.c_str(), g_port);
    Serial.printf("[MQTT5] Conectando a %s:%d como %s...\n",
                  g_server.c_str(), g_port, g_clientId.c_str());

    return mqtt.connect(g_clientId, g_apiKey, "");
}

void disconnectMQTT() {
    mqtt.disconnect();
}

void mqttLoop() {
    mqtt.loop();
}

bool isMQTTConnected() {
    return mqtt.connected();
}

bool publishMQTT(const String& topic, const String& payload, uint8_t qos, bool retain) {
    if (!mqtt.connected()) return false;
    return mqtt.publish(topic, payload, qos, retain);
}

bool subscribeMQTT(const String& topic, uint8_t qos) {
    if (!mqtt.connected()) return false;
    return mqtt.subscribe(topic, qos);
}

}
