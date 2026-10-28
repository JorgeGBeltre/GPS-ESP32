#pragma once

#include <string>

namespace relay::domain {

/**
 * @brief Entidad pura de dominio que contiene la configuración del dispositivo.
 * Desacoplada de frameworks o hardware para permitir compilación y pruebas en host.
 * Incluye campos para conectividad celular (APN), WiFi, MQTT, hardware IDs y OTA.
 */
struct DeviceConfig {
    // Conectividad celular (APN)
    std::string apn;
    std::string apnUser;
    std::string apnPass;

    // Conectividad WiFi
    std::string wifiSsid;
    std::string wifiPass;

    // Broker MQTT
    std::string mqttBroker;
    int         mqttPort = 8883;
    std::string mqttUser;
    std::string mqttPass;

    // Identidad y vinculación
    std::string deviceId;
    std::string deviceKey;    // Clave privada de hardware / Device Key para autenticación mutua
    std::string bindToken;    // Token de registro / reclamo emitido por la app para enlazar a la cuenta
    std::string otaHmacKey;

    bool        provisioned = false;
};

} // namespace relay::domain
