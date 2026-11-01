#pragma once

#include <cstdint>
#include <string>

namespace relay::domain {

/**
 * @brief Modo de arranque del relé tras un corte de energía:
 * - Off:  Arranca siempre apagado (por defecto).
 * - On:   Arranca siempre encendido.
 * - Stay: Restaura el último estado persistido en NVS (con escritura diferida de 150 ms).
 */
enum class StartupMode : uint8_t {
    Off  = 0,
    On   = 1,
    Stay = 2
};

inline const char* startupModeToString(StartupMode mode) {
    switch (mode) {
        case StartupMode::On:   return "on";
        case StartupMode::Stay: return "stay";
        case StartupMode::Off:
        default:                return "off";
    }
}

inline StartupMode startupModeFromString(const std::string& str) {
    if (str == "on" || str == "ON" || str == "1") return StartupMode::On;
    if (str == "stay" || str == "STAY" || str == "2") return StartupMode::Stay;
    return StartupMode::Off;
}

/**
 * @brief Entidad pura de dominio que contiene la configuración del dispositivo.
 * Desacoplada de frameworks o hardware para permitir compilación y pruebas en host.
 * Incluye campos para conectividad celular (APN), WiFi, MQTT, identidad de hardware,
 * OTA y comportamiento operativo (StartupMode, Inching/Pulse, SLED Online).
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
    std::string deviceKey;    // Clave privada de hardware / Device Key para autenticación mutua y cifrado LAN
    std::string bindToken;    // Token de registro / reclamo emitido por la app para enlazar a la cuenta
    std::string otaHmacKey;

    // Comportamiento operativo
    StartupMode startupMode    = StartupMode::Off;
    bool        lastRelayState = false;
    bool        pulseEnabled   = false;
    uint32_t    pulseWidthMs   = 500;
    bool        sledOnline     = true;

    bool        provisioned    = false;
};

} // namespace relay::domain
