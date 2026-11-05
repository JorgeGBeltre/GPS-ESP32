#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <cstddef>
#include <cstdint>
#include "Domain/Entities/ScheduledTimer.h"

struct AppConfig {
    String   apn;
    String   apn_user;
    String   apn_pass;
    String   emergency_phone = "+18090000000";
    bool     sms_enabled     = true;
    int      sensitivity     = 2000;
    int      rollover_angle  = 60;

    String   wifi_ssid;
    String   wifi_pass;
    String   mqtt_broker;
    int      mqtt_port      = 8883;
    String   mqtt_user;       // Usuario / API Key / Cuenta
    String   mqtt_pass;       // Clave o Token de sesión
    String   device_id;       // Identificador único (Chip ID / Device ID)
    String   device_key;      // Clave privada de hardware / Device Key de fábrica
    String   bind_token;      // Token de registro / reclamo emitido por la app móvil para enlazar la cuenta
    String   ota_hmac_key;    // Clave HMAC para OTA segura

    uint8_t  startup_mode     = 0;     // 0=Off, 1=On, 2=Stay
    bool     last_relay_state = false; // Último estado persistido para StartupMode::Stay
    bool     pulse_enabled    = false; // Modo pulso (Inching)
    uint32_t pulse_width_ms   = 500;   // Ancho de pulso en ms (500..36000000, paso 500)
    bool     sled_online      = true;  // Indicador LED de estado en línea (sledOnline)
    bool     account_bound    = false; // Vinculación activa a cuenta de plataforma
    String   last_wifi_error;          // Último error de conexión Wi-Fi
};

extern AppConfig config;

void initConfig(const char* defaultDeviceId = nullptr);
bool isProvisioned();
bool isAccountBound();
bool hasValidWifiCredentials();
bool hasValidCellularCredentials();
bool hasValidBrokerCredentials();
void computeUnlinkProofHex(const char* deviceKey, const char* chipId, char outHex[33]);
bool verifyUnlinkProof(const char* deviceKey, const char* chipId, const char* providedHex);
const char* resolveDeviceConfigState(bool wifiConnected, bool cellularConnected, bool mqttConnected);
const char* resolveDeviceCredentialMode(bool wifiConnected);

void setCellularProvisioningSnapshot(bool simPresent, bool apnAutoDetected, bool cellularConnected, const char* resolvedApn);
bool isCellularSimPresent();
bool isCellularApnAutoDetected();
bool isCellularDataConnected();
const char* getCellularResolvedApn();

void saveConfig();
void loadConfig();
void resetConfig();

bool saveLastRelayStateNvs(bool state);
bool saveStartupModeNvs(uint8_t mode);
bool savePulseConfigNvs(bool enabled, uint32_t widthMs);
bool saveSledOnlineNvs(bool enabled);
bool saveTimersNvs(const relay::domain::ScheduledTimer* timers, size_t count);
size_t loadTimersNvs(relay::domain::ScheduledTimer* timers, size_t maxCount);
