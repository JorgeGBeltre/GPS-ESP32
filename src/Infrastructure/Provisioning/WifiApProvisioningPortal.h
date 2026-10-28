#pragma once

#include <Arduino.h>
#include <WiFiManager.h>
#include "Infrastructure/Config/nvs_config.h"

namespace relay::infra {

class WifiApProvisioningPortal {
public:
  static void startPortal(uint32_t timeoutSec = 180) {
    WiFiManager wifiManager;
    uint64_t chipID = ESP.getEfuseMac();
    String apSSID = "GPS-" + String((uint16_t)(chipID >> 32), DEC) +
                    String((uint32_t)chipID, DEC);
    const char *apPassword = "12345678";

    WiFiManagerParameter custom_apn("apn", "APN Celular", config.apn.c_str(), 64);
    WiFiManagerParameter custom_apn_user("apn_user", "Usuario APN", config.apn_user.c_str(), 32);
    WiFiManagerParameter custom_apn_pass("apn_pass", "Contrasena APN", config.apn_pass.c_str(), 32, "type=\"password\"");

    wifiManager.addParameter(&custom_apn);
    wifiManager.addParameter(&custom_apn_user);
    wifiManager.addParameter(&custom_apn_pass);

    Serial.printf("[PROVISION] Iniciando Access Point SoftAP: %s (Clave: %s)\n",
                  apSSID.c_str(), apPassword);

    wifiManager.setConfigPortalTimeout(timeoutSec);
    if (wifiManager.startConfigPortal(apSSID.c_str(), apPassword)) {
      config.apn = custom_apn.getValue();
      config.apn_user = custom_apn_user.getValue();
      config.apn_pass = custom_apn_pass.getValue();
      saveConfig();
      Serial.println("[PROVISION] Parametros APN y WiFi guardados en NVS.");
    }

    Serial.println("[PROVISION] Portal finalizado. Reiniciando...");
    delay(100);
    ESP.restart();
  }
};

} // namespace relay::infra
