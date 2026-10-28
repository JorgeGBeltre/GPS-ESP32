#include "Infrastructure/Net/NetworkClient.h"
#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

namespace relay::infra {

static WiFiClient g_plainClient;
static WiFiClientSecure g_secureClient;
static bool g_useTls = false;

WiFiClient& getActiveNetworkClient() {
    return g_useTls ? static_cast<WiFiClient&>(g_secureClient) : g_plainClient;
}

void configureNetworkClient(const char* rootCa) {
    if (rootCa && strstr(rootCa, "YOUR_SSL_CERTIFICATE_HERE") == nullptr && strlen(rootCa) > 50) {
        g_useTls = true;
        g_secureClient.setCACert(rootCa);
        Serial.println("[NET] TLS Root CA configurado correctamente");
    } else {
        g_useTls = false;
        Serial.println("[NET] Modo TCP en claro (puerto 1883 / desarrollo)");
    }
}

}
