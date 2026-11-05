#include "Infrastructure/Provisioning/ble_provisioning.h"
#include "Infrastructure/Config/nvs_config.h"
#include "Infrastructure/Config/config.h"
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <cstring>
#include <sstream>

#define SERVICE_UUID          "12345678-1234-1234-1234-123456789abc"
#define CHAR_APN_UUID         "aaaaaaaa-bbbb-cccc-dddd-000000000001"
#define CHAR_APNUSER_UUID     "aaaaaaaa-bbbb-cccc-dddd-000000000002"
#define CHAR_APNPASS_UUID     "aaaaaaaa-bbbb-cccc-dddd-000000000003"
#define CHAR_BROKER_UUID      "aaaaaaaa-bbbb-cccc-dddd-000000000004"
#define CHAR_PORT_UUID        "aaaaaaaa-bbbb-cccc-dddd-000000000005"
#define CHAR_MQTTUSER_UUID    "aaaaaaaa-bbbb-cccc-dddd-000000000006"
#define CHAR_MQTTPASS_UUID    "aaaaaaaa-bbbb-cccc-dddd-000000000007"
#define CHAR_DEVID_UUID       "aaaaaaaa-bbbb-cccc-dddd-000000000008"
#define CHAR_OTAKEY_UUID      "aaaaaaaa-bbbb-cccc-dddd-000000000011"
#define CHAR_WIFISSID_UUID    "aaaaaaaa-bbbb-cccc-dddd-000000000012"
#define CHAR_WIFIPASS_UUID    "aaaaaaaa-bbbb-cccc-dddd-000000000013"
#define CHAR_FRAME_JSON_UUID  "aaaaaaaa-bbbb-cccc-dddd-000000000014"
#define CHAR_DEVKEY_UUID      "aaaaaaaa-bbbb-cccc-dddd-000000000015"
#define CHAR_BINDTOKEN_UUID   "aaaaaaaa-bbbb-cccc-dddd-000000000016"
#define CHAR_SAVE_UUID        "aaaaaaaa-bbbb-cccc-dddd-000000000099"

static volatile bool _configSaved = false;
static volatile bool _bleStopping = false;
static std::string _rxFrameBuffer;
static unsigned long _lastRxChunkMs = 0;

static uint8_t computeAdvStateByte() {
    const bool wifiConn = (WiFi.status() == WL_CONNECTED);
    const bool cellConn = isCellularDataConnected();
    const bool anyConn  = wifiConn || cellConn;
    if (!hasValidWifiCredentials() && !hasValidCellularCredentials() && !anyConn) {
        return 0x00; // unconfigured
    }
    if (!hasValidBrokerCredentials() && !isAccountBound()) {
        return anyConn ? 0x04 : 0x01; // wifi_connected (or cellular ready) vs incomplete
    }
    if (anyConn) return 0x04;
    if (config.last_wifi_error.length() > 0) return 0x03;
    return 0x02;
}

static uint8_t computeAdvProvFlags() {
    const bool wifiConn = (WiFi.status() == WL_CONNECTED);
    const char* credMode = resolveDeviceCredentialMode(wifiConn);
    uint8_t credBits = 0x01U; // default: apn
    if (std::strcmp(credMode, "adopt_only") == 0) {
        credBits = 0x02U;
    } else if (std::strcmp(credMode, "wifi") == 0) {
        credBits = 0x00U;
    } else if (std::strcmp(credMode, "wifi_and_apn") == 0) {
        credBits = 0x03U;
    }

    uint8_t flags = credBits;
    if (isCellularApnAutoDetected()) flags |= 0x04U; // bit 2: apn_auto_detected
    if (isCellularDataConnected())   flags |= 0x08U; // bit 3: cellular_connected
    flags |= 0x10U;                                  // bit 4: supports_wifi
    if (isAccountBound())            flags |= 0x20U; // bit 5: account_bound
    return flags;
}

static void buildBleStateJson(String& out) {
    const bool wifiConn = (WiFi.status() == WL_CONNECTED);
    const bool cellConn = isCellularDataConnected();
    const bool autoApn  = isCellularApnAutoDetected();
    const char* credMode = resolveDeviceCredentialMode(wifiConn);
    const bool adoptReady = (std::strcmp(credMode, "adopt_only") == 0);
    const String effectiveApn = config.apn.length() > 0 ? config.apn : String(getCellularResolvedApn());

    StaticJsonDocument<768> idDoc;
    idDoc["device_family"]          = "gps_esp32";
    idDoc["device_code"]            = 2;
    idDoc["device_type"]            = "GPS-Relay";
    idDoc["model"]                  = "VESIS-GPS-ESP32";
    idDoc["channel_count"]          = 1;
    idDoc["device_id"]              = config.device_id;
    idDoc["chip_id"]                = config.device_id;
    idDoc["device_key"]             = config.device_key;
    idDoc["mac"]                    = WiFi.macAddress();
    idDoc["version"]                = FIRMWARE_VERSION;
    idDoc["config_state"]           = resolveDeviceConfigState(wifiConn, cellConn, false);
    idDoc["credential_mode"]        = credMode;
    idDoc["requires_wifi"]          = (std::strcmp(credMode, "wifi") == 0 || std::strcmp(credMode, "wifi_and_apn") == 0);
    idDoc["requires_apn"]           = (std::strcmp(credMode, "apn") == 0 || std::strcmp(credMode, "wifi_and_apn") == 0);
    idDoc["supports_wifi"]          = true;
    idDoc["supports_apn"]           = true;
    idDoc["sim_present"]            = isCellularSimPresent();
    idDoc["apn_auto_detected"]      = autoApn;
    idDoc["cellular_connected"]     = cellConn;
    idDoc["adopt_ready"]            = adoptReady;
    idDoc["apn"]                    = effectiveApn;
    idDoc["emergency_phone"]        = config.emergency_phone;
    idDoc["wifi_ssid"]              = config.wifi_ssid;
    idDoc["configured_ssid"]        = config.wifi_ssid;
    idDoc["wifi_connected"]         = wifiConn;
    idDoc["has_wifi_credentials"]   = hasValidWifiCredentials();
    idDoc["has_broker_credentials"] = hasValidBrokerCredentials();
    idDoc["account_bound"]          = isAccountBound();
    idDoc["provisioning_mode"]      = true;
    idDoc["server"]                 = config.mqtt_broker;
    idDoc["port"]                   = config.mqtt_port;
    idDoc["status"]                 = _configSaved ? "configured" : "ready_to_claim";
    JsonArray methods = idDoc.createNestedArray("methods");
    methods.add("ble");
    methods.add("softap");
    methods.add("smartconfig");
    methods.add("lan");

    out = "";
    serializeJson(idDoc, out);
}

static bool isCompleteJsonObject(const std::string& s) {
    size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || s[first] != '{') return false;
    int depth = 0;
    bool inStr = false;
    bool esc = false;
    for (size_t i = first; i < s.size(); ++i) {
        char ch = s[i];
        if (inStr) {
            if (esc) esc = false;
            else if (ch == '\\') esc = true;
            else if (ch == '"') inStr = false;
        } else {
            if (ch == '"') inStr = true;
            else if (ch == '{') ++depth;
            else if (ch == '}') {
                --depth;
                if (depth == 0) return true;
            }
        }
    }
    return false;
}

static bool parseJsonCredentials(const std::string& raw) {
    size_t first = raw.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || raw[first] != '{') return false;

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, raw.substr(first));
    if (error) {
        Serial.printf("[BLE] Error parseando trama JSON: %s\n", error.c_str());
        return false;
    }

    if (doc.containsKey("apn")) {
        String nextApn = doc["apn"].as<String>();
        if (nextApn.length() > 0) config.apn = nextApn;
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

    String nextSsid;
    bool hasSsidKey = false;
    if (doc.containsKey("wifi_ssid")) { nextSsid = doc["wifi_ssid"].as<String>(); hasSsidKey = true; }
    else if (doc.containsKey("ssid")) { nextSsid = doc["ssid"].as<String>(); hasSsidKey = true; }
    if (hasSsidKey && nextSsid.length() > 0) {
        config.wifi_ssid = nextSsid;
    }

    if (doc.containsKey("wifi_pass")) {
        String p = doc["wifi_pass"].as<String>();
        if (p.length() > 0 || nextSsid.length() > 0) config.wifi_pass = p;
    } else if (doc.containsKey("pass")) {
        String p = doc["pass"].as<String>();
        if (p.length() > 0 || nextSsid.length() > 0) config.wifi_pass = p;
    } else if (doc.containsKey("password")) {
        String p = doc["password"].as<String>();
        if (p.length() > 0 || nextSsid.length() > 0) config.wifi_pass = p;
    }

    if (doc.containsKey("mqtt_broker")) config.mqtt_broker = doc["mqtt_broker"].as<String>();
    else if (doc.containsKey("broker")) config.mqtt_broker = doc["broker"].as<String>();
    else if (doc.containsKey("server")) config.mqtt_broker = doc["server"].as<String>();

    if (doc.containsKey("mqtt_port")) config.mqtt_port = doc["mqtt_port"].as<int>();
    else if (doc.containsKey("port")) config.mqtt_port = doc["port"].as<int>();

    if (doc.containsKey("account")) config.mqtt_user = doc["account"].as<String>();
    else if (doc.containsKey("user")) config.mqtt_user = doc["user"].as<String>();
    else if (doc.containsKey("user_id")) config.mqtt_user = doc["user_id"].as<String>();
    else if (doc.containsKey("account_id")) config.mqtt_user = doc["account_id"].as<String>();
    else if (doc.containsKey("email")) config.mqtt_user = doc["email"].as<String>();
    else if (doc.containsKey("mqtt_user")) config.mqtt_user = doc["mqtt_user"].as<String>();
    else if (doc.containsKey("api_key")) config.mqtt_user = doc["api_key"].as<String>();

    if (doc.containsKey("mqtt_pass")) config.mqtt_pass = doc["mqtt_pass"].as<String>();
    else if (doc.containsKey("pass_mqtt")) config.mqtt_pass = doc["pass_mqtt"].as<String>();

    if (doc.containsKey("device_id")) config.device_id = doc["device_id"].as<String>();
    else if (doc.containsKey("id")) config.device_id = doc["id"].as<String>();
    else if (doc.containsKey("deviceid")) config.device_id = doc["deviceid"].as<String>();

    if (doc.containsKey("device_key")) config.device_key = doc["device_key"].as<String>();
    else if (doc.containsKey("devicekey")) config.device_key = doc["devicekey"].as<String>();
    else if (doc.containsKey("apikey") && !doc.containsKey("mqtt_user") && !doc.containsKey("api_key")) {
        String ak = doc["apikey"].as<String>();
        if (ak.startsWith("iotk_") || ak.startsWith("dvk_")) {
            config.mqtt_user = ak;
        } else {
            config.device_key = ak;
        }
    }

    if (doc.containsKey("bind_token")) config.bind_token = doc["bind_token"].as<String>();
    else if (doc.containsKey("token")) config.bind_token = doc["token"].as<String>();
    else if (doc.containsKey("claim_token")) config.bind_token = doc["claim_token"].as<String>();
    else if (doc.containsKey("account_token")) config.bind_token = doc["account_token"].as<String>();

    if (doc.containsKey("ota_hmac_key")) config.ota_hmac_key = doc["ota_hmac_key"].as<String>();
    else if (doc.containsKey("ota_key")) config.ota_hmac_key = doc["ota_key"].as<String>();

    if (doc.containsKey("startup")) {
        String st = doc["startup"].as<String>();
        st.toLowerCase();
        config.startup_mode = (st == "on") ? 1 : (st == "stay" ? 2 : 0);
    }
    if (doc.containsKey("pulse")) {
        String p = doc["pulse"].as<String>();
        p.toLowerCase();
        config.pulse_enabled = (p == "on" || p == "1" || p == "true");
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

    if (hasValidBrokerCredentials() || config.bind_token.length() > 0) {
        config.account_bound = true;
    }

    Serial.println("[BLE] Trama JSON de aprovisionamiento procesada con éxito");
    return true;
}

static bool parseKeyValueFrame(const std::string& raw) {
    auto trim = [](const std::string& s) -> std::string {
        size_t start = s.find_first_not_of(" \t\r\n\"'");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n\"'");
        return s.substr(start, end - start + 1);
    };

    bool matched = false;
    std::string cur = raw;
    for (char& c : cur) {
        if (c == '&' || c == '\n' || c == '\r') c = ';';
    }

    std::stringstream ss(cur);
    std::string token;
    while (std::getline(ss, token, ';')) {
        if (token.empty()) continue;
        size_t sep = token.find('=');
        if (sep == std::string::npos) sep = token.find(':');
        if (sep == std::string::npos) continue;

        std::string k = trim(token.substr(0, sep));
        std::string v = trim(token.substr(sep + 1));
        for (char& ch : k) ch = tolower((unsigned char)ch);

        if (k == "apn" && !v.empty()) { config.apn = v.c_str(); matched = true; }
        else if (k == "apn_user") { config.apn_user = v.c_str(); matched = true; }
        else if (k == "apn_pass") { config.apn_pass = v.c_str(); matched = true; }
        else if ((k == "emergency_phone" || k == "phone") && !v.empty()) { config.emergency_phone = v.c_str(); matched = true; }
        else if (k == "sensitivity") { config.sensitivity = atoi(v.c_str()); matched = true; }
        else if (k == "rollover_angle") { config.rollover_angle = atoi(v.c_str()); matched = true; }
        else if ((k == "wifi_ssid" || k == "ssid") && !v.empty()) { config.wifi_ssid = v.c_str(); matched = true; }
        else if (k == "wifi_pass" || k == "pass" || k == "password") { config.wifi_pass = v.c_str(); matched = true; }
        else if (k == "mqtt_broker" || k == "broker" || k == "server") { config.mqtt_broker = v.c_str(); matched = true; }
        else if (k == "mqtt_port" || k == "port") { config.mqtt_port = atoi(v.c_str()); matched = true; }
        else if (k == "account" || k == "user" || k == "user_id" || k == "account_id" || k == "email" || k == "mqtt_user" || k == "api_key") { config.mqtt_user = v.c_str(); matched = true; }
        else if (k == "mqtt_pass" || k == "pass_mqtt") { config.mqtt_pass = v.c_str(); matched = true; }
        else if (k == "device_id" || k == "id" || k == "deviceid") { config.device_id = v.c_str(); matched = true; }
        else if (k == "device_key" || k == "devicekey" || k == "apikey") { config.device_key = v.c_str(); matched = true; }
        else if (k == "bind_token" || k == "token" || k == "claim_token" || k == "account_token") { config.bind_token = v.c_str(); matched = true; }
        else if (k == "ota_hmac_key" || k == "ota_key") { config.ota_hmac_key = v.c_str(); matched = true; }
    }

    if (matched && (hasValidBrokerCredentials() || config.bind_token.length() > 0)) {
        config.account_bound = true;
    }
    if (matched) {
        Serial.println("[BLE] Trama Clave=Valor de aprovisionamiento procesada con éxito");
    }
    return matched;
}

static bool parseFrame(const std::string& raw) {
    if (raw.empty()) return false;
    if (parseJsonCredentials(raw)) return true;
    if (parseKeyValueFrame(raw)) return true;
    return false;
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        (void)pServer;
        Serial.println("[BLE] App móvil conectada al dispositivo.");
    }
    void onDisconnect(NimBLEServer* pServer) override {
        (void)pServer;
        Serial.println("[BLE] App móvil desconectada.");
        if (!_bleStopping && !_configSaved) {
            NimBLEDevice::startAdvertising();
        }
    }
};

class StringWriteCallback : public NimBLECharacteristicCallbacks {
public:
    String* _target;
    bool _secret;
    explicit StringWriteCallback(String* target, bool secret = false) : _target(target), _secret(secret) {}
    void onRead(NimBLECharacteristic* c) override {
        c->setValue(_secret ? "" : _target->c_str());
    }
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        *_target = val.c_str();
        c->setValue(_secret ? "" : val.c_str());
        if (!_secret) {
            Serial.printf("[BLE] %s = %s\n", c->getUUID().toString().c_str(), _target->c_str());
        } else {
            Serial.printf("[BLE] %s actualizado (%u bytes)\n", c->getUUID().toString().c_str(), (unsigned int)val.size());
        }
    }
};

class PortWriteCallback : public NimBLECharacteristicCallbacks {
public:
    int* _target;
    explicit PortWriteCallback(int* target) : _target(target) {}
    void onRead(NimBLECharacteristic* c) override {
        c->setValue(String(*_target).c_str());
    }
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        if (val.size() == 2 && (!isdigit((unsigned char)val[0]) || !isdigit((unsigned char)val[1]))) {
            uint16_t p = (uint8_t)val[0] | ((uint8_t)val[1] << 8);
            *_target = p;
        } else if (!val.empty()) {
            *_target = atoi(val.c_str());
        }
        c->setValue(String(*_target).c_str());
        Serial.printf("[BLE] Puerto MQTT/Servidor = %d\n", *_target);
    }
};

class DevIdCallback : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c) override {
        c->setValue(config.device_id.c_str());
    }
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        if (!val.empty()) {
            config.device_id = val.c_str();
            c->setValue(config.device_id.c_str());
        }
    }
};

class DevKeyCallback : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c) override {
        c->setValue(config.device_key.c_str());
    }
};

class BindTokenCallback : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c) override {
        c->setValue("");
    }
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        config.bind_token = val.c_str();
        c->setValue("");
        Serial.printf("[BLE] Token de enlace recibido (%u bytes)\n", (unsigned int)val.size());
    }
};

class SaveCallback : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c) override {
        c->setValue(_configSaved ? "OK" : "READY");
    }
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        if (!val.empty()) {
            parseFrame(val);
        }
        saveConfig();
        _configSaved = true;
        Serial.println("[BLE] Configuración y enlace guardados en NVS. Enviando ACK 'OK'...");
        c->setValue("OK");
        c->notify();
    }
};

class FrameCallback : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c) override {
        String json;
        buildBleStateJson(json);
        c->setValue(json.c_str());
        Serial.printf("[BLE] Reporte de identidad entregado a la app (%u bytes)\n", json.length());
    }

    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        if (val.empty()) return;

        unsigned long now = millis();
        if (!_rxFrameBuffer.empty() && (now - _lastRxChunkMs > 4000 || _rxFrameBuffer.size() + val.size() > 2048)) {
            _rxFrameBuffer.clear();
        }
        _lastRxChunkMs = now;

        size_t firstNonWs = val.find_first_not_of(" \t\r\n");
        bool startsJson = (firstNonWs != std::string::npos && val[firstNonWs] == '{');

        if (_rxFrameBuffer.empty() && startsJson && !isCompleteJsonObject(val)) {
            _rxFrameBuffer = val;
            return;
        }

        if (!_rxFrameBuffer.empty()) {
            _rxFrameBuffer += val;
            if (!isCompleteJsonObject(_rxFrameBuffer)) {
                return;
            }
            val = _rxFrameBuffer;
            _rxFrameBuffer.clear();
        }

        if (parseFrame(val)) {
            saveConfig();
            _configSaved = true;
            Serial.println("[BLE] Trama procesada y vinculación guardada en NVS. Enviando ACK 'OK'...");
            c->setValue("OK");
            c->notify();
        } else {
            Serial.println("[BLE] Error: Trama de aprovisionamiento inválida.");
            c->setValue("ERROR:INVALID_FRAME");
            c->notify();
        }
    }
};

void initBLEProvisioning() {
}

QuickPairingResult runQuickPairing(unsigned long timeoutMs,
                                   relay::domain::IIndicator* indicator,
                                   relay::domain::IButtonHandler* button) {
    Serial.println("\n[PAIRING] =========================================================");
    Serial.println("[PAIRING] Iniciando QUICK PAIRING (BLE GATT + SmartConfig ESP-Touch)");
    Serial.println("[PAIRING] - Pulsación larga (5s): Cambiar a SoftAP GPS-<ChipID>");
    Serial.println("[PAIRING] - Pulsación corta:      Salir de modo emparejamiento");
    Serial.println("[PAIRING] =========================================================");
    _configSaved = false;
    _bleStopping = false;
    _rxFrameBuffer.clear();

    if (indicator) {
        indicator->show(relay::domain::IndicatorStatus::QuickPairing);
    }

    if (config.device_id.length() == 0) {
        uint64_t chipId = ESP.getEfuseMac();
        char buf[16];
        snprintf(buf, sizeof(buf), "%04X%08X", (uint16_t)(chipId >> 32), (uint32_t)chipId);
        config.device_id = String(buf);
    }

    if (config.device_key.length() == 0) {
        uint64_t mac = ESP.getEfuseMac();
        char buf[37];
        snprintf(buf, sizeof(buf), "%08X-%04X-4%03X-%04X-%012llX",
                 (uint32_t)(mac & 0xFFFFFFFF),
                 (uint16_t)((mac >> 32) & 0xFFFF),
                 (uint16_t)((mac >> 16) & 0x0FFF),
                 (uint16_t)(((mac >> 48) & 0x3FFF) | 0x8000),
                 (unsigned long long)(mac ^ 0x5A5A5A5A5A5AULL));
        config.device_key = String(buf);
    }

    // 1. Activar SmartConfig (ESP-Touch / AirKiss) en paralelo solo si Wi-Fi no está ya conectado
    const bool alreadyWifiConn = (WiFi.status() == WL_CONNECTED);
    if (!alreadyWifiConn) {
        WiFi.mode(WIFI_STA);
        WiFi.beginSmartConfig();
    }

    // 2. Activar servidor BLE GATT 'GPS-<DeviceID>'
    String advName = "GPS-" + config.device_id;
    NimBLEDevice::init(advName.c_str());
    NimBLEDevice::setMTU(512);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->advertiseOnDisconnect(false);
    server->setCallbacks(new ServerCallbacks());

    NimBLEService* dis = server->createService(NimBLEUUID((uint16_t)0x180A));
    if (dis) {
        NimBLECharacteristic* mChar = dis->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A24), NIMBLE_PROPERTY::READ);
        mChar->setValue("VESIS-GPS-ESP32");

        NimBLECharacteristic* sChar = dis->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A25), NIMBLE_PROPERTY::READ);
        sChar->setValue(config.device_id.c_str());

        NimBLECharacteristic* fwChar = dis->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A26), NIMBLE_PROPERTY::READ);
        fwChar->setValue(FIRMWARE_VERSION);

        NimBLECharacteristic* hwChar = dis->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A27), NIMBLE_PROPERTY::READ);
        hwChar->setValue("ESP32");

        NimBLECharacteristic* mfChar = dis->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
        mfChar->setValue("VESIS");

        dis->start();
    }

    NimBLEService* service = server->createService(SERVICE_UUID);

    auto mkChar = [&](const char* uuid, String* field, bool secret = false) {
        const uint32_t props = secret ? NIMBLE_PROPERTY::WRITE : (NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
        NimBLECharacteristic* ch = service->createCharacteristic(uuid, props);
        ch->setCallbacks(new StringWriteCallback(field, secret));
        ch->setValue(secret ? "" : field->c_str());
        return ch;
    };

    mkChar(CHAR_APN_UUID,       &config.apn, false);
    mkChar(CHAR_APNUSER_UUID,   &config.apn_user, false);
    mkChar(CHAR_APNPASS_UUID,   &config.apn_pass, true);
    mkChar(CHAR_WIFISSID_UUID,  &config.wifi_ssid, false);
    mkChar(CHAR_WIFIPASS_UUID,  &config.wifi_pass, true);
    mkChar(CHAR_BROKER_UUID,    &config.mqtt_broker, false);
    mkChar(CHAR_MQTTUSER_UUID,  &config.mqtt_user, false);
    mkChar(CHAR_MQTTPASS_UUID,  &config.mqtt_pass, true);
    mkChar(CHAR_OTAKEY_UUID,    &config.ota_hmac_key, true);

    NimBLECharacteristic* devIdChar = service->createCharacteristic(
        CHAR_DEVID_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
    devIdChar->setCallbacks(new DevIdCallback());
    devIdChar->setValue(config.device_id.c_str());

    NimBLECharacteristic* devKeyChar = service->createCharacteristic(
        CHAR_DEVKEY_UUID, NIMBLE_PROPERTY::READ);
    devKeyChar->setCallbacks(new DevKeyCallback());
    devKeyChar->setValue(config.device_key.c_str());

    NimBLECharacteristic* bindTokenChar = service->createCharacteristic(
        CHAR_BINDTOKEN_UUID, NIMBLE_PROPERTY::WRITE);
    bindTokenChar->setCallbacks(new BindTokenCallback());
    bindTokenChar->setValue("");

    NimBLECharacteristic* portChar = service->createCharacteristic(
        CHAR_PORT_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
    portChar->setCallbacks(new PortWriteCallback(&config.mqtt_port));
    portChar->setValue(String(config.mqtt_port).c_str());

    NimBLECharacteristic* frameChar = service->createCharacteristic(
        CHAR_FRAME_JSON_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    frameChar->setCallbacks(new FrameCallback());

    String initJson;
    buildBleStateJson(initJson);
    frameChar->setValue(initJson.c_str());

    NimBLECharacteristic* saveChar = service->createCharacteristic(
        CHAR_SAVE_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    saveChar->setCallbacks(new SaveCallback());
    saveChar->setValue("READY");

    service->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setName(advName.c_str());
    // 7-byte Manufacturer Data: [0xFF, 0xFF, 'V', 'S', stateByte, deviceCode=0x02, provFlags]
    const uint8_t mfgPayload[7] = {
        0xFF, 0xFF, 'V', 'S',
        computeAdvStateByte(),
        0x02, // gps_esp32
        computeAdvProvFlags()
    };
    adv->setManufacturerData(std::string(reinterpret_cast<const char*>(mfgPayload), sizeof(mfgPayload)));
    adv->start();

    Serial.printf("[BLE] Publicitando como '%s' (mode=%s, autoApn=%d) durante %lu ms\n",
                  advName.c_str(),
                  resolveDeviceCredentialMode(alreadyWifiConn),
                  isCellularApnAutoDetected() ? 1 : 0,
                  timeoutMs);

    if (button) {
        unsigned long releaseWaitStart = millis();
        while (digitalRead(button->getPin()) == LOW && (millis() - releaseWaitStart) < 3000) {
            if (indicator) indicator->update();
            delay(20);
        }
        button->init();
    }

    auto cleanupRadios = [&]() {
        _bleStopping = true;
        if (!alreadyWifiConn) {
            WiFi.stopSmartConfig();
        }
        if (adv) {
            adv->stop();
        }
        if (server && server->getConnectedCount() > 0) {
            auto peers = server->getPeerDevices();
            for (uint16_t connHandle : peers) {
                server->disconnect(connHandle);
            }
            unsigned long discWait = millis();
            while (server->getConnectedCount() > 0 && (millis() - discWait) < 600) {
                delay(20);
            }
        }
        delay(80);
        NimBLEDevice::deinit(true);
    };

    unsigned long start = millis();
    while ((millis() - start) < timeoutMs) {
        if (indicator) {
            indicator->update();
        }

        if (_configSaved) {
            Serial.println("[PAIRING] ¡Aprovisionamiento BLE completado con éxito!");
            delay(600);
            cleanupRadios();
            return QuickPairingResult::Provisioned;
        }

        if (!alreadyWifiConn && WiFi.smartConfigDone()) {
            String scSsid = WiFi.SSID();
            String scPass = WiFi.psk();
            if (scSsid.length() > 0) {
                config.wifi_ssid = scSsid;
                config.wifi_pass = scPass;
                saveConfig();
                Serial.printf("[PAIRING] ¡SmartConfig completado! SSID: %s\n", scSsid.c_str());
                cleanupRadios();
                return QuickPairingResult::Provisioned;
            }
        }

        if (button) {
            auto ev = button->update();
            if (ev == relay::domain::ButtonEvent::LongPress) {
                Serial.println("[PAIRING] 2ª pulsación larga (5s) -> Conmutando a SoftAP Compatible (GPS-<ChipID>)...");
                cleanupRadios();
                return QuickPairingResult::SwitchToSoftAp;
            } else if (ev == relay::domain::ButtonEvent::ShortPress) {
                Serial.println("[PAIRING] Pulsación corta detectada -> Cancelando modo emparejamiento.");
                cleanupRadios();
                return QuickPairingResult::CancelledByButton;
            }
        }

        delay(20);
    }

    Serial.println("[PAIRING] Tiempo de espera de Quick Pairing agotado.");
    cleanupRadios();
    return QuickPairingResult::Timeout;
}

bool runBLEProvisioning(unsigned long timeoutMs) {
    return runQuickPairing(timeoutMs, nullptr, nullptr) == QuickPairingResult::Provisioned;
}
