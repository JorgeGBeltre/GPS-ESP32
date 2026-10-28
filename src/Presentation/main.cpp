#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "Infrastructure/Config/config.h"
#include "Infrastructure/Config/BoardConfig.h"
#include "Infrastructure/Config/NvsConfigRepository.h"

#include "Application/Controllers/RelayFirmwareController.h"
#include "Application/Messaging/MqttTopics.h"
#include "Application/Services/AccidentService.h"
#include "Application/Services/ConnectivityService.h"
#include "Application/Services/RelayService.h"
#include "Application/Services/TelemetryService.h"

#include "Infrastructure/Actuators/GpioRelay.h"
#include "Infrastructure/Button/GpioButton.h"
#include "Infrastructure/Modem/Sim7000Modem.h"
#include "Infrastructure/Indicator/GpioIndicator.h"
#include "Infrastructure/Logging/SerialLogger.h"
#include "Infrastructure/Messaging/Mqttv5Transport.h"
#include "Infrastructure/Messaging/mqtt_client.h"
#include "Infrastructure/Net/WifiLink.h"
#include "Infrastructure/Ota/Mqttv5OtaService.h"
#include "Infrastructure/Provisioning/ProvisioningPortals.h"
#include "Infrastructure/Sensors/Mpu6050Detector.h"
#include "Infrastructure/Serialization/ArduinoJsonPayloadSerializer.h"
#include "Infrastructure/System/EspDeviceRepository.h"
#include "Infrastructure/System/EspSystemControl.h"
#include "Infrastructure/Time/ArduinoClock.h"
#include "Infrastructure/Time/NtpTimeProvider.h"

using namespace relay;

static infra::ArduinoClock                 g_clock;
static infra::SerialLogger                 g_logger;
static infra::EspSystemControl             g_system;
static infra::GpioIndicator                g_indicator{wifiLed, FAST_BLINK_INTERVAL, SLOW_BLINK_INTERVAL};
static infra::GpioRelay                    g_relay1{RelayPin1};
static infra::GpioButton                   g_button1{BUTTON_PIN1, DEBOUNCE_TIME, BUTTON_LONG_PRESS_TIME};
static infra::WifiLink                     g_wifi{WIFI_CONNECT_TIMEOUT};
static infra::Mqttv5Transport              g_transport;
static infra::Mqttv5OtaService             g_ota;
static infra::EspDeviceRepository          g_deviceRepo{FIRMWARE_VERSION};
static infra::NtpTimeProvider              g_timeProvider{gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2};
static infra::ArduinoJsonPayloadSerializer g_serializer;
static infra::NvsConfigRepository          g_configRepo;
static infra::BleProvisioningPortal        g_blePortal;

// Hardware MPU6050 (Wire on ESP32: SDA=GPIO 21, SCL=GPIO 22)
static infra::Mpu6050Detector              g_accidentDetector{
    I2C_SDA_PIN, I2C_SCL_PIN,
    ACCIDENT_SENSITIVITY, ROLLOVER_ANGLE_DEG,
    HARSH_ACCEL_THRESHOLD_G, HARSH_BRAKE_THRESHOLD_G,
    ACCIDENT_CHECK_INTERVAL, ACCIDENT_COOLDOWN};

// SIM7000G Integrated LTE & GNSS Modem (Serial2: RX=GPIO 16, TX=GPIO 17, PWRKEY=GPIO 23, 115200 baud)
static infra::Sim7000Modem                 g_sim7000{
    Serial2, SIM7000_RX_PIN, SIM7000_TX_PIN, SIM7000_PWRKEY_PIN, SIM7000_BAUD, EMERGENCY_PHONE};

static app::RelayService*            g_relayService = nullptr;
static app::ConnectivityService*     g_connectivity = nullptr;
static app::TelemetryService*        g_telemetry = nullptr;
static app::AccidentService*         g_accidentService = nullptr;
static app::RelayFirmwareController* g_controller = nullptr;

static bool shouldForceProvisioning() {
    pinMode(BUTTON_PIN1, INPUT_PULLUP);
    if (digitalRead(BUTTON_PIN1) == LOW) {
        const unsigned long start = millis();
        while (digitalRead(BUTTON_PIN1) == LOW) {
            if (millis() - start > BUTTON_LONG_PRESS_TIME) return true;
            delay(50);
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=============================================");
    Serial.println("  VESIS GPS ESP32 - Clean Architecture Firmware ");
    Serial.printf("  Version: %s\n", FIRMWARE_VERSION);
    Serial.println("=============================================\n");

    auto identity = g_deviceRepo.identity();
    g_configRepo.begin(identity.chipId.c_str());

    domain::DeviceConfig devConfig;
    bool isProv = g_configRepo.load(devConfig);

    // Initial provisioning check
    if (!isProv || shouldForceProvisioning()) {
        Serial.println("[MAIN] Dispositivo en modo aprovisionamiento...");
        g_indicator.show(domain::IndicatorStatus::Connecting);

        bool bleOk = g_blePortal.run(60000);
        if (bleOk) {
            Serial.println("[MAIN] Aprovisionamiento BLE completado. Reiniciando...");
            delay(500);
            ESP.restart();
        } else if (!isProv) {
            Serial.println("[MAIN] BLE timeout. Iniciando portal WiFi SoftAP de respaldo...");
            infra::WifiApProvisioningPortal::startPortal(180);
        }
    }

    std::string activeDeviceId = !devConfig.deviceId.empty() ? devConfig.deviceId : identity.chipId;
    std::string deviceId1 = activeDeviceId + "-01";

    Serial.printf("[INIT] Chip ID:       %s\n", identity.chipId.c_str());
    Serial.printf("[INIT] Device ID:     %s\n", activeDeviceId.c_str());
    Serial.printf("[INIT] MAC:           %s\n", identity.macAddress.c_str());
    Serial.printf("[INIT] Relay ID:      %s\n", deviceId1.c_str());

    // Initialize sensors and communication hardware
    g_accidentDetector.begin();
    g_sim7000.begin();

    if (SMS_ALERTS_ENABLED) {
        g_sim7000.setCaCert(root_ca_pem);
    }

    g_wifi.setCredentials(devConfig.wifiSsid, devConfig.wifiPass);

    bool brokerIsPlaceholder = devConfig.mqttBroker.empty() || 
                               devConfig.mqttBroker == "YOUR_MQTT_SERVER_HERE" || 
                               devConfig.mqttBroker == "localhost";
    const char* broker = !brokerIsPlaceholder ? devConfig.mqttBroker.c_str() : MQTT_SERVER;
    int port = (!brokerIsPlaceholder && devConfig.mqttPort > 0 && devConfig.mqttPort != 8883) ? devConfig.mqttPort : MQTT_PORT;

    bool userIsPlaceholder = devConfig.mqttUser.empty() || 
                             devConfig.mqttUser == "YOUR_API_KEY_HERE";
    const char* apiKey = !userIsPlaceholder ? devConfig.mqttUser.c_str() : API_KEY;

    if (brokerIsPlaceholder || userIsPlaceholder) {
        devConfig.mqttBroker = MQTT_SERVER;
        devConfig.mqttPort = MQTT_PORT;
        devConfig.mqttUser = API_KEY;
        g_configRepo.save(devConfig);
    }

    app::MqttTopics topics = app::MqttTopics::forDevice(activeDeviceId);

    infra::initMQTT(broker, port, activeDeviceId.c_str(),
                    apiKey, root_ca_pem, FIRMWARE_VERSION);

    static app::RelayService relayService{g_logger};
    relayService.registerRelay(deviceId1, g_relay1, "device_switch1");
    g_relayService = &relayService;

    static app::ConnectivityService connectivity{g_wifi, g_transport, g_logger};
    g_connectivity = &connectivity;

    static app::TelemetryService telemetry{
        relayService, connectivity, g_deviceRepo, g_timeProvider,
        g_clock, g_serializer, topics, g_logger, &g_sim7000};
    g_telemetry = &telemetry;

    static app::AccidentService accidentService{
        g_accidentDetector, g_sim7000, connectivity,
        g_deviceRepo, g_timeProvider, g_serializer,
        topics, g_logger, &g_sim7000};
    g_accidentService = &accidentService;

    auto provisioningCallback = []() {
        Serial.println("[PROVISION] Pulsacion larga detectada (>3s) -> Iniciando portal BLE...");
        g_indicator.show(domain::IndicatorStatus::Connecting);
        bool bleOk = g_blePortal.run(60000);
        if (bleOk) {
            Serial.println("[PROVISION] Aprovisionamiento BLE completado. Reiniciando...");
            delay(500);
            ESP.restart();
        } else {
            Serial.println("[PROVISION] BLE timeout. Abriendo portal SoftAP de respaldo...");
            infra::WifiApProvisioningPortal::startPortal(180);
        }
    };

    static app::RelayFirmwareController controller{
        relayService, connectivity, telemetry, g_ota, g_indicator,
        g_system, g_timeProvider, g_clock, g_logger, topics,
        provisioningCallback, &accidentService, &g_sim7000};

    controller.registerButton(g_button1, deviceId1);

    controller.setup();
    g_controller = &controller;

    g_timeProvider.sync();
}

void loop() {
    if (g_controller) {
        g_controller->tick();
    }
    g_sim7000.update();

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "diag" || cmd == "gps") {
            g_sim7000.diagnose();
        } else if (cmd == "pwr_on") {
            g_sim7000.powerOn();
        } else if (cmd == "pwr_off") {
            g_sim7000.powerOff();
        } else if (cmd == "restart_modem") {
            g_sim7000.restartModem();
        }
    }

    delay(10);
}
