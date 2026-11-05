#include "Infrastructure/Config/NvsConfigRepository.h"
#include "Infrastructure/Config/config.h"
#include <Arduino.h>
#include <WiFi.h>

#include "Application/Controllers/RelayFirmwareController.h"
#include "Application/Messaging/MqttTopics.h"
#include "Application/Services/AccidentService.h"
#include "Application/Services/ConnectivityService.h"
#include "Application/Services/RelayService.h"
#include "Application/Services/TelemetryService.h"
#include "Application/Services/TimerSchedulerService.h"

#include "Infrastructure/Actuators/GpioRelay.h"
#include "Infrastructure/Button/GpioButton.h"
#include "Infrastructure/Indicator/GpioIndicator.h"
#include "Infrastructure/Logging/SerialLogger.h"
#include "Infrastructure/Messaging/Mqttv5Transport.h"
#include "Infrastructure/Messaging/mqtt_client.h"
#include "Infrastructure/Modem/Sim7000Modem.h"
#include "Infrastructure/Net/LanZeroconfServer.h"
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

static infra::ArduinoClock g_clock;
static infra::SerialLogger g_logger;
static infra::EspSystemControl g_system;
static infra::GpioIndicator g_indicator{wifiLed, FAST_BLINK_INTERVAL,
                                        SLOW_BLINK_INTERVAL};
static infra::GpioRelay g_relay1{RelayPin1};
static infra::GpioButton g_button1{BUTTON_PIN1, DEBOUNCE_TIME,
                                   BUTTON_LONG_PRESS_TIME};
static infra::WifiLink g_wifi{WIFI_CONNECT_TIMEOUT};
static infra::Mqttv5Transport g_transport;
static infra::Mqttv5OtaService g_ota;
static infra::EspDeviceRepository g_deviceRepo{FIRMWARE_VERSION};
static infra::NtpTimeProvider g_timeProvider{gmtOffset_sec, daylightOffset_sec,
                                             ntpServer1, ntpServer2};
static infra::ArduinoJsonPayloadSerializer g_serializer;
static infra::NvsConfigRepository g_configRepo;
static infra::BleProvisioningPortal g_blePortal;

// SIM7000G (Serial2: GNSS + LTE/SMS) & MPU6050 (I2C: Impact/Rollover/Harsh
// events)
static infra::Sim7000Modem g_sim7000{Serial2,        SIM7000_RX_PIN,
                                     SIM7000_TX_PIN, SIM7000_PWRKEY_PIN,
                                     SIM7000_BAUD,   g_logger};
static infra::Mpu6050Detector g_motionDetector{I2C_SDA_PIN,
                                               I2C_SCL_PIN,
                                               ACCIDENT_SENSITIVITY,
                                               ROLLOVER_ANGLE_DEG,
                                               HARSH_ACCEL_THRESHOLD_G,
                                               HARSH_BRAKE_THRESHOLD_G,
                                               g_logger};

static app::RelayService *g_relayService = nullptr;
static app::ConnectivityService *g_connectivity = nullptr;
static app::TelemetryService *g_telemetry = nullptr;
static app::AccidentService *g_accidentService = nullptr;
static app::TimerSchedulerService *g_timerScheduler = nullptr;
static app::RelayFirmwareController *g_controller = nullptr;
static infra::LanZeroconfServer *g_lanServer = nullptr;

static void executeFactoryUnlinkToQuickPairing() {
  Serial.println("[UNLINK] Dispositivo desvinculado de la cuenta. Borrando credenciales y entrando en Quick Pairing...");
  g_relay1.setState(false);
  resetConfig();
  WiFi.disconnect(true, true);
  delay(200);
  ESP.restart();
}

static void runPairingWorkflow(bool isInitialUnprovisionedBoot) {
  g_sim7000.probeCellularProvisioningState();
  Serial.println(
      "[PAIRING] Entrando en Etapa 1: Quick Pairing (BLE + SmartConfig)...");
  QuickPairingResult res = g_blePortal.runInteractive(
      isInitialUnprovisionedBoot ? 60000UL : 180000UL, g_indicator, g_button1);

  if (res == QuickPairingResult::Provisioned) {
    Serial.println(
        "[PAIRING] Credenciales recibidas en Quick Pairing. Reiniciando...");
    delay(400);
    ESP.restart();
    return;
  }

  if (res == QuickPairingResult::CancelledByButton) {
    Serial.println("[PAIRING] Emparejamiento cancelado por pulsación corta. "
                   "Retomando operación normal.");
    return;
  }

  if (res == QuickPairingResult::SwitchToSoftAp || isInitialUnprovisionedBoot) {
    Serial.println(
        "[PAIRING] Entrando en Etapa 2: Compatible SoftAP (GPS-<ChipID>)...");
    infra::WifiApProvisioningPortal::startPortal(180, &g_indicator, &g_button1);
  }
}

void setup() {
  Serial.begin(115200);

  auto identity = g_deviceRepo.identity();
  g_configRepo.begin(identity.chipId.c_str());

  domain::DeviceConfig devConfig;
  bool isProv = g_configRepo.load(devConfig);

  std::string activeDeviceId =
      !devConfig.deviceId.empty() ? devConfig.deviceId : identity.chipId;
  std::string deviceId1 = activeDeviceId + "-01";

  // Restaurar estado del relé (< 20 ms, Offline-First) según StartupMode (Off,
  // On, Stay) y pulso
  g_indicator.setSledOnline(devConfig.sledOnline);
  static app::RelayService relayService{g_logger, &g_configRepo};
  relayService.registerRelay(deviceId1, g_relay1, "device_switch1");
  relayService.initWithStartupConfig(
      devConfig.startupMode, devConfig.lastRelayState, devConfig.pulseEnabled,
      devConfig.pulseWidthMs, millis());
  g_relayService = &relayService;

  Serial.println("\n=============================================");
  Serial.println("  GPS-ESP32 Clean Architecture - Firmware    ");
  Serial.printf("  Versión: %s\n", FIRMWARE_VERSION);
  Serial.println("=============================================\n");
  Serial.printf("[INIT] Chip ID:       %s\n", identity.chipId.c_str());
  Serial.printf("[INIT] Device ID:     %s\n", activeDeviceId.c_str());
  Serial.printf("[INIT] MAC:           %s\n", identity.macAddress.c_str());
  Serial.printf("[INIT] Relay ID:      %s\n", deviceId1.c_str());
  Serial.printf("[INIT] StartupMode:   %s (Pulso=%s, %u ms, SLED=%s)\n",
                domain::startupModeToString(devConfig.startupMode),
                devConfig.pulseEnabled ? "on" : "off", devConfig.pulseWidthMs,
                devConfig.sledOnline ? "on" : "off");

  if (!isProv) {
    g_button1.init();
    runPairingWorkflow(true);
    g_configRepo.load(devConfig);
  }

  if (!devConfig.apn.empty()) {
    g_sim7000.setApn(devConfig.apn, devConfig.apnUser, devConfig.apnPass);
  }
  if (!devConfig.emergencyPhone.empty()) {
    g_sim7000.setEmergencyPhone(devConfig.emergencyPhone);
  }
  if (devConfig.accidentSensitivity > 0) {
    g_motionDetector.setSensitivity(devConfig.accidentSensitivity);
  }
  if (devConfig.rolloverAngleDeg > 0) {
    g_motionDetector.setRolloverAngle(devConfig.rolloverAngleDeg);
  }

  g_wifi.setCredentials(devConfig.wifiSsid, devConfig.wifiPass);

  const char *broker = !devConfig.mqttBroker.empty()
                           ? devConfig.mqttBroker.c_str()
                           : MQTT_SERVER;
  int port = devConfig.mqttPort > 0 ? devConfig.mqttPort : MQTT_PORT;
  const char *apiKey =
      !devConfig.mqttUser.empty() ? devConfig.mqttUser.c_str() : API_KEY;

  app::MqttTopics topics =
      app::MqttTopics::forDevice(activeDeviceId, "device_switch1");

  infra::initMQTT(broker, port, activeDeviceId.c_str(), apiKey, root_ca_pem,
                  FIRMWARE_VERSION);
  infra::setMqttUnlinkVerifiedCallback(executeFactoryUnlinkToQuickPairing);

  static app::ConnectivityService connectivity{g_wifi, g_transport, g_logger};
  g_connectivity = &connectivity;

  static app::TelemetryService telemetry{
      relayService, connectivity, g_deviceRepo, g_timeProvider,
      g_clock,      g_serializer, topics,       g_logger,
      &g_sim7000};
  g_telemetry = &telemetry;

  static app::AccidentService accidentService{
      g_motionDetector,  g_sim7000,
      g_sim7000,         telemetry,
      g_clock,           g_logger,
      !devConfig.emergencyPhone.empty() ? devConfig.emergencyPhone : std::string(EMERGENCY_PHONE),
      devConfig.smsEnabled,
      ACCIDENT_COOLDOWN, ACCIDENT_CHECK_INTERVAL};
  g_accidentService = &accidentService;

  static app::TimerSchedulerService timerScheduler{relayService, deviceId1,
                                                   &g_configRepo};
  g_timerScheduler = &timerScheduler;

  static infra::LanZeroconfServer lanServer{
      relayService, telemetry,      timerScheduler, g_indicator,
      g_configRepo, activeDeviceId, deviceId1,      LAN_HTTP_PORT};
  lanServer.setUnlinkCallback(executeFactoryUnlinkToQuickPairing);
  g_lanServer = &lanServer;

  auto provisioningCallback = []() { runPairingWorkflow(false); };

  static app::RelayFirmwareController controller{relayService,
                                                 connectivity,
                                                 telemetry,
                                                 g_ota,
                                                 g_indicator,
                                                 g_system,
                                                 g_timeProvider,
                                                 g_clock,
                                                 g_logger,
                                                 topics,
                                                 provisioningCallback,
                                                 &accidentService,
                                                 &g_sim7000,
                                                 &timerScheduler,
                                                 &g_configRepo,
                                                 STATUS_PUBLISH_INTERVAL,
                                                 TIME_SYNC_INTERVAL,
                                                 GPS_PUBLISH_INTERVAL};

  controller.registerButton(g_button1, deviceId1);
  controller.setOnLocalStateChangeCallback([]() {
    if (g_lanServer) {
      g_lanServer->notifyMdnsStateChange();
    }
  });

  controller.setup(true);
  g_controller = &controller;

  if (g_wifi.isConnected()) {
    g_timeProvider.sync();
  }

  Serial.println(
      "[SERIAL] Comandos de diagnóstico GPS disponibles por consola serial:");
  Serial.println(
      "         - diag    : Diagnóstico completo SIM7000G + GNSS + Satélites");
  Serial.println(
      "         - nmea    : Activar/desactivar tramas crudas AT+CGNSINF");
  Serial.println(
      "         - pwr_on  : Forzar pulso de encendido PWRKEY y AT+CGNSPWR=1");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("diag") || cmd.equalsIgnoreCase("gps")) {
      g_sim7000.diagnose();
    } else if (cmd.equalsIgnoreCase("nmea")) {
      static bool rawLog = false;
      rawLog = !rawLog;
      g_sim7000.setRawLogging(rawLog);
      Serial.printf("[SERIAL] Log crudo GNSS %s\n",
                    rawLog ? "ACTIVADO" : "DESACTIVADO");
    } else if (cmd.equalsIgnoreCase("pwr_on")) {
      g_sim7000.powerOn();
      g_sim7000.enableGnss(true);
    }
  }

  if (g_controller) {
    g_controller->tick();
  }
  if (g_lanServer) {
    g_lanServer->loop(g_wifi.isConnected());
  }
  delay(5);
}
