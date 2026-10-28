#ifndef CONFIG_H
#define CONFIG_H

#include <cstddef>
#include <cstdint>

#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

constexpr unsigned long WIFI_CONNECT_TIMEOUT = 5000;

constexpr const char *ntpServer1 = "pool.ntp.org";
constexpr const char *ntpServer2 = "time.nist.gov";
constexpr long gmtOffset_sec = -4 * 3600;
constexpr int daylightOffset_sec = 0;

constexpr const char *MQTT_SERVER = "192.168.100.7";
constexpr int MQTT_PORT = 1883;
constexpr const char *API_KEY = "dvk_8e64c24388e36214db296ef144f866c1f1ec75571658b736b755743a1936c646";
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 10000;
constexpr int MQTT_SOCKET_TIMEOUT = 30;
constexpr size_t MQTT_BUFFER_SIZE = 4096;
constexpr unsigned long MQTT_KEEP_ALIVE = 60;

inline const char *root_ca_pem = "-----BEGIN CERTIFICATE-----\n"
                                 "YOUR_SSL_CERTIFICATE_HERE\n"
                                 "-----END CERTIFICATE-----\n";

constexpr uint8_t RelayPin1 = 4;
constexpr uint8_t BUTTON_PIN1 = 0;
constexpr uint8_t wifiLed = 2;
constexpr uint8_t DEBOUNCE_TIME = 50;
constexpr uint16_t BUTTON_LONG_PRESS_TIME = 3000;

// GPS & MPU6050 Hardware Pins
constexpr uint8_t GPS_RX_PIN = 16;
constexpr uint8_t GPS_TX_PIN = 17;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t GSM_RX_PIN = 26;
constexpr uint8_t GSM_TX_PIN = 27;

// Motion & Accident Detection Thresholds
constexpr int ACCIDENT_SENSITIVITY = 2000;
constexpr int ROLLOVER_ANGLE_DEG = 60;
constexpr float HARSH_ACCEL_THRESHOLD_G = 0.45f;
constexpr float HARSH_BRAKE_THRESHOLD_G = -0.55f;
constexpr unsigned long ACCIDENT_COOLDOWN = 60000UL;
constexpr unsigned long ACCIDENT_CHECK_INTERVAL = 100UL;
constexpr unsigned long GPS_PUBLISH_INTERVAL = 30000UL;

// Emergency SMS Alerts
constexpr const char *EMERGENCY_PHONE = "+18090000000";
constexpr bool SMS_ALERTS_ENABLED = true;

constexpr unsigned long STATUS_PUBLISH_INTERVAL = 3600000;
constexpr unsigned long TIME_SYNC_INTERVAL = 3600000;
constexpr int FAST_BLINK_INTERVAL = 100;
constexpr int SLOW_BLINK_INTERVAL = 400;

constexpr const char *FIRMWARE_VERSION = "v0.0.9";

#endif
