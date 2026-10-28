#pragma once

#include <cstdint>

namespace relay::board {

constexpr uint8_t RELAY_PIN_1          = 4;
constexpr uint8_t BUTTON_PIN_1         = 0;
constexpr uint8_t WIFI_LED_PIN         = 2;

// GPS UART (Serial2 on ESP32)
constexpr uint8_t GPS_RX_PIN           = 16;
constexpr uint8_t GPS_TX_PIN           = 17;
constexpr uint32_t GPS_BAUD_RATE       = 9600;

// MPU6050 I2C
constexpr uint8_t I2C_SDA_PIN          = 21;
constexpr uint8_t I2C_SCL_PIN          = 22;

// GSM / SIM7000G UART
constexpr uint8_t GSM_RX_PIN           = 26;
constexpr uint8_t GSM_TX_PIN           = 27;
constexpr uint32_t GSM_BAUD_RATE       = 115200;

constexpr uint8_t  DEBOUNCE_TIME_MS      = 50;
constexpr uint16_t LONG_PRESS_TIME_MS    = 3000;

constexpr int FAST_BLINK_INTERVAL_MS   = 100;
constexpr int SLOW_BLINK_INTERVAL_MS   = 400;

}
