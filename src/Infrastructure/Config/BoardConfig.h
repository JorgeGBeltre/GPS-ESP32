#pragma once

#include <cstdint>

namespace relay::board {

constexpr uint8_t RELAY_PIN_1          = 4;
constexpr uint8_t BUTTON_PIN_1         = 0;
constexpr uint8_t WIFI_LED_PIN         = 2;

// SIM7000G UART (Serial2 on ESP32: LTE & GNSS)
constexpr uint8_t SIM7000_RX_PIN       = 16;
constexpr uint8_t SIM7000_TX_PIN       = 17;
constexpr uint32_t SIM7000_BAUD_RATE   = 115200;

// MPU6050 I2C
constexpr uint8_t I2C_SDA_PIN          = 21;
constexpr uint8_t I2C_SCL_PIN          = 22;

constexpr uint8_t  DEBOUNCE_TIME_MS      = 50;
constexpr uint16_t LONG_PRESS_TIME_MS    = 3000;

constexpr int FAST_BLINK_INTERVAL_MS   = 100;
constexpr int SLOW_BLINK_INTERVAL_MS   = 400;

}
