# VESIS GPS-ESP32

**Smart IoT Vehicle Tracker, Accident & Rollover Detection System, Smart Actuator & Dual Provisioning Firmware for ESP32 with Clean Architecture, Native MQTT 5.0, MPU6050 Motion Dynamics, Cellular APN Provisioning, and Emergency SMS Alerting**

[![Platform](https://img.shields.io/badge/platform-ESP32-blue)](https://www.espressif.com/en/products/socs/esp32)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-green)](https://en.cppreference.com/w/cpp/17)
[![MQTT 5.0](https://img.shields.io/badge/MQTT-5.0-orange)](https://mqtt.org)
[![GPS](https://img.shields.io/badge/GPS-TinyGPSPlus-blueviolet)](https://github.com/mikalhart/TinyGPSPlus)
[![IMU](https://img.shields.io/badge/IMU-MPU6050%20I2C-yellow)](https://github.com/ElectronicCats/mpu6050)
[![BLE](https://img.shields.io/badge/Bluetooth-BLE%20GATT-blueviolet)](https://github.com/h2zero/NimBLE-Arduino)
[![Architecture](https://img.shields.io/badge/Architecture-Clean%20Architecture-purple)](#architecture)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-teal)](https://platformio.org)
[![CMake](https://img.shields.io/badge/CMake-Supported-red)](https://cmake.org)

---

**VESIS GPS-ESP32** is a mission-critical, production-grade connected vehicle telemetry, accident detection, and remote actuation firmware built for the Espressif ESP32 SoC. Engineered following strict **Clean Architecture (Hexagonal / Onion Architecture)** principles, the project completely decouples domain business and safety rules from external peripheral drivers, cloud protocols, and hardware dependencies.

The firmware combines high-frequency GPS positioning (`TinyGPSPlus`), 6-axis inertial motion dynamics (`MPU6050` with STA/LTA shock impact analysis, rollover tilt angles, and harsh acceleration/braking detection), backup emergency GSM SMS alerting (`SIM7000G`/`SIM800L`), physical actuator/relay control, mobile BLE onboarding with cellular APN provisioning (`NimBLE-Arduino`), captive SoftAP fallback (`WiFiManager`), session-aware native MQTT 5.0 (`ESP32_MQTTv5`), and secure chunked OTA (`MQTTOTAv5`).

---

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
  - [Layer Separation](#layer-separation)
  - [Directory Tree](#directory-tree)
- [Hardware Specifications & Pinout](#hardware-specifications--pinout)
- [Motion Dynamics & Accident Detection Algorithm](#motion-dynamics--accident-detection-algorithm)
  - [STA/LTA Shock Impact Detection](#stalta-shock-impact-detection)
  - [Rollover & Tilt Angle Detection](#rollover--tilt-angle-detection)
  - [Aggressive Driving Dynamics (Harsh Acceleration & Braking)](#aggressive-driving-dynamics-harsh-acceleration--braking)
  - [Speed Filtering & GPS Cross-Validation](#speed-filtering--gps-cross-validation)
- [GPS Telemetry & Navigation](#gps-telemetry--navigation)
- [Emergency GSM / Cellular SMS Alerting](#emergency-gsm--cellular-sms-alerting)
- [Dependencies](#dependencies)
- [Configuration & Placeholders](#configuration--placeholders)
- [Bluetooth BLE Provisioning & Cellular APN Configuration](#bluetooth-ble-provisioning--cellular-apn-configuration)
  - [Overview & Workflow](#overview--workflow)
  - [GATT Service & Characteristics Layout](#gatt-service--characteristics-layout)
  - [Standard Bluetooth Device Information Service (0x180A)](#standard-bluetooth-device-information-service-0x180a)
  - [Provisioning via Unified Data Frames (JSON & Key=Value)](#provisioning-via-unified-data-frames-json--keyvalue)
  - [Dual Provisioning Flow & Fallback Architecture](#dual-provisioning-flow--fallback-architecture)
- [MQTT 5.0 Messaging & Topics](#mqtt-50-messaging--topics)
  - [Topic Topology](#topic-topology)
  - [GPS Telemetry Payloads](#gps-telemetry-payloads)
  - [Accident & Safety Alert Payloads](#accident--safety-alert-payloads)
  - [Device Status & Actuator Payloads](#device-status--actuator-payloads)
  - [Inbound Command Handling](#inbound-command-handling)
- [Over-The-Air (OTA) Updates](#over-the-air-ota-updates)
- [Physical Controls & Status Indicator (LED)](#physical-controls--status-indicator-led)
- [Build & Flashing Instructions](#build--flashing-instructions)
  - [Using PlatformIO](#using-platformio)
  - [Dual OTA Partition Scheme](#dual-ota-partition-scheme)
  - [Host Compilation with CMake](#host-compilation-with-cmake)
- [Backend Implementation](#backend-implementation)
- [Broker Compatibility](#broker-compatibility)
- [License](#-license)
- [Contact](#contact)
- [Support](#support)

---

## Features

| Feature | Description |
|---|---|
| **Clean Architecture** | Pure Domain (zero-dependency C++17 entities/interfaces), decoupled Application use cases, isolated Infrastructure drivers, and unified Presentation composition root |
| **High-Precision GPS** | Powered by `TinyGPSPlus` on HardwareSerial2 (`GPIO 16/17`) — Latitude, Longitude, Altitude, Speed (km/h), Course/Heading, HDOP, Satellites, and UTC timestamp |
| **Accident & Impact Detection** | Real-time STA/LTA (Short-Term Average / Long-Term Average) ratio algorithm computing dynamic G-force shocks on `MPU6050` (`SDA 21/SCL 22`) to avoid false alarms from road bumps |
| **Vehicle Rollover Detection** | Continuous roll and pitch angle monitoring via trigonometric accelerometer projection. Immediate alert dispatch if roll or pitch exceeds `60°` |
| **Driving Dynamics Monitoring** | Real-time detection of harsh acceleration (`> +0.45g`) and harsh braking (`< -0.55g`) with GPS speed cross-validation |
| **Emergency GSM SMS Dispatch** | Powered by `Sim7000AlertSender` on HardwareSerial1 (`GPIO 26/27`) — automatically sends emergency SMS with Google Maps coordinates and collision metrics to configured SOS contacts |
| **Native MQTT 5.0** | Powered by `ESP32_MQTTv5` — TLS encryption (`WiFiClientSecure`), session resumption, user properties, keep-alive, and JSON Last Will & Testament (LWT) |
| **Secure OTA Updates** | Powered by `MQTTOTAv5` — chunked binary transfers over MQTT 5.0, SHA-256 HMAC integrity verification, automatic rollback protection, and non-blocking restarts |
| **Cellular APN & BLE Provisioning** | Powered by `NimBLE-Arduino` — Sonoff/eWeLink-style onboarding configuring cellular APN (`apn`, `apn_user`, `apn_pass`), WiFi, and MQTT broker endpoints via mobile app frames or individual GATT characteristics |
| **Dual Provisioning Portals** | Primary BLE Provisioning with automatic fallback to SoftAP Web Portal (`WiFiManager` / `GPS-<ChipID>`) on timeout or button long-press, including APN configuration fields |
| **Smart Actuation / Relay** | Solid-state or mechanical relay driver on `GPIO 4` with remote MQTT 5.0 control (`devices/<chipId>/switch/set`), physical button toggle on `GPIO 0`, and cloud state synchronization |
| **NVS Persistent Storage** | Dynamic credentials, APN, device keys, and calibration persisted non-volatilely via `Preferences` in the `relay-config` namespace |
| **Dual Build Toolchain** | Full target firmware builds via PlatformIO (`platformio.ini`) and pure host test compilation via CMake (`CMakeLists.txt`) |

---

## Architecture

The project adheres to Robert C. Martin's Clean Architecture pattern, engineered for modularity, testability, and clean separation of concerns.

```
+---------------------------------------------------------------------------------------+
|                                     Presentation                                      |
|                                       main.cpp                                        |
+-------------------------------------------+-------------------------------------------+
                                            |
+-------------------------------------------v-------------------------------------------+
|                                      Application                                      |
|  AccidentAlgorithm  |  AccidentService  |  TelemetryService  |  RelayService          |
|  ConnectivityService|  RelayFirmwareController  |  MqttTopics  |  IPayloadSerializer   |
+-------------------------------------------+-------------------------------------------+
                                            |
+-------------------------------------------v-------------------------------------------+
|                                        Domain                                         |
|  Entities:   GpsLocation, AccidentEvent, DeviceConfig, RelayState, DeviceIdentity    |
|  Interfaces: IGpsProvider, IAccidentDetector, IGsmAlertSender, IRelayController,      |
|              ITelemetryTransport, IOtaService, INetworkLink, IConfigRepository        |
+-------------------------------------------^-------------------------------------------+
                                            |
+-------------------------------------------+-------------------------------------------+
|                                    Infrastructure                                     |
|  TinyGpsPlusProvider  |  Mpu6050Detector  |  Sim7000AlertSender  |  GpioRelay         |
|  Mqttv5Transport      |  WifiLink         |  ble_provisioning    |  NvsConfigRepository|
+---------------------------------------------------------------------------------------+
```

### Layer Separation

1. **Domain Layer (`src/Domain/`)**:
   - Contains pure C++17 entities (`GpsLocation`, `AccidentEvent`, `RelayState`, `DeviceIdentity`, `SystemMetrics`, `DeviceEvent`, `DeviceConfig`).
   - Contains abstract interfaces (`IGpsProvider`, `IAccidentDetector`, `IGsmAlertSender`, `IRelayController`, `IButtonHandler`, `ITelemetryTransport`, `IOtaService`, `INetworkLink`, `IIndicator`, `ILogger`, `IClock`, `ITimeProvider`, `IDeviceRepository`, `ISystemControl`, `IConfigRepository`, `IProvisioningPortal`).
   - Zero external libraries, Arduino headers, or hardware dependencies.
2. **Application Layer (`src/Application/`)**:
   - Implements use cases and business orchestration:
     - `AccidentAlgorithm`: Evaluates STA/LTA shock ratios, rollover tilt angles, and harsh acceleration/braking with speed filtering.
     - `AccidentService`: Orchestrates accident events, publishing immediate high-priority MQTT 5.0 alerts and triggering emergency SMS alerts.
     - `TelemetryService`: Formats and streams GPS telemetry and periodic system health.
     - `RelayService`: Manages physical/virtual relay states, desired/current state delta matching, and toggles.
     - `RelayFirmwareController`: Finite state machine managing system transitions, button events, and command routing.
     - `ConnectivityService`: Coordinates network links (WiFi) and MQTT 5.0 transports.
     - `MqttTopics`: Generates pure string topic paths based on device identity.
     - `IPayloadSerializer`: Abstract data transfer serialization interface.
3. **Infrastructure Layer (`src/Infrastructure/`)**:
   - Implements concrete domain interfaces using hardware drivers and external libraries:
     - `Gps`: `TinyGpsPlusProvider` (HardwareSerial2 on `RX=16, TX=17`).
     - `Sensors`: `Mpu6050Detector` (I2C Wire on `SDA=21, SCL=22`).
     - `Gsm`: `Sim7000AlertSender` (HardwareSerial1 on `RX=26, TX=27`).
     - `Actuators`: `GpioRelay` (`GPIO 4` pin control).
     - `Button`: `GpioButton` (non-blocking debouncing and long press detector on `GPIO 0`).
     - `Config`: `BoardConfig.h` (pin mapping), `config.h` (defaults), `nvs_config.h`/`cpp` (NVS Preferences persistence with APN support), and `NvsConfigRepository.h`.
     - `Indicator`: `GpioIndicator` (non-blocking status LED on `GPIO 2`).
     - `Logging`: `SerialLogger` (serial logging output).
     - `Messaging`: `mqtt_client` and `Mqttv5Transport` (wrapping `ESP32_MQTTv5`).
     - `Net`: `WifiLink` and `NetworkClient` (WiFiClientSecure with TLS and dynamic credentials).
     - `Ota`: `ota_manager` and `Mqttv5OtaService` (wrapping `MQTTOTAv5`).
     - `Provisioning`: `ble_provisioning.h`/`cpp` (NimBLE GATT server with APN UUIDs & frame parser), `ProvisioningPortals.h`, and `WifiApProvisioningPortal.h` (`WiFiManager` captive portal).
     - `Serialization`: `ArduinoJsonPayloadSerializer` (ArduinoJson v6 schema formatter for GPS, alerts, and telemetry).
     - `System`: `EspDeviceRepository` (eFuse MAC, chip model, heap) and `EspSystemControl` (reboot).
     - `Time`: `ArduinoClock` and `NtpTimeProvider` (SNTP + ISO 8601 formatting).
4. **Presentation Layer (`src/Presentation/`)**:
   - `main.cpp`: Composition Root. Instantiates concrete infrastructure adapters, wires dependencies into application services, registers button handlers and sensor loops, and exposes standard `setup()` and `loop()` entry points.

### Directory Tree

```
GPS_Esp32/
├── CMakeLists.txt
├── platformio.ini
├── partitions.csv
├── .gitignore
├── README.md
└── src/
    ├── Domain/
    │   ├── Entities/
    │   │   ├── AccidentEvent.h
    │   │   ├── DeviceConfig.h
    │   │   ├── DeviceEvent.h
    │   │   ├── DeviceIdentity.h
    │   │   ├── GpsLocation.h
    │   │   ├── RelayState.h
    │   │   └── SystemMetrics.h
    │   └── Interfaces/
    │       ├── IAccidentDetector.h
    │       ├── IButtonHandler.h
    │       ├── IClock.h
    │       ├── IConfigRepository.h
    │       ├── IDeviceRepository.h
    │       ├── IGpsProvider.h
    │       ├── IGsmAlertSender.h
    │       ├── IIndicator.h
    │       ├── ILogger.h
    │       ├── INetworkLink.h
    │       ├── IOtaService.h
    │       ├── IProvisioningPortal.h
    │       ├── IRelayController.h
    │       ├── ISystemControl.h
    │       ├── ITelemetryTransport.h
    │       └── ITimeProvider.h
    ├── Application/
    │   ├── AccidentAlgorithm.h
    │   ├── AccidentAlgorithm.cpp
    │   ├── Controllers/
    │   │   ├── RelayFirmwareController.h
    │   │   └── RelayFirmwareController.cpp
    │   ├── Dto/
    │   │   └── DeviceCommand.h
    │   ├── Messaging/
    │   │   └── MqttTopics.h
    │   ├── Serialization/
    │   │   └── IPayloadSerializer.h
    │   └── Services/
    │       ├── AccidentService.h
    │       ├── AccidentService.cpp
    │       ├── ConnectivityService.h
    │       ├── ConnectivityService.cpp
    │       ├── RelayService.h
    │       ├── RelayService.cpp
    │       ├── TelemetryService.h
    │       └── TelemetryService.cpp
    ├── Infrastructure/
    │   ├── Actuators/
    │   │   ├── GpioRelay.h
    │   │   └── GpioRelay.cpp
    │   ├── Button/
    │   │   ├── GpioButton.h
    │   │   └── GpioButton.cpp
    │   ├── Config/
    │   │   ├── BoardConfig.h
    │   │   ├── config.h
    │   │   ├── nvs_config.h
    │   │   ├── nvs_config.cpp
    │   │   └── NvsConfigRepository.h
    │   ├── Gps/
    │   │   ├── TinyGpsPlusProvider.h
    │   │   └── TinyGpsPlusProvider.cpp
    │   ├── Gsm/
    │   │   ├── Sim7000AlertSender.h
    │   │   └── Sim7000AlertSender.cpp
    │   ├── Indicator/
    │   │   └── GpioIndicator.h
    │   ├── Logging/
    │   │   └── SerialLogger.h
    │   ├── Messaging/
    │   │   ├── mqtt_client.h
    │   │   ├── mqtt_client.cpp
    │   │   ├── Mqttv5Transport.h
    │   │   └── Mqttv5Transport.cpp
    │   ├── Net/
    │   │   ├── NetworkClient.h
    │   │   ├── NetworkClient.cpp
    │   │   ├── WifiLink.h
    │   │   └── WifiLink.cpp
    │   ├── Ota/
    │   │   ├── Mqttv5OtaService.h
    │   │   ├── ota_manager.h
    │   │   └── ota_manager.cpp
    │   ├── Provisioning/
    │   │   ├── ble_provisioning.h
    │   │   ├── ble_provisioning.cpp
    │   │   ├── ProvisioningPortals.h
    │   │   └── WifiApProvisioningPortal.h
    │   ├── Sensors/
    │   │   ├── Mpu6050Detector.h
    │   │   └── Mpu6050Detector.cpp
    │   ├── Serialization/
    │   │   ├── ArduinoJsonPayloadSerializer.h
    │   │   └── ArduinoJsonPayloadSerializer.cpp
    │   ├── System/
    │   │   ├── EspDeviceRepository.h
    │   │   ├── EspDeviceRepository.cpp
    │   │   └── EspSystemControl.h
    │   └── Time/
    │       ├── ArduinoClock.h
    │       ├── NtpTimeProvider.h
    │       └── NtpTimeProvider.cpp
    └── Presentation/
        └── main.cpp
```

---

## Hardware Specifications & Pinout

Default board wiring configured in `src/Infrastructure/Config/BoardConfig.h`:

| Peripheral | ESP32 GPIO | Peripheral Pin | Bus / Interface | Function |
|---|---|---|---|---|
| **GPS Module** (NEO-6M / NEO-8M) | `GPIO 16` | `TX` of GPS | HardwareSerial 2 (RX) | 9600 baud NMEA GPS sentence reception |
| **GPS Module** (NEO-6M / NEO-8M) | `GPIO 17` | `RX` of GPS | HardwareSerial 2 (TX) | 9600 baud GPS configuration transmission |
| **MPU6050 IMU** | `GPIO 21` | `SDA` | I2C Wire Bus (Data) | 400 kHz Fast-Mode 6-axis accelerometer & gyroscope |
| **MPU6050 IMU** | `GPIO 22` | `SCL` | I2C Wire Bus (Clock) | 400 kHz I2C Bus Clock |
| **GSM Modem** (SIM7000 / SIM800) | `GPIO 26` | `TX` of Modem | HardwareSerial 1 (RX) | Cellular modem AT communication |
| **GSM Modem** (SIM7000 / SIM800) | `GPIO 27` | `RX` of Modem | HardwareSerial 1 (TX) | Emergency SMS command dispatch |
| **Relay Actuator** | `GPIO 4` | Control In | Digital Output | Primary relay output (Active High) |
| **Push Button** | `GPIO 0` | Button Pin | Digital Input (`INPUT_PULLUP`) | Short press: Toggle relay / Long press: Provisioning |
| **Status LED** | `GPIO 2` | Anode | Digital Output | Non-blocking system & network state indicator |

---

## Motion Dynamics & Accident Detection Algorithm

The device executes a real-time motion and crash detection engine in `src/Application/AccidentAlgorithm.cpp` combining inertial sensing and GPS velocity cross-validation:

### STA/LTA Shock Impact Detection
Traditional impact thresholds (e.g. fixed `> 2.5g`) frequently produce false positives when vehicles drive over potholes, speed bumps, or rail crossings. 
The firmware implements the **Short-Term Average / Long-Term Average (STA/LTA)** ratio algorithm:
- **STA (Short-Term Average)**: Window of ~100 ms capturing sudden impulse energy.
- **LTA (Long-Term Average)**: Window of ~2000 ms capturing baseline vehicle vibration and road roughness.
- **Trigger**: When the ratio `STA / LTA` exceeds the sensitivity threshold (default: `ratio > 3.0` with absolute acceleration `total_g > 2.8g`), an **IMPACT** accident event is detected.

### Rollover & Tilt Angle Detection
Vehicle attitude is calculated continuously from the 3-axis accelerometer gravity vectors:
$$\text{Pitch} = \arctan2\left(a_x, \sqrt{a_y^2 + a_z^2}\right) \times \frac{180}{\pi}$$
$$\text{Roll} = \arctan2\left(a_y, \sqrt{a_x^2 + a_z^2}\right) \times \frac{180}{\pi}$$

When either $|\text{Roll}| > 60^\circ$ or $|\text{Pitch}| > 60^\circ$ persists beyond the debounce window, a **ROLLOVER** accident event is generated.

### Aggressive Driving Dynamics (Harsh Acceleration & Braking)
Forward and lateral vehicle kinematics are monitored to detect hazardous driving behavior:
- **Harsh Acceleration**: Detected when sustained longitudinal acceleration along the vehicle forward axis exceeds `+0.45g`.
- **Harsh Braking**: Detected when longitudinal deceleration exceeds `-0.55g`.

### Speed Filtering & GPS Cross-Validation
To prevent false alarms while the vehicle is stationary (such as when parked or during door slamming), high-severity alerts are cross-validated against GPS ground speed. If the vehicle is stationary (`speed_kmh < 3.0 km/h`), impact events are down-weighted unless accompanied by a sustained rollover tilt.

---

## GPS Telemetry & Navigation

The GPS driver (`TinyGpsPlusProvider`) continuously parses standard NMEA sentences (`$GPRMC`, `$GPGGA`, etc.) non-blockingly over HardwareSerial2:
- **Fix Acquisition**: Monitors satellite count and 3D fix state.
- **Precision Metrics**: Continuously reports HDOP (Horizontal Dilution of Precision).
- **Positioning Data**: Latitude, Longitude, Altitude (meters above sea level), Speed (km/h), and Course/Heading (degrees clockwise from True North).
- **Time Synchronization**: Extracts UTC atomic time to validate and supplement NTP time.

GPS telemetry packets are streamed periodically (default: every 10 seconds, or immediately upon significant heading/speed change) to the MQTT 5.0 broker under `telemetry/<chipId>/gps`.

---

## Emergency GSM / Cellular SMS Alerting

In severe road incidents, WiFi connectivity may be unavailable. The `Sim7000AlertSender` provides a critical hardware redundancy link:
1. **Trigger**: When an `IMPACT` or `ROLLOVER` event occurs, the `AccidentService` dispatches an alert payload to the cellular modem via AT commands.
2. **Message Content**: Generates an emergency SOS SMS formatted with:
   - Alert type (`CRASH / IMPACT` or `VEHICLE ROLLOVER`)
   - Peak G-force and tilt angles
   - Direct clickable **Google Maps link** (`https://maps.google.com/?q=<lat>,<lon>`)
   - Vehicle Speed and UTC timestamp
3. **Transmission**: Dispatched over HardwareSerial1 directly to configured emergency phone numbers.

---

## Dependencies

Linked and managed automatically via PlatformIO:

- **[ESP32_MQTTv5](https://github.com/JorgeGBeltre/ESP32_MQTTv5)** (`^1.0.3`): Native MQTT 5.0 client library for ESP32.
- **[MQTTOTAv5](https://github.com/JorgeGBeltre/MQTTOTAv5)** (`^1.0.2`): Over-The-Air firmware updater over MQTT 5.0.
- **[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)** (`^1.4.1`): High-performance, lightweight Bluetooth Low Energy stack.
- **[TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)** (`^1.1.0`): High-performance NMEA GPS parser.
- **[MPU6050](https://github.com/ElectronicCats/mpu6050)** (`^1.4.5`): 6-axis IMU driver with I2Cdevlib.
- **[ArduinoJson](https://arduinojson.org/)** (`^6.21.6`): Memory-efficient JSON serialization and parsing.
- **[WiFiManager](https://github.com/tzapu/WiFiManager)** (`^2.0.17`): SoftAP web portal for runtime credentials provisioning.
- **Arduino-ESP32 Core** (`>= 2.0.0`): Official Espressif framework for ESP32.

---

## Configuration & Placeholders

Default fallback credentials and endpoint settings are organized in `src/Infrastructure/Config/config.h`:

```cpp
// src/Infrastructure/Config/config.h

constexpr const char* MQTT_SERVER = "YOUR_MQTT_SERVER_HERE";
constexpr int MQTT_PORT = 8883;
constexpr const char* API_KEY = "YOUR_API_KEY_HERE";
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 10000;
constexpr int MQTT_SOCKET_TIMEOUT = 30;
constexpr size_t MQTT_BUFFER_SIZE = 4096;
constexpr unsigned long MQTT_KEEP_ALIVE = 60;
```

> **Note**: Dynamic credentials provided via BLE Provisioning or SoftAP are stored non-volatilely in NVS (`relay-config`) and take precedence over static defaults in `config.h`.

---

## Bluetooth BLE Provisioning & Cellular APN Configuration

The device implements an on-demand Bluetooth Low Energy (BLE) provisioning system powered by `NimBLE-Arduino`, advertising as `GPS-<DeviceID>`.

### Overview & Workflow

```
+--------------------+                    +---------------------+                    +----------------------+
|  Mobile App (BLE)  |                    |  VESIS GPS (ESP32)  |                    |   NVS Flash Memory   |
+--------------------+                    +---------------------+                    +----------------------+
          |                                         |                                          |
          | --- 1. Scan ('GPS-<DeviceID>') -------> |                                          |
          | --- 2. Connect to Service UUID -------> |                                          |
          |                                         |                                          |
          | <-- 3. Read Identity / JSON Frame ----- | (Hardware authenticity verified)         |
          |                                         |                                          |
          | --- 4. Write APN, WiFi, Broker creds -> |                                          |
          |        (Per-characteristic or JSON)     |                                          |
          |                                         |                                          |
          | --- 5. Write to CHAR_SAVE ------------> | --- Commit to NVS ---------------------> |
          |                                         |                                          |
          | <-- 6. ACK Notification ('OK') -------- |                                          |
          |                                         | --- Reboot into operational mode ------> |
```

### GATT Service & Characteristics Layout

- **Service UUID**: `12345678-1234-1234-1234-123456789abc`
- **Advertisement Name**: `GPS-<DeviceID>` (e.g. `GPS-0000A1B2C3D4`)

| Characteristic | UUID | Permissions | Description | Example / Default Value |
|---|---|---|---|---|
| **Cellular APN** | `aaaaaaaa-bbbb-cccc-dddd-000000000001` | `READ` / `WRITE` | Cellular Access Point Name for GSM/LTE modem | `"internet.claro.com"` |
| **APN Username** | `aaaaaaaa-bbbb-cccc-dddd-000000000002` | `READ` / `WRITE` | Cellular APN username | `""` or `"claro"` |
| **APN Password** | `aaaaaaaa-bbbb-cccc-dddd-000000000003` | `READ` / `WRITE` | Cellular APN password | `""` or `"claro"` |
| **Cloud Broker** | `aaaaaaaa-bbbb-cccc-dddd-000000000004` | `READ` / `WRITE` | MQTT 5.0 broker endpoint hostname/IP | `"broker.hivemq.com"` |
| **Broker Port** | `aaaaaaaa-bbbb-cccc-dddd-000000000005` | `READ` / `WRITE` | TCP/TLS broker port | `8883` (TLS) or `1883` |
| **User / Account** | `aaaaaaaa-bbbb-cccc-dddd-000000000006` | `READ` / `WRITE` | User account / registered email / MQTT user | `"user@example.com"` |
| **MQTT Password** | `aaaaaaaa-bbbb-cccc-dddd-000000000007` | `READ` / `WRITE` | MQTT broker password | `""` |
| **Device ID** | `aaaaaaaa-bbbb-cccc-dddd-000000000008` | `READ` / `WRITE` | Unique hardware identifier presented to cloud | `0000A1B2C3D4` |
| **OTA HMAC Key** | `aaaaaaaa-bbbb-cccc-dddd-000000000011` | `READ` / `WRITE` | HMAC signature key for OTA verification | `""` |
| **WiFi SSID** | `aaaaaaaa-bbbb-cccc-dddd-000000000012` | `READ` / `WRITE` | 2.4 GHz wireless network SSID | `"Fleet_Depot_WiFi"` |
| **WiFi Password** | `aaaaaaaa-bbbb-cccc-dddd-000000000013` | `READ` / `WRITE` | WPA/WPA2 passphrase | `"SecretPassword123"` |
| **JSON / Data Frame** | `aaaaaaaa-bbbb-cccc-dddd-000000000014` | `READ` / `WRITE` / `NOTIFY` | Unified frame channel (read identity; write JSON/Key-Value) | See JSON frame format |
| **Device Key** | `aaaaaaaa-bbbb-cccc-dddd-000000000015` | `READ` | Factory private hardware key (`apikey`) | Unique cryptographic UUID |
| **Bind Token** | `aaaaaaaa-bbbb-cccc-dddd-000000000016` | `READ` / `WRITE` | User account claim token from mobile app | `"claim_tok_9876543210"` |
| **Save / ACK** | `aaaaaaaa-bbbb-cccc-dddd-000000000099` | `WRITE` / `READ` / `NOTIFY` | Save trigger into NVS and ACK notification (`"OK"`) | `"READY"` -> Notifies `"OK"` |

### Standard Bluetooth Device Information Service (`0x180A`)

| Standard Characteristic | SIG UUID | Permissions | Returned Value |
|---|---|---|---|
| **Model Number String** | `0x2A24` | `READ` | `"VESIS GPS-Tracker ESP32"` |
| **Serial Number String** | `0x2A25` | `READ` | `<DeviceID>` (e.g. `0000A1B2C3D4`) |
| **Firmware Revision** | `0x2A26` | `READ` | Firmware version (e.g. `"v0.0.8"`) |
| **Hardware Revision** | `0x2A27` | `READ` | `"ESP32"` |
| **Manufacturer Name** | `0x2A29` | `READ` | `"VESIS"` |

### Provisioning via Unified Data Frames (JSON & Key=Value)

#### Option A: JSON Frame (Including APN)

Written to `CHAR_FRAME_JSON_UUID` (`...0014`) or `CHAR_SAVE_UUID` (`...0099`):

```json
{
  "apn": "internet.claro.com",
  "apn_user": "",
  "apn_pass": "",
  "ssid": "Fleet_Depot_WiFi",
  "password": "MySuperSecretPassword",
  "server": "broker.hivemq.com",
  "port": 8883,
  "account": "user@example.com",
  "bind_token": "claim_tok_9876543210abcdef",
  "device_id": "GPS-0000A1B2C3D4",
  "ota_key": "s3cr3t_k3y"
}
```

*Accepted JSON key aliases:*
- `apn`: `"apn"`
- `apn_user`: `"apn_user"` or `"apnuser"`
- `apn_pass`: `"apn_pass"` or `"apnpass"`
- `ssid`: `"wifi_ssid"` or `"ssid"`
- `password`: `"wifi_pass"`, `"pass"`, or `"password"`
- `server`: `"mqtt_broker"`, `"broker"`, or `"server"`
- `port`: `"mqtt_port"` or `"port"`
- `account`: `"account"`, `"user"`, `"user_id"`, `"email"`, or `"mqtt_user"`
- `bind_token`: `"bind_token"`, `"token"`, or `"claim_token"`

#### Option B: Delimited Key=Value Frame

```text
apn=internet.claro.com;ssid=Fleet_Depot_WiFi;pass=SecretPassword;server=broker.hivemq.com;port=8883;account=user@example.com;token=claim_tok_12345
```

### Dual Provisioning Flow & Fallback Architecture

1. **Unprovisioned First Boot**: The device enters BLE advertising as `GPS-<DeviceID>` for 60 seconds. If no mobile app connects, it automatically falls back to the SoftAP captive web portal:
   - **SSID**: `GPS-<ChipID>`
   - **Password**: `12345678`
   - **Web Portal**: Automatically serves captive form with APN, APN User, APN Password, WiFi SSID, and WiFi Password fields.
2. **Button Long-Press (> 3000 ms)**: Holding GPIO 0 triggers the BLE provisioning mode at any time.
3. **Boot Hold**: Powering up while holding GPIO 0 forces immediate entry into provisioning mode.

---

## MQTT 5.0 Messaging & Topics

### Topic Topology

Topics are dynamically derived at runtime using the chip's unique eFuse MAC address:

| Topic | Direction | Description |
|---|---|---|
| `telemetry/<ChipID>/gps` | Outbound | Continuous high-precision GPS positioning and kinematics |
| `event/<ChipID>/alerts` | Outbound | High-priority safety events (Impacts, Rollovers, Harsh Driving) |
| `status` | Outbound | Periodic system metrics and relay status report; also MQTT 5.0 LWT topic |
| `events` | Outbound | System lifecycle events (`MQTT_CONNECTED`, `BUTTON_PRESS`, etc.) |
| `telemetry` | Outbound | System health snapshot (heap, uptime, RSSI) |
| `commands` | Inbound | Control commands (`reset`, `status`, `toggle_all`, `switch1:...`) |
| `device_switch1/state` | Inbound | Turn relay on/off/toggle via MQTT payload (`on`, `off`, `toggle`) |
| `device_switch1_state` | Inbound/Outbound | Desired state from cloud / Confirmed state from device |
| `ota` | Inbound | Handled automatically by `MQTTOTAv5` |

### GPS Telemetry Payloads

Published periodically to `telemetry/<ChipID>/gps`:

```json
{
  "chip_id": "0000A1B2C3D4",
  "timestamp": "2026-10-28T05:00:00.000Z",
  "gps": {
    "valid": true,
    "lat": 18.486058,
    "lng": -69.931211,
    "altitude_m": 42.5,
    "speed_kmh": 48.2,
    "course_deg": 135.4,
    "hdop": 0.9,
    "satellites": 9
  }
}
```

### Accident & Safety Alert Payloads

Published immediately with QoS 1 to `event/<ChipID>/alerts`:

```json
{
  "event_type": "IMPACT",
  "severity": "CRITICAL",
  "timestamp": "2026-10-28T05:01:23.450Z",
  "metrics": {
    "sta_lta_ratio": 4.82,
    "peak_g": 3.45,
    "pitch_deg": 12.4,
    "roll_deg": -8.1,
    "speed_kmh": 62.1
  },
  "location": {
    "lat": 18.486120,
    "lng": -69.931450,
    "maps_url": "https://maps.google.com/?q=18.486120,-69.931450"
  },
  "sms_dispatched": true
}
```

### Device Status & Actuator Payloads

Published to `status`:

```json
{
  "Device": {
    "status": "connected",
    "ChipId": "0000A1B2C3D4",
    "MacAddress": "24:6F:28:XX:XX:XX",
    "ChipType": "ESP32-D0WDQ6",
    "FirmwareVersion": "v0.0.8"
  },
  "Timestamp": "2026-10-28T05:00:00.000Z",
  "Details": {
    "chipId": "0000A1B2C3D4",
    "sync_completed": true,
    "devices_count": 1,
    "wifi_status": "connected",
    "rssi": -58,
    "free_heap": 178240,
    "uptime": 145020,
    "devices": {
      "0000A1B2C3D4-01": {
        "state": "on",
        "desired_state": "on",
        "pin": 4
      }
    }
  }
}
```

### Inbound Command Handling

Sending strings to `commands`:
- `"reset"`: Triggers a clean system restart via `ESP.restart()`.
- `"status"`: Forces immediate publication of the full device status JSON.
- `"toggle_all"`: Inverts all active relays.
- `"switch1:on"`, `"switch1:off"`, `"switch1:toggle"`: Directly controls Relay 1.

---

## Over-The-Air (OTA) Updates

Over-The-Air updates are fully handled by **`MQTTOTAv5`**:
1. **Chunked Delivery**: Firmware binaries are delivered in discrete chunks over MQTT 5.0 and streamed directly to the standby OTA partition (`app0` or `app1`).
2. **Cryptographic Integrity**: SHA-256 HMAC hash is checked against metadata transmitted in MQTT 5.0 User Properties.
3. **Rollback Protection**: Uses native ESP32 dual OTA boot partitions (`esp_ota_ops.h`) to automatically roll back to the previous stable firmware if the new image fails to boot.
4. **Non-blocking Execution**: Managed via `ota.handle()` inside `loop()` without interrupting GPS parsing or sensor sampling.

---

## Physical Controls & Status Indicator (LED)

### Push Button (GPIO 0)
- **Short Press (< 3000 ms)**: Toggles Relay 1 locally, emits a `"BUTTON_PRESS"` event to `events`, and publishes the confirmed state (`"on"`/`"off"`) to `device_switch1_state`.
- **Long Press (> 3000 ms)**: Launches the **BLE Provisioning Portal** (advertising as `GPS-<DeviceID>` for 60 seconds with rapid LED blinking). If timeout expires without receiving configuration, automatically falls back to the SoftAP captive portal (`GPS-<ChipID>`).

### Status LED (GPIO 2)
- **Fast Blink (100 ms interval)**: Disconnected from WiFi / BLE Provisioning active / Searching for network.
- **Slow Blink (400 ms interval)**: WiFi connected, attempting connection to MQTT 5.0 broker.
- **Solid ON**: Fully operational (WiFi and MQTT 5.0 connected).

---

## Build & Flashing Instructions

### Using PlatformIO

Ensure [PlatformIO Core](https://platformio.org/install/cli) is installed.

1. **Compile Firmware**:
   ```bash
   pio run
   ```

2. **Upload to ESP32**:
   ```bash
   pio run -t upload
   ```

3. **Monitor Serial Output (115200 baud)**:
   ```bash
   pio run -t monitor
   ```

### Dual OTA Partition Scheme

The firmware relies on `partitions.csv` configured for two 1.9 MB application slots:

```csv
nvs,      data, nvs,     0x9000,   0x5000,
otadata,  data, ota,     0xe000,   0x2000,
app0,     app,  ota_0,   0x10000,  0x1E0000,
app1,     app,  ota_1,   0x1F0000, 0x1E0000,
spiffs,   data, spiffs,  0x3D0000, 0x30000,
```

### Host Compilation with CMake

The Domain and Application layers can be compiled on a host machine (Linux, macOS, or Windows) for testing without ESP32 toolchains:

```bash
mkdir build && cd build
cmake .. -DHOST_BUILD=ON
cmake --build .
```

---

## Backend Implementation & Broker Compatibility

To handle MQTT 5.0 telemetry and OTA firmware updates, deploy an MQTT 5.0 compliant broker:
- **[Mosquitto](https://mosquitto.org/) 2.0+** (with MQTT 5.0 enabled)
- **[EMQX](https://www.emqx.com/) 5.0+**
- **[HiveMQ Cloud](https://www.hivemq.com/)** (MQTT 5.0 tier)
- **[AWS IoT Core](https://aws.amazon.com/iot-core/)**

Reference broker backend repository:
- **Repository**: [github.com/Ruben890/Mqtt-Broker](https://github.com/Ruben890/Mqtt-Broker)

---

## License

This library is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

## Contact

Author: **Jorge Gaspar Beltre Rivera**  
Project: VESIS GPS-ESP32

<p align="center">
  <a href="https://www.linkedin.com/in/jorge-gaspar-beltre-rivera/" target="_blank"><img src="https://user-images.githubusercontent.com/74038190/235294012-0a55e343-37ad-4b0f-924f-c8431d9d2483.gif" alt="LinkedIn" width="100"></a>
  <a href="https://github.com/JorgeGBeltre" target="_blank"><img src="https://user-images.githubusercontent.com/74038190/212257468-1e9a91f1-b626-4baa-b15d-5c385dfa7ed2.gif" alt="GitHub" width="100"></a>
  <a href="mailto:Jorgegaspar3021@gmail.com"><img src="https://user-images.githubusercontent.com/74038190/216122065-2f028bae-25d6-4a3c-bc9f-175394ed5011.png" alt="E-Mail" width="100"></a>
</p>

## Support

This project is developed independently. Even a small contribution helps me dedicate more time to development, testing, and releasing new features.

<p align="center">
  <a href="https://www.paypal.com/donate/?hosted_button_id=2VLA8BWT967LU">
    <img src="https://www.paypalobjects.com/webstatic/icon/pp258.png"
         alt="Donate with PayPal"
         height="60">
  </a>
</p>
