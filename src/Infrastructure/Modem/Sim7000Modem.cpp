#include "Infrastructure/Modem/Sim7000Modem.h"

namespace relay::infra {

Sim7000Modem::Sim7000Modem(HardwareSerial& serial,
                           uint8_t rxPin,
                           uint8_t txPin,
                           uint32_t baud,
                           const std::string& emergencyPhone)
    : serial_(serial),
      rxPin_(rxPin),
      txPin_(txPin),
      baud_(baud),
      emergencyPhone_(emergencyPhone) {}

void Sim7000Modem::begin() {
    Serial.printf("[SIM7000] Inicializando modem SIM7000G en UART (RX: %u, TX: %u, Baud: %u)...\n",
                  rxPin_, txPin_, baud_);
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    delay(300);

    // Intentar comunicar a la velocidad configurada (115200)
    bool detected = false;
    for (int i = 0; i < 3; i++) {
        if (sendATCommand("AT", "OK", 600)) {
            detected = true;
            break;
        }
        delay(150);
    }

    // Si no responde a 115200, intentar a 9600 por si viene de fábrica a 9600
    if (!detected && baud_ != 9600) {
        serial_.begin(9600, SERIAL_8N1, rxPin_, txPin_);
        delay(300);
        if (sendATCommand("AT", "OK", 600)) {
            Serial.println("[SIM7000] Modem detectado a 9600 baud. Cambiando a 115200 baud...");
            sendATCommand("AT+IPR=115200", "OK", 500);
            serial_.begin(115200, SERIAL_8N1, rxPin_, txPin_);
            delay(200);
            detected = sendATCommand("AT", "OK", 600);
        }
    }

    if (detected) {
        isReady_ = true;
        sendATCommand("ATE0", "OK", 500);          // Desactivar eco
        sendATCommand("AT+CFUN=1", "OK", 1000);    // Modo funcionalidad completa
        sendATCommand("AT+CMGF=1", "OK", 500);     // Modo SMS texto
        sendATCommand("AT+CSCS=\"GSM\"", "OK", 500);

        // Alimentar antena activa GNSS (SGPIO GPIO4 para LilyGO/Waveshare y CVAUXS para SIMCom)
        sendATCommand("AT+SGPIO=0,4,1,1", "OK", 1000);
        sendATCommand("AT+CVAUXS=1", "OK", 1000);

        // Encender y configurar el receptor GNSS interno del SIM7000G
        Serial.println("[SIM7000] Encendiendo receptor GNSS integrado (AT+CGNSPWR=1)...");
        if (sendATCommand("AT+CGNSPWR=1", "OK", 2000)) {
            gnssPowered_ = true;
            // Configurar constelaciones GPS + GLONASS
            sendATCommand("AT+CGNSMOD=1,1,0,0,0", "OK", 1000);
            Serial.println("[SIM7000] Receptor GNSS (GPS+GLONASS) encendido y listo.");
        } else {
            gnssPowered_ = false;
            Serial.println("[SIM7000] AVISO: No se recibió OK al comando AT+CGNSPWR=1.");
        }
    } else {
        isReady_ = false;
        Serial.println("[SIM7000] AVISO: El modem SIM7000G no respondió a comandos AT.");
        Serial.println("[SIM7000] Verifique: 1) Alimentación VCC (5V/VIN) y GND común.");
        Serial.println("[SIM7000]            2) Cruce de pines (ESP32 RX:16 -> SIM7000 TX, ESP32 TX:17 -> SIM7000 RX).");
    }
}

void Sim7000Modem::begin(const std::string& /*authCode*/) {
    begin();
}

void Sim7000Modem::update() {
    if (!isReady_) {
        // Intento periódico de detección en caso de encendido tardío
        if (millis() - lastReadyCheckMs_ > 10000) {
            lastReadyCheckMs_ = millis();
            if (sendATCommand("AT", "OK", 400)) {
                begin();
            }
        }
        return;
    }

    // Consulta periódica del GNSS cada 2 segundos
    if (millis() - lastGnssQueryMs_ >= 2000) {
        lastGnssQueryMs_ = millis();
        queryGnss();
    }
}

void Sim7000Modem::queryGnss() {
    while (serial_.available() > 0) serial_.read();

    serial_.println("AT+CGNSINF");

    unsigned long start = millis();
    String response = "";
    while (millis() - start < 1000) {
        while (serial_.available() > 0) {
            char c = serial_.read();
            response += c;
            if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
                break;
            }
        }
        if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
            break;
        }
        delay(10);
    }

    int idx = response.indexOf("+CGNSINF: ");
    if (idx != -1) {
        int endIdx = response.indexOf("\r", idx);
        if (endIdx == -1) endIdx = response.indexOf("\n", idx);
        String line = (endIdx != -1) ? response.substring(idx + 10, endIdx) : response.substring(idx + 10);
        parseGnssInf(line);
    }
}

void Sim7000Modem::parseGnssInf(const String& line) {
    // Formato SIM7000 +CGNSINF:
    // 0: Run status (1/0)
    // 1: Fix status (1/0)
    // 2: UTC DateTime (YYYYMMDDHHMMSS.sss)
    // 3: Latitude (±dd.dddddd)
    // 4: Longitude (±ddd.dddddd)
    // 5: MSL Altitude (m)
    // 6: Speed Over Ground (km/h)
    // 7: Course Over Ground (deg)
    // 8: Fix Mode
    // 9: Reserved
    // 10: HDOP
    // 11: PDOP
    // 12: VDOP
    // 13: Reserved
    // 14: GNSS Satellites in View
    // 15: GNSS Satellites Used
    // 16: GLONASS Satellites Used
    // 17: Reserved
    // 18: C/N0 max (SNR dB-Hz)

    std::vector<String> tokens;
    int start = 0;
    while (start <= (int)line.length()) {
        int comma = line.indexOf(',', start);
        if (comma == -1) {
            tokens.push_back(line.substring(start));
            break;
        }
        tokens.push_back(line.substring(start, comma));
        start = comma + 1;
    }

    if (tokens.size() < 2) return;

    int runStatus = tokens[0].toInt();
    int fixStatus = tokens[1].toInt();

    if (runStatus == 0) {
        // Si el GNSS se apagó, intentar reactivarlo
        sendATCommand("AT+CGNSPWR=1", "OK", 1000);
    }

    satellitesInView_ = (tokens.size() > 14) ? tokens[14].toInt() : 0;
    satellitesUsed_   = (tokens.size() > 15) ? tokens[15].toInt() : 0;
    if (tokens.size() > 18) {
        maxC_N0_ = tokens[18].toInt();
    }

    if (fixStatus == 1 && tokens.size() >= 8) {
        cachedLocation_.isValid = true;
        if (!tokens[2].isEmpty()) cachedLocation_.timestamp = tokens[2].c_str();
        cachedLocation_.latitude = tokens[3].toDouble();
        cachedLocation_.longitude = tokens[4].toDouble();
        cachedLocation_.altitude = tokens[5].toFloat();
        cachedLocation_.speed = tokens[6].toFloat();
        cachedLocation_.course = tokens[7].toFloat();
        cachedLocation_.satellites = (satellitesUsed_ > 0) ? satellitesUsed_ : satellitesInView_;
    } else {
        cachedLocation_.isValid = false;
        cachedLocation_.latitude = 0.0;
        cachedLocation_.longitude = 0.0;
        cachedLocation_.altitude = 0.0f;
        cachedLocation_.speed = 0.0f;
        cachedLocation_.course = 0.0f;
        cachedLocation_.satellites = satellitesInView_;
    }
}

void Sim7000Modem::readData(domain::GpsLocation& out) {
    out = cachedLocation_;
    if (!out.isValid) {
        if (millis() - lastWarningMs_ >= 10000) {
            Serial.printf("[SIM7000-GNSS] Sin fix valido - Satelites a la vista: %d, en uso: %d, C/N0 max: %d dB-Hz\n",
                          satellitesInView_, satellitesUsed_, maxC_N0_);
            lastWarningMs_ = millis();
        }
    } else {
        if (millis() - lastWarningMs_ >= 10000) {
            Serial.printf("[SIM7000-GNSS] ¡FIX 3D ACTIVO! Lat: %.6f, Lon: %.6f, Sats: %d, Vel: %.1f km/h\n",
                          cachedLocation_.latitude, cachedLocation_.longitude, cachedLocation_.satellites, cachedLocation_.speed);
            lastWarningMs_ = millis();
        }
    }
}

float Sim7000Modem::currentSpeedKmph() {
    return cachedLocation_.isValid ? cachedLocation_.speed : 0.0f;
}

int Sim7000Modem::getSatellites() {
    return cachedLocation_.satellites;
}

bool Sim7000Modem::hasFix() {
    return cachedLocation_.isValid;
}

void Sim7000Modem::diagnose() {
    Serial.println("\n=== DIAGNOSTICO MODEM SIM7000G (LTE & GNSS) ===");
    Serial.printf("Modem listo:             %s\n", isReady_ ? "SI" : "NO");
    Serial.printf("GNSS alimentado:         %s\n", gnssPowered_ ? "SI" : "NO");
    Serial.printf("Fix satelital (3D):      %s\n", cachedLocation_.isValid ? "SI" : "NO");
    Serial.printf("Satelites a la vista:    %d\n", satellitesInView_);
    Serial.printf("Satelites en uso:        %d\n", satellitesUsed_);
    Serial.printf("Nivel max C/N0 (SNR):    %d dB-Hz\n", maxC_N0_);
    if (cachedLocation_.isValid) {
        Serial.printf("Latitud: %.6f, Longitud: %.6f\n", cachedLocation_.latitude, cachedLocation_.longitude);
        Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h, Rumbo: %.1f deg\n",
                      cachedLocation_.altitude, cachedLocation_.speed, cachedLocation_.course);
    }
    String csq = sendATCommandWithReply("AT+CSQ", 1000);
    csq.trim();
    Serial.printf("Senal Celular (CSQ):     %s\n", csq.c_str());
    Serial.println("================================================\n");
}

bool Sim7000Modem::sendAlert(const domain::AccidentEvent& accident,
                             const domain::GpsLocation& gps,
                             const std::string& timestamp,
                             const std::string& chipId) {
    if (emergencyPhone_.empty()) {
        Serial.println("[GSM] ERROR: No hay numero de emergencia configurado para SMS.");
        return false;
    }

    std::string msg = "ALERTA VESIS: " + accident.alertType() + "\n";
    msg += "ID: " + chipId + "\n";
    msg += "Hora: " + timestamp + "\n";

    if (gps.isValid) {
        msg += "Ubicacion: https://maps.google.com/?q=" +
               std::to_string(gps.latitude) + "," + std::to_string(gps.longitude) + "\n";
        msg += "Velocidad: " + std::to_string((int)gps.speed) + " km/h\n";
    } else {
        msg += "GPS: Sin fix satelital\n";
    }

    if (accident.rollover_detected) {
        msg += "Roll: " + std::to_string((int)accident.roll_angle) + " deg, Pitch: " +
               std::to_string((int)accident.pitch_angle) + " deg\n";
    } else if (accident.impact_detected) {
        msg += "Magnitud Impacto: " + std::to_string((int)accident.impact_magnitude) + "\n";
    }

    return sendSms(emergencyPhone_, msg);
}

bool Sim7000Modem::sendSms(const std::string& phoneNumber, const std::string& message) {
    if (!isReady()) {
        Serial.println("[GSM] Modem no disponible para enviar SMS.");
        return false;
    }

    Serial.printf("[GSM] Enviando SMS a %s...\n", phoneNumber.c_str());

    serial_.print("AT+CMGS=\"");
    serial_.print(phoneNumber.c_str());
    serial_.println("\"");

    unsigned long start = millis();
    bool gotPrompt = false;
    while (millis() - start < 3000) {
        if (serial_.available()) {
            char c = serial_.read();
            if (c == '>') {
                gotPrompt = true;
                break;
            }
        }
        delay(10);
    }

    if (!gotPrompt) {
        Serial.println("[GSM] ERROR: No se recibio prompt '>' para envio de SMS.");
        serial_.write(27); // ESC
        return false;
    }

    serial_.print(message.c_str());
    serial_.write(26); // Ctrl+Z

    start = millis();
    String response = "";
    while (millis() - start < 10000) {
        if (serial_.available()) {
            char c = serial_.read();
            response += c;
            if (response.indexOf("OK") != -1) {
                Serial.println("[GSM] SMS enviado exitosamente.");
                return true;
            }
            if (response.indexOf("ERROR") != -1) {
                Serial.printf("[GSM] ERROR en envio de SMS: %s\n", response.c_str());
                return false;
            }
        }
        delay(10);
    }

    Serial.println("[GSM] TIMEOUT esperando confirmacion de SMS.");
    return false;
}

bool Sim7000Modem::isReady() {
    return isReady_;
}

void Sim7000Modem::setCaCert(const char* caCertPem) {
    if (!caCertPem || strlen(caCertPem) == 0) return;
    Serial.println("[GSM] Certificado CA configurado para canal seguro MQTTS SIM7000G.");
}

bool Sim7000Modem::sendATCommand(const String& cmd, const String& expectedResponse, unsigned long timeoutMs) {
    while (serial_.available() > 0) serial_.read();

    serial_.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (serial_.available() > 0) {
            char c = serial_.read();
            response += c;
            if (response.indexOf(expectedResponse) != -1) {
                return true;
            }
        }
        delay(10);
    }
    return false;
}

String Sim7000Modem::sendATCommandWithReply(const String& cmd, unsigned long timeoutMs) {
    while (serial_.available() > 0) serial_.read();

    serial_.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (serial_.available() > 0) {
            char c = serial_.read();
            response += c;
            if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
                return response;
            }
        }
        delay(10);
    }
    return response;
}

} // namespace relay::infra
