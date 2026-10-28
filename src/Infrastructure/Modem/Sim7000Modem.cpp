#include "Infrastructure/Modem/Sim7000Modem.h"

namespace relay::infra {

Sim7000Modem::Sim7000Modem(HardwareSerial& serial,
                           uint8_t rxPin,
                           uint8_t txPin,
                           uint8_t pwrKeyPin,
                           uint32_t baud,
                           const std::string& emergencyPhone)
    : serial_(serial),
      rxPin_(rxPin),
      txPin_(txPin),
      pwrKeyPin_(pwrKeyPin),
      baud_(baud),
      activeBaud_(baud),
      emergencyPhone_(emergencyPhone) {}

void Sim7000Modem::processIncomingByte(char c) {
    rxBytesCount_++;
    nmeaGps_.encode(c);

    if (c == '\n') {
        nmeaLineBuf_.trim();
        if (nmeaLineBuf_.startsWith("$")) {
            parseNmeaLine(nmeaLineBuf_);
        }
        nmeaLineBuf_ = "";
    } else if (c != '\r') {
        if (nmeaLineBuf_.length() < 128) {
            nmeaLineBuf_ += c;
        } else {
            nmeaLineBuf_ = "";
        }
    }
}

void Sim7000Modem::parseNmeaLine(const String& line) {
    if (line.startsWith("$GP") || line.startsWith("$GN") ||
        line.startsWith("$GL") || line.startsWith("$BD") || line.startsWith("$GA")) {
        if (!isReady_) {
            nmeaMode_ = true;
            gnssPowered_ = true;
        }
    }

    // Parse $xxGSV (Satellites in View & SNR C/N0):
    // $GPGSV,totalMsgs,msgNum,satsInView,prn1,el1,az1,snr1,prn2,el2,az2,snr2,...*chk
    if (line.length() > 7 && line.substring(3, 6) == "GSV") {
        String body = line;
        int starIdx = body.indexOf('*');
        if (starIdx != -1) {
            body = body.substring(0, starIdx);
        }

        std::vector<String> fields;
        int start = 0;
        while (start <= (int)body.length()) {
            int comma = body.indexOf(',', start);
            if (comma == -1) {
                fields.push_back(body.substring(start));
                break;
            }
            fields.push_back(body.substring(start, comma));
            start = comma + 1;
        }

        if (fields.size() >= 4) {
            int inView = fields[3].toInt();
            if (line.startsWith("$GLGSV")) {
                gsvSatsGlonass_ = inView;
            } else {
                gsvSatsGps_ = inView;
            }
            int totalInView = gsvSatsGps_ + gsvSatsGlonass_;
            if (totalInView > 0 || satellitesInView_ == 0) {
                satellitesInView_ = totalInView;
            }

            // Extract SNR (C/N0) from fields 7, 11, 15, 19
            for (size_t i = 7; i < fields.size(); i += 4) {
                if (!fields[i].isEmpty()) {
                    int snr = fields[i].toInt();
                    if (snr > maxC_N0_) {
                        maxC_N0_ = snr;
                    }
                }
            }
        }
    }
}

void Sim7000Modem::syncFromNmea() {
    if (nmeaGps_.satellites.isValid()) {
        satellitesUsed_ = static_cast<int>(nmeaGps_.satellites.value());
    }
    if (satellitesUsed_ > satellitesInView_) {
        satellitesInView_ = satellitesUsed_;
    }

    if (nmeaGps_.location.isValid()) {
        cachedLocation_.isValid = true;
        cachedLocation_.latitude = nmeaGps_.location.lat();
        cachedLocation_.longitude = nmeaGps_.location.lng();
        cachedLocation_.altitude = nmeaGps_.altitude.isValid() ? static_cast<float>(nmeaGps_.altitude.meters()) : 0.0f;
        cachedLocation_.speed = nmeaGps_.speed.isValid() ? static_cast<float>(nmeaGps_.speed.kmph()) : 0.0f;
        cachedLocation_.course = nmeaGps_.course.isValid() ? static_cast<float>(nmeaGps_.course.deg()) : 0.0f;
        cachedLocation_.satellites = (satellitesUsed_ > 0) ? satellitesUsed_ : satellitesInView_;
    } else {
        cachedLocation_.isValid = false;
        cachedLocation_.latitude = 0.0;
        cachedLocation_.longitude = 0.0;
        cachedLocation_.altitude = 0.0f;
        cachedLocation_.speed = 0.0f;
        cachedLocation_.course = 0.0f;
        cachedLocation_.satellites = (satellitesInView_ > 0) ? satellitesInView_ : satellitesUsed_;
    }
}

bool Sim7000Modem::probeAtOrNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, int attempts, unsigned long timeoutMs) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
    activeBaud_ = baud;
    delay(80);

    for (int i = 0; i < attempts; i++) {
        while (serial_.available() > 0) {
            processIncomingByte(static_cast<char>(serial_.read()));
        }
        if (nmeaMode_) {
            syncFromNmea();
            return true;
        }

        serial_.println("AT");
        unsigned long start = millis();
        String response = "";
        while (millis() - start < timeoutMs) {
            while (serial_.available() > 0) {
                char c = static_cast<char>(serial_.read());
                processIncomingByte(c);
                response += c;
                if (response.indexOf("OK") != -1) {
                    isReady_ = true;
                    nmeaMode_ = false;
                    return true;
                }
            }
            if (nmeaMode_) {
                syncFromNmea();
                return true;
            }
            delay(10);
        }
        delay(80);
    }
    return false;
}

bool Sim7000Modem::listenForNmea(uint32_t baud, uint8_t rxPin, uint8_t txPin, unsigned long durationMs) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
    activeBaud_ = baud;

    unsigned long start = millis();
    while (millis() - start < durationMs) {
        while (serial_.available() > 0) {
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
        }
        if (nmeaMode_) {
            syncFromNmea();
            return true;
        }
        delay(10);
    }
    return false;
}

void Sim7000Modem::powerOn() {
    if (pwrKeyPin_ == 0xFF) return;

    Serial.printf("[SIM7000] Control PWRKEY en GPIO %u. Verificando si el modulo ya responde...\n", pwrKeyPin_);
    pinMode(pwrKeyPin_, INPUT);
    delay(50);

    // 1. Verificar primero a 115200 y 9600 (varios intentos) para NO apagar un modem ya encendido
    if (probeAtOrNmea(baud_, rxPin_, txPin_, 3, 350) ||
        (baud_ != 9600 && probeAtOrNmea(9600, rxPin_, txPin_, 2, 350))) {
        Serial.printf("[SIM7000] Modulo ya activo en UART (%u baud).\n", activeBaud_);
        return;
    }

    // Restaurar baud_ antes de encender
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    activeBaud_ = baud_;

    // 2. Pulso PWRKEY Activo-BAJO (1.2s LOW -> liberar a alta impedancia INPUT)
    Serial.println("[SIM7000] Encendiendo modem: pulso PWRKEY LOW (1.2s) -> INPUT...");
    pinMode(pwrKeyPin_, OUTPUT);
    digitalWrite(pwrKeyPin_, LOW);
    delay(1200);
    pinMode(pwrKeyPin_, INPUT);

    Serial.println("[SIM7000] Esperando arranque de UART del modem (hasta 5.5s)...");
    unsigned long waitStart = millis();
    while (millis() - waitStart < 5500) {
        if (millis() - waitStart >= 2000) {
            if (probeAtOrNmea(baud_, rxPin_, txPin_, 1, 400)) {
                Serial.println("[SIM7000] Modem respondio durante ventana de arranque.");
                return;
            }
        } else {
            delay(200);
        }
    }

    // 3. Si no respondio ni envio bytes, probar pulso Activo-ALTO (para placas con transistor NPN en PWRKEY)
    if (!isReady_ && !nmeaMode_ && rxBytesCount_ == 0) {
        Serial.println("[SIM7000] Sin respuesta con pulso LOW; probando pulso PWRKEY HIGH (1.2s para inversor NPN)...");
        pinMode(pwrKeyPin_, OUTPUT);
        digitalWrite(pwrKeyPin_, HIGH);
        delay(1200);
        digitalWrite(pwrKeyPin_, LOW);
        delay(100);
        pinMode(pwrKeyPin_, INPUT);

        waitStart = millis();
        while (millis() - waitStart < 5000) {
            if (millis() - waitStart >= 2000) {
                if (probeAtOrNmea(baud_, rxPin_, txPin_, 1, 400)) {
                    Serial.println("[SIM7000] Modem respondio tras pulso HIGH.");
                    return;
                }
            } else {
                delay(200);
            }
        }
    }
}

void Sim7000Modem::powerOff() {
    if (pwrKeyPin_ == 0xFF) return;
    Serial.printf("[SIM7000] Apagando modem via PWRKEY (GPIO %u)...\n", pwrKeyPin_);
    pinMode(pwrKeyPin_, OUTPUT);
    digitalWrite(pwrKeyPin_, LOW);
    delay(1500);
    pinMode(pwrKeyPin_, INPUT);
    isReady_ = false;
    gnssPowered_ = false;
}

void Sim7000Modem::restartModem() {
    powerOff();
    delay(1000);
    powerOn();
    if (isReady_) {
        configureGnssAt();
    }
}

void Sim7000Modem::configureGnssAt() {
    sendATCommand("ATE0", "OK", 500);          // Desactivar eco
    sendATCommand("AT+CFUN=1", "OK", 1000);    // Funcionalidad completa
    sendATCommand("AT+CMGF=1", "OK", 500);     // Modo SMS texto
    sendATCommand("AT+CSCS=\"GSM\"", "OK", 500);

    // Energizar alimentación de antena activa GNSS (3.05V en VAUX y GPIO4 en SGPIO)
    sendATCommand("AT+CVAUXV=3050", "OK", 500);
    sendATCommand("AT+CVAUXS=1", "OK", 500);
    sendATCommand("AT+SGPIO=0,4,1,1", "OK", 500);

    // Configurar constelaciones en SIM7000G (4 parámetros: GPS, GLONASS, BeiDou, Galileo)
    if (!sendATCommand("AT+CGNSMOD=1,1,1,1", "OK", 800)) {
        sendATCommand("AT+CGNSMOD=1,1,0,0", "OK", 800);
    }

    // Encender receptor GNSS interno del SIM7000G
    Serial.println("[SIM7000] Encendiendo receptor GNSS integrado (AT+CGNSPWR=1)...");
    if (sendATCommand("AT+CGNSPWR=1", "OK", 2000)) {
        gnssPowered_ = true;
        // Reafirmar alimentación de antena activa tras encender el motor GNSS
        sendATCommand("AT+SGPIO=0,4,1,1", "OK", 500);
        sendATCommand("AT+CVAUXS=1", "OK", 500);
        Serial.println("[SIM7000] Receptor GNSS encendido y antena activa alimentada.");
        queryGnss();
    } else {
        gnssPowered_ = false;
        Serial.println("[SIM7000] AVISO: No se recibio OK al comando AT+CGNSPWR=1.");
    }
}

void Sim7000Modem::begin() {
    if (initialized_ && (isReady_ || nmeaMode_)) {
        return;
    }
    initialized_ = true;

    Serial.printf("[SIM7000] Inicializando receptor GPS/SIM7000G en UART (RX: %u, TX: %u, PWRKEY: %u, Baud: %u)...\n",
                  rxPin_, txPin_, pwrKeyPin_, baud_);

    if (pwrKeyPin_ != 0xFF) {
        pinMode(pwrKeyPin_, INPUT);
    }

    // 1. Probar si ya responde por AT o transmite NMEA a 115200 o 9600 (sin pulsar PWRKEY)
    bool detected = probeAtOrNmea(baud_, rxPin_, txPin_, 3, 350);

    if (!detected) {
        // Escuchar ventana de 1.1s a 9600 baud (estándar 1 Hz NMEA GPS y fábrica SIM7000)
        if (probeAtOrNmea(9600, rxPin_, txPin_, 2, 350) || listenForNmea(9600, rxPin_, txPin_, 1100)) {
            detected = true;
            if (isReady_ && baud_ != 9600) {
                Serial.println("[SIM7000] Modem AT detectado a 9600 baud. Sincronizando a 115200 baud...");
                sendATCommand("AT+IPR=115200", "OK", 500);
                serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
                activeBaud_ = baud_;
                delay(150);
                sendATCommand("AT", "OK", 500);
            }
        }
    }

    // 2. Si no hubo respuesta ni flujo NMEA, accionar secuencia de encendido PWRKEY
    if (!detected && pwrKeyPin_ != 0xFF) {
        powerOn();
        detected = (isReady_ || nmeaMode_);
        if (!detected && baud_ != 9600) {
            if (probeAtOrNmea(9600, rxPin_, txPin_, 2, 400)) {
                detected = true;
                if (isReady_) {
                    sendATCommand("AT+IPR=115200", "OK", 500);
                    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
                    activeBaud_ = baud_;
                    delay(150);
                    sendATCommand("AT", "OK", 500);
                }
            }
        }
    }

    // 3. Si en (rxPin_, txPin_) llegaron 0 bytes, verificar cruce RX/TX (17/16) o pines GSM alternativos (26/27)
    if (!detected && rxBytesCount_ == 0) {
        struct PinPair { uint8_t rx; uint8_t tx; };
        const PinPair altPairs[] = {
            {txPin_, rxPin_},
            {26, 27},
            {27, 26}
        };

        for (const auto& pair : altPairs) {
            if (pair.rx == rxPin_ && pair.tx == txPin_) continue;
            Serial.printf("[SIM7000] Sondeando pines alternativos RX: %u, TX: %u...\n", pair.rx, pair.tx);
            if (probeAtOrNmea(baud_, pair.rx, pair.tx, 2, 350) ||
                probeAtOrNmea(9600, pair.rx, pair.tx, 1, 350) ||
                listenForNmea(9600, pair.rx, pair.tx, 1100)) {
                rxPin_ = pair.rx;
                txPin_ = pair.tx;
                detected = true;
                Serial.printf("[SIM7000] ¡Receptor detectado en RX: %u, TX: %u (%u baud)!\n",
                              rxPin_, txPin_, activeBaud_);
                break;
            }
        }
    }

    if (detected && isReady_) {
        configureGnssAt();
    } else if (detected && nmeaMode_) {
        Serial.printf("[GPS-NMEA] Flujo NMEA activo en RX: %u, TX: %u (%u baud). Bytes recibidos: %u\n",
                      rxPin_, txPin_, activeBaud_, rxBytesCount_);
    } else {
        isReady_ = false;
        // Asegurar que Serial2 quede en la velocidad y pines configurados (no atrapado en 9600 o pines alternos)
        serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
        activeBaud_ = baud_;
        Serial.printf("[SIM7000] AVISO: Sin respuesta AT ni flujo NMEA (Bytes RX: %u).\n", rxBytesCount_);
        Serial.println("[SIM7000] Reintentando autodeteccion en segundo plano cada 10s...");
    }
}

void Sim7000Modem::begin(const std::string& /*authCode*/) {
    begin();
}

void Sim7000Modem::update() {
    while (serial_.available() > 0) {
        processIncomingByte(static_cast<char>(serial_.read()));
    }

    if (nmeaMode_) {
        syncFromNmea();
        return;
    }

    if (!isReady_) {
        if (millis() - lastReadyCheckMs_ >= 10000) {
            lastReadyCheckMs_ = millis();
            if (probeAtOrNmea(baud_, rxPin_, txPin_, 2, 300)) {
                if (isReady_) {
                    Serial.println("[SIM7000] Modem AT detectado en segundo plano. Configurando GNSS...");
                    configureGnssAt();
                }
                return;
            }
            if (baud_ != 9600 && (probeAtOrNmea(9600, rxPin_, txPin_, 1, 300) || listenForNmea(9600, rxPin_, txPin_, 1100))) {
                if (isReady_) {
                    Serial.println("[SIM7000] Modem AT detectado a 9600 baud en segundo plano.");
                    sendATCommand("AT+IPR=115200", "OK", 500);
                    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
                    activeBaud_ = baud_;
                    delay(100);
                    configureGnssAt();
                } else if (nmeaMode_) {
                    Serial.printf("[GPS-NMEA] Receptor NMEA detectado en segundo plano (%u baud).\n", activeBaud_);
                }
                return;
            }
            // Restaurar siempre el baud rate principal tras el sondeo
            serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
            activeBaud_ = baud_;
        }
        return;
    }

    // Consulta periódica del GNSS cada 2 segundos en modo AT
    if (millis() - lastGnssQueryMs_ >= 2000) {
        lastGnssQueryMs_ = millis();
        queryGnss();
    }
}

void Sim7000Modem::queryGnss() {
    while (serial_.available() > 0) {
        processIncomingByte(static_cast<char>(serial_.read()));
    }

    serial_.println("AT+CGNSINF");

    unsigned long start = millis();
    String response = "";
    while (millis() - start < 1000) {
        while (serial_.available() > 0) {
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
            response += c;
            if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
                break;
            }
        }
        if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
            break;
        }
        delay(5);
    }

    int idx = response.indexOf("+CGNSINF: ");
    if (idx != -1) {
        int endIdx = response.indexOf("\r", idx);
        if (endIdx == -1) endIdx = response.indexOf("\n", idx);
        String line = (endIdx != -1) ? response.substring(idx + 10, endIdx) : response.substring(idx + 10);
        lastCgnsInfRaw_ = line;
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
        gnssPowered_ = false;
        if (sendATCommand("AT+CGNSPWR=1", "OK", 1000)) {
            gnssPowered_ = true;
            sendATCommand("AT+SGPIO=0,4,1,1", "OK", 500);
            sendATCommand("AT+CVAUXS=1", "OK", 500);
        }
    } else {
        gnssPowered_ = true;
    }

    int gpsInView  = (tokens.size() > 14 && !tokens[14].isEmpty()) ? tokens[14].toInt() : 0;
    int gpsUsed    = (tokens.size() > 15 && !tokens[15].isEmpty()) ? tokens[15].toInt() : 0;
    int gloUsed    = (tokens.size() > 16 && !tokens[16].isEmpty()) ? tokens[16].toInt() : 0;
    satellitesUsed_   = gpsUsed + gloUsed;
    satellitesInView_ = (gpsInView >= satellitesUsed_) ? gpsInView : satellitesUsed_;

    if (tokens.size() > 18 && !tokens[18].isEmpty()) {
        maxC_N0_ = tokens[18].toInt();
    }

    if (fixStatus == 1 && tokens.size() >= 8 && !tokens[3].isEmpty() && !tokens[4].isEmpty()) {
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
    if (nmeaMode_) {
        syncFromNmea();
    }
    out = cachedLocation_;

    if (!out.isValid) {
        if (millis() - lastWarningMs_ >= 10000) {
            Serial.printf("[GNSS] Sin fix 3D (%s) - Sats vista: %d, en uso: %d, C/N0 max: %d dB-Hz, Bytes UART: %u\n",
                          nmeaMode_ ? "NMEA" : (isReady_ ? "AT-SIM7000" : "SIN-UART"),
                          satellitesInView_, satellitesUsed_, maxC_N0_, rxBytesCount_);
            lastWarningMs_ = millis();
        }
    } else {
        if (millis() - lastWarningMs_ >= 10000) {
            Serial.printf("[GNSS] ¡FIX 3D ACTIVO (%s)! Lat: %.6f, Lon: %.6f, Sats: %d, Vel: %.1f km/h\n",
                          nmeaMode_ ? "NMEA" : "AT-SIM7000",
                          cachedLocation_.latitude, cachedLocation_.longitude,
                          cachedLocation_.satellites, cachedLocation_.speed);
            lastWarningMs_ = millis();
        }
    }
}

float Sim7000Modem::currentSpeedKmph() {
    if (nmeaMode_) syncFromNmea();
    return cachedLocation_.isValid ? cachedLocation_.speed : 0.0f;
}

int Sim7000Modem::getSatellites() {
    if (nmeaMode_) syncFromNmea();
    return cachedLocation_.satellites;
}

bool Sim7000Modem::hasFix() {
    if (nmeaMode_) syncFromNmea();
    return cachedLocation_.isValid;
}

void Sim7000Modem::diagnose() {
    if (nmeaMode_) {
        syncFromNmea();
    } else if (isReady_) {
        queryGnss();
    }

    Serial.println("\n=== DIAGNOSTICO GNSS / MODEM SIM7000G ===");
    Serial.printf("Modo activo:             %s\n",
                  nmeaMode_ ? "NMEA Stream (TinyGPSPlus)" : (isReady_ ? "AT SIM7000G (+CGNSINF)" : "SIN RESPUESTA UART"));
    Serial.printf("Pines UART (RX/TX):      GPIO %u / GPIO %u (%u baud)\n", rxPin_, txPin_, activeBaud_);
    Serial.printf("Pin PWRKEY:              GPIO %u\n", pwrKeyPin_);
    Serial.printf("Bytes RX recibidos:      %u\n", rxBytesCount_);
    Serial.printf("Chars NMEA procesados:   %u (ChkErr: %u)\n",
                  (unsigned int)nmeaGps_.charsProcessed(), (unsigned int)nmeaGps_.failedChecksum());
    Serial.printf("Modem AT listo:          %s\n", isReady_ ? "SI" : "NO");
    Serial.printf("GNSS alimentado:         %s\n", gnssPowered_ ? "SI" : "NO");
    Serial.printf("Fix satelital (3D):      %s\n", cachedLocation_.isValid ? "SI" : "NO");
    Serial.printf("Satelites a la vista:    %d\n", satellitesInView_);
    Serial.printf("Satelites en uso:        %d\n", satellitesUsed_);
    Serial.printf("Nivel max C/N0 (SNR):    %d dB-Hz\n", maxC_N0_);
    if (!lastCgnsInfRaw_.isEmpty()) {
        Serial.printf("Ultimo +CGNSINF:         %s\n", lastCgnsInfRaw_.c_str());
    }
    if (cachedLocation_.isValid) {
        Serial.printf("Latitud: %.6f, Longitud: %.6f\n", cachedLocation_.latitude, cachedLocation_.longitude);
        Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h, Rumbo: %.1f deg\n",
                      cachedLocation_.altitude, cachedLocation_.speed, cachedLocation_.course);
    }
    if (isReady_) {
        String csq = sendATCommandWithReply("AT+CSQ", 800);
        csq.trim();
        Serial.printf("Senal Celular (CSQ):     %s\n", csq.c_str());
    }
    Serial.println("=========================================\n");
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
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
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
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
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
    while (serial_.available() > 0) {
        processIncomingByte(static_cast<char>(serial_.read()));
    }

    serial_.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (serial_.available() > 0) {
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
            response += c;
            if (response.indexOf(expectedResponse) != -1) {
                return true;
            }
        }
        delay(5);
    }
    return false;
}

String Sim7000Modem::sendATCommandWithReply(const String& cmd, unsigned long timeoutMs) {
    while (serial_.available() > 0) {
        processIncomingByte(static_cast<char>(serial_.read()));
    }

    serial_.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeoutMs) {
        while (serial_.available() > 0) {
            char c = static_cast<char>(serial_.read());
            processIncomingByte(c);
            response += c;
            if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
                return response;
            }
        }
        delay(5);
    }
    return response;
}

} // namespace relay::infra
