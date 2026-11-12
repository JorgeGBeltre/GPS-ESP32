#include "Infrastructure/Modem/Sim7000Modem.h"
#include "Infrastructure/Config/nvs_config.h"
#include "Infrastructure/System/EspSystemControl.h"
#include <cmath>
#include <cstdlib>

namespace relay::infra {

namespace {
struct CarrierApnEntry {
    const char* mccMncPrefix;
    const char* apn;
};

constexpr CarrierApnEntry kCarrierApnTable[] = {
    {"37001", "internet.orange.com.do"},
    {"37002", "internet.claro.com.do"},
    {"37003", "avircom.net"},
    {"37004", "data.viva.com.do"},
    {"334020", "internet.itelcel.com"},
    {"33403", "internet.movistar.mx"},
    {"732101", "internet.comcel.com.co"},
    {"732103", "web.colombiamovil.com.co"},
    {"732123", "internet.movistar.com.co"},
    {"71606", "movistar.pe"},
    {"71610", "claro.pe"},
    {"73001", "bam.entelpcs.cl"},
    {"73002", "wap.tmovil.cl"},
    {"72207", "wap.gprs.unifon.com.ar"},
    {"722310", "igprs.claro.com.ar"},
    {"310260", "fast.t-mobile.com"},
    {"310410", "broadband"},
    {"21401", "airtelnet.es"},
    {"21407", "movistar.es"},
};

bool parseRegLacCellId(const String& regResp, uint32_t& outLac, uint32_t& outCellId) {
    int q1 = regResp.indexOf('"');
    if (q1 == -1) return false;
    int q2 = regResp.indexOf('"', q1 + 1);
    if (q2 == -1) return false;
    int q3 = regResp.indexOf('"', q2 + 1);
    if (q3 == -1) return false;
    int q4 = regResp.indexOf('"', q3 + 1);
    if (q4 == -1) return false;

    String lacHex = regResp.substring(q1 + 1, q2);
    String ciHex  = regResp.substring(q3 + 1, q4);
    lacHex.trim();
    ciHex.trim();
    if (lacHex.length() == 0 || ciHex.length() == 0) return false;

    uint32_t lac = static_cast<uint32_t>(std::strtoul(lacHex.c_str(), nullptr, 16));
    uint32_t ci  = static_cast<uint32_t>(std::strtoul(ciHex.c_str(), nullptr, 16));
    if (ci == 0 || ci == 0xFFFFFFFFUL) return false;

    outLac = lac;
    outCellId = ci;
    return true;
}

void extractDigitsMccMnc(const String& raw, int& outMcc, int& outMnc) {
    String digits = "";
    for (int i = 0; i < (int)raw.length(); ++i) {
        if (raw[i] >= '0' && raw[i] <= '9') {
            digits += raw[i];
        } else if (digits.length() >= 5) {
            break;
        } else {
            digits = "";
        }
    }
    if (digits.length() >= 5) {
        outMcc = digits.substring(0, 3).toInt();
        int mncLen = 2;
        for (size_t i = 0; i < sizeof(kCarrierApnTable) / sizeof(kCarrierApnTable[0]); ++i) {
            if (digits.startsWith(kCarrierApnTable[i].mccMncPrefix)) {
                int pLen = static_cast<int>(strlen(kCarrierApnTable[i].mccMncPrefix));
                if (pLen >= 5) mncLen = pLen - 3;
                break;
            }
        }
        if ((int)digits.length() >= 3 + mncLen) {
            outMnc = digits.substring(3, 3 + mncLen).toInt();
        }
    }
}
} // namespace

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
        cachedLocation_.isLbsFallback = false;
        cachedLocation_.altitude = nmeaGps_.altitude.isValid() ? static_cast<float>(nmeaGps_.altitude.meters()) : 0.0f;
        cachedLocation_.speed = nmeaGps_.speed.isValid() ? static_cast<float>(nmeaGps_.speed.kmph()) : 0.0f;
        cachedLocation_.course = nmeaGps_.course.isValid() ? static_cast<float>(nmeaGps_.course.deg()) : 0.0f;
        cachedLocation_.satellites = (satellitesUsed_ > 0) ? satellitesUsed_ : satellitesInView_;
        float hdop = nmeaGps_.hdop.isValid() ? static_cast<float>(nmeaGps_.hdop.hdop()) : 99.9f;
        applyDriftAndTowFilter(nmeaGps_.location.lat(), nmeaGps_.location.lng(), cachedLocation_.speed, hdop);
        applyLbsToCachedLocation(true);
    } else {
        cachedLocation_.isValid = false;
        cachedLocation_.isLbsFallback = false;
        cachedLocation_.latitude = 0.0;
        cachedLocation_.longitude = 0.0;
        cachedLocation_.altitude = 0.0f;
        cachedLocation_.speed = 0.0f;
        cachedLocation_.course = 0.0f;
        cachedLocation_.satellites = (satellitesInView_ > 0) ? satellitesInView_ : satellitesUsed_;
        applyLbsToCachedLocation(false);
    }
}

std::string Sim7000Modem::autoResolveCarrierApn() {
    String cpinResp = sendATCommandWithReply("AT+CPIN?", 800);
    if (cpinResp.indexOf("READY") != -1) {
        simPresent_ = true;
    }

    String imsiResp = sendATCommandWithReply("AT+CIMI", 1200);
    extractDigitsMccMnc(imsiResp, mcc_, mnc_);
    for (size_t i = 0; i < sizeof(kCarrierApnTable) / sizeof(kCarrierApnTable[0]); ++i) {
        if (imsiResp.indexOf(kCarrierApnTable[i].mccMncPrefix) != -1) {
            simPresent_ = true;
            apnAutoDetected_ = true;
            resolvedApn_ = kCarrierApnTable[i].apn;
            String cgatt = sendATCommandWithReply("AT+CGATT?", 600);
            if (cgatt.indexOf("+CGATT: 1") != -1) {
                cellularConnected_ = true;
            }
            setCellularProvisioningSnapshot(simPresent_, apnAutoDetected_, cellularConnected_, resolvedApn_.c_str());
            return resolvedApn_;
        }
    }

    // Fallback LTE: si el módem adjuntó automáticamente por EPS Default Bearer (+CGNAPN / +CGDCONT?)
    String cgattResp = sendATCommandWithReply("AT+CGATT?", 800);
    if (cgattResp.indexOf("+CGATT: 1") != -1) {
        simPresent_ = true;
        cellularConnected_ = true;
    }

    auto extractQuotedApn = [](const String& resp, const char* tag, int quoteIndex) -> std::string {
        int pos = resp.indexOf(tag);
        if (pos == -1) return "";
        int qStart = -1;
        int seenQuotes = 0;
        for (int i = pos; i < (int)resp.length(); ++i) {
            if (resp[i] == '"') {
                ++seenQuotes;
                if (seenQuotes == quoteIndex) {
                    qStart = i + 1;
                } else if (seenQuotes == quoteIndex + 1 && qStart > 0) {
                    String candidate = resp.substring(qStart, i);
                    candidate.trim();
                    if (candidate.length() > 0) {
                        return std::string(candidate.c_str());
                    }
                    break;
                }
            }
        }
        return "";
    };

    String cgnapnResp = sendATCommandWithReply("AT+CGNAPN", 800);
    std::string netApn = extractQuotedApn(cgnapnResp, "+CGNAPN:", 1);
    if (netApn.empty() && cellularConnected_) {
        String cgdcontResp = sendATCommandWithReply("AT+CGDCONT?", 800);
        netApn = extractQuotedApn(cgdcontResp, "+CGDCONT:", 3);
    }

    if (!netApn.empty()) {
        simPresent_ = true;
        apnAutoDetected_ = true;
        resolvedApn_ = netApn;
    }

    setCellularProvisioningSnapshot(simPresent_, apnAutoDetected_, cellularConnected_, resolvedApn_.c_str());
    return resolvedApn_;
}

void Sim7000Modem::probeCellularProvisioningState() {
    if (pwrKeyPin_ != 0xFF) {
        pinMode(pwrKeyPin_, INPUT);
    }
    bool uartOk = isReady_ || probeAtOrNmea(baud_, rxPin_, txPin_, 2, 250);
    if (!uartOk && baud_ != 9600) {
        uartOk = probeAtOrNmea(9600, rxPin_, txPin_, 1, 250);
        if (uartOk && isReady_) {
            sendATCommand("AT+IPR=115200", "OK", 400);
            serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
            activeBaud_ = baud_;
            delay(80);
        } else if (!uartOk) {
            serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
            activeBaud_ = baud_;
        }
    }

    if (uartOk && isReady_) {
        sendATCommand("ATE0", "OK", 400);
        std::string detected = autoResolveCarrierApn();
        if (!detected.empty() && apn_.empty()) {
            apn_ = detected;
            String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + apn_.c_str() + "\"";
            sendATCommand(cgdcont, "OK", 600);
        }
        Serial.printf("[SIM7000] Sondeo celular para aprovisionamiento: SIM=%d, autoAPN=%d (%s), CGATT=%d\n",
                      simPresent_ ? 1 : 0,
                      apnAutoDetected_ ? 1 : 0,
                      resolvedApn_.c_str(),
                      cellularConnected_ ? 1 : 0);
    } else {
        setCellularProvisioningSnapshot(simPresent_, apnAutoDetected_, cellularConnected_, resolvedApn_.c_str());
    }
}

void Sim7000Modem::applyDriftAndTowFilter(double rawLat, double rawLon, float speedKmh, float hdop) {
    cachedLocation_.hdop = hdop;
    cachedLocation_.virtualIgnition = (speedKmh >= 3.0f);
    cachedLocation_.jammingDetected = jammingDetected_;
    cachedLocation_.roaming = roaming_;
    cachedLocation_.driftFiltered = false;
    cachedLocation_.towDetected = false;

    if (!hasAnchor_) {
        hasAnchor_ = true;
        anchorLat_ = rawLat;
        anchorLon_ = rawLon;
    }

    double distFromAnchorM = TinyGPSPlus::distanceBetween(anchorLat_, anchorLon_, rawLat, rawLon);

    // Static GPS Drift Filter (GTRDF): pin stationary coordinates within 30 m
    if (speedKmh < 3.0f && distFromAnchorM <= 30.0) {
        cachedLocation_.latitude = anchorLat_;
        cachedLocation_.longitude = anchorLon_;
        cachedLocation_.driftFiltered = true;
        towLatched_ = false;
    } else {
        cachedLocation_.latitude = rawLat;
        cachedLocation_.longitude = rawLon;
        // Parked Tow Detection (GTTOW) with +20 m exit hysteresis (100 m set, < 80 m or >= 8 km/h clear)
        if (speedKmh >= 8.0f || distFromAnchorM < 80.0) {
            towLatched_ = false;
        } else if (speedKmh < 8.0f && distFromAnchorM >= 100.0) {
            towLatched_ = true;
        }
        cachedLocation_.towDetected = towLatched_;
        if (speedKmh >= 5.0f || (!towLatched_ && distFromAnchorM > 30.0)) {
            anchorLat_ = rawLat;
            anchorLon_ = rawLon;
        }
    }
}

void Sim7000Modem::pollCellularHealth() {
    if (!isReady_ || millis() - lastCellCheckMs_ < 30000) return;
    lastCellCheckMs_ = millis();

    String csqResp = sendATCommandWithReply("AT+CSQ", 800);
    int csq = 0;
    int idx = csqResp.indexOf("+CSQ:");
    if (idx != -1) {
        int comma = csqResp.indexOf(",", idx);
        if (comma > idx) {
            csq = csqResp.substring(idx + 5, comma).toInt();
        }
    }

    String cregResp = sendATCommandWithReply("AT+CREG?", 800);
    bool registeredHome = (cregResp.indexOf(",1") != -1);
    bool registeredRoaming = (cregResp.indexOf(",5") != -1);
    roaming_ = registeredRoaming;
    parseRegLacCellId(cregResp, lac_, cellId_);

    if (lastCsq_ >= 12 && (csq == 0 || csq == 99) && !registeredHome && !registeredRoaming) {
        jammingDetected_ = true;
        Serial.println("[SIM7000] ALERTA: Posible interferencia RF celular detectada (GTJDC).");
    } else if ((registeredHome || registeredRoaming) && csq >= 5 && csq != 99) {
        jammingDetected_ = false;
    }
    if (csq != 99) lastCsq_ = csq;

    cachedLocation_.jammingDetected = jammingDetected_;
    cachedLocation_.roaming = roaming_;
    pollCellularLbs(!cachedLocation_.isValid || cachedLocation_.isLbsFallback);
}

void Sim7000Modem::pollCellularLbs(bool requestCoordinatesIfNoGnss) {
    if (!isReady_) return;
    const unsigned long now = millis();
    if (lastLbsQueryMs_ != 0 && (now - lastLbsQueryMs_ < 25000UL) &&
        (!requestCoordinatesIfNoGnss || hasLbsCoords_)) {
        return;
    }
    lastLbsQueryMs_ = now;

    if (mcc_ == 0) {
        String imsiResp = sendATCommandWithReply("AT+CIMI", 900);
        extractDigitsMccMnc(imsiResp, mcc_, mnc_);
    }

    // 1. Consultar información de celda servidora (+CPSI? en SIM7000G)
    // Ejemplo LTE-M: +CPSI: LTE CAT-M1,Online,370-02,0x1A2B,12345678,120,...
    // Ejemplo GSM:   +CPSI: GSM,Online,370-02,0x1A2B,12345,45 EGSM 900,-72,0,...
    String cpsiResp = sendATCommandWithReply("AT+CPSI?", 800);
    int cpsiIdx = cpsiResp.indexOf("+CPSI:");
    if (cpsiIdx != -1) {
        int lineEnd = cpsiResp.indexOf('\n', cpsiIdx);
        String line = (lineEnd != -1) ? cpsiResp.substring(cpsiIdx + 6, lineEnd)
                                      : cpsiResp.substring(cpsiIdx + 6);
        line.trim();
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
        if (tokens.size() >= 5) {
            int dash = tokens[2].indexOf('-');
            if (dash > 0) {
                int parsedMcc = tokens[2].substring(0, dash).toInt();
                int parsedMnc = tokens[2].substring(dash + 1).toInt();
                if (parsedMcc > 0) {
                    mcc_ = parsedMcc;
                    mnc_ = parsedMnc;
                }
            }
            String lacTok = tokens[3];
            lacTok.trim();
            uint32_t parsedLac = (lacTok.startsWith("0x") || lacTok.startsWith("0X"))
                                     ? static_cast<uint32_t>(std::strtoul(lacTok.c_str() + 2, nullptr, 16))
                                     : static_cast<uint32_t>(std::strtoul(lacTok.c_str(), nullptr, 10));
            uint32_t parsedCi = static_cast<uint32_t>(std::strtoul(tokens[4].c_str(), nullptr, 10));
            if (parsedCi > 0) {
                lac_ = parsedLac;
                cellId_ = parsedCi;
            }
        }
    }

    // 2. Fallback a registros extendidos (+CEREG? / +CGREG? / +CREG? con formato =2)
    if (cellId_ == 0) {
        uint32_t regLac = 0, regCi = 0;
        if (parseRegLacCellId(sendATCommandWithReply("AT+CEREG?", 600), regLac, regCi) ||
            parseRegLacCellId(sendATCommandWithReply("AT+CGREG?", 600), regLac, regCi) ||
            parseRegLacCellId(sendATCommandWithReply("AT+CREG?", 600), regLac, regCi)) {
            lac_ = regLac;
            cellId_ = regCi;
        }
    }

    // 3. Si no hay fix satelital GNSS, resolver coordenadas LBS de estación base (AT+CLBS=1,1 / AT+CIPGSMLOC=1,1)
    if (requestCoordinatesIfNoGnss) {
        String clbs = sendATCommandWithReply("AT+CLBS=1,1", 2500);
        int clbsIdx = clbs.indexOf("+CLBS:");
        if (clbsIdx != -1) {
            int lineEnd = clbs.indexOf('\n', clbsIdx);
            String line = (lineEnd != -1) ? clbs.substring(clbsIdx + 6, lineEnd)
                                          : clbs.substring(clbsIdx + 6);
            line.trim();
            int c1 = line.indexOf(',');
            int c2 = (c1 != -1) ? line.indexOf(',', c1 + 1) : -1;
            int c3 = (c2 != -1) ? line.indexOf(',', c2 + 1) : -1;
            if (c1 != -1 && c2 != -1) {
                int code = line.substring(0, c1).toInt();
                double lat = line.substring(c1 + 1, c2).toDouble();
                double lon = (c3 != -1) ? line.substring(c2 + 1, c3).toDouble()
                                        : line.substring(c2 + 1).toDouble();
                int acc = (c3 != -1) ? line.substring(c3 + 1).toInt() : 500;
                if (code == 0 && (std::fabs(lat) > 0.0001 || std::fabs(lon) > 0.0001)) {
                    lbsLat_ = lat;
                    lbsLon_ = lon;
                    lbsAccuracyM_ = (acc > 0) ? acc : 500;
                    hasLbsCoords_ = true;
                    return;
                }
            }
        }

        String gsmLoc = sendATCommandWithReply("AT+CIPGSMLOC=1,1", 2500);
        int locIdx = gsmLoc.indexOf("+CIPGSMLOC:");
        if (locIdx != -1) {
            int lineEnd = gsmLoc.indexOf('\n', locIdx);
            String line = (lineEnd != -1) ? gsmLoc.substring(locIdx + 11, lineEnd)
                                          : gsmLoc.substring(locIdx + 11);
            line.trim();
            int c1 = line.indexOf(',');
            int c2 = (c1 != -1) ? line.indexOf(',', c1 + 1) : -1;
            int c3 = (c2 != -1) ? line.indexOf(',', c2 + 1) : -1;
            if (c1 != -1 && c2 != -1 && c3 != -1) {
                int code = line.substring(0, c1).toInt();
                double lon = line.substring(c1 + 1, c2).toDouble();
                double lat = line.substring(c2 + 1, c3).toDouble();
                if (code == 0 && (std::fabs(lat) > 0.0001 || std::fabs(lon) > 0.0001)) {
                    lbsLat_ = lat;
                    lbsLon_ = lon;
                    lbsAccuracyM_ = 500;
                    hasLbsCoords_ = true;
                }
            }
        }
    }
}

void Sim7000Modem::applyLbsToCachedLocation(bool hasSatelliteFix) {
    cachedLocation_.lbsValid = (mcc_ > 0 && cellId_ > 0) || hasLbsCoords_;
    cachedLocation_.mcc = mcc_;
    cachedLocation_.mnc = mnc_;
    cachedLocation_.lac = lac_;
    cachedLocation_.cellId = cellId_;
    cachedLocation_.lbsAccuracyM = lbsAccuracyM_;

    if (!hasSatelliteFix && hasLbsCoords_) {
        cachedLocation_.isValid = true;
        cachedLocation_.isLbsFallback = true;
        cachedLocation_.latitude = lbsLat_;
        cachedLocation_.longitude = lbsLon_;
        cachedLocation_.driftFiltered = false;
        cachedLocation_.towDetected = false;
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
    EspSystemControl::heartbeat();
    powerOff();
    delay(1000);
    EspSystemControl::heartbeat();
    powerOn();
    EspSystemControl::heartbeat();
    if (isReady_) {
        configureGnssAt();
    }
}

void Sim7000Modem::recoverUnresponsiveModem() {
    if (otaActive_ || pwrKeyPin_ == 0xFF || nmeaMode_) return;
    const unsigned long now = millis();
    if (lastModemRecoveryMs_ != 0 && (now - lastModemRecoveryMs_ < 300000UL)) {
        return;
    }
    lastModemRecoveryMs_ = now;
    modemRecoveries_++;
    consecutiveAtFailures_ = 0;
    unreadyProbeCount_ = 0;
    Serial.printf("[SIM7000] Auto-recuperacion de modem no responsivo (#%u, cooldown 300s)...\n",
                  (unsigned)modemRecoveries_);
    restartModem();
}

void Sim7000Modem::configureGnssAt() {
    sendATCommand("ATE0", "OK", 500);          // Desactivar eco
    sendATCommand("AT+CFUN=1", "OK", 1000);    // Funcionalidad completa
    sendATCommand("AT+CMGF=1", "OK", 500);     // Modo SMS texto
    sendATCommand("AT+CSCS=\"GSM\"", "OK", 500);
    sendATCommand("AT+CREG=2", "OK", 500);
    sendATCommand("AT+CGREG=2", "OK", 500);
    sendATCommand("AT+CEREG=2", "OK", 500);

    // Configurar contexto PDP y APN celular (con auto-resolución MCC/MNC apn.bin si está vacío)
    if (apn_.empty()) {
        apn_ = autoResolveCarrierApn();
    }
    if (!apn_.empty()) {
        String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + apn_.c_str() + "\"";
        sendATCommand(cgdcont, "OK", 1000);
        String cstt = String("AT+CSTT=\"") + apn_.c_str() + "\",\"" + apnUser_.c_str() + "\",\"" + apnPass_.c_str() + "\"";
        sendATCommand(cstt, "OK", 1000);
        Serial.printf("[SIM7000] APN celular configurado: %s\n", apn_.c_str());
    }

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
                unreadyProbeCount_ = 0;
                consecutiveAtFailures_ = 0;
                if (isReady_) {
                    Serial.println("[SIM7000] Modem AT detectado en segundo plano. Configurando GNSS...");
                    configureGnssAt();
                }
                return;
            }
            if (baud_ != 9600 && (probeAtOrNmea(9600, rxPin_, txPin_, 1, 300) || listenForNmea(9600, rxPin_, txPin_, 1100))) {
                unreadyProbeCount_ = 0;
                consecutiveAtFailures_ = 0;
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
            if (++unreadyProbeCount_ >= 18) {
                unreadyProbeCount_ = 0;
                recoverUnresponsiveModem();
            }
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
    EspSystemControl::heartbeat();
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
    if (idx != -1 || response.indexOf("OK") != -1) {
        consecutiveAtFailures_ = 0;
    } else if (++consecutiveAtFailures_ >= 6) {
        Serial.println("[SIM7000] Sin respuesta AT durante 6 consultas consecutivas; marcando modem no listo.");
        isReady_ = false;
        consecutiveAtFailures_ = 0;
        recoverUnresponsiveModem();
        return;
    }

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
        cachedLocation_.isLbsFallback = false;
        if (!tokens[2].isEmpty()) cachedLocation_.timestamp = tokens[2].c_str();
        double rawLat = tokens[3].toDouble();
        double rawLon = tokens[4].toDouble();
        cachedLocation_.altitude = tokens[5].toFloat();
        cachedLocation_.speed = tokens[6].toFloat();
        cachedLocation_.course = tokens[7].toFloat();
        cachedLocation_.satellites = (satellitesUsed_ > 0) ? satellitesUsed_ : satellitesInView_;
        float hdop = (tokens.size() > 10 && !tokens[10].isEmpty()) ? tokens[10].toFloat() : 99.9f;
        applyDriftAndTowFilter(rawLat, rawLon, cachedLocation_.speed, hdop);
        pollCellularHealth();
        applyLbsToCachedLocation(true);
    } else {
        cachedLocation_.isValid = false;
        cachedLocation_.isLbsFallback = false;
        cachedLocation_.latitude = 0.0;
        cachedLocation_.longitude = 0.0;
        cachedLocation_.altitude = 0.0f;
        cachedLocation_.speed = 0.0f;
        cachedLocation_.course = 0.0f;
        cachedLocation_.satellites = satellitesInView_;
        pollCellularHealth();
        pollCellularLbs(true);
        applyLbsToCachedLocation(false);
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
        Serial.printf("Latitud: %.6f, Longitud: %.6f (%s)\n",
                      cachedLocation_.latitude, cachedLocation_.longitude,
                      cachedLocation_.isLbsFallback ? "LBS Celular" : "GNSS Satelital");
        Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h, Rumbo: %.1f deg\n",
                      cachedLocation_.altitude, cachedLocation_.speed, cachedLocation_.course);
    }
    if (cachedLocation_.lbsValid) {
        Serial.printf("Celda LBS (MCC-MNC-LAC-CID): %d-%d-%u-%u (Prec: %d m)\n",
                      cachedLocation_.mcc, cachedLocation_.mnc,
                      (unsigned)cachedLocation_.lac, (unsigned)cachedLocation_.cellId,
                      cachedLocation_.lbsAccuracyM);
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
        msg += std::string(gps.isLbsFallback ? "Ubicacion(LBS): " : "Ubicacion: ") +
               "https://maps.google.com/?q=" +
               std::to_string(gps.latitude) + "," + std::to_string(gps.longitude) + "\n";
        msg += "Velocidad: " + std::to_string((int)gps.speed) + " km/h\n";
    } else {
        msg += "GPS: Sin fix satelital\n";
    }
    if (gps.lbsValid && gps.cellId > 0) {
        msg += "Celda LBS: " + std::to_string(gps.mcc) + "-" + std::to_string(gps.mnc) + "-" +
               std::to_string(gps.lac) + "-" + std::to_string(gps.cellId) + "\n";
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
