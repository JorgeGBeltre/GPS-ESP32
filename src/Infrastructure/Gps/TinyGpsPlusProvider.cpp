#include "Infrastructure/Gps/TinyGpsPlusProvider.h"

namespace relay::infra {

TinyGpsPlusProvider::TinyGpsPlusProvider(HardwareSerial& serial,
                                         uint8_t rxPin,
                                         uint8_t txPin,
                                         uint32_t baud)
    : gpsSerial_(serial), rxPin_(rxPin), txPin_(txPin), baud_(baud) {}

void TinyGpsPlusProvider::begin() {
    Serial.printf("[GPS] Inicializando UART en baud %u (RX: %u, TX: %u)...\n", baud_, rxPin_, txPin_);
    gpsSerial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    delay(100);

    int bytesAvailable = gpsSerial_.available();
    Serial.printf("[GPS] Bytes disponibles en buffer: %d\n", bytesAvailable);
    if (bytesAvailable > 0) {
        Serial.println("[GPS] Modulo GPS detectado y transmitiendo");
    } else {
        Serial.println("[GPS] Esperando transmision NMEA del receptor GPS...");
    }
}

void TinyGpsPlusProvider::update() {
    while (gpsSerial_.available() > 0) {
        gps_.encode(gpsSerial_.read());
    }
}

void TinyGpsPlusProvider::readData(domain::GpsLocation& out) {
    out.isValid = gps_.location.isValid();
    if (out.isValid) {
        out.latitude = gps_.location.lat();
        out.longitude = gps_.location.lng();
        out.altitude = gps_.altitude.meters();
        out.speed = gps_.speed.kmph();
        out.course = gps_.course.deg();
        out.satellites = gps_.satellites.value();
    } else {
        static unsigned long lastWarning = 0;
        if (millis() - lastWarning > 30000) {
            Serial.printf("[GPS] Sin fix valido - Satelites visibles: %d\n", gps_.satellites.value());
            lastWarning = millis();
        }
    }
}

float TinyGpsPlusProvider::currentSpeedKmph() {
    return gps_.speed.isValid() ? static_cast<float>(gps_.speed.kmph()) : 0.0f;
}

int TinyGpsPlusProvider::getSatellites() {
    return gps_.satellites.value();
}

bool TinyGpsPlusProvider::hasFix() {
    return gps_.location.isValid();
}

void TinyGpsPlusProvider::diagnose() {
    Serial.println("\n=== DIAGNOSTICO GPS ===");
    Serial.printf("Bytes en UART:          %d\n", gpsSerial_.available());
    Serial.printf("Caracteres procesados:  %u\n", (unsigned int)gps_.charsProcessed());
    Serial.printf("Checksums fallidos:     %u\n", (unsigned int)gps_.failedChecksum());
    Serial.printf("Satelites visibles:     %d\n", gps_.satellites.value());
    Serial.printf("Fix valido:             %s\n", gps_.location.isValid() ? "SI" : "NO");

    if (gps_.location.isValid()) {
        Serial.printf("Latitud:   %.6f, Longitud: %.6f\n", gps_.location.lat(), gps_.location.lng());
        Serial.printf("Altitud:   %.1f m, Velocidad: %.1f km/h, Rumbo: %.1f deg\n",
                      gps_.altitude.meters(), gps_.speed.kmph(), gps_.course.deg());
    } else {
        Serial.printf("Antigüedad ultimo fix:  %u ms\n", (unsigned int)gps_.location.age());
    }
    Serial.println("========================\n");
}

} // namespace relay::infra
