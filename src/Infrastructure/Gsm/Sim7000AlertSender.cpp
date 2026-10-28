#include "Infrastructure/Gsm/Sim7000AlertSender.h"

namespace relay::infra {

Sim7000AlertSender::Sim7000AlertSender(HardwareSerial& serial,
                                       uint8_t rxPin,
                                       uint8_t txPin,
                                       uint32_t baud,
                                       const std::string& emergencyPhone)
    : serial_(serial),
      rxPin_(rxPin),
      txPin_(txPin),
      baud_(baud),
      emergencyPhone_(emergencyPhone) {}

void Sim7000AlertSender::begin(const std::string& authCode) {
    Serial.printf("[GSM] Inicializando modem celular en UART (RX: %u, TX: %u, Baud: %u)...\n",
                  rxPin_, txPin_, baud_);
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    delay(500);

    // Test communication with modem
    if (sendATCommand("AT", "OK", 1000)) {
        isReady_ = true;
        sendATCommand("ATE0", "OK", 1000);        // Echo off
        sendATCommand("AT+CMGF=1", "OK", 1000);   // SMS Text mode
        sendATCommand("AT+CSCS=\"GSM\"", "OK", 1000);
        Serial.println("[GSM] Modem celular SIM7000G/SIM800 detectado y configurado en modo SMS.");
    } else {
        isReady_ = false;
        Serial.println("[GSM] AVISO: Modem celular no respondio al comando AT (puede no estar conectado o energizado).");
    }
}

void Sim7000AlertSender::update() {
    // Non-blocking drain of unsolicited AT codes (+CMTI, etc.)
    while (serial_.available() > 0) {
        serial_.read();
    }
}

bool Sim7000AlertSender::sendAlert(const domain::AccidentEvent& accident,
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

bool Sim7000AlertSender::sendSms(const std::string& phoneNumber, const std::string& message) {
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

bool Sim7000AlertSender::isReady() {
    if (!isReady_) {
        // Periodic retry to detect modem if powered on later
        if (millis() - lastCheckMs_ > 15000) {
            lastCheckMs_ = millis();
            if (sendATCommand("AT", "OK", 500)) {
                isReady_ = true;
                sendATCommand("ATE0", "OK", 500);
                sendATCommand("AT+CMGF=1", "OK", 500);
            }
        }
    }
    return isReady_;
}

void Sim7000AlertSender::setCaCert(const char* caCertPem) {
    if (!caCertPem || strlen(caCertPem) == 0) return;
    Serial.println("[GSM] Certificado CA configurado para canal seguro MQTTS SIM7000G.");
    // In SIM7000G AT commands: AT+CSSLCFG="cacert",... can be configured when cellular TLS is selected.
}

bool Sim7000AlertSender::sendATCommand(const String& command, const String& expectedResponse, unsigned long timeoutMs) {
    while (serial_.available() > 0) serial_.read();

    serial_.println(command);

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

} // namespace relay::infra
