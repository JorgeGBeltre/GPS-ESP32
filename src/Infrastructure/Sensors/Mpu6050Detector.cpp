#include "Infrastructure/Sensors/Mpu6050Detector.h"

namespace relay::infra {

Mpu6050Detector::Mpu6050Detector(uint8_t sdaPin,
                                 uint8_t sclPin,
                                 int sensitivity,
                                 int rolloverAngleDeg,
                                 float harshAccelG,
                                 float harshBrakeG,
                                 unsigned long checkIntervalMs,
                                 unsigned long cooldownMs)
    : sdaPin_(sdaPin),
      sclPin_(sclPin),
      sensitivity_(sensitivity),
      rolloverAngleDeg_(rolloverAngleDeg),
      harshAccelG_(harshAccelG),
      harshBrakeG_(harshBrakeG),
      checkIntervalMs_(checkIntervalMs),
      cooldownMs_(cooldownMs) {}

void Mpu6050Detector::begin() {
    Serial.printf("[MPU6050] Iniciando I2C en SDA: %u, SCL: %u...\n", sdaPin_, sclPin_);
    Wire.begin(sdaPin_, sclPin_);
    mpu_.initialize();

    isConnected_ = mpu_.testConnection();
    if (!isConnected_) {
        Serial.println("[MPU6050] ERROR: Sensor MPU6050 no detectado en bus I2C!");
    } else {
        Serial.println("[MPU6050] Sensor detectado. Calibrando acelerometro y giroscopio...");
        mpu_.CalibrateAccel(6);
        mpu_.CalibrateGyro(6);
        mpu_.PrintActiveOffsets();
        Serial.println("[MPU6050] Calibracion completada exitosamente.");
    }
}

bool Mpu6050Detector::check(domain::AccidentEvent& out, float currentSpeedKmph) {
    if (!isConnected_) return false;

    unsigned long now = millis();
    if (now - lastCheckMs_ < checkIntervalMs_) return false;
    lastCheckMs_ = now;

    int16_t ax = 0, ay = 0, az = 0;
    mpu_.getAcceleration(&ax, &ay, &az);

    app::AccidentSample sample = algorithm_.evaluate(ax, ay, az,
                                                     sensitivity_,
                                                     rolloverAngleDeg_,
                                                     currentSpeedKmph,
                                                     harshAccelG_,
                                                     harshBrakeG_);

    if (!sample.evaluated) return false;

    if (sample.impact_detected || sample.rollover_detected ||
        sample.harsh_accel_detected || sample.harsh_brake_detected) {

        if (now - lastEventMs_ >= cooldownMs_) {
            lastEventMs_ = now;

            out.impact_detected      = sample.impact_detected;
            out.rollover_detected    = sample.rollover_detected;
            out.harsh_accel_detected = sample.harsh_accel_detected;
            out.harsh_brake_detected = sample.harsh_brake_detected;
            out.impact_magnitude     = sample.impact_magnitude;
            out.roll_angle           = sample.roll_angle;
            out.pitch_angle          = sample.pitch_angle;
            out.accel_g              = sample.accel_g;
            out.detection_time       = now;

            if (sample.impact_detected) {
                Serial.printf("[MPU6050] IMPACTO VALIDADO POR STA/LTA! Magnitud: %.1f\n", sample.impact_magnitude);
            }
            if (sample.rollover_detected) {
                Serial.printf("[MPU6050] VUELCO (ROLLOVER) DETECTADO! Roll: %.1f deg, Pitch: %.1f deg\n",
                              sample.roll_angle, sample.pitch_angle);
            }
            if (sample.harsh_accel_detected) {
                Serial.printf("[MPU6050] ACELERACION BRUSCA! Aceleracion: %.2fg\n", sample.accel_g);
            }
            if (sample.harsh_brake_detected) {
                Serial.printf("[MPU6050] FRENADA BRUSCA! Aceleracion: %.2fg\n", sample.accel_g);
            }
            return true;
        } else {
            Serial.println("[MPU6050] Evento detectado dentro de ventana de cooldown, ignorando.");
        }
    }

    return false;
}

} // namespace relay::infra
