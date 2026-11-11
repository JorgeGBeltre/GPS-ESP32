#include "Infrastructure/Serialization/ArduinoJsonPayloadSerializer.h"
#include <ArduinoJson.h>

namespace relay::infra {

std::string ArduinoJsonPayloadSerializer::formatStatus(
    const domain::DeviceIdentity& id,
    const domain::SystemMetrics& metrics,
    const std::vector<domain::RelayState>& relays,
    const std::string& timestamp,
    bool syncCompleted) {

    DynamicJsonDocument doc(2048);

    JsonObject dev = doc.createNestedObject("Device");
    dev["status"] = metrics.mqttConnected ? "connected" : "disconnected";
    dev["ChipId"] = id.chipId;
    dev["MacAddress"] = id.macAddress;
    dev["ChipType"] = id.chipType;
    dev["FirmwareVersion"] = id.firmwareVersion;

    doc["Timestamp"] = timestamp;

    JsonObject details = doc.createNestedObject("Details");
    details["chipId"] = id.chipId;
    details["sync_completed"] = syncCompleted;
    details["devices_count"] = relays.size();
    details["wifi_status"] = metrics.wifiConnected ? "connected" : "disconnected";
    details["rssi"] = metrics.rssi;
    details["free_heap"] = metrics.freeHeap;
    details["uptime"] = metrics.uptime;

    JsonObject devicesObj = details.createNestedObject("devices");
    for (const auto& r : relays) {
        JsonObject rObj = devicesObj.createNestedObject(r.deviceId);
        rObj["state"] = r.currentState ? "on" : "off";
        rObj["desired_state"] = r.desiredState ? "on" : "off";
        rObj["pin"] = r.pin;
    }

    String output;
    serializeJson(doc, output);
    return output.c_str();
}

std::string ArduinoJsonPayloadSerializer::formatEvent(
    const domain::DeviceIdentity& id,
    const domain::DeviceEvent& event,
    const std::string& timestamp,
    bool isConnected) {

    DynamicJsonDocument doc(1536);

    JsonObject dev = doc.createNestedObject("Device");
    dev["status"] = isConnected ? "connected" : "disconnected";
    dev["ChipId"] = id.chipId;
    dev["MacAddress"] = id.macAddress;
    dev["ChipType"] = id.chipType;
    dev["FirmwareVersion"] = id.firmwareVersion;

    doc["Timestamp"] = timestamp;

    JsonObject details = doc.createNestedObject("Details");
    details["chipId"] = id.chipId;

    JsonObject info = details.createNestedObject("info");
    info["event"] = event.eventName;
    info["description"] = event.description;
    if (!event.state.empty()) {
        info["state"] = event.state;
    }
    info["send_interval"] = 30;
    info["timestamp"] = millis();

    String output;
    serializeJson(doc, output);
    return output.c_str();
}

std::string ArduinoJsonPayloadSerializer::formatTelemetry(
    const domain::DeviceIdentity& id,
    const domain::SystemMetrics& metrics,
    const std::vector<domain::RelayState>& relays,
    const std::string& timestamp) {

    DynamicJsonDocument doc(1536);

    JsonObject dev = doc.createNestedObject("Device");
    dev["status"] = metrics.mqttConnected ? "connected" : "disconnected";
    dev["ChipId"] = id.chipId;
    dev["MacAddress"] = id.macAddress;
    dev["ChipType"] = id.chipType;
    dev["FirmwareVersion"] = id.firmwareVersion;

    doc["Timestamp"] = timestamp;

    JsonObject details = doc.createNestedObject("Details");
    details["rssi"] = metrics.rssi;
    details["free_heap"] = metrics.freeHeap;
    details["uptime"] = metrics.uptime;

    JsonObject relaysObj = details.createNestedObject("relays");
    for (const auto& r : relays) {
        relaysObj[r.deviceId] = r.currentState ? "on" : "off";
    }

    String output;
    serializeJson(doc, output);
    return output.c_str();
}

std::string ArduinoJsonPayloadSerializer::formatAccidentAlert(
    const domain::DeviceIdentity& id,
    const domain::AccidentEvent& event,
    const std::string& timestamp) {

    DynamicJsonDocument doc(2048);

    JsonObject dev = doc.createNestedObject("Device");
    dev["status"] = "alert";
    dev["ChipId"] = id.chipId;
    dev["MacAddress"] = id.macAddress;
    dev["FirmwareVersion"] = id.firmwareVersion;

    doc["Timestamp"] = timestamp;

    JsonObject details = doc.createNestedObject("Details");
    details["chipId"] = id.chipId;

    JsonObject alertObj = details.createNestedObject("alert");
    alertObj["type"] = event.alertType();
    alertObj["impact"] = event.impact_detected;
    alertObj["rollover"] = event.rollover_detected;
    alertObj["harsh_accel"] = event.harsh_accel_detected;
    alertObj["harsh_brake"] = event.harsh_brake_detected;
    alertObj["harsh_cornering"] = event.harsh_cornering_detected;
    alertObj["tow"] = event.tow_detected;
    alertObj["impact_magnitude"] = event.impact_magnitude;
    alertObj["peak_g"] = event.peak_g;
    alertObj["roll_angle"] = event.roll_angle;
    alertObj["pitch_angle"] = event.pitch_angle;
    alertObj["accel_g"] = event.accel_g;
    alertObj["lateral_g"] = event.lateral_g;

    JsonObject gpsObj = details.createNestedObject("gps");
    gpsObj["valid"] = event.location.isValid;
    gpsObj["positioning_source"] = event.location.isValid
                                       ? (event.location.isLbsFallback ? "LBS" : "GPS")
                                       : (event.location.lbsValid ? "LBS_CELL_ID" : "NONE");
    gpsObj["lbs_fallback"] = event.location.isLbsFallback;
    gpsObj["latitude"] = event.location.latitude;
    gpsObj["longitude"] = event.location.longitude;
    gpsObj["speed_kmph"] = event.location.speed;
    gpsObj["altitude_m"] = event.location.altitude;
    gpsObj["course_deg"] = event.location.course;
    gpsObj["hdop"] = event.location.hdop;
    gpsObj["satellites"] = event.location.satellites;
    gpsObj["drift_filtered"] = event.location.driftFiltered;
    gpsObj["tow_detected"] = event.location.towDetected;
    gpsObj["virtual_ignition"] = event.location.virtualIgnition;
    gpsObj["jamming_detected"] = event.location.jammingDetected;
    gpsObj["roaming"] = event.location.roaming;
    if (event.location.lbsValid) {
        JsonObject lbs = gpsObj.createNestedObject("lbs");
        lbs["mcc"] = event.location.mcc;
        lbs["mnc"] = event.location.mnc;
        lbs["lac"] = event.location.lac;
        lbs["cell_id"] = event.location.cellId;
        if (event.location.lbsAccuracyM > 0) {
            lbs["accuracy_m"] = event.location.lbsAccuracyM;
        }
        char cellKey[48];
        snprintf(cellKey, sizeof(cellKey), "%d-%d-%u-%u",
                 event.location.mcc, event.location.mnc,
                 (unsigned)event.location.lac, (unsigned)event.location.cellId);
        lbs["cell_key"] = cellKey;
    }

    String output;
    serializeJson(doc, output);
    return output.c_str();
}

std::string ArduinoJsonPayloadSerializer::formatGpsTelemetry(
    const domain::DeviceIdentity& id,
    const domain::GpsLocation& gps,
    const std::string& timestamp) {

    DynamicJsonDocument doc(2048);

    JsonObject dev = doc.createNestedObject("Device");
    dev["status"] = "operational";
    dev["ChipId"] = id.chipId;
    dev["FirmwareVersion"] = id.firmwareVersion;

    doc["Timestamp"] = timestamp;

    JsonObject details = doc.createNestedObject("Details");
    details["chipId"] = id.chipId;

    JsonObject gpsObj = details.createNestedObject("gps");
    gpsObj["valid"] = gps.isValid;
    gpsObj["positioning_source"] = gps.isValid
                                       ? (gps.isLbsFallback ? "LBS" : "GPS")
                                       : (gps.lbsValid ? "LBS_CELL_ID" : "NONE");
    gpsObj["lbs_fallback"] = gps.isLbsFallback;
    gpsObj["latitude"] = gps.latitude;
    gpsObj["longitude"] = gps.longitude;
    gpsObj["speed_kmph"] = gps.speed;
    gpsObj["altitude_m"] = gps.altitude;
    gpsObj["course_deg"] = gps.course;
    gpsObj["hdop"] = gps.hdop;
    gpsObj["satellites"] = gps.satellites;
    gpsObj["drift_filtered"] = gps.driftFiltered;
    gpsObj["tow_detected"] = gps.towDetected;
    gpsObj["virtual_ignition"] = gps.virtualIgnition;
    gpsObj["jamming_detected"] = gps.jammingDetected;
    gpsObj["roaming"] = gps.roaming;
    if (gps.lbsValid) {
        JsonObject lbs = gpsObj.createNestedObject("lbs");
        lbs["mcc"] = gps.mcc;
        lbs["mnc"] = gps.mnc;
        lbs["lac"] = gps.lac;
        lbs["cell_id"] = gps.cellId;
        if (gps.lbsAccuracyM > 0) {
            lbs["accuracy_m"] = gps.lbsAccuracyM;
        }
        char cellKey[48];
        snprintf(cellKey, sizeof(cellKey), "%d-%d-%u-%u",
                 gps.mcc, gps.mnc, (unsigned)gps.lac, (unsigned)gps.cellId);
        lbs["cell_key"] = cellKey;
    }

    String output;
    serializeJson(doc, output);
    return output.c_str();
}

}
