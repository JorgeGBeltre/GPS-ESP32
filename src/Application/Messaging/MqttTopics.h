#pragma once

#include <string>

namespace relay::app {

// Tópicos MQTT del equipo, TODOS con ámbito por dispositivo.
//
// Antes eran literales compartidos ("telemetry", "events", "commands", "ota", "status") y
// los de cada relé no llevaban identificador ("device_switch1/command"). En un broker
// multi-tenant eso significa que todos los equipos comparten los mismos tópicos: cualquiera
// recibía los comandos y la telemetría de los demás, y no había forma de que el servidor
// dirigiera un comando a UN relé concreto. Ahora cada tópico lleva el chipId, que es lo que
// el broker verifica contra la ApiKey del dispositivo al conectar.
struct MqttTopics {
    std::string telemetry;     // telemetry/<chipId>
    std::string events;        // events/<chipId>
    std::string commands;      // commands/<chipId>
    std::string status;        // status/<chipId>
    std::string deviceEvents;  // event/<chipId>
    std::string devicePrefix;  // devices/<chipId>
    std::string gps;           // telemetry/<chipId>/gps
    std::string alerts;        // telemetry/<chipId>/accidents

    static MqttTopics forDevice(const std::string& chipId) {
        MqttTopics t;
        t.telemetry    = "telemetry/" + chipId;
        t.events       = "events/" + chipId;
        t.commands     = "commands/" + chipId;
        t.status       = "status/" + chipId;
        t.deviceEvents = "event/" + chipId;
        t.devicePrefix = "devices/" + chipId;
        t.gps          = "telemetry/" + chipId + "/gps";
        t.alerts       = "telemetry/" + chipId + "/accidents";
        return t;
    }

    // Tópico de un relé concreto: devices/<chipId>/<base><suffix>
    // Ejemplos: devices/A1B2C3/device_switch1/command, devices/A1B2C3/device_switch1_state
    std::string relay(const std::string& base, const std::string& suffix) const {
        return devicePrefix + "/" + base + suffix;
    }
};

// El tópico de ACTUALIZACIÓN OTA (ota/<chipId>) lo gestiona directamente la librería
// MQTTOTAv5, que se suscribe por sí misma en ota.begin(). No se declara aquí para no
// suscribirse dos veces al mismo tópico.

}
