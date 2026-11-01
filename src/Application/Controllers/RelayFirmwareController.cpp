#include "Application/Controllers/RelayFirmwareController.h"
#include <cstdlib>

namespace relay::app {

RelayFirmwareController::RelayFirmwareController(
    RelayService& relays,
    ConnectivityService& connectivity,
    TelemetryService& telemetry,
    domain::IOtaService& ota,
    domain::IIndicator& indicator,
    domain::ISystemControl& system,
    domain::ITimeProvider& timeProvider,
    domain::IClock& clock,
    domain::ILogger& logger,
    const MqttTopics& topics,
    ApTriggerCallback apCallback,
    AccidentService* accidentService,
    domain::IGpsProvider* gpsProvider,
    TimerSchedulerService* timerScheduler,
    domain::IConfigRepository* configRepo,
    unsigned long statusIntervalMs,
    unsigned long timeSyncIntervalMs,
    unsigned long gpsIntervalMs)
    : relays_(relays), connectivity_(connectivity), telemetry_(telemetry),
      ota_(ota), indicator_(indicator), system_(system), timeProvider_(timeProvider),
      clock_(clock), log_(logger), topics_(topics), apCallback_(std::move(apCallback)),
      accidentService_(accidentService), gpsProvider_(gpsProvider),
      timerScheduler_(timerScheduler), configRepo_(configRepo),
      statusIntervalMs_(statusIntervalMs), timeSyncIntervalMs_(timeSyncIntervalMs),
      gpsIntervalMs_(gpsIntervalMs) {}

void RelayFirmwareController::registerButton(domain::IButtonHandler& button,
                                             const std::string& targetDeviceId) {
    button.init();
    buttons_.push_back(ButtonBinding{&button, targetDeviceId});
}

void RelayFirmwareController::updateIndicatorForConnectivity() {
    if (ota_.isInProgress()) {
        indicator_.show(domain::IndicatorStatus::OtaProgress);
        return;
    }
    if (!connectivity_.network().isConnected()) {
        indicator_.show(domain::IndicatorStatus::Disconnected);
    } else if (!connectivity_.transport().isConnected()) {
        indicator_.show(domain::IndicatorStatus::Connecting);
    } else {
        indicator_.show(domain::IndicatorStatus::Operational);
    }
}

void RelayFirmwareController::setup(bool relaysAlreadyInitialized) {
    if (!relaysAlreadyInitialized) {
        log_.info("[CTRL] Inicializando sistema de relés...");
        relays_.init();
    }

    if (timerScheduler_) {
        timerScheduler_->loadFromStorage();
    }

    if (gpsProvider_) {
        log_.info("[CTRL] Inicializando proveedor GPS...");
        gpsProvider_->begin();
    }

    indicator_.show(domain::IndicatorStatus::Disconnected);

    connectivity_.transport().onMessage([this](const std::string& topic, const std::string& payload) {
        this->handleIncomingMessage(topic, payload);
    });

    connectivity_.transport().onConnect([this]() {
        this->onMqttConnected();
    });

    if (connectivity_.connect()) {
        state_ = State::Operational;
        indicator_.show(domain::IndicatorStatus::Operational);
        log_.info("[CTRL] Sistema OPERACIONAL");
    } else {
        state_ = State::Connecting;
        lastReconnectAttemptMs_ = clock_.nowMs();
        hasAttemptedReconnect_ = true;
        updateIndicatorForConnectivity();
        log_.info("[CTRL] Operando en modo OFFLINE-FIRST (Botón físico, GNSS/MPU6050 y LAN activos)");
    }
}

void RelayFirmwareController::onMqttConnected() {
    log_.info("[CTRL] MQTT conectado: suscribiendo a tópicos...");

    connectivity_.transport().subscribe(topics_.commands);
    connectivity_.transport().subscribe(topics_.deviceEvents);

    for (const auto& r : relays_.getAllStates()) {
        connectivity_.transport().subscribe(topics_.relay(r.topic, "/state"));
        connectivity_.transport().subscribe(topics_.relay(r.topic, "/command"));
        connectivity_.transport().subscribe(topics_.relay(r.topic, "_state"));
    }

    telemetry_.publishEvent("MQTT_CONNECTED", "Conexión MQTTv5 establecida");
    telemetry_.requestCurrentStates();
    telemetry_.publishStatus();

    if (gpsProvider_) {
        telemetry_.publishGpsTelemetry();
    }
}

void RelayFirmwareController::tick() {
    const unsigned long now = clock_.nowMs();

    // 1. Entrada física y temporizadores locales siempre activos (Offline-First)
    handleButtonEvents();
    relays_.flushDeferredSave(now);

    std::string pulsedDevice;
    if (relays_.updatePulseTimer(now, pulsedDevice)) {
        log_.info("[PULSE] Auto-OFF por temporizador de pulso en " + pulsedDevice);
        if (onLocalStateChangeCb_) {
            onLocalStateChangeCb_();
        }
        if (connectivity_.isOnline()) {
            telemetry_.publishEvent("PULSE_AUTO_OFF", "Pulso finalizado automáticamente", "off");
            telemetry_.publishRelayState(pulsedDevice);
        }
    }

    if (timerScheduler_) {
        const uint32_t epochSec = timeProvider_.epochSeconds();
        if (epochSec > 1000000000U && epochSec != lastEvaluatedEpochSec_) {
            lastEvaluatedEpochSec_ = epochSec;
            if (timerScheduler_->evaluate(epochSec, now) > 0) {
                if (onLocalStateChangeCb_) {
                    onLocalStateChangeCb_();
                }
                if (connectivity_.isOnline()) {
                    telemetry_.publishEvent("TIMER_TRIGGERED", "Temporizador programado ejecutado");
                    telemetry_.publishAllRelayStates();
                }
            }
        }
    }

    // 2. GPS + Detección de Accidentes (operan en modo Offline-First y reportan por SMS/GPRS si no hay Wi-Fi)
    if (gpsProvider_) {
        gpsProvider_->update();
    }
    if (accidentService_) {
        accidentService_->process();
    }

    // 3. Indicador visual (máquina de estados de 100 ms no bloqueante)
    indicator_.update();

    // 4. Gestión OTA
    if (ota_.isInProgress()) {
        indicator_.show(domain::IndicatorStatus::OtaProgress);
        ota_.handle();
        clock_.sleepMs(10);
        return;
    }
    ota_.handle();

    // 5. Máquina de estados de conectividad red / MQTTv5
    if (state_ == State::Connecting) {
        updateIndicatorForConnectivity();
        if (gpsProvider_ && now - lastGpsPublishMs_ >= RECONNECT_INTERVAL_MS) {
            lastGpsPublishMs_ = now;
            domain::GpsLocation loc;
            gpsProvider_->readData(loc);
        }

        if (!hasAttemptedReconnect_ || now - lastReconnectAttemptMs_ >= RECONNECT_INTERVAL_MS) {
            hasAttemptedReconnect_ = true;
            lastReconnectAttemptMs_ = now;
            if (connectivity_.ensureConnected()) {
                state_ = State::Operational;
                indicator_.show(domain::IndicatorStatus::Operational);
                log_.info("[CTRL] Estado -> OPERATIONAL");
            } else {
                updateIndicatorForConnectivity();
            }
        }
    } else if (state_ == State::Operational) {
        if (!connectivity_.isOnline()) {
            updateIndicatorForConnectivity();
            lastReconnectAttemptMs_ = now;
            hasAttemptedReconnect_ = true;
            if (!connectivity_.ensureConnected()) {
                state_ = State::Connecting;
                updateIndicatorForConnectivity();
                return;
            }
        }

        indicator_.show(domain::IndicatorStatus::Operational);
        connectivity_.transport().loop();

        if (now - lastStatusPublishMs_ >= statusIntervalMs_) {
            lastStatusPublishMs_ = now;
            telemetry_.publishStatus();
        }

        if (gpsProvider_ && (now - lastGpsPublishMs_ >= gpsIntervalMs_)) {
            lastGpsPublishMs_ = now;
            telemetry_.publishGpsTelemetry();
        }

        if (now - lastTimeSyncMs_ >= timeSyncIntervalMs_) {
            lastTimeSyncMs_ = now;
            timeProvider_.sync();
        }
    }
}

void RelayFirmwareController::handleButtonEvents() {
    const unsigned long now = clock_.nowMs();
    for (auto& btn : buttons_) {
        if (!btn.handler) continue;
        auto ev = btn.handler->update();

        if (ev == domain::ButtonEvent::ShortPress) {
            log_.info("[BUTTON] Pulsación corta detectada para " + btn.targetDeviceId);
            relays_.toggle(btn.targetDeviceId, now);

            if (onLocalStateChangeCb_) {
                onLocalStateChangeCb_();
            }

            if (connectivity_.isOnline()) {
                const auto* state = relays_.getState(btn.targetDeviceId);
                std::string stateStr = (state && state->currentState) ? "on" : "off";
                std::string desc = "Cambio por botón físico - " + btn.targetDeviceId + ": " +
                                   (state && state->currentState ? "ENCENDIDO" : "APAGADO");

                telemetry_.publishEvent("BUTTON_PRESS", desc, stateStr);
                telemetry_.publishRelayState(btn.targetDeviceId);
            }
        } else if (ev == domain::ButtonEvent::LongPress) {
            log_.info("[BUTTON] Pulsación larga (>= 5 s) detectada -> Iniciando emparejamiento");
            if (connectivity_.isOnline()) {
                telemetry_.publishEvent("PAIRING_MODE_ACTIVATED", "Modo emparejamiento activado por botón físico");
                telemetry_.publishStatus();
            }

            if (apCallback_) {
                apCallback_();
                updateIndicatorForConnectivity();
            } else {
                system_.reboot();
            }
        }
    }
}

void RelayFirmwareController::handleIncomingMessage(const std::string& topic, const std::string& payload) {
    log_.info("[CTRL] Mensaje recibido en [" + topic + "]: " + payload);

    if (topic == topics_.commands) {
        processCommand(payload);
        return;
    }

    const unsigned long now = clock_.nowMs();

    for (const auto& r : relays_.getAllStates()) {
        std::string setTopic   = topics_.relay(r.topic, "/state");
        std::string cmdTopic   = topics_.relay(r.topic, "/command");
        std::string stateTopic = topics_.relay(r.topic, "_state");

        if (topic == setTopic || topic == cmdTopic) {
            bool turnOn = (payload == "on" || payload == "1" || payload == "true");
            bool turnOff = (payload == "off" || payload == "0" || payload == "false");
            bool doToggle = (payload == "toggle");

            if (turnOn) {
                relays_.setDesiredState(r.deviceId, true);
                relays_.applyDesiredState(r.deviceId, now);
            } else if (turnOff) {
                relays_.setDesiredState(r.deviceId, false);
                relays_.applyDesiredState(r.deviceId, now);
            } else if (doToggle) {
                relays_.toggle(r.deviceId, now);
            }

            if (onLocalStateChangeCb_) {
                onLocalStateChangeCb_();
            }

            const auto* st = relays_.getState(r.deviceId);
            std::string stateStr = (st && st->currentState) ? "on" : "off";
            telemetry_.publishEvent("DEVICE_SET_COMMAND", "Comando recibido: " + payload, stateStr);
            telemetry_.publishRelayState(r.deviceId);
            return;
        } else if (topic == stateTopic) {
            bool serverState = (payload == "on" || payload == "1" || payload == "true");

            if (!relays_.isInitialSyncCompleted()) {
                relays_.setInitialSyncCompleted(true);
                if (relays_.getStartupMode() != domain::StartupMode::Stay) {
                    relays_.setDesiredState(r.deviceId, serverState);
                    relays_.applyDesiredState(r.deviceId, now);
                }
                const auto* st = relays_.getState(r.deviceId);
                telemetry_.publishEvent("INITIAL_STATE", "Sincronización inicial completada",
                                        (st && st->currentState) ? "on" : "off");
            } else {
                relays_.setDesiredState(r.deviceId, serverState);
                relays_.applyDesiredState(r.deviceId, now);
                telemetry_.publishEvent("STATE_UPDATE", "Actualización de estado desde servidor",
                                        serverState ? "on" : "off");
            }

            if (onLocalStateChangeCb_) {
                onLocalStateChangeCb_();
            }
            return;
        }
    }
}

void RelayFirmwareController::processCommand(const std::string& cmd) {
    const unsigned long now = clock_.nowMs();
    if (cmd == "reset") {
        relays_.flushDeferredSave(now + 10000UL);
        telemetry_.publishEvent("SYSTEM_RESET", "Reinicio del sistema solicitado");
        clock_.sleepMs(500);
        system_.reboot();
    } else if (cmd == "status") {
        telemetry_.publishStatus();
    } else if (cmd == "toggle_all") {
        telemetry_.publishEvent("RELAY_TOGGLE_ALL", "Todos los relays cambiados de estado");
        relays_.toggleAll(now);
        if (onLocalStateChangeCb_) onLocalStateChangeCb_();
        telemetry_.publishStatus();
        telemetry_.publishAllRelayStates();
    } else if (cmd == "sync_states") {
        telemetry_.publishEvent("STATE_SYNC", "Sincronización de estados solicitada");
        relays_.setInitialSyncCompleted(false);
        telemetry_.requestCurrentStates();
    } else if (cmd == "gps") {
        telemetry_.publishGpsTelemetry();
    } else if (cmd == "diagnose_gps") {
        if (gpsProvider_) gpsProvider_->diagnose();
    } else if (cmd.rfind("startup:", 0) == 0) {
        std::string modeStr = cmd.substr(8);
        auto mode = domain::startupModeFromString(modeStr);
        relays_.setStartupMode(mode);
        telemetry_.publishEvent("STARTUP_CONFIG", "Modo de arranque actualizado a " +
                                std::string(domain::startupModeToString(mode)));
    } else if (cmd.rfind("pulse:", 0) == 0) {
        std::string rest = cmd.substr(6);
        auto sep = rest.find_first_of(":,");
        bool en = false;
        uint32_t widthMs = relays_.getPulseWidthMs();
        if (sep == std::string::npos) {
            en = (rest == "on" || rest == "1" || rest == "true");
        } else {
            std::string statePart = rest.substr(0, sep);
            std::string widthPart = rest.substr(sep + 1);
            en = (statePart == "on" || statePart == "1" || statePart == "true");
            uint32_t parsed = static_cast<uint32_t>(std::strtoul(widthPart.c_str(), nullptr, 10));
            if (parsed >= 500) widthMs = parsed;
        }
        relays_.configurePulse(en, widthMs, now);
        telemetry_.publishEvent("PULSE_CONFIG", "Configuración de pulso actualizada", en ? "on" : "off");
    } else if (cmd.rfind("sled:", 0) == 0) {
        std::string sub = cmd.substr(5);
        bool en = (sub == "on" || sub == "1" || sub == "true");
        indicator_.setSledOnline(en);
        if (configRepo_) {
            configRepo_->saveSledOnline(en);
        }
        telemetry_.publishEvent("SLED_CONFIG", "LED de estado en línea actualizado", en ? "on" : "off");
    } else if (cmd.rfind("switch1:", 0) == 0) {
        std::string subcmd = cmd.substr(8);
        telemetry_.publishEvent("DEVICE_COMMAND", "Comando switch1: " + subcmd);
        if (subcmd == "on" || subcmd == "1" || subcmd == "true") {
            for (const auto& r : relays_.getAllStates()) {
                relays_.setDesiredState(r.deviceId, true);
                relays_.applyDesiredState(r.deviceId, now);
                if (onLocalStateChangeCb_) onLocalStateChangeCb_();
                telemetry_.publishRelayState(r.deviceId);
                break;
            }
        } else if (subcmd == "off" || subcmd == "0" || subcmd == "false") {
            for (const auto& r : relays_.getAllStates()) {
                relays_.setDesiredState(r.deviceId, false);
                relays_.applyDesiredState(r.deviceId, now);
                if (onLocalStateChangeCb_) onLocalStateChangeCb_();
                telemetry_.publishRelayState(r.deviceId);
                break;
            }
        } else if (subcmd == "toggle") {
            for (const auto& r : relays_.getAllStates()) {
                relays_.toggle(r.deviceId, now);
                if (onLocalStateChangeCb_) onLocalStateChangeCb_();
                telemetry_.publishRelayState(r.deviceId);
                break;
            }
        }
    }
}

} // namespace relay::app
