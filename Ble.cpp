#include "Ble.h"

// ============================================================
//  BLE CALLBACKS
// ============================================================

class CBu16 : public NimBLECharacteristicCallbacks {
    volatile uint16_t* t; const char* n;
public:
    CBu16(volatile uint16_t* t, const char* n) : t(t), n(n) {}
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() >= 2) {
            *t = *(uint16_t*)c->getValue().data();
            Serial.printf("[BLE] %s = %d\n", n, *t);
        }
    }
};

class CBu32 : public NimBLECharacteristicCallbacks {
    volatile uint32_t* t; const char* n;
public:
    CBu32(volatile uint32_t* t, const char* n) : t(t), n(n) {}
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() >= 4) {
            *t = *(uint32_t*)c->getValue().data();
            Serial.printf("[BLE] %s = %d\n", n, *t);
        }
    }
};

// Float stored as uint16 scaled x100 (e.g. 120.0 → 12000, 17.0 → 1700)
class CBfloat : public NimBLECharacteristicCallbacks {
    volatile float* t; const char* n; float scale;
public:
    CBfloat(volatile float* t, const char* n, float scale) : t(t), n(n), scale(scale) {}
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() >= 2) {
            uint16_t raw = *(uint16_t*)c->getValue().data();
            *t = raw / scale;
            Serial.printf("[BLE] %s = %.2f\n", n, *t);
        }
    }
};

class CBcommand : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() < 1) return;

        uint8_t cmd = c->getValue()[0];
        Serial.printf("[BLE] COMMAND = %d\n", cmd);

        switch (cmd) {
            case CMD_HOVER_TEST:
                if (state == IDLE || state == DONE) startHoverTest();
                else Serial.println("[BLE] Ignored hover test command: not idle");
                break;

            case CMD_START_MISSION:
                if (state == IDLE || state == DONE) startMission();
                else Serial.println("[BLE] Ignored mission command: not idle");
                break;

            case CMD_DISARM:
                if (state == HOVER_TEST || state == ALT_HOLD || state == AUTO_HOVER_CAL)
                    startLanding(currentRelAlt);
                else
                    disarmToIdle("[BLE] Disarm command");
                break;

            case CMD_AUTO_HOVER_CAL:
                if (state == IDLE || state == DONE) startAutoHoverCal();
                else Serial.println("[BLE] Ignored auto hover calibration command: not idle");
                break;

            case CMD_ALT_HOLD:
                if (state == IDLE || state == DONE) startAltHold();
                else Serial.println("[BLE] Ignored alt hold command: not idle");
                break;

            default:
                Serial.println("[BLE] Unknown command");
                break;
        }
    }
};

class ServerCB : public NimBLEServerCallbacks {
    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("[BLE] Client disconnected, restarting advertising");
        NimBLEDevice::startAdvertising();
        // Test states land on BLE loss. Mission states continue autonomously.
        if (state == HOVER_TEST || state == ALT_HOLD || state == AUTO_HOVER_CAL) {
            bleSafetyLand = true;
        }
    }
};

class CBbenchMode : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() < 1) return;

        uint8_t requested = c->getValue()[0] ? 1 : 0;
        if (state != IDLE && state != DONE) {
            Serial.println("[BLE] Ignored bench mode change: not idle");
            c->setValue((uint8_t*)&BENCH_MODE_ENABLED, 1);
            return;
        }

        BENCH_MODE_ENABLED = requested;
        benchAlt = 0;
        benchLastMs = 0;
        c->setValue((uint8_t*)&BENCH_MODE_ENABLED, 1);
        Serial.printf("[BLE] BENCH_MODE = %s\n", BENCH_MODE_ENABLED ? "ON" : "OFF");
        if (BENCH_MODE_ENABLED) {
            Serial.println("[BLE] WARNING: simulated altitude only. DO NOT FLY.");
        }
    }
};

static NimBLECharacteristic* makeChar(NimBLEService* svc, const char* uuid,
                                       NimBLECharacteristicCallbacks* cb,
                                       uint8_t* initData, size_t initLen) {
    auto c = svc->createCharacteristic(uuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    c->setCallbacks(cb);
    c->setValue(initData, initLen);
    return c;
}

void setupBLE() {
    NimBLEDevice::init("Quad-Tuner");
    auto* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCB());
    auto* svc    = server->createService(SERVICE_UUID);

    hoverChar = makeChar(svc, HOVER_UUID,
        new CBu16(&HOVER_THROTTLE, "HOVER_THROTTLE"),
        (uint8_t*)&HOVER_THROTTLE, 2);

    makeChar(svc, SPRINT_THROT_UUID,
        new CBu16(&SPRINT_THROTTLE, "SPRINT_THROTTLE"),
        (uint8_t*)&SPRINT_THROTTLE, 2);

    // SPRINT_CUTOFF stored x100 (17.0m → 1700)
    uint16_t scInit = (uint16_t)(SPRINT_CUTOFF_M * 100);
    makeChar(svc, SPRINT_CUTOFF_UUID,
        new CBfloat(&SPRINT_CUTOFF_M, "SPRINT_CUTOFF_M", 100.0f),
        (uint8_t*)&scInit, 2);

    // TARGET_ALT stored x10 (18.3m → 183)
    uint16_t taInit = (uint16_t)(TARGET_ALT_M * 10);
    makeChar(svc, TARGET_ALT_UUID,
        new CBfloat(&TARGET_ALT_M, "TARGET_ALT_M", 10.0f),
        (uint8_t*)&taInit, 2);

    // HOLD_KP/KI/KD stored x10 (120.0 → 1200)
    uint16_t kpInit = (uint16_t)(HOLD_KP * 10);
    makeChar(svc, HOLD_KP_UUID,
        new CBfloat(&HOLD_KP, "HOLD_KP", 10.0f),
        (uint8_t*)&kpInit, 2);

    uint16_t kiInit = (uint16_t)(HOLD_KI * 10);
    makeChar(svc, HOLD_KI_UUID,
        new CBfloat(&HOLD_KI, "HOLD_KI", 10.0f),
        (uint8_t*)&kiInit, 2);

    uint16_t kdInit = (uint16_t)(HOLD_KD * 10);
    makeChar(svc, HOLD_KD_UUID,
        new CBfloat(&HOLD_KD, "HOLD_KD", 10.0f),
        (uint8_t*)&kdInit, 2);

    makeChar(svc, PUNCH_START_UUID,
        new CBu32(&PUNCH_START_MS, "PUNCH_START_MS"),
        (uint8_t*)&PUNCH_START_MS, 4);

    makeChar(svc, PUNCH_THROT_UUID,
        new CBu16(&PUNCH_THROTTLE, "PUNCH_THROTTLE"),
        (uint8_t*)&PUNCH_THROTTLE, 2);

    uint8_t commandInit = 0;
    makeChar(svc, COMMAND_UUID,
        new CBcommand(),
        (uint8_t*)&commandInit, 1);

    makeChar(svc, BENCH_MODE_UUID,
        new CBbenchMode(),
        (uint8_t*)&BENCH_MODE_ENABLED, 1);

    // Telemetry: read + notify, pushed from ESP32 side (no write callback)
    telemetryChar = svc->createCharacteristic(TELEMETRY_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t telInit[7] = {0};
    telemetryChar->setValue(telInit, 7);

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->enableScanResponse(true);
    adv->setName("Quad-Tuner");
    adv->start();
    Serial.println("[BLE] Advertising as 'Quad-Tuner'");
}
