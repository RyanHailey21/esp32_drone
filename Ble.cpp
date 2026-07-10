#include "Ble.h"
#include <NimBLEDevice.h>
#include "State.h"
#include "Control.h"
#include "FlightLog.h"

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

// Float stored as uint16 with per-characteristic scale (e.g. HOLD_KP x10, altitude x100).
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
                    bleRequestedLand = true;
                else
                    disarmToIdle("[BLE] Disarm command");
                break;

            case CMD_KILL:
                disarmToIdle("[BLE] Kill command");
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
        bool armingTestMode = state == ARMING
            && (armTarget == ARM_HOVER_TEST || armTarget == ARM_AUTO_HOVER_CAL || armTarget == ARM_ALT_HOLD);
        if (state == HOVER_TEST || state == ALT_HOLD || state == AUTO_HOVER_CAL || armingTestMode) {
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

class CBangleMode : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() < 1) return;

        uint8_t requested = c->getValue()[0] ? 1 : 0;
        if (state != IDLE && state != DONE) {
            Serial.println("[BLE] Ignored angle mode change: not idle");
            c->setValue((uint8_t*)&ANGLE_MODE_ENABLED, 1);
            return;
        }

        ANGLE_MODE_ENABLED = requested;
        channels[CH_ANGLE] = ANGLE_MODE_ENABLED ? 1800 : 1000;
        c->setValue((uint8_t*)&ANGLE_MODE_ENABLED, 1);
        Serial.printf("[BLE] ANGLE_MODE = %s\n", ANGLE_MODE_ENABLED ? "ON" : "OFF");
    }
};

class CBmissionType : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (c->getValue().length() < 1) return;

        uint8_t requested = c->getValue()[0] ? 1 : 0;
        if (state != IDLE && state != DONE) {
            Serial.println("[BLE] Ignored mission type change: not idle");
            c->setValue((uint8_t*)&MISSION_TYPE, 1);
            return;
        }

        MISSION_TYPE = requested;
        c->setValue((uint8_t*)&MISSION_TYPE, 1);
        Serial.printf("[BLE] MISSION_TYPE = %s\n",
            MISSION_TYPE ? "AUTOROTOR_CUT" : "POWERED_LAND");
    }
};

class CBflightLogOffset : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (state != IDLE && state != DONE) return;
        if (c->getValue().length() < 2) return;
        uint16_t offset = *(uint16_t*)c->getValue().data();
        flightLogSetReadOffset(offset);
    }
};

class CBflightLogChunk : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (state != IDLE && state != DONE) {
            uint8_t empty = 0;
            c->setValue(&empty, 0);
            return;
        }
        uint8_t chunk[FLIGHT_LOG_CHUNK_BYTES];
        uint16_t n = flightLogReadChunk(chunk, sizeof(chunk));
        c->setValue(chunk, n);
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

static void makeFloat(NimBLEService* svc, const char* uuid,
                      volatile float* var, const char* name, float scale) {
    uint16_t init = (uint16_t)(*var * scale);
    makeChar(svc, uuid, new CBfloat(var, name, scale), (uint8_t*)&init, 2);
}

void setupBLE() {
    NimBLEDevice::init("Quad-Tuner");
    NimBLEDevice::setMTU(128);
    auto* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCB());
    auto* svc    = server->createService(SERVICE_UUID);

    hoverChar = makeChar(svc, HOVER_UUID,
        new CBu16(&HOVER_THROTTLE, "HOVER_THROTTLE"),
        (uint8_t*)&HOVER_THROTTLE, 2);

    makeChar(svc, SPRINT_THROT_UUID,
        new CBu16(&SPRINT_THROTTLE, "SPRINT_THROTTLE"),
        (uint8_t*)&SPRINT_THROTTLE, 2);

    makeFloat(svc, SPRINT_CUTOFF_UUID, &SPRINT_CUTOFF_M, "SPRINT_CUTOFF_M", 100.0f);
    makeFloat(svc, TARGET_ALT_UUID,    &TARGET_ALT_M,    "TARGET_ALT_M",    10.0f);
    makeFloat(svc, ALT_HOLD_TARGET_UUID, &ALT_HOLD_TARGET_M, "ALT_HOLD_TARGET_M", 10.0f);
    makeFloat(svc, HOLD_KP_UUID,       &HOLD_KP,         "HOLD_KP",         10.0f);
    makeFloat(svc, HOLD_KI_UUID,       &HOLD_KI,         "HOLD_KI",         10.0f);
    makeFloat(svc, HOLD_KD_UUID,       &HOLD_KD,         "HOLD_KD",         10.0f);
    makeFloat(svc, ALT_RAMP_RATE_UUID, &ALT_RAMP_RATE_MPS, "ALT_RAMP_RATE_MPS", 100.0f);
    makeFloat(svc, MAX_CLIMB_TEST_UUID, &MAX_CLIMB_MPS_TEST, "MAX_CLIMB_MPS_TEST", 100.0f);
    makeFloat(svc, MAX_DESCENT_TEST_UUID, &MAX_DESCENT_MPS_TEST, "MAX_DESCENT_MPS_TEST", 100.0f);
    makeFloat(svc, BF_GROUND_EFFECT_UUID, &BF_VARIO_GROUND_EFFECT_M, "BF_VARIO_GROUND_EFFECT_M", 100.0f);

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

    makeChar(svc, ANGLE_MODE_UUID,
        new CBangleMode(),
        (uint8_t*)&ANGLE_MODE_ENABLED, 1);

    makeChar(svc, MISSION_TYPE_UUID,
        new CBmissionType(),
        (uint8_t*)&MISSION_TYPE, 1);

    uint16_t logOffsetInit = 0;
    makeChar(svc, FLIGHT_LOG_OFFSET_UUID,
        new CBflightLogOffset(),
        (uint8_t*)&logOffsetInit, 2);

    auto* flightLogChunkChar = svc->createCharacteristic(FLIGHT_LOG_CHUNK_UUID,
        NIMBLE_PROPERTY::READ);
    flightLogChunkChar->setCallbacks(new CBflightLogChunk());
    uint8_t emptyLogInit = 0;
    flightLogChunkChar->setValue(&emptyLogInit, 0);

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
