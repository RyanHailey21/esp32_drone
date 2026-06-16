#include <NimBLEDevice.h>

// Runtime BLE-controlled bench mode for desk testing without a flight controller.
// Defaults off on every boot. Never fly with bench mode enabled.
volatile uint8_t BENCH_MODE_ENABLED = 0;

// ── Pins ─────────────────────────────────────────────────────
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define MOTOR_PWM_PIN   6
#define STATUS_LED      8

// ── Brushed Motor PWM ────────────────────────────────────────
#define PWM_FREQ        25000
#define PWM_RESOLUTION  8
#define MOTOR_DUTY      255

// ── MSP Commands ─────────────────────────────────────────────
#define MSP_SET_RAW_RC  200
#define MSP_ALTITUDE    109

// ── Tunable Parameters (BLE-writable) ────────────────────────

// Hover / baseline
volatile uint16_t HOVER_THROTTLE  = 1420;  // neutral hover, find in hover test
// Sprint phase
volatile uint16_t SPRINT_THROTTLE = 1850;  // full climb throttle (~85%)
volatile float    SPRINT_CUTOFF_M = 17.0;  // ~56ft: transition sprint→hold early
                                            // avoids baro lag overshoot past 60ft
// Hold phase — simple P controller
volatile float    TARGET_ALT_M    = 18.3;  // 60ft target
volatile float    HOLD_KP         = 120.0; // P gain: throttle_add = KP * error_m
                                            // e.g. 0.5m low → adds 60µs to hover
// Punch phase
volatile uint32_t PUNCH_START_MS  = 7500;  // ms from launch when punch begins
volatile uint16_t PUNCH_THROTTLE  = 2000;  // max throttle for final burst

// ── BLE UUIDs ────────────────────────────────────────────────
#define SERVICE_UUID        "ab0828b1-198e-4351-b779-901fa0e0371e"
#define HOVER_UUID          "ab0828b2-198e-4351-b779-901fa0e0371e"
#define CEILING_UUID        "ab0828b3-198e-4351-b779-901fa0e0371e"  // unused now, kept for compat
#define PRESPIN_UUID        "ab0828b4-198e-4351-b779-901fa0e0371e"  // unused now, kept for compat
#define SPRINT_THROT_UUID   "ab0828b5-198e-4351-b779-901fa0e0371e"
#define SPRINT_CUTOFF_UUID  "ab0828b6-198e-4351-b779-901fa0e0371e"
#define HOLD_KP_UUID        "ab0828b7-198e-4351-b779-901fa0e0371e"
#define PUNCH_START_UUID    "ab0828b8-198e-4351-b779-901fa0e0371e"
#define PUNCH_THROT_UUID    "ab0828b9-198e-4351-b779-901fa0e0371e"
#define COMMAND_UUID        "ab0828ba-198e-4351-b779-901fa0e0371e"
#define BENCH_MODE_UUID     "ab0828bb-198e-4351-b779-901fa0e0371e"

#define CMD_HOVER_TEST      1
#define CMD_START_MISSION   2
#define CMD_DISARM          3
#define CMD_AUTO_HOVER_CAL  4

#define CAL_START_THROTTLE  1150
#define CAL_MAX_THROTTLE    1650
#define CAL_STEP_US         5
#define CAL_STEP_MS         250
#define CAL_LIFTOFF_M       0.12f
#define CAL_BACKOFF_US      25
#define CAL_TIMEOUT_MS      30000

#define BENCH_SPRINT_RATE_MPS     9.0f
#define BENCH_PUNCH_RATE_MPS      7.0f
#define BENCH_HOVER_LIFTOFF_US    1325
#define BENCH_HOVER_CAL_RATE_MPS  0.8f

// ── RC Channels ──────────────────────────────────────────────
// CH1=Roll CH2=Pitch CH3=Throttle CH4=Yaw CH5=AUX1(Arm) CH6=AUX2(Angle)
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};

// ── State Machine ────────────────────────────────────────────
enum MissionState {
    IDLE,
    ARMING,
    SPRINTING,   // full throttle to SPRINT_CUTOFF_M
    HOLDING,     // P-controller holds TARGET_ALT_M
    PUNCHING,    // max throttle final burst
    CUT,
    HOVER_TEST,
    AUTO_HOVER_CAL,
    LANDING,
    DONE
};
MissionState state = IDLE;

uint32_t launchTime = 0;
uint32_t armTime    = 0;
bool     armingForHover   = false;
bool     armingForAutoCal = false;
uint32_t calTime    = 0;
uint32_t calStepTime = 0;
uint16_t calThrottle = CAL_START_THROTTLE;
float    launchAlt  = 0;
bool     prespunUp  = false;
float    benchAlt = 0;
uint32_t benchLastMs = 0;

uint32_t landingStartMs       = 0;
uint16_t landingStartThrottle = 1000;
#define  LANDING_TIME_MS      2500

NimBLECharacteristic* hoverChar = nullptr;

HardwareSerial fcSerial(1);

void startHoverTest();
void startMission();
void startAutoHoverCal();
void startLanding(uint16_t currentThrottle);
void disarmToIdle(const char* reason);


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
                if (state == HOVER_TEST || state == AUTO_HOVER_CAL)
                    startLanding(channels[2]);
                else
                    disarmToIdle("[BLE] Disarm command");
                break;

            case CMD_AUTO_HOVER_CAL:
                if (state == IDLE || state == DONE) startAutoHoverCal();
                else Serial.println("[BLE] Ignored auto hover calibration command: not idle");
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

NimBLECharacteristic* makeChar(NimBLEService* svc, const char* uuid,
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

    // HOLD_KP stored x10 (120.0 → 1200)
    uint16_t kpInit = (uint16_t)(HOLD_KP * 10);
    makeChar(svc, HOLD_KP_UUID,
        new CBfloat(&HOLD_KP, "HOLD_KP", 10.0f),
        (uint8_t*)&kpInit, 2);

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

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->enableScanResponse(true);
    adv->setName("Quad-Tuner");
    adv->start();
    Serial.println("[BLE] Advertising as 'Quad-Tuner'");
}


// ============================================================
//  MSP
// ============================================================

void sendMSP(uint8_t cmd, uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    fcSerial.write('$');
    fcSerial.write('M');
    fcSerial.write('<');
    fcSerial.write(len); cs ^= len;
    fcSerial.write(cmd); cs ^= cmd;
    for (int i = 0; i < len; i++) { fcSerial.write(data[i]); cs ^= data[i]; }
    fcSerial.write(cs);
}

void sendRC() {
    uint8_t data[16];
    for (int i = 0; i < 8; i++) {
        data[i*2]   = channels[i] & 0xFF;
        data[i*2+1] = channels[i] >> 8;
    }
    sendMSP(MSP_SET_RAW_RC, data, 16);
}

float getAltitude() {
    if (!BENCH_MODE_ENABLED) {
        while (fcSerial.available()) fcSerial.read();
        uint8_t empty = 0;
        sendMSP(MSP_ALTITUDE, &empty, 0);
        uint32_t timeout = millis() + 50;
        while (fcSerial.available() < 9 && millis() < timeout);
        if (fcSerial.available() < 9) return 0;
        while (fcSerial.available() >= 9) {
            if (fcSerial.read() != '$') continue;
            if (fcSerial.read() != 'M') continue;
            if (fcSerial.read() != '>') continue;
            fcSerial.read(); fcSerial.read();  // len, cmd
            int32_t alt_cm = 0;
            alt_cm  = (uint32_t)fcSerial.read();
            alt_cm |= (uint32_t)fcSerial.read() << 8;
            alt_cm |= (uint32_t)fcSerial.read() << 16;
            alt_cm |= (uint32_t)fcSerial.read() << 24;
            fcSerial.read();  // checksum
            return alt_cm / 100.0f;
        }
        return 0;
    }

    uint32_t now = millis();
    if (benchLastMs == 0) benchLastMs = now;
    float dt = (now - benchLastMs) / 1000.0f;
    benchLastMs = now;

    switch (state) {
        case IDLE:
            benchAlt = 0;
            break;

        case ARMING:
        case HOVER_TEST:
            benchAlt = launchAlt;
            break;

        case AUTO_HOVER_CAL:
            if (calThrottle >= BENCH_HOVER_LIFTOFF_US) {
                benchAlt += BENCH_HOVER_CAL_RATE_MPS * dt;
            } else {
                benchAlt = launchAlt;
            }
            break;

        case SPRINTING:
            benchAlt += BENCH_SPRINT_RATE_MPS * dt;
            break;

        case HOLDING:
            benchAlt = launchAlt + TARGET_ALT_M;
            break;

        case PUNCHING:
            benchAlt += BENCH_PUNCH_RATE_MPS * dt;
            break;

        case CUT:
        case DONE:
            break;
    }

    return benchAlt;
}


// ============================================================
//  ALTITUDE HOLD — simple P controller
//  error > 0: below target → add throttle
//  error < 0: above target → reduce throttle
// ============================================================

uint16_t holdThrottle(float altitude) {
    float error = TARGET_ALT_M - altitude;
    int16_t correction = (int16_t)(HOLD_KP * error);
    return (uint16_t)constrain(HOVER_THROTTLE + correction, 1200, 1700);
}

void startHoverTest() {
    launchAlt = getAltitude();
    armTime = millis();
    armingForHover = true;
    state = ARMING;
    Serial.println("[STATE] -> ARMING (hover test)");
}

void startMission() {
    launchAlt = getAltitude();
    armTime = millis();
    state   = ARMING;
    Serial.println("[STATE] -> ARMING");
}

void startAutoHoverCal() {
    launchAlt = getAltitude();
    calTime = millis();
    calStepTime = millis();
    calThrottle = CAL_START_THROTTLE;
    armTime = millis();
    armingForAutoCal = true;
    state = ARMING;
    Serial.println("[STATE] -> ARMING (auto hover cal)");
}

void startLanding(uint16_t currentThrottle) {
    landingStartMs       = millis();
    landingStartThrottle = currentThrottle;
    state = LANDING;
    Serial.printf("[STATE] -> LANDING from throttle=%d (%.1fs ramp)\n",
        currentThrottle, LANDING_TIME_MS / 1000.0f);
}

void disarmToIdle(const char* reason) {
    ledcWrite(MOTOR_PWM_PIN, 0);
    channels[4] = 1000;
    channels[5] = 1000;
    channels[2] = 1000;
    sendRC();
    state = IDLE;
    Serial.println(reason);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    fcSerial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_PWM_PIN, 0);

    pinMode(STATUS_LED, OUTPUT);

    setupBLE();

    Serial.println("[BOOT] Ready. Control via BLE — open quad_tuner.html in Chrome.");
    Serial.println("[BOOT] Mission profile: SPRINT → HOLD → PUNCH → CUT");
}


// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
    float    altitude     = getAltitude() - launchAlt;
    uint32_t missionTime  = millis() - launchTime;  // time since motors started

    switch (state) {

        // ── IDLE ─────────────────────────────────────────────
        case IDLE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 1000 < 100);

            break;

        // ── ARMING ───────────────────────────────────────────
        case ARMING:
            channels[4] = 1800;
            channels[5] = 1800;
            channels[2] = 1000;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 200 < 100);

            if (millis() - armTime > 1500) {
                if (armingForHover) {
                    armingForHover = false;
                    state = HOVER_TEST;
                    Serial.println("[STATE] → HOVER TEST");
                } else if (armingForAutoCal) {
                    armingForAutoCal = false;
                    calTime = millis();
                    calStepTime = millis();
                    state = AUTO_HOVER_CAL;
                    Serial.println("[STATE] → AUTO HOVER CAL");
                } else {
                    launchTime = millis();
                    prespunUp  = false;
                    state      = SPRINTING;
                    Serial.println("[STATE] → SPRINTING");
                }
            }
            break;

        // ── SPRINTING ────────────────────────────────────────
        // Full throttle until SPRINT_CUTOFF_M (~56ft)
        // Transitions slightly below 60ft to avoid baro-lag overshoot
        case SPRINTING:
            channels[2] = SPRINT_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);  // rapid blink = climbing hard

            Serial.printf("[SPRINT] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[2]);

            // Safety: if somehow 8s passes during sprint, cut
            if (missionTime >= 8000) { state = CUT; break; }

            if (altitude >= SPRINT_CUTOFF_M) {
                // Kick autorotation motor on entry to hold
                ledcWrite(MOTOR_PWM_PIN, MOTOR_DUTY);
                prespunUp = true;
                state     = HOLDING;
                Serial.printf("[STATE] → HOLDING at %.2fm (target %.2fm)\n",
                    altitude, TARGET_ALT_M);
            }
            break;

        // ── HOLDING ──────────────────────────────────────────
        // P controller keeps quad at TARGET_ALT_M (60ft)
        // Autorotation motor already spinning from SPRINTING exit
        // Transitions to PUNCH at PUNCH_START_MS
        case HOLDING: {
            uint16_t thr = holdThrottle(altitude);
            channels[2]  = thr;
            sendRC();
            digitalWrite(STATUS_LED, HIGH);

            Serial.printf("[HOLD] t=%dms  alt=%.2fm  err=%.2fm  throttle=%d\n",
                missionTime, altitude, TARGET_ALT_M - altitude, thr);

            // Safety
            if (missionTime >= 8000) { state = CUT; break; }

            if (missionTime >= PUNCH_START_MS) {
                state = PUNCHING;
                Serial.println("[STATE] → PUNCHING");
            }
            break;
        }

        // ── PUNCHING ─────────────────────────────────────────
        // Max throttle for final burst — builds upward velocity before cut
        // Quad will coast above 60ft after motor cut, maximising descent time
        case PUNCHING:
            channels[2] = PUNCH_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 50 < 25);  // very fast strobe

            Serial.printf("[PUNCH] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[2]);

            if (missionTime >= 8000) {
                state = CUT;
                Serial.println("[STATE] → CUT");
            }
            break;

        // ── CUT ──────────────────────────────────────────────
        case CUT:
            channels[4] = 1000;  // disarm
            channels[2] = 1000;
            sendRC();
            // Brushed autorotation motor stays running
            Serial.printf("[CUT] Final alt=%.2fm  mission_time=%dms\n",
                altitude, missionTime);
            state = DONE;
            break;

        // ── HOVER TEST ───────────────────────────────────────
        case HOVER_TEST:
            channels[4] = 1800;
            channels[5] = 1800;
            channels[2] = HOVER_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            Serial.printf("[HOVER] throttle=%d  (tune HOVER_THROTTLE via BLE)\n",
                HOVER_THROTTLE);

            break;

        // ── DONE ─────────────────────────────────────────────
        // Auto hover calibration ramps throttle until liftoff is detected,
        // then backs off slightly and leaves the quad in hover test mode.
        case AUTO_HOVER_CAL:
            channels[4] = 1800;
            channels[5] = 1800;
            channels[2] = calThrottle;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 300 < 150);

            if (millis() - calStepTime >= CAL_STEP_MS) {
                calStepTime = millis();
                if (calThrottle < CAL_MAX_THROTTLE) {
                    calThrottle += CAL_STEP_US;
                }
            }

            Serial.printf("[AUTO_HOVER] alt=%.2fm  throttle=%d\n",
                altitude, calThrottle);

            if (altitude >= CAL_LIFTOFF_M) {
                HOVER_THROTTLE = constrain(calThrottle - CAL_BACKOFF_US, 1200, 1600);
                if (hoverChar) hoverChar->setValue((uint8_t*)&HOVER_THROTTLE, 2);
                channels[2] = HOVER_THROTTLE;
                sendRC();
                state = HOVER_TEST;
                Serial.printf("[AUTO_HOVER] liftoff detected, HOVER_THROTTLE=%d -> HOVER TEST\n",
                    HOVER_THROTTLE);
                break;
            }

            if (calThrottle >= CAL_MAX_THROTTLE || millis() - calTime >= CAL_TIMEOUT_MS) {
                disarmToIdle("[AUTO_HOVER] Calibration failed -> IDLE");
                break;
            }

            break;

        // ── LANDING ──────────────────────────────────────────
        case LANDING: {
            channels[4] = 1800;
            channels[5] = 1800;
            uint32_t elapsed = millis() - landingStartMs;
            if (elapsed >= LANDING_TIME_MS) {
                disarmToIdle("[LANDING] complete");
            } else {
                float t = (float)elapsed / LANDING_TIME_MS;
                channels[2] = (uint16_t)(landingStartThrottle * (1.0f - t) + 1000 * t);
            }
            sendRC();
            digitalWrite(STATUS_LED, millis() % 200 < 30);
            break;
        }

        case DONE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);
            break;
    }

    delay(20);  // 50Hz
}
