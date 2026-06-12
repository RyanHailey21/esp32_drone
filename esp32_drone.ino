// ============================================================
//  Quad Mission Controller
//  Hardware: ESP32-C3 Super Mini
//
//  Arduino IDE Board Settings:
//    Board:             ESP32C3 Dev Module
//    USB CDC On Boot:   Enabled
//    CPU Frequency:     160 MHz
//    Flash Size:        4MB
//    Partition Scheme:  Default 4MB with spiffs
//    Upload Speed:      921600
//
//  Libraries (install via Library Manager):
//    - NimBLE-Arduino by h2zero
//
//  Note: uses Arduino core 2.x PWM API (ledcSetup/ledcAttachPin)
//  If on core 3.x replace with: ledcAttach(pin, freq, resolution)
//                                ledcWrite(pin, duty)
//
//  Mission Profile:
//    SPRINT  → full throttle to 60ft as fast as possible
//    HOLD    → P-controller holds 60ft, clock runs down
//    PUNCH   → max throttle final burst before 8s cut
//    CUT     → motors off, autorotation descent
// ============================================================

#include <HardwareSerial.h>
#include <NimBLEDevice.h>

// ── Pins ─────────────────────────────────────────────────────
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define MOTOR_PWM_PIN   6
#define LAUNCH_PIN      3
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
    DONE
};
MissionState state = IDLE;

uint32_t launchTime = 0;
uint32_t armTime    = 0;
float    launchAlt  = 0;
bool     prespunUp  = false;

HardwareSerial fcSerial(1);


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
    auto* svc    = server->createService(SERVICE_UUID);

    makeChar(svc, HOVER_UUID,
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
    while (fcSerial.available()) fcSerial.read();
    uint8_t empty = 0;
    sendMSP(MSP_ALTITUDE, &empty, 0);
    uint32_t timeout = millis() + 20;
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


// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    fcSerial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_PWM_PIN, 0);

    pinMode(LAUNCH_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);

    setupBLE();

    Serial.println("[BOOT] Ready.");
    Serial.println("[BOOT] Short press = hover test | Long press (1s+) = full mission");
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

            if (digitalRead(LAUNCH_PIN) == LOW) {
                uint32_t t = millis();
                while (digitalRead(LAUNCH_PIN) == LOW);
                launchAlt = getAltitude();

                if (millis() - t < 1000) {
                    state = HOVER_TEST;
                    Serial.println("[STATE] → HOVER TEST");
                } else {
                    armTime = millis();
                    state   = ARMING;
                    Serial.println("[STATE] → ARMING");
                }
            }
            break;

        // ── ARMING ───────────────────────────────────────────
        case ARMING:
            channels[4] = 1800;
            channels[5] = 1800;
            channels[2] = 1050;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 200 < 100);

            if (millis() - armTime > 1500) {
                launchTime = millis();
                prespunUp  = false;
                state      = SPRINTING;
                Serial.println("[STATE] → SPRINTING");
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

            if (digitalRead(LAUNCH_PIN) == LOW) {
                while (digitalRead(LAUNCH_PIN) == LOW);
                channels[4] = 1000;
                channels[2] = 1000;
                sendRC();
                state = IDLE;
                Serial.println("[HOVER] Disarmed → IDLE");
            }
            break;

        // ── DONE ─────────────────────────────────────────────
        case DONE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);
            break;
    }

    delay(20);  // 50Hz
}
