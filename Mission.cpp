#include "Mission.h"
#include <NimBLEDevice.h>
#include "State.h"
#include "Msp.h"
#include "Control.h"
#include "Ble.h"

static void pushTelemetry(float rawAlt, float altitude) {
    if (!telemetryChar) return;
    static uint8_t tick = 0;
    if (++tick < 5) return;
    tick = 0;
    int32_t altCm = (int32_t)(rawAlt * 100.0f);
    int32_t relCm = (int32_t)(altitude * 100.0f);
    uint8_t pkt[13];
    memcpy(pkt,      &altCm,                 4);
    memcpy(pkt + 4,  &relCm,                 4);
    pkt[8] = (uint8_t)state;
    memcpy(pkt + 9,  &channels[CH_THROTTLE], 2);
    memcpy(pkt + 11, &lastVario,             2);
    telemetryChar->setValue(pkt, 13);
    telemetryChar->notify();
}

void runMissionLoop() {
    float    rawAlt      = getAltitude();
    float    altitude    = rawAlt - launchAlt;
    currentRelAlt        = altitude;
    uint32_t missionTime = millis() - launchTime;

    pushTelemetry(rawAlt, altitude);

    // Keep vario filter current every loop so LANDING and other states see fresh data
    if ((millis() - lastVarioMs) < VARIO_STALE_MS && abs(lastVario) <= VARIO_MAX_PLAUSIBLE_CMS) {
        float rawMs = lastVario / 100.0f;
        filteredVario = VARIO_ALPHA * rawMs + (1.0f - VARIO_ALPHA) * filteredVario;
    }

    // BLE disconnect safety: land if triggered while in a test state
    if (bleSafetyLand) {
        bleSafetyLand = false;
        Serial.println("[BLE] Disconnect safety → LANDING");
        startLanding(altitude);
    }

    switch (state) {

        // ── IDLE ─────────────────────────────────────────────
        case IDLE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 1000 < 100);
            break;

        // ── ARMING ───────────────────────────────────────────
        case ARMING:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = 1800;
            channels[CH_THROTTLE] = 1000;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 200 < 100);

            if (millis() - armTime > ARMING_MS) {
                switch (armTarget) {
                    case ARM_HOVER_TEST:
                        state = HOVER_TEST;
                        Serial.println("[STATE] → HOVER TEST");
                        break;

                    case ARM_AUTO_HOVER_CAL:
                        launchAlt       = rawAlt;
                        calTime         = millis();
                        calStepTime     = millis();
                        calLiftoffCount = 0;
                        state = AUTO_HOVER_CAL;
                        Serial.printf("[STATE] → AUTO HOVER CAL (launchAlt=%.2fm)\n", launchAlt);
                        break;

                    case ARM_ALT_HOLD:
                        launchAlt = rawAlt;
                        resetCascadeController(0.0f);
                        state = ALT_HOLD;
                        Serial.printf("[STATE] → ALT HOLD (launchAlt=%.2fm, target=%.1fm)\n",
                            launchAlt, (float)TARGET_ALT_M);
                        break;

                    default:  // ARM_MISSION
                        launchAlt  = rawAlt;
                        launchTime = millis();
                        state      = SPRINTING;
                        Serial.printf("[STATE] → SPRINTING (launchAlt=%.2fm)\n", launchAlt);
                        break;
                }
            }
            break;

        // ── SPRINTING ────────────────────────────────────────
        case SPRINTING:
            channels[CH_THROTTLE] = SPRINT_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);

            Serial.printf("[SPRINT] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[CH_THROTTLE]);

            if (missionTime >= 8000) { state = CUT; break; }

            if (altitude >= SPRINT_CUTOFF_M) {
                ledcWrite(MOTOR_PWM_PIN, MOTOR_DUTY);
                resetCascadeController(altitude);
                state = HOLDING;
                Serial.printf("[STATE] → HOLDING at %.2fm (target %.2fm)\n",
                    altitude, (float)TARGET_ALT_M);
            }
            break;

        // ── HOLDING ──────────────────────────────────────────
        case HOLDING: {
            uint16_t thr          = holdCascaded(altitude, true);
            channels[CH_THROTTLE] = thr;
            sendRC();
            digitalWrite(STATUS_LED, HIGH);

            Serial.printf("[HOLD] t=%dms  alt=%.2fm  setpt=%.2fm  thr=%d\n",
                missionTime, altitude, internalSetpoint, thr);

            if (missionTime >= 8000) { state = CUT; break; }

            if (missionTime >= PUNCH_START_MS) {
                state = PUNCHING;
                Serial.println("[STATE] → PUNCHING");
            }
            break;
        }

        // ── PUNCHING ─────────────────────────────────────────
        case PUNCHING:
            channels[CH_THROTTLE] = PUNCH_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 50 < 25);

            Serial.printf("[PUNCH] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[CH_THROTTLE]);

            if (missionTime >= 8000) {
                state = CUT;
                Serial.println("[STATE] → CUT");
            }
            break;

        // ── CUT ──────────────────────────────────────────────
        case CUT:
            channels[CH_ARM]      = 1000;
            channels[CH_THROTTLE] = 1000;
            sendRC();
            Serial.printf("[CUT] Final alt=%.2fm  mission_time=%dms\n",
                altitude, missionTime);
            state = DONE;
            break;

        // ── HOVER TEST ───────────────────────────────────────
        case HOVER_TEST:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = 1800;
            channels[CH_THROTTLE] = HOVER_THROTTLE;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            Serial.printf("[HOVER] throttle=%d  (tune HOVER_THROTTLE via BLE)\n",
                HOVER_THROTTLE);
            break;

        // ── AUTO HOVER CAL ───────────────────────────────────
        case AUTO_HOVER_CAL:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = 1800;
            channels[CH_THROTTLE] = calThrottle;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 300 < 150);

            if (millis() - calStepTime >= CAL_STEP_MS) {
                calStepTime = millis();
                if (calThrottle < CAL_MAX_THROTTLE) calThrottle += CAL_STEP_US;
            }

            Serial.printf("[AUTO_HOVER] alt=%.2fm  throttle=%d\n", altitude, calThrottle);

            if (altitude >= CAL_LIFTOFF_M) calLiftoffCount++;
            else                            calLiftoffCount = 0;

            if (calLiftoffCount >= 5) {
                HOVER_THROTTLE = constrain(calThrottle + CAL_GE_OFFSET_US, 1200, 1600);
                if (hoverChar) hoverChar->setValue((uint8_t*)&HOVER_THROTTLE, 2);
                channels[CH_THROTTLE] = HOVER_THROTTLE;
                sendRC();
                state = HOVER_TEST;
                Serial.printf("[AUTO_HOVER] liftoff confirmed (%d counts), HOVER_THROTTLE=%d -> HOVER TEST\n",
                    calLiftoffCount, HOVER_THROTTLE);
                break;
            }

            if (calThrottle >= CAL_MAX_THROTTLE || millis() - calTime >= CAL_TIMEOUT_MS) {
                disarmToIdle("[AUTO_HOVER] Calibration failed -> IDLE");
            }
            break;

        // ── ALT HOLD ─────────────────────────────────────────
        case ALT_HOLD: {
            uint16_t thr          = holdCascaded(altitude, false);
            channels[CH_THROTTLE] = thr;
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = 1800;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            if (altitude > ALT_MAX_M) {
                Serial.printf("[ALT_HOLD] ceiling exceeded %.2fm, landing\n", altitude);
                startLanding(altitude);
            }
            break;
        }

        // ── LANDING ──────────────────────────────────────────
        case LANDING: {
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = 1800;
            digitalWrite(STATUS_LED, millis() % 200 < 30);

            if (altitude <= LANDING_GROUND_M) {
                disarmToIdle("[LANDING] ground detected");
                break;
            }
            if (millis() - landingStartMs >= LANDING_TIMEOUT_MS) {
                disarmToIdle("[LANDING] timeout");
                break;
            }

            float rateError           = (-DESCENT_RATE_MPS) - filteredVario;
            int16_t correction        = (int16_t)(LANDING_KP_VSPEED * rateError);
            channels[CH_THROTTLE]     = (uint16_t)constrain(HOVER_THROTTLE + correction, 1000, 1600);
            sendRC();

            Serial.printf("[LANDING] alt=%.2fm  filtV=%.2f  thr=%d\n",
                altitude, filteredVario, channels[CH_THROTTLE]);
            break;
        }

        case DONE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);
            break;
    }
}
