#include "Mission.h"
#include <NimBLEDevice.h>
#include "State.h"
#include "Msp.h"
#include "Control.h"
#include "Ble.h"

// 0=settling, 1=waiting for liftoff, 2=cascade active. Updated each ALT_HOLD iteration.
static uint8_t altHoldGuardPhase = 2;

static uint16_t angleModeChannelValue() {
    return ANGLE_MODE_ENABLED ? 1800 : 1000;
}

static void pushTelemetry(float rawAlt, float altitude) {
    if (!telemetryChar) return;
    static uint8_t tick = 0;
    if (++tick < 5) return;
    tick = 0;
    int32_t altCm     = (int32_t)(rawAlt * 100.0f);
    int32_t relCm     = (int32_t)(altitude * 100.0f);
    int16_t filtVarCs = (int16_t)constrain(filteredVario    * 100.0f, -32768.0f, 32767.0f);
    int16_t setptCm   = (int16_t)constrain(internalSetpoint * 100.0f, -32768.0f, 32767.0f);
    int16_t tofCm     = lastTofValid ? (int16_t)constrain(lastTofAltM * 100.0f, -32768.0f, 32767.0f) : -1;
    int32_t baroCm    = (int32_t)(lastBaroAltM * 100.0f);
    uint8_t tofValid  = lastTofValid ? 1 : 0;

    uint8_t pkt[38];
    memcpy(pkt,      &altCm,                 4);
    memcpy(pkt + 4,  &relCm,                 4);
    pkt[8] = (uint8_t)state;
    memcpy(pkt + 9,  &channels[CH_THROTTLE], 2);
    memcpy(pkt + 11, &lastVario,             2);
    memcpy(pkt + 13, &filtVarCs,             2);
    memcpy(pkt + 15, &setptCm,              2);
    pkt[17] = altHoldGuardPhase;
    memcpy(pkt + 18, &lastFcVario,           2);
    memcpy(pkt + 20, &lastDerivedVario,      2);
    memcpy(pkt + 22, lastMspAltitudePayload, 6);
    memcpy(pkt + 28, &channels[CH_ANGLE],    2);
    memcpy(pkt + 30, &tofCm,                 2);
    pkt[32] = lastTofWeightPct;
    pkt[33] = tofValid;
    memcpy(pkt + 34, &baroCm,                4);
    telemetryChar->setValue(pkt, 38);
    telemetryChar->notify();
}

void runMissionLoop() {
    float    rawAlt      = getAltitude();
    float    altitude    = rawAlt - launchAlt;
    currentRelAlt        = altitude;
    uint32_t missionTime = millis() - launchTime;

    pushTelemetry(rawAlt, altitude);

    // Keep vario filter current every loop so LANDING and other states see fresh data.
    // Use a time-based filter so smoothing does not change with loop-rate jitter.
    bool varioFresh = (millis() - lastVarioMs) < VARIO_STALE_MS
                      && abs(lastVario) <= VARIO_MAX_PLAUSIBLE_CMS;
    if (varioFresh) {
        static uint32_t varioFilterLastMs = 0;
        uint32_t nowMs = millis();
        if (varioFilterLastMs == 0) varioFilterLastMs = nowMs;
        float dt = constrain((nowMs - varioFilterLastMs) / 1000.0f, 0.001f, 0.2f);
        varioFilterLastMs = nowMs;
        float rawMs = lastVario / 100.0f;
        float alpha = dt / (VARIO_TAU_S + dt);
        filteredVario += alpha * (rawMs - filteredVario);
    } else if (state == ALT_HOLD || state == HOLDING) {
        static uint32_t lastVarioWarnMs = 0;
        if (millis() - lastVarioWarnMs >= 500) {
            lastVarioWarnMs = millis();
            Serial.printf("[VARIO] stale! lastVarioMs=%u raw=%d filtV=%.2f\n",
                lastVarioMs, lastVario, filteredVario);
        }
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
            channels[CH_ANGLE] = angleModeChannelValue();
            sendRC();
            digitalWrite(STATUS_LED, millis() % 1000 < 100);
            break;

        // ── ARMING ───────────────────────────────────────────
        case ARMING:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = angleModeChannelValue();
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
                            launchAlt, constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M));
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
            channels[CH_ANGLE]    = angleModeChannelValue();
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
            channels[CH_ANGLE]    = angleModeChannelValue();
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
            channels[CH_ANGLE]    = angleModeChannelValue();
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
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = angleModeChannelValue();
            // Ramp toward HOVER_THROTTLE at ~200 µs/s so a rapid step from the
            // ARMING low-throttle doesn't trigger Betaflight's ANTI_GRAVITY I-term boost.
            if (channels[CH_THROTTLE] < (uint16_t)HOVER_THROTTLE) {
                channels[CH_THROTTLE] = min((uint16_t)(channels[CH_THROTTLE] + HOVER_RAMP_STEP_US),
                                            (uint16_t)HOVER_THROTTLE);
            } else {
                channels[CH_THROTTLE] = HOVER_THROTTLE;
            }
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            Serial.printf("[HOVER] throttle=%d  target=%d  angleAux=%d\n",
                channels[CH_THROTTLE], (int)HOVER_THROTTLE, channels[CH_ANGLE]);
            break;

        // ── AUTO HOVER CAL ───────────────────────────────────
        case AUTO_HOVER_CAL:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = angleModeChannelValue();
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
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = angleModeChannelValue();

            uint16_t thr;
            uint32_t timeInAltHold = millis() - (armTime + ARMING_MS);
            if (timeInAltHold < BARO_SETTLE_MS) {
                altHoldGuardPhase = 0;
                launchAlt = rawAlt;
                thr = HOVER_THROTTLE + TAKEOFF_NUDGE_US;
                resetCascadeController(0.0f);
                if (timeInAltHold % 100 < 20)
                    Serial.printf("[ALT_HOLD] settle  t=%ums rawAlt=%.2f launchAlt=%.2f rel=%.2f\n",
                        timeInAltHold, rawAlt, launchAlt, altitude);
            } else if (altitude < TAKEOFF_ALT_M && timeInAltHold < GROUND_GUARD_TIMEOUT_MS) {
                altHoldGuardPhase = 1;
                thr = HOVER_THROTTLE + TAKEOFF_NUDGE_US;
                resetCascadeController(0.0f);
                if (timeInAltHold % 100 < 20)
                    Serial.printf("[ALT_HOLD] guard   t=%ums rawAlt=%.2f launchAlt=%.2f rel=%.2f\n",
                        timeInAltHold, rawAlt, launchAlt, altitude);
            } else if (altitude < TAKEOFF_ALT_M) {
                altHoldGuardPhase = 1;
                thr = HOVER_THROTTLE;
                channels[CH_THROTTLE] = thr;
                sendRC();
                Serial.printf("[ALT_HOLD] ground guard expired without liftoff confirmation, landing rel=%.2f\n",
                    altitude);
                startLanding(altitude);
                break;
            } else {
                altHoldGuardPhase = 2;
                thr = holdCascaded(altitude, false);
            }
            channels[CH_THROTTLE] = thr;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            // Tight safety ceiling: if baro or controller misbehaves, land before getting dangerous.
            float holdTarget  = constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M);
            float safeCeiling = holdTarget + 2.0f;
            if (altitude > safeCeiling) {
                Serial.printf("[ALT_HOLD] safety ceiling %.1fm exceeded at %.2fm, landing\n", safeCeiling, altitude);
                startLanding(altitude);
            }
            break;
        }

        // ── LANDING ──────────────────────────────────────────
        case LANDING: {
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = angleModeChannelValue();
            digitalWrite(STATUS_LED, millis() % 200 < 30);

            // Only trust baro ground detection after we've actually descended from start altitude.
            // Guards against a stuck/zero baro reading killing motors mid-air.
            bool hasDescended = (landingStartAlt - altitude) > 0.3f;
            if (hasDescended && altitude <= LANDING_GROUND_M) {
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
