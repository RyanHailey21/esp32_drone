#include "Mission.h"

void runMissionLoop() {
    float    rawAlt       = getAltitude();
    float    altitude     = rawAlt - launchAlt;
    currentRelAlt         = altitude;
    uint32_t missionTime  = millis() - launchTime;  // time since motors started

    // Push telemetry at ~10Hz (every 5 loops)
    // Sends relative altitude (above launch point) so preflight panel shows climb height
    static uint8_t telTick = 0;
    if (telemetryChar && ++telTick >= 5) {
        telTick = 0;
        int32_t altCm = (int32_t)(rawAlt * 100.0f);
        int32_t relCm = (int32_t)(altitude * 100.0f);
        uint8_t pkt[13];
        memcpy(pkt,      &altCm,       4);
        memcpy(pkt + 4,  &relCm,       4);
        pkt[8] = (uint8_t)state;
        memcpy(pkt + 9,  &channels[2], 2);
        memcpy(pkt + 11, &lastVario,   2);
        telemetryChar->setValue(pkt, 13);
        telemetryChar->notify();
    }

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
            channels[4] = 1800;
            channels[5] = 1800;
            channels[2] = 1000;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 200 < 100);

            if (millis() - armTime > ARMING_MS) {
                if (armingForHover) {
                    armingForHover = false;
                    state = HOVER_TEST;
                    Serial.println("[STATE] → HOVER TEST");
                } else if (armingForAutoCal) {
                    armingForAutoCal = false;
                    launchAlt        = rawAlt;
                    calTime          = millis();
                    calStepTime      = millis();
                    calLiftoffCount  = 0;
                    state = AUTO_HOVER_CAL;
                    Serial.printf("[STATE] → AUTO HOVER CAL (launchAlt=%.2fm)\n", launchAlt);
                } else if (armingForAltHold) {
                    armingForAltHold = false;
                    launchAlt        = rawAlt;
                    resetCascadeController(0.0f);   // setpoint starts at ground, ramps to target
                    state = ALT_HOLD;
                    Serial.printf("[STATE] → ALT HOLD (launchAlt=%.2fm, target=%.1fm)\n", launchAlt, (float)TARGET_ALT_M);
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
                ledcWrite(MOTOR_PWM_PIN, MOTOR_DUTY);
                prespunUp = true;
                resetCascadeController(altitude);   // bumpless: setpoint = current alt
                state     = HOLDING;
                Serial.printf("[STATE] → HOLDING at %.2fm (target %.2fm)\n",
                    altitude, (float)TARGET_ALT_M);
            }
            break;

        // ── HOLDING ──────────────────────────────────────────
        // PID controller keeps quad at TARGET_ALT_M (60ft)
        // D term uses FC vario (vertical velocity) to damp oscillations
        case HOLDING: {
            uint16_t thr = holdCascaded(altitude, true);
            channels[2]  = thr;
            sendRC();
            digitalWrite(STATUS_LED, HIGH);

            Serial.printf("[HOLD] t=%dms  alt=%.2fm  setpt=%.2fm  thr=%d\n",
                missionTime, altitude, internalSetpoint, thr);

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
                calLiftoffCount++;
            } else {
                calLiftoffCount = 0;
            }

            if (calLiftoffCount >= 5) {
                HOVER_THROTTLE = constrain(calThrottle + CAL_GE_OFFSET_US, 1200, 1600);
                if (hoverChar) hoverChar->setValue((uint8_t*)&HOVER_THROTTLE, 2);
                channels[2] = HOVER_THROTTLE;
                sendRC();
                state = HOVER_TEST;
                Serial.printf("[AUTO_HOVER] liftoff confirmed (%d counts), HOVER_THROTTLE=%d -> HOVER TEST\n",
                    calLiftoffCount, HOVER_THROTTLE);
                break;
            }

            if (calThrottle >= CAL_MAX_THROTTLE || millis() - calTime >= CAL_TIMEOUT_MS) {
                disarmToIdle("[AUTO_HOVER] Calibration failed -> IDLE");
                break;
            }

            break;

        // ── ALT HOLD ─────────────────────────────────────────
        // Test mode: same PID as HOLDING but lands on BLE disconnect
        case ALT_HOLD: {
            uint16_t thr = holdCascaded(altitude, false);
            channels[2]  = thr;
            channels[4]  = 1800;
            channels[5]  = 1800;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            if (altitude > ALT_MAX_M) {
                Serial.printf("[ALT_HOLD] ceiling exceeded %.2fm, landing\n", altitude);
                startLanding(altitude);
                break;
            }
            break;
        }

        // ── LANDING ──────────────────────────────────────────
        // Velocity-based descent: targets DESCENT_RATE_MPS downward
        // using FC vario as feedback. Cuts motors at ground threshold.
        case LANDING: {
            channels[4] = 1800;
            channels[5] = 1800;
            digitalWrite(STATUS_LED, millis() % 200 < 30);

            // Ground detected or timeout → disarm
            if (altitude <= LANDING_GROUND_M) {
                disarmToIdle("[LANDING] ground detected");
                break;
            }
            if (millis() - landingStartMs >= LANDING_TIMEOUT_MS) {
                disarmToIdle("[LANDING] timeout");
                break;
            }

            // Velocity controller: target DESCENT_RATE_MPS downward using filtered vario
            float rateError    = (-DESCENT_RATE_MPS) - filteredVario;
            int16_t correction = (int16_t)(LANDING_KP_VSPEED * rateError);
            channels[2] = (uint16_t)constrain(HOVER_THROTTLE + correction, 1000, 1600);
            sendRC();

            Serial.printf("[LANDING] alt=%.2fm  filtV=%.2f  thr=%d\n",
                altitude, filteredVario, channels[2]);
            break;
        }

        case DONE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);
            break;
    }
}
