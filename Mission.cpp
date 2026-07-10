#include "Mission.h"
#include <NimBLEDevice.h>
#include "State.h"
#include "Msp.h"
#include "Control.h"
#include "Ble.h"
#include "FlightLog.h"

// 0=settling, 2=cascade active. Updated each ALT_HOLD iteration.
static uint8_t altHoldGuardPhase = 2;
static bool altHoldCascadeLatched = false;

static uint16_t angleModeChannelValue() {
    return ANGLE_MODE_ENABLED ? 1800 : 1000;
}

static uint16_t hoverTestAngleModeChannelValue() {
    return HOVER_TEST_ANGLE_MODE ? 1800 : 1000;
}

static bool attitudeAbortActive() {
    if ((lastFcDiagMask & (1 << 1)) == 0) return false;
    if (millis() - lastFcAttitudeMs > ATTITUDE_ABORT_MAX_AGE_MS) return false;
    return fabsf(lastFcRollDeciDeg / 10.0f) >= ATTITUDE_ABORT_DEG
        || fabsf(lastFcPitchDeciDeg / 10.0f) >= ATTITUDE_ABORT_DEG;
}

static bool testModeOrTestArming() {
    if (state == HOVER_TEST || state == ALT_HOLD || state == AUTO_HOVER_CAL || state == LANDING) {
        return true;
    }
    return state == ARMING
        && (armTarget == ARM_HOVER_TEST || armTarget == ARM_AUTO_HOVER_CAL || armTarget == ARM_ALT_HOLD);
}

static void finishMissionOrLand(float altitude, const char* reason) {
    Serial.println(reason);
    if (MISSION_TYPE == 0) {
        startLanding(altitude);
    } else {
        state = CUT;
    }
}

static bool missionCeilingExceeded(float altitude) {
    return isfinite(altitude) && altitude >= ALT_MAX_M;
}

static uint16_t takeoffGuardThrottle(uint32_t timeInAltHoldMs, bool tofReady) {
    uint32_t rampMs = timeInAltHoldMs > BARO_SETTLE_MS ? timeInAltHoldMs - BARO_SETTLE_MS : 0;
    int32_t rampUs = (int32_t)((rampMs * (uint32_t)TAKEOFF_RAMP_US_PER_S) / 1000U);
    int32_t maxOffsetUs = tofReady ? TAKEOFF_MAX_OFFSET_US : TAKEOFF_INVALID_TOF_MAX_OFFSET_US;
    int32_t offsetUs = min(maxOffsetUs, (int32_t)TAKEOFF_NUDGE_US + rampUs);
    return (uint16_t)constrain((int32_t)HOVER_THROTTLE + offsetUs,
                               (int32_t)MIN_ALT_HOLD_THROTTLE_US,
                               (int32_t)HOVER_THROTTLE + maxOffsetUs);
}

static void printFlightLogHeader(const char* label, float targetM) {
    flightLogReset();
    flightLogAppendf("[RUN] %s hover=%u target=%.2f kp=%.2f ki=%.2f kd=%.2f tofFull=%.2f tofZero=%.2f\n",
        label, HOVER_THROTTLE, targetM, (float)HOLD_KP, (float)HOLD_KI,
        (float)HOLD_KD, (float)TOF_BLEND_FULL_M, (float)TOF_BLEND_ZERO_M);
    flightLogAppend("[FLT] ms,state,phase,alt,lowRel,tof,tofW,baro,cbaro,src,setpt,fV,usedV,bfV,derV,vsrc,desV,aErr,vErr,P,I,rawThr,thr,minThr,maxThr,sat,accX,accY,accZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,cycle,sensors,rcThr,rcArm,rcAngle,vbat,amps,diag,tofRaw,tofReadOk,tofReject,tofDt,tofStatus,tofI2c\n");
#if SERIAL_FLIGHT_DEBUG
    Serial.printf("[RUN] %s hover=%u target=%.2f kp=%.2f ki=%.2f kd=%.2f tofFull=%.2f tofZero=%.2f\n",
        label, HOVER_THROTTLE, targetM, (float)HOLD_KP, (float)HOLD_KI,
        (float)HOLD_KD, (float)TOF_BLEND_FULL_M, (float)TOF_BLEND_ZERO_M);
    Serial.println("[FLT] ms,state,phase,alt,lowRel,tof,tofW,baro,cbaro,src,setpt,fV,usedV,bfV,derV,vsrc,desV,aErr,vErr,P,I,rawThr,thr,minThr,maxThr,sat,accX,accY,accZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,cycle,sensors,rcThr,rcArm,rcAngle,vbat,amps,diag,tofRaw,tofReadOk,tofReject,tofDt,tofStatus,tofI2c");
#endif
}

static void markOpenLoopControl(uint16_t throttle) {
    lastAltError = 0.0f;
    lastDesiredVspeed = 0.0f;
    lastVspeedError = 0.0f;
    lastControlPUs = 0.0f;
    lastControlIUs = 0.0f;
    lastRawThrottle = throttle;
    lastThrMin = throttle;
    lastThrMax = throttle;
    lastClampedThrottle = throttle;
    lastThrottleSat = 0;
}

static void logFlightSample(uint32_t elapsedMs, const char* phase, float altitude,
                            float compatAlt, uint16_t throttle) {
    static uint32_t lastLogMs = 0;
    if (elapsedMs < lastLogMs || elapsedMs - lastLogMs >= 50) {
        lastLogMs = elapsedMs;
        flightLogAppendf("[FLT] %lu,%d,%s,%.3f,%.3f,%.3f,%u,%.3f,%.3f,%u,%.3f,%.3f,%d,%d,%d,%u,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%d,%d,%d,%d,%d,%d,%d,%.1f,%.1f,%d,%u,%u,%u,%u,%u,%.1f,%.2f,%u,%.3f,%u,%u,%u,%u,%u\n",
            (unsigned long)elapsedMs,
            (int)state,
            phase,
            altitude,
            compatAlt,
            lastTofValid ? lastTofAltM : -1.0f,
            lastTofWeightPct,
            lastBaroAltM,
            lastCorrectedBaroAltM,
            lastAltitudeSource,
            internalSetpoint,
            filteredVario,
            lastVario,
            lastFcVario,
            lastDerivedVario,
            lastVarioSource,
            lastDesiredVspeed,
            lastAltError,
            lastVspeedError,
            lastControlPUs,
            lastControlIUs,
            lastRawThrottle,
            throttle,
            lastThrMin,
            lastThrMax,
            (int)lastThrottleSat,
            lastFcAccX,
            lastFcAccY,
            lastFcAccZ,
            lastFcGyroX,
            lastFcGyroY,
            lastFcGyroZ,
            lastFcRollDeciDeg / 10.0f,
            lastFcPitchDeciDeg / 10.0f,
            lastFcYawDeg,
            lastFcCycleTimeUs,
            lastFcSensorsMask,
            lastFcRcThrottle,
            lastFcRcArm,
            lastFcRcAngle,
            lastFcVbatDeciV / 10.0f,
            lastFcAmperageCentiA / 100.0f,
            lastFcDiagMask,
            lastTofRawM,
            lastTofReadOk ? 1 : 0,
            lastTofRejectReason,
            lastTofReadDtMs,
            lastTofRangeStatus,
            lastTofI2cStatus);
    }
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

    bool activeFlight = state == ARMING
                     || state == SPRINTING
                     || state == HOLDING
                     || state == PUNCHING
                     || state == HOVER_TEST
                     || state == AUTO_HOVER_CAL
                     || state == LANDING
                     || state == ALT_HOLD;

    if (activeFlight) {
        uint16_t diagMask = lastFcDiagMask;
        uint8_t sensorFlags = 0;
        if (mspAltitudeFresh()) sensorFlags |= 0x01;
        if (lastTofValid) sensorFlags |= 0x02;
        if (millis() - lastFcAttitudeMs <= ATTITUDE_ABORT_MAX_AGE_MS) sensorFlags |= 0x04;

        uint8_t pkt[28];
        memcpy(pkt,      &altCm,                 4);
        memcpy(pkt + 4,  &relCm,                 4);
        pkt[8] = (uint8_t)state;
        memcpy(pkt + 9,  &channels[CH_THROTTLE], 2);
        memcpy(pkt + 11, &lastVario,             2);
        memcpy(pkt + 13, &filtVarCs,             2);
        memcpy(pkt + 15, &setptCm,               2);
        pkt[17] = altHoldGuardPhase;
        memcpy(pkt + 18, &lastFcVario,           2);
        memcpy(pkt + 20, &lastDerivedVario,      2);
        memcpy(pkt + 22, &diagMask,              2);
        pkt[24] = sensorFlags;
        pkt[25] = lastTofWeightPct;
        pkt[26] = lastAltitudeSource;
        pkt[27] = lastVarioSource;
        telemetryChar->setValue(pkt, sizeof(pkt));
        telemetryChar->notify();
        return;
    }

    int32_t cbaroCm   = (int32_t)(lastCorrectedBaroAltM * 100.0f);

    uint8_t pkt[77];
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
    pkt[38] = lastAltitudeSource;
    memcpy(pkt + 39, &cbaroCm,               4);
    memcpy(pkt + 43, &lastFcDiagMask,        2);
    memcpy(pkt + 45, &lastFcAccX,            2);
    memcpy(pkt + 47, &lastFcAccY,            2);
    memcpy(pkt + 49, &lastFcAccZ,            2);
    memcpy(pkt + 51, &lastFcGyroX,           2);
    memcpy(pkt + 53, &lastFcGyroY,           2);
    memcpy(pkt + 55, &lastFcGyroZ,           2);
    memcpy(pkt + 57, &lastFcRollDeciDeg,     2);
    memcpy(pkt + 59, &lastFcPitchDeciDeg,    2);
    memcpy(pkt + 61, &lastFcYawDeg,          2);
    memcpy(pkt + 63, &lastFcCycleTimeUs,     2);
    memcpy(pkt + 65, &lastFcSensorsMask,     2);
    memcpy(pkt + 67, &lastFcRcThrottle,      2);
    memcpy(pkt + 69, &lastFcRcArm,           2);
    memcpy(pkt + 71, &lastFcRcAngle,         2);
    pkt[73] = lastFcVbatDeciV;
    memcpy(pkt + 74, &lastFcAmperageCentiA,  2);
    pkt[76] = lastVarioSource;
    telemetryChar->setValue(pkt, 77);
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
#if SERIAL_FLIGHT_DEBUG
        Serial.printf("[VARIO] stale! lastVarioMs=%u raw=%d filtV=%.2f\n",
            lastVarioMs, lastVario, filteredVario);
#endif
        }
    }

    // BLE callback requests are consumed in the main loop to avoid racing MSP/RC writes.
    if (bleRequestedLand) {
        bleRequestedLand = false;
        bleSafetyLand = false;
        if (testModeOrTestArming()) {
            Serial.println("[BLE] Land request -> LANDING");
            startLanding(altitude);
        } else {
            Serial.println("[BLE] Land request ignored outside test mode");
        }
    }

    // BLE disconnect safety: land if triggered while in a test state
    if (bleSafetyLand) {
        bleSafetyLand = false;
        if (testModeOrTestArming()) {
            Serial.println("[BLE] Disconnect safety -> LANDING");
            startLanding(altitude);
        } else {
            Serial.println("[BLE] Disconnect ignored outside test mode");
        }
    }

    if ((state == SPRINTING || state == HOLDING || state == PUNCHING) && missionCeilingExceeded(altitude)) {
        finishMissionOrLand(altitude, "[MISSION] altitude ceiling exceeded");
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
                        resetAltitudeFusion();
                        launchAlt       = getAltitude();
                        calTime         = millis();
                        calStepTime     = millis();
                        calLiftoffCount = 0;
                        state = AUTO_HOVER_CAL;
                        Serial.printf("[STATE] → AUTO HOVER CAL (launchAlt=%.2fm)\n", launchAlt);
                        break;

                    case ARM_ALT_HOLD:
                        resetAltitudeFusion();
                        launchAlt = getAltitude();
                        altHoldCascadeLatched = false;
                        resetCascadeController(0.0f);
                        state = ALT_HOLD;
                        printFlightLogHeader("ALT_HOLD",
                            constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M));
                        Serial.printf("[STATE] → ALT HOLD (launchAlt=%.2fm, target=%.1fm)\n",
                            launchAlt, constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M));
                        break;

                    default:  // ARM_MISSION
                        resetAltitudeFusion();
                        launchAlt  = getAltitude();
                        launchTime = millis();
                        printFlightLogHeader("MISSION", (float)TARGET_ALT_M);
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
            markOpenLoopControl(channels[CH_THROTTLE]);
            sendRC();
            logFlightSample(missionTime, "sprint", altitude, altitude, channels[CH_THROTTLE]);
            digitalWrite(STATUS_LED, millis() % 100 < 50);

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[SPRINT] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[CH_THROTTLE]);
#endif

            if (missionCeilingExceeded(altitude)) {
                finishMissionOrLand(altitude, "[MISSION] altitude ceiling exceeded");
                break;
            }

            if (missionTime >= 8000) {
                finishMissionOrLand(altitude, "[MISSION] time limit reached before hold");
                break;
            }

            if (altitude >= SPRINT_CUTOFF_M) {
                if (MISSION_TYPE == 1) {
                    ledcWrite(MOTOR_PWM_PIN, MOTOR_DUTY);
                }
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
            logFlightSample(missionTime, "hold", altitude, altitude, thr);
            digitalWrite(STATUS_LED, HIGH);

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[HOLD] t=%dms  alt=%.2fm  setpt=%.2fm  thr=%d\n",
                missionTime, altitude, internalSetpoint, thr);
#endif

            if (missionCeilingExceeded(altitude)) {
                finishMissionOrLand(altitude, "[MISSION] altitude ceiling exceeded");
                break;
            }

            if (missionTime >= 8000) {
                finishMissionOrLand(altitude, "[MISSION] time limit reached in hold");
                break;
            }

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
            markOpenLoopControl(channels[CH_THROTTLE]);
            sendRC();
            logFlightSample(missionTime, "punch", altitude, altitude, channels[CH_THROTTLE]);
            digitalWrite(STATUS_LED, millis() % 50 < 25);

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[PUNCH] t=%dms  alt=%.2fm  throttle=%d\n",
                missionTime, altitude, channels[CH_THROTTLE]);
#endif

            if (missionCeilingExceeded(altitude)) {
                finishMissionOrLand(altitude, "[MISSION] altitude ceiling exceeded");
                break;
            }

            if (missionTime >= 8000) {
                finishMissionOrLand(altitude, "[MISSION] final time reached");
            }
            break;

        // ── CUT ──────────────────────────────────────────────
        case CUT:
            channels[CH_ARM]      = 1000;
            channels[CH_THROTTLE] = 1000;
            sendRC();
#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[CUT] Final alt=%.2fm  mission_time=%dms\n",
                altitude, missionTime);
#endif
            state = DONE;
            break;

        // ── HOVER TEST ───────────────────────────────────────
        case HOVER_TEST:
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = hoverTestAngleModeChannelValue();
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

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[HOVER] throttle=%d  target=%d  angleAux=%d\n",
                channels[CH_THROTTLE], (int)HOVER_THROTTLE, channels[CH_ANGLE]);
#endif
            break;

        // ── AUTO HOVER CAL ───────────────────────────────────
        case AUTO_HOVER_CAL:
            channels[CH_ARM]      = 1800;
            channels[CH_ANGLE]    = hoverTestAngleModeChannelValue();
            channels[CH_THROTTLE] = calThrottle;
            sendRC();
            digitalWrite(STATUS_LED, millis() % 300 < 150);

            if (millis() - calStepTime >= CAL_STEP_MS) {
                calStepTime = millis();
                if (calThrottle < CAL_MAX_THROTTLE) calThrottle += CAL_STEP_US;
            }

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[AUTO_HOVER] alt=%.2fm  throttle=%d\n", altitude, calThrottle);
#endif

            if (altitude >= CAL_LIFTOFF_M) calLiftoffCount++;
            else                            calLiftoffCount = 0;

            if (calLiftoffCount >= 5) {
                HOVER_THROTTLE = constrain(calThrottle + CAL_GE_OFFSET_US, 1200, 1600);
                if (hoverChar) hoverChar->setValue((uint8_t*)&HOVER_THROTTLE, 2);
                channels[CH_THROTTLE] = HOVER_THROTTLE;
                sendRC();
                state = HOVER_TEST;
#if SERIAL_FLIGHT_DEBUG
                Serial.printf("[AUTO_HOVER] liftoff confirmed (%d counts), HOVER_THROTTLE=%d -> HOVER TEST\n",
                    calLiftoffCount, HOVER_THROTTLE);
#endif
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
            float controlAlt = altitude;
            float tofAgl = lastTofValid ? lastTofAltM : altitude;
            if (timeInAltHold < BARO_SETTLE_MS) {
                altHoldGuardPhase = 0;
                launchAlt = rawAlt;
                altHoldCascadeLatched = false;
                thr = takeoffGuardThrottle(timeInAltHold, lastTofValid);
                resetCascadeController(0.0f);
                if (timeInAltHold % 100 < 20) {
#if SERIAL_FLIGHT_DEBUG
                    Serial.printf("[ALT_HOLD] settle  t=%ums rawAlt=%.2f launchAlt=%.2f rel=%.2f tof=%.2f tofW=%u\n",
                        timeInAltHold, rawAlt, launchAlt, altitude, tofAgl, lastTofWeightPct);
#endif
                }
            } else if (!altHoldCascadeLatched) {
                altHoldCascadeLatched = true;
                altHoldGuardPhase = 2;
                primeCascadeController(controlAlt);
                thr = holdCascaded(controlAlt, false);
#if SERIAL_FLIGHT_DEBUG
                Serial.printf("[ALT_HOLD] cascade active after settle rel=%.2f tof=%.2f tofW=%u\n",
                    altitude, tofAgl, lastTofWeightPct);
#endif
            } else {
                altHoldGuardPhase = 2;
                thr = holdCascaded(controlAlt, false);
            }
            if (altHoldGuardPhase != 2) {
                lastAltError = 0.0f;
                lastDesiredVspeed = 0.0f;
                lastVspeedError = 0.0f;
                lastControlPUs = 0.0f;
                lastControlIUs = 0.0f;
                lastRawThrottle = thr;
                lastThrMin = thr;
                lastThrMax = thr;
                lastClampedThrottle = thr;
                lastThrottleSat = 0;
            }
            channels[CH_THROTTLE] = thr;
            sendRC();
            logFlightSample(timeInAltHold, altHoldGuardPhase == 0 ? "settle" : "cascade",
                controlAlt, controlAlt, thr);
            digitalWrite(STATUS_LED, millis() % 500 < 250);

            // Tight safety ceiling: if baro or controller misbehaves, land before getting dangerous.
            float holdTarget  = constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M);
            float safeCeiling = holdTarget + 2.0f;
            if (controlAlt > safeCeiling) {
#if SERIAL_FLIGHT_DEBUG
                Serial.printf("[ALT_HOLD] safety ceiling %.1fm exceeded at %.2fm, landing\n", safeCeiling, controlAlt);
#endif
                startLanding(controlAlt);
            }
            if (attitudeAbortActive()) {
                disarmToIdle("[ALT_HOLD] attitude abort");
            }
            break;
        }

        // ── LANDING ──────────────────────────────────────────
        case LANDING: {
            channels[CH_ARM]   = 1800;
            channels[CH_ANGLE] = angleModeChannelValue();
            digitalWrite(STATUS_LED, millis() % 200 < 30);
            if (attitudeAbortActive()) {
                disarmToIdle("[LANDING] attitude abort");
                break;
            }

            // ToF can detect ground immediately. Baro/fused fallback must first
            // show real descent so a stale zero cannot kill motors mid-air.
            float landingAlt = altitude;
            float tofAgl = lastTofValid ? lastTofAltM : altitude;
            bool hasDescended = (landingStartAlt - landingAlt) > 0.3f;
            bool tofGround = lastTofValid && lastTofWeightPct >= 80 && lastTofAltM <= LANDING_GROUND_M;
            bool fallbackGround = !tofGround && hasDescended && landingAlt <= LANDING_GROUND_M;
            bool startedOnGround = landingStartAlt <= LANDING_GROUND_M && landingAlt <= LANDING_GROUND_M;
            if (tofGround || fallbackGround || startedOnGround) {
                disarmToIdle("[LANDING] ground detected");
                break;
            }
            if (millis() - landingStartMs >= LANDING_TIMEOUT_MS) {
                disarmToIdle("[LANDING] timeout");
                break;
            }

            bool landingVarioFresh = (millis() - lastVarioMs) < VARIO_STALE_MS
                                      && abs(lastVario) <= VARIO_MAX_PLAUSIBLE_CMS;
            float landingVario = landingVarioFresh ? lastVario / 100.0f : filteredVario;
            float predictedLandingAlt = max(0.0f, landingAlt + landingVario * LANDING_LOOKAHEAD_S);
            float targetDescentMps = DESCENT_RATE_MPS;
            float landingOffsetUs = LANDING_THROTTLE_OFFSET_US;
            if (predictedLandingAlt <= LANDING_FINAL_ALT_M) {
                targetDescentMps = LANDING_FINAL_DESCENT_MPS;
                landingOffsetUs = LANDING_FINAL_OFFSET_US;
            } else if (predictedLandingAlt <= LANDING_FLARE_ALT_M) {
                float u = (predictedLandingAlt - LANDING_FINAL_ALT_M) / (LANDING_FLARE_ALT_M - LANDING_FINAL_ALT_M);
                targetDescentMps = LANDING_FINAL_DESCENT_MPS
                    + u * (LANDING_FLARE_DESCENT_MPS - LANDING_FINAL_DESCENT_MPS);
                landingOffsetUs = LANDING_FINAL_OFFSET_US
                    + u * (LANDING_FLARE_OFFSET_US - LANDING_FINAL_OFFSET_US);
            }

            uint32_t landingElapsedMs = millis() - landingStartMs;
            if (landingElapsedMs < LANDING_ENTRY_RAMP_MS) {
                float ramp = landingElapsedMs / (float)LANDING_ENTRY_RAMP_MS;
                landingOffsetUs = LANDING_ENTRY_START_OFFSET_US
                    + ramp * (landingOffsetUs - LANDING_ENTRY_START_OFFSET_US);
            }

            float rateError           = (-targetDescentMps) - landingVario;
            int16_t correction        = (int16_t)(LANDING_KP_VSPEED * rateError);
            float landingBaseThrottle = max((float)MIN_ALT_HOLD_THROTTLE_US,
                                            (float)HOVER_THROTTLE - landingOffsetUs);
            float landingRawThrottle  = landingBaseThrottle + correction;
            float landingMinThrottle  = max(1000.0f, (float)HOVER_THROTTLE - (float)LANDING_MIN_BELOW_HOVER_US);
            if (predictedLandingAlt <= LANDING_LOW_ALT_FLOOR_M && landingVario < -LANDING_FINAL_DESCENT_MPS) {
                float descentMps = -landingVario;
                float speedU = (descentMps - LANDING_FINAL_DESCENT_MPS)
                    / (LANDING_FAST_DESCENT_MPS - LANDING_FINAL_DESCENT_MPS);
                speedU = constrain(speedU, 0.0f, 1.0f);
                float floorOffsetUs = LANDING_LOW_ALT_MIN_OFFSET_US
                    + speedU * (LANDING_FAST_DESCENT_MIN_OFFSET_US - LANDING_LOW_ALT_MIN_OFFSET_US);
                landingMinThrottle = max(landingMinThrottle, (float)HOVER_THROTTLE - floorOffsetUs);
            }
            float landingMaxThrottle  = (float)HOVER_THROTTLE + 80.0f;
            float landingClampedThrottle = constrain(landingRawThrottle, landingMinThrottle, landingMaxThrottle);
            if (lastClampedThrottle > 0) {
                float minSlewThrottle = max(landingMinThrottle, (float)lastClampedThrottle - (float)THROTTLE_SLEW_DOWN_US);
                float maxSlewThrottle = min(landingMaxThrottle, (float)lastClampedThrottle + (float)THROTTLE_SLEW_UP_US);
                landingClampedThrottle = constrain(landingClampedThrottle, minSlewThrottle, maxSlewThrottle);
            }
            channels[CH_THROTTLE]     = (uint16_t)landingClampedThrottle;
            lastAltError = 0.0f;
            lastDesiredVspeed = -targetDescentMps;
            lastVspeedError = rateError;
            lastControlPUs = correction;
            lastControlIUs = 0.0f;
            lastRawThrottle = landingRawThrottle;
            lastThrMin = landingMinThrottle;
            lastThrMax = landingMaxThrottle;
            lastClampedThrottle = channels[CH_THROTTLE];
            lastThrottleSat = landingRawThrottle > landingClampedThrottle ? 1 : (landingRawThrottle < landingClampedThrottle ? -1 : 0);
            sendRC();
            logFlightSample(millis() - landingStartMs, "landing", landingAlt, landingAlt, channels[CH_THROTTLE]);

#if SERIAL_FLIGHT_DEBUG
            Serial.printf("[LANDING] alt=%.2fm  tof=%.2f  controlV=%.2f filtV=%.2f  thr=%d\n",
                landingAlt, tofAgl, landingVario, filteredVario, channels[CH_THROTTLE]);
#endif
            break;
        }

        case DONE:
            sendRC();
            digitalWrite(STATUS_LED, millis() % 100 < 50);
            break;
    }
}
