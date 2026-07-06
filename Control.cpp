#include "Control.h"

// ============================================================
//  ALTITUDE HOLD — cascaded controller
//
//  Outer loop: altitude error → desired vertical speed
//    HOLD_KP: alt error (m) → desired speed (m/s)
//
//  Inner loop: vertical speed error → throttle (PI)
//    HOLD_KD: speed error (m/s) → throttle offset (µs)  [inner P]
//    HOLD_KI: speed integral   → throttle offset (µs)   [inner I]
//
//  Reference shaping: internalSetpoint ramps toward the active target.
//  at ALT_RAMP_RATE_MPS to avoid stepping the setpoint.
// ============================================================

void resetCascadeController(float currentAlt) {
    internalSetpoint = currentAlt;
    filteredVario    = 0.0f;
    vspeedIntegral   = 0.0f;
    vspeedLastMs     = millis();
    lastVarioMs      = millis();
}

void primeCascadeController(float currentAlt) {
    internalSetpoint = currentAlt;
    vspeedIntegral   = 0.0f;
    vspeedLastMs     = millis();
}

uint16_t holdCascaded(float altitude, bool isMission) {
    uint32_t now = millis();
    float dt = (now - vspeedLastMs) / 1000.0f;
    vspeedLastMs = now;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.15f) {
        dt = 0.02f;
        vspeedIntegral = 0.0f;
    }

    bool varioValid = (now - lastVarioMs) <= VARIO_STALE_MS
                      && isfinite(filteredVario)
                      && fabsf(filteredVario) <= (VARIO_MAX_PLAUSIBLE_CMS / 100.0f);
    if (!varioValid || !isfinite(altitude)) {
        vspeedIntegral = 0.0f;
        lastAltError = 0.0f;
        lastDesiredVspeed = 0.0f;
        lastVspeedError = 0.0f;
        lastControlPUs = 0.0f;
        lastControlIUs = 0.0f;
        lastRawThrottle = HOVER_THROTTLE;
        lastThrMin = HOVER_THROTTLE;
        lastThrMax = HOVER_THROTTLE;
        lastClampedThrottle = HOVER_THROTTLE;
        lastThrottleSat = 0;
#if SERIAL_FLIGHT_DEBUG
        Serial.printf("[CASCADE] invalid sensor data: alt=%.2f filtV=%.2f age=%ums -> LANDING\n",
            altitude, filteredVario, now - lastVarioMs);
#endif
        startLanding(altitude);
        return HOVER_THROTTLE;
    }

    // 1. Reference shaping: ramp internal setpoint toward the active target.
    float target = isMission
        ? TARGET_ALT_M
        : constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M);
    if (internalSetpoint < target) {
        internalSetpoint = min(internalSetpoint + ALT_RAMP_RATE_MPS * dt, target);
    } else if (internalSetpoint > target) {
        internalSetpoint = max(internalSetpoint - ALT_RAMP_RATE_MPS * dt, target);
    }

    // 2. Outer loop: altitude error → desired vertical speed
    // Predict altitude slightly ahead to compensate the observed control lag.
    float predictionVario = constrain(filteredVario,
                                      -ALT_HOLD_LOOKAHEAD_MAX_V_MPS,
                                      ALT_HOLD_LOOKAHEAD_MAX_V_MPS);
    float predictedAlt = altitude + predictionVario * ALT_HOLD_LOOKAHEAD_S;
    float altError = internalSetpoint - predictedAlt;
    float maxClimb = isMission ? MAX_CLIMB_MPS_HOLD  : MAX_CLIMB_MPS_TEST;
    float maxDesc  = isMission ? MAX_DESCENT_MPS_HOLD : MAX_DESCENT_MPS_TEST;

    // Near target: scale down speed limits to cushion final approach
    if (fabsf(altError) < NEAR_TARGET_M) {
        float factor = fabsf(altError) / NEAR_TARGET_M;
        factor = NEAR_TARGET_FACTOR + (1.0f - NEAR_TARGET_FACTOR) * factor;
        maxClimb *= factor;
        maxDesc  *= factor;
    }
    float desiredVspeed = constrain(HOLD_KP * altError, -maxDesc, maxClimb);
    bool captureNeedsHelp = !isMission
        && altitude < target - ALT_HOLD_CAPTURE_MARGIN_M
        && filteredVario < ALT_HOLD_CAPTURE_MIN_CLIMB_MPS;
    if (captureNeedsHelp) {
        desiredVspeed = max(desiredVspeed, min((float)ALT_HOLD_CAPTURE_MIN_CLIMB_MPS, maxClimb));
    }

    // filteredVario is maintained by runMissionLoop() every iteration.
    // Sensor validity is checked above before throttle corrections are made.

    // Inner PI loop with conditional anti-windup
    float vspeedError  = desiredVspeed - filteredVario;
    float minControlThrottle = isMission ? (float)MIN_MISSION_THROTTLE_US : (float)MIN_ALT_HOLD_THROTTLE_US;
    float thrMin = max(minControlThrottle,
                       (float)HOVER_THROTTLE - THR_DOWN_OFFSET_US);
    float thrMax = (float)HOVER_THROTTLE + THR_UP_OFFSET_US;

    float iLimit = (HOLD_KI > 0.001f) ? (VSPEED_I_MAX_US / HOLD_KI) : 0.0f;
    float candidateIntegral = constrain(vspeedIntegral + vspeedError * dt, -iLimit, iLimit);
    float candidateThrottle = (float)HOVER_THROTTLE
                            + HOLD_KD * vspeedError
                            + HOLD_KI * candidateIntegral;
    bool satHigh = candidateThrottle > thrMax && vspeedError > 0.0f;
    bool satLow  = candidateThrottle < thrMin && vspeedError < 0.0f;
    if (!satHigh && !satLow) {
        vspeedIntegral = candidateIntegral;
    }

    float finalThrottle = (float)HOVER_THROTTLE + HOLD_KD * vspeedError + HOLD_KI * vspeedIntegral;
    float clampedThrottle = constrain(finalThrottle, thrMin, thrMax);
    int8_t sat = finalThrottle > thrMax ? 1 : (finalThrottle < thrMin ? -1 : 0);
    if (captureNeedsHelp && filteredVario < ALT_HOLD_CAPTURE_FLOOR_MAX_V_MPS) {
        float captureOffsetUs = filteredVario < ALT_HOLD_RECOVERY_DESCENT_MPS
            ? (float)ALT_HOLD_RECOVERY_MIN_OFFSET_US
            : (float)ALT_HOLD_CAPTURE_MIN_OFFSET_US;
        float captureMinThrottle = min(thrMax, (float)HOVER_THROTTLE + captureOffsetUs);
        if (clampedThrottle < captureMinThrottle) {
            clampedThrottle = captureMinThrottle;
            sat = 1;
        }
    }

    lastAltError = altError;
    lastDesiredVspeed = desiredVspeed;
    lastVspeedError = vspeedError;
    lastControlPUs = HOLD_KD * vspeedError;
    lastControlIUs = HOLD_KI * vspeedIntegral;
    lastRawThrottle = finalThrottle;
    lastThrMin = thrMin;
    lastThrMax = thrMax;
    lastClampedThrottle = (uint16_t)clampedThrottle;
    lastThrottleSat = sat;

#if SERIAL_FLIGHT_DEBUG
    static uint32_t lastLogMs = 0;
    if (now - lastLogMs >= 100) {
        lastLogMs = now;
        Serial.printf("[CASCADE] setpt=%.2f aErr=%.2f desV=%.2f fV=%.2f vErr=%.2f "
                      "P=%.0f I=%.0f raw=%.0f out=%.0f min=%.0f max=%.0f sat=%s\n",
            internalSetpoint, altError, desiredVspeed, filteredVario, vspeedError,
            lastControlPUs, lastControlIUs, finalThrottle, clampedThrottle, thrMin, thrMax,
            sat > 0 ? "HI" : (sat < 0 ? "LO" : "ok"));
    }
#endif

    return (uint16_t)clampedThrottle;
}

void startHoverTest() {
    armTime   = millis();
    armTarget = ARM_HOVER_TEST;
    state     = ARMING;
    Serial.println("[STATE] -> ARMING (hover test)");
}

void startMission() {
    armTime   = millis();
    armTarget = ARM_MISSION;
    state     = ARMING;
    Serial.println("[STATE] -> ARMING");
}

void startAutoHoverCal() {
    calThrottle = CAL_START_THROTTLE;
    armTime     = millis();
    armTarget   = ARM_AUTO_HOVER_CAL;
    state       = ARMING;
    Serial.println("[STATE] -> ARMING (auto hover cal)");
}

void startAltHold() {
    armTime   = millis();
    armTarget = ARM_ALT_HOLD;
    state     = ARMING;
    Serial.printf("[STATE] -> ARMING (alt hold, target=%.1fm)\n",
        constrain((float)ALT_HOLD_TARGET_M, ALT_HOLD_TARGET_MIN_M, ALT_HOLD_TARGET_MAX_M));
}

void startLanding(float currentAlt) {
    landingStartMs  = millis();
    landingStartAlt = currentAlt;
    state           = LANDING;
    Serial.printf("[STATE] -> LANDING from alt=%.2fm\n", currentAlt);
}

void disarmToIdle(const char* reason) {
    ledcWrite(MOTOR_PWM_PIN, 0);
    channels[CH_ARM]      = 1000;
    channels[CH_ANGLE]    = 1000;
    channels[CH_THROTTLE] = 1000;
    launchAlt = 0;
    state     = IDLE;
    Serial.println(reason);
    // Do NOT call sendRC() here — invoked from NimBLE task, races with main loop fcSerial.
    // The main loop sends RC on its next iteration within one cycle (<70ms).
}
