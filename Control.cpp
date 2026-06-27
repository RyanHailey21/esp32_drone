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
//  Reference shaping: internalSetpoint ramps toward TARGET_ALT_M
//  at ALT_RAMP_RATE_MPS to avoid stepping the setpoint.
// ============================================================

void resetCascadeController(float currentAlt) {
    internalSetpoint = currentAlt;   // bumpless: start from current altitude
    filteredVario    = 0.0f;
    vspeedIntegral   = 0.0f;
    vspeedLastMs     = millis();
    lastVarioMs      = millis();     // treat vario as fresh at reset
}

uint16_t holdCascaded(float altitude, bool isMission) {
    uint32_t now = millis();
    float dt = constrain((now - vspeedLastMs) / 1000.0f, 0.005f, 0.2f);
    vspeedLastMs = now;

    // 1. Reference shaping: ramp internal setpoint toward TARGET_ALT_M
    float target = TARGET_ALT_M;
    if (internalSetpoint < target) {
        internalSetpoint = min(internalSetpoint + ALT_RAMP_RATE_MPS * dt, target);
    } else if (internalSetpoint > target) {
        internalSetpoint = max(internalSetpoint - ALT_RAMP_RATE_MPS * dt, target);
    }

    // 2. Outer loop: altitude error → desired vertical speed
    float altError = internalSetpoint - altitude;
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

    // filteredVario is maintained by runMissionLoop() every iteration.
    // If vario goes stale the filter just holds its last value — safe degraded behaviour.

    // 4. Inner PI loop with conditional anti-windup
    float vspeedError  = desiredVspeed - filteredVario;
    float thrMin = (float)HOVER_THROTTLE - THR_DOWN_OFFSET_US;
    float thrMax = (float)HOVER_THROTTLE + THR_UP_OFFSET_US;

    // Only integrate when not saturated in the direction of the error
    float rawThrottle = (float)HOVER_THROTTLE + HOLD_KD * vspeedError + HOLD_KI * vspeedIntegral;
    bool satHigh = rawThrottle > thrMax && vspeedError > 0;
    bool satLow  = rawThrottle < thrMin && vspeedError < 0;
    if (!satHigh && !satLow) {
        vspeedIntegral += vspeedError * dt;
        vspeedIntegral  = constrain(vspeedIntegral, -KI_VSPEED_LIMIT, KI_VSPEED_LIMIT);
    }

    float finalThrottle = (float)HOVER_THROTTLE + HOLD_KD * vspeedError + HOLD_KI * vspeedIntegral;

    // Rate-limited debug log at 10 Hz
    static uint32_t lastLogMs = 0;
    if (now - lastLogMs >= 100) {
        lastLogMs = now;
        Serial.printf("[CASCADE] setpt=%.2f aErr=%.2f desV=%.2f fV=%.2f vErr=%.2f "
                      "P=%.0f I=%.0f raw=%.0f sat=%s\n",
            internalSetpoint, altError, desiredVspeed, filteredVario, vspeedError,
            HOLD_KD * vspeedError, HOLD_KI * vspeedIntegral, finalThrottle,
            satHigh ? "HI" : (satLow ? "LO" : "ok"));
    }

    return (uint16_t)constrain(finalThrottle, thrMin, thrMax);
}

void startHoverTest() {
    armTime = millis();
    armingForHover = true;
    state = ARMING;
    Serial.println("[STATE] -> ARMING (hover test)");
}

void startMission() {
    armTime = millis();
    state   = ARMING;
    Serial.println("[STATE] -> ARMING");
}

void startAutoHoverCal() {
    calThrottle = CAL_START_THROTTLE;
    armTime = millis();
    armingForAutoCal = true;
    state = ARMING;
    Serial.println("[STATE] -> ARMING (auto hover cal)");
}

void startAltHold() {
    armTime           = millis();
    armingForAltHold  = true;
    state = ARMING;
    Serial.printf("[STATE] -> ARMING (alt hold, target=%.1fm)\n", TARGET_ALT_M);
}

void startLanding(float currentAlt) {
    landingStartMs  = millis();
    landingStartAlt = currentAlt;
    state = LANDING;
    Serial.printf("[STATE] -> LANDING from alt=%.2fm\n", currentAlt);
}

void disarmToIdle(const char* reason) {
    ledcWrite(MOTOR_PWM_PIN, 0);
    channels[4] = 1000;
    channels[5] = 1000;
    channels[2] = 1000;
    launchAlt = 0;
    state = IDLE;
    Serial.println(reason);
    // Do NOT call sendRC() here — this may be invoked from the NimBLE task,
    // which would race with the main loop's fcSerial writes and corrupt MSP framing.
    // The main loop sends RC on its next iteration within one cycle (<70ms).
}
