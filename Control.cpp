#include "Control.h"

// ============================================================
//  ALTITUDE HOLD — simple PID controller
//  error > 0: below target → add throttle
//  error < 0: above target → reduce throttle
// ============================================================

uint16_t holdPID(float altitude) {
    uint32_t now = millis();
    float dt = constrain((now - holdLastMs) / 1000.0f, 0.005f, 0.2f);
    holdLastMs = now;

    float error = TARGET_ALT_M - altitude;

    holdIntegral += error * dt;
    holdIntegral  = constrain(holdIntegral, -10.0f, 10.0f);  // windup limit

    float varioMs = lastVario / 100.0f;  // cm/s → m/s

    float p = HOLD_KP * error;
    float i = HOLD_KI * holdIntegral;
    float d = -HOLD_KD * varioMs;        // negative: climbing → reduce throttle

    return (uint16_t)constrain(HOVER_THROTTLE + p + i + d, 1200, 1700);
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
    holdIntegral    = 0;
    state = LANDING;
    Serial.printf("[STATE] -> LANDING from alt=%.2fm\n", currentAlt);
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
