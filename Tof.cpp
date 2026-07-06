#include "Tof.h"
#include "Config.h"
#include "State.h"

#if TOF_ENABLED
#include <Wire.h>
#include <VL53L1X.h>

static VL53L1X tofSensor;
static bool tofReady = false;
static float lastGoodTofM = 0.0f;
static uint32_t lastGoodTofMs = 0;

void setupTof() {
#if TOF_SHUT_PIN >= 0
    pinMode(TOF_SHUT_PIN, OUTPUT);
    digitalWrite(TOF_SHUT_PIN, LOW);
    delay(5);
    digitalWrite(TOF_SHUT_PIN, HIGH);
    delay(10);
#endif

    Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN, TOF_I2C_HZ);
    tofSensor.setTimeout(TOF_TIMEOUT_MS);

    if (!tofSensor.init()) {
        Serial.println("[TOF] VL53L1X init failed");
        return;
    }

    tofSensor.setDistanceMode(VL53L1X::Long);
    tofSensor.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tofSensor.startContinuous(TOF_PERIOD_MS);
    tofReady = true;
    Serial.printf("[TOF] VL53L1X ready SDA=%d SCL=%d\n", TOF_SDA_PIN, TOF_SCL_PIN);
}

bool readTofAltitude(float& altitudeM) {
    static uint32_t lastReadAttemptMs = 0;
    uint32_t nowMs = millis();
    lastTofReadDtMs = lastReadAttemptMs == 0 ? 0 : (uint16_t)min(nowMs - lastReadAttemptMs, 65535UL);
    lastReadAttemptMs = nowMs;
    lastTofReadOk = false;
    lastTofRejectReason = 0;
    lastTofI2cStatus = tofSensor.last_status;

    if (!tofReady) {
        lastTofRejectReason = 1;
        return false;
    }

    if (!tofSensor.dataReady()) {
        lastTofRejectReason = 2;
        lastTofI2cStatus = tofSensor.last_status;
        if (lastGoodTofMs != 0 && nowMs - lastGoodTofMs <= TOF_RECENT_VALID_MS) {
            altitudeM = lastGoodTofM;
            return true;
        }
        return false;
    }

    uint16_t distanceMm = tofSensor.read(false);
    lastTofRangeStatus = (uint8_t)tofSensor.ranging_data.range_status;
    lastTofI2cStatus = tofSensor.last_status;
    if (tofSensor.timeoutOccurred()) {
        lastTofRejectReason = 3;
        return false;
    }

    float measuredM = distanceMm / 1000.0f;
    lastTofRawM = measuredM;
    if (measuredM > TOF_VALID_MAX_M) {
        lastTofRejectReason = 4;
        return false;
    }
    if (measuredM < TOF_VALID_MIN_M) {
        // Mounted close to the ground, the VL53L1X can report below its useful
        // range while the aircraft is still sitting on the floor. Treat that
        // as a valid ground reference instead of blocking ToF baseline setup.
        measuredM = 0.0f;
    }

    if (lastGoodTofMs != 0) {
        float dt = max((nowMs - lastGoodTofMs) / 1000.0f, 0.02f);
        float maxStepM = max((float)TOF_MAX_STEP_MIN_M, (float)TOF_MAX_STEP_MPS * dt);
        if (fabsf(measuredM - lastGoodTofM) > maxStepM) {
            lastTofRejectReason = 5;
            return false;
        }
    }

    lastGoodTofM = measuredM;
    lastGoodTofMs = nowMs;
    altitudeM = measuredM;
    lastTofReadOk = true;
    lastTofRejectReason = 0;
    return true;
}

#else

void setupTof() {}

bool readTofAltitude(float& altitudeM) {
    (void)altitudeM;
    lastTofReadOk = false;
    lastTofRejectReason = 1;
    lastTofRangeStatus = 255;
    lastTofI2cStatus = 0;
    return false;
}

#endif

void resetTofFilter() {
#if TOF_ENABLED
    lastGoodTofM = 0.0f;
    lastGoodTofMs = 0;
#endif
    lastTofReadOk = false;
    lastTofRawM = 0.0f;
    lastTofRejectReason = 0;
    lastTofRangeStatus = 255;
    lastTofI2cStatus = 0;
    lastTofReadDtMs = 0;
}

float tofBlendWeight(float altitudeM, bool valid) {
    if (!valid) return 0.0f;
    if (altitudeM <= TOF_BLEND_FULL_M) return 1.0f;
    if (altitudeM >= TOF_BLEND_ZERO_M) return 0.0f;
    return (TOF_BLEND_ZERO_M - altitudeM) / (TOF_BLEND_ZERO_M - TOF_BLEND_FULL_M);
}
