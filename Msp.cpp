#include "Msp.h"
#include "Tof.h"

static bool baroOffsetInitialized = false;
static float baroToTofOffsetM = 0.0f;
static float trustedTofAltM = 0.0f;
static uint32_t trustedTofMs = 0;
static bool fusedOutputOffsetInitialized = false;
static float fusedOutputOffsetM = 0.0f;

void resetAltitudeFusion() {
    baroOffsetInitialized = false;
    baroToTofOffsetM = 0.0f;
    trustedTofAltM = 0.0f;
    trustedTofMs = 0;
    fusedOutputOffsetInitialized = false;
    fusedOutputOffsetM = 0.0f;
    resetTofFilter();
}

static void sendMSP(uint8_t cmd, const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    fcSerial.write('$');
    fcSerial.write('M');
    fcSerial.write('<');
    fcSerial.write(len); cs ^= len;
    fcSerial.write(cmd); cs ^= cmd;
    for (int i = 0; i < len; i++) { fcSerial.write(data[i]); cs ^= data[i]; }
    fcSerial.write(cs);
}

static bool readByteUntil(uint8_t& value, uint32_t deadline) {
    while (!fcSerial.available() && millis() < deadline);
    if (!fcSerial.available()) return false;
    value = (uint8_t)fcSerial.read();
    return true;
}

static int16_t constrainVario(float cms) {
    return (int16_t)constrain(cms, -32768.0f, 32767.0f);
}

static float fuseAltitude(float baroAltM) {
    float tofAltM = 0.0f;
    bool rawTofValid = readTofAltitude(tofAltM);
    uint32_t nowMs = millis();

    bool tofHoldoverValid = false;
    if (rawTofValid) {
        trustedTofAltM = tofAltM;
        trustedTofMs = nowMs;
    } else if (trustedTofMs != 0 && nowMs - trustedTofMs <= TOF_HOLDOVER_MS
               && trustedTofAltM <= TOF_BLEND_FULL_M) {
        tofAltM = trustedTofAltM;
        tofHoldoverValid = true;
    }

    bool tofValid = rawTofValid || tofHoldoverValid;
    float tofWeight = tofBlendWeight(tofAltM, tofValid);

    if (rawTofValid && tofWeight >= 0.25f) {
        float measuredOffset = tofAltM - baroAltM;
        if (!baroOffsetInitialized) {
            baroToTofOffsetM = measuredOffset;
            baroOffsetInitialized = true;
        } else {
            baroToTofOffsetM += TOF_OFFSET_ALPHA * (measuredOffset - baroToTofOffsetM);
        }
    }

    float correctedBaroM = baroAltM + (baroOffsetInitialized ? baroToTofOffsetM : 0.0f);
    float fusedAltM = tofValid
        ? correctedBaroM + tofWeight * (tofAltM - correctedBaroM)
        : correctedBaroM;
    if (!fusedOutputOffsetInitialized) {
        fusedOutputOffsetM = fusedAltM - baroAltM;
        fusedOutputOffsetInitialized = true;
    }
    fusedAltM -= fusedOutputOffsetM;
    correctedBaroM -= fusedOutputOffsetM;

    lastTofValid = tofValid;
    lastTofAltM = tofValid ? tofAltM : 0.0f;
    lastTofWeightPct = (uint8_t)constrain(tofWeight * 100.0f + 0.5f, 0.0f, 100.0f);
    lastBaroAltM = baroAltM;
    lastCorrectedBaroAltM = correctedBaroM;
    lastFusedAltM = fusedAltM;
    lastAltitudeSource = tofHoldoverValid ? 3 : (tofWeight >= 0.95f ? 1 : (tofWeight > 0.0f ? 2 : 0));

    return fusedAltM;
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
        static float lastAlt = 0;
        while (fcSerial.available()) fcSerial.read();
        sendMSP(MSP_ALTITUDE, nullptr, 0);
        uint32_t timeout = millis() + 60;
        uint16_t rxBytes = 0;
        uint8_t lastByte = 0;
        while (millis() < timeout) {
            uint8_t b = 0;
            if (!readByteUntil(b, timeout)) break;
            rxBytes++;
            lastByte = b;
            if (b != '$') continue;
            if (!readByteUntil(b, timeout)) break;
            rxBytes++;
            lastByte = b;
            if (b != 'M') continue;
            if (!readByteUntil(b, timeout)) break;
            rxBytes++;
            lastByte = b;
            if (b != '>') continue;

            uint8_t len = 0, cmd = 0;
            if (!readByteUntil(len, timeout)) break;
            rxBytes++;
            lastByte = len;
            if (!readByteUntil(cmd, timeout)) break;
            rxBytes++;
            lastByte = cmd;

            uint8_t checksum = len ^ cmd;
            uint8_t payload[16];
            bool keepPayload = (cmd == MSP_ALTITUDE && len == 6);
            for (uint8_t i = 0; i < len; i++) {
                if (!readByteUntil(b, timeout)) return lastAlt;
                rxBytes++;
                lastByte = b;
                checksum ^= b;
                if (keepPayload && i < sizeof(payload)) payload[i] = b;
            }
            if (!readByteUntil(b, timeout)) break;
            rxBytes++;
            lastByte = b;
            if (checksum != b) continue;
            if (!keepPayload) continue;

            int32_t alt_cm = 0;
            alt_cm  = (uint32_t)payload[0];
            alt_cm |= (uint32_t)payload[1] << 8;
            alt_cm |= (uint32_t)payload[2] << 16;
            alt_cm |= (uint32_t)payload[3] << 24;
            memcpy(lastMspAltitudePayload, payload, sizeof(lastMspAltitudePayload));
            int16_t vario = 0;
            vario  = (uint16_t)payload[4];
            vario |= (uint16_t)payload[5] << 8;
            static uint32_t mspRawLogMs = 0;
            uint32_t nowMs = millis();
            if (nowMs - mspRawLogMs >= 500) {
                mspRawLogMs = nowMs;
                Serial.printf("[MSP_ALTITUDE] len=%u raw=", len);
                for (uint8_t i = 0; i < len; i++) {
                    Serial.printf("%02X ", payload[i]);
                }
                Serial.printf("alt=%ld fcVario=%d\n", (long)alt_cm, vario);
            }

            float baroAltM = alt_cm / 100.0f;
            float fusedAltM = fuseAltitude(baroAltM);

            // If Betaflight sends 0 vario, derive speed from altitude. Use a
            // shorter ToF window when valid; fall back to a slower fused/baro
            // derivative to avoid baro quantization pulses.
            static float derivBaseAltM = 0.0f;
            static uint32_t derivBaseMs = 0;
            static float tofDerivBaseAltM = 0.0f;
            static uint32_t tofDerivBaseMs = 0;
            static int16_t heldDerivedVario = 0;
            static float filteredDerivedCms = 0.0f;
            static float filteredTofDerivedCms = 0.0f;
            if (lastTofValid && lastTofWeightPct >= 80) {
                if (tofDerivBaseMs == 0) {
                    tofDerivBaseAltM = lastTofAltM;
                    tofDerivBaseMs = nowMs;
                } else if (nowMs - tofDerivBaseMs >= 100) {
                    float dtSec = (nowMs - tofDerivBaseMs) / 1000.0f;
                    float rawTofCms = (lastTofAltM - tofDerivBaseAltM) * 100.0f / dtSec;
                    constexpr float TOF_DERIVED_ALPHA = 0.55f;
                    filteredTofDerivedCms += TOF_DERIVED_ALPHA * (rawTofCms - filteredTofDerivedCms);
                    heldDerivedVario = constrainVario(filteredTofDerivedCms);
                    tofDerivBaseAltM = lastTofAltM;
                    tofDerivBaseMs = nowMs;
                }
            } else {
                tofDerivBaseMs = 0;
                if (derivBaseMs == 0) {
                    derivBaseAltM = fusedAltM;
                    derivBaseMs = nowMs;
                } else if (nowMs - derivBaseMs >= 400) {
                    float dtSec = (nowMs - derivBaseMs) / 1000.0f;
                    float rawDerivedCms = (fusedAltM - derivBaseAltM) * 100.0f / dtSec;
                    constexpr float DERIVED_ALPHA = 0.25f;
                    filteredDerivedCms += DERIVED_ALPHA * (rawDerivedCms - filteredDerivedCms);
                    heldDerivedVario = constrainVario(filteredDerivedCms);
                    derivBaseAltM = fusedAltM;
                    derivBaseMs = nowMs;
                }
            }

            // Betaflight 4.4.x MSP_ALTITUDE often reports a zero vario field
            // unless firmware is built with USE_VARIO. Keep FC vario for
            // diagnostics, but use the derived/smoothed estimate for control.
            int16_t usedVario = heldDerivedVario;

            lastAlt     = fusedAltM;
            lastVario   = usedVario;
            lastFcVario = vario;
            lastDerivedVario = heldDerivedVario;
            lastVarioMs = nowMs;
            static uint32_t varioLogMs = 0;
            if (nowMs - varioLogMs >= 500) {
                varioLogMs = nowMs;
                Serial.printf("[ALT] fused=%.2f baro=%.2f cbaro=%.2f src=%u tof=%s%.2f w=%u%% fc_vario=%d derived=%d used=%d cm/s\n",
                    lastAlt, baroAltM, lastCorrectedBaroAltM, lastAltitudeSource,
                    lastTofValid ? "" : "invalid:",
                    lastTofAltM, lastTofWeightPct, vario, heldDerivedVario, usedVario);
            }
            return lastAlt;
        }
        static uint32_t mspTimeoutLogMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - mspTimeoutLogMs >= 500) {
            mspTimeoutLogMs = nowMs;
            Serial.printf("[MSP_ALTITUDE] no valid response rxBytes=%u last=0x%02X lastAlt=%.2f\n",
                rxBytes, lastByte, lastAlt);
        }
        return lastAlt;
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
        case ALT_HOLD:
            benchAlt = launchAlt + (state == ALT_HOLD ? ALT_HOLD_TARGET_M : TARGET_ALT_M);
            lastVario   = 0;
            lastFcVario = 0;
            lastDerivedVario = 0;
            memset(lastMspAltitudePayload, 0, sizeof(lastMspAltitudePayload));
            lastVarioMs = millis();   // keep vario "fresh" so cascade filter stays active
            break;

        case PUNCHING:
            benchAlt += BENCH_PUNCH_RATE_MPS * dt;
            break;

        case CUT:
        case DONE:
            break;
    }

    lastBaroAltM = benchAlt;
    lastCorrectedBaroAltM = benchAlt;
    lastFusedAltM = benchAlt;
    lastTofValid = false;
    lastTofAltM = 0.0f;
    lastTofWeightPct = 0;
    lastAltitudeSource = 0;

    return benchAlt;
}
