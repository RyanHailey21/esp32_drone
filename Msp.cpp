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

static int16_t readS16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t readU16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readU32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool requestMSP(uint8_t cmd, uint8_t* payload, uint8_t payloadCapacity,
                       uint8_t& payloadLen, uint32_t timeoutMs) {
    payloadLen = 0;
    while (fcSerial.available()) fcSerial.read();
    sendMSP(cmd, nullptr, 0);
    uint32_t timeout = millis() + timeoutMs;

    while (millis() < timeout) {
        uint8_t b = 0;
        if (!readByteUntil(b, timeout)) break;
        if (b != '$') continue;
        if (!readByteUntil(b, timeout) || b != 'M') continue;
        if (!readByteUntil(b, timeout) || b != '>') continue;

        uint8_t len = 0;
        uint8_t responseCmd = 0;
        if (!readByteUntil(len, timeout)) break;
        if (!readByteUntil(responseCmd, timeout)) break;

        uint8_t checksum = len ^ responseCmd;
        uint8_t scratch[32];
        for (uint8_t i = 0; i < len; i++) {
            if (!readByteUntil(b, timeout)) return false;
            checksum ^= b;
            if (i < sizeof(scratch)) scratch[i] = b;
        }
        if (!readByteUntil(b, timeout)) break;
        if (checksum != b || responseCmd != cmd) continue;

        payloadLen = min(len, payloadCapacity);
        memcpy(payload, scratch, payloadLen);
        return true;
    }

    return false;
}

static int16_t constrainVario(float cms) {
    return (int16_t)constrain(cms, -32768.0f, 32767.0f);
}

static int16_t selectVarioForControl(int16_t fcVario, int16_t derivedVario) {
    // Betaflight 4.4.3 with VARIO enabled exposes a filtered FC-side vertical
    // speed estimate through MSP_ALTITUDE. Prefer it directly; true zero is a
    // valid hover reading. Keep the ESP32-derived estimate only as a sanity
    // fallback for impossible values or for builds without BF vario support.
#if USE_BF_VARIO_PRIMARY
    if (abs(fcVario) <= VARIO_MAX_PLAUSIBLE_CMS) {
        lastVarioSource = 1;
        return fcVario;
    }
#endif

    lastVarioSource = 0;
    return derivedVario;
}

static void pollFcDiagnostics(uint32_t nowMs) {
    static uint8_t slot = 0;
    static uint32_t lastPollMs = 0;
    if (nowMs - lastPollMs < 20) return;
    lastPollMs = nowMs;

    uint8_t payload[32] = {0};
    uint8_t len = 0;
    uint8_t cmd = MSP_RAW_IMU;
    uint16_t bit = 0;

    switch (slot++ % 5) {
        case 0: cmd = MSP_RAW_IMU;  bit = 1 << 0; break;
        case 1: cmd = MSP_ATTITUDE; bit = 1 << 1; break;
        case 2: cmd = MSP_STATUS;   bit = 1 << 2; break;
        case 3: cmd = MSP_ANALOG;   bit = 1 << 3; break;
        default: cmd = MSP_RC;      bit = 1 << 4; break;
    }

    if (!requestMSP(cmd, payload, sizeof(payload), len, 15)) return;

    switch (cmd) {
        case MSP_RAW_IMU:
            if (len >= 18) {
                lastFcAccX = readS16(payload + 0);
                lastFcAccY = readS16(payload + 2);
                lastFcAccZ = readS16(payload + 4);
                lastFcGyroX = readS16(payload + 6);
                lastFcGyroY = readS16(payload + 8);
                lastFcGyroZ = readS16(payload + 10);
                lastFcMagX = readS16(payload + 12);
                lastFcMagY = readS16(payload + 14);
                lastFcMagZ = readS16(payload + 16);
                lastFcDiagMask |= bit;
                lastFcDiagMs = nowMs;
            }
            break;
        case MSP_ATTITUDE:
            if (len >= 6) {
                lastFcRollDeciDeg = readS16(payload + 0);
                lastFcPitchDeciDeg = readS16(payload + 2);
                lastFcYawDeg = readS16(payload + 4);
                lastFcDiagMask |= bit;
                lastFcDiagMs = nowMs;
            }
            break;
        case MSP_STATUS:
            if (len >= 6) {
                lastFcCycleTimeUs = readU16(payload + 0);
                lastFcI2cErrors = readU16(payload + 2);
                lastFcSensorsMask = readU16(payload + 4);
                lastFcDiagMask |= bit;
                lastFcDiagMs = nowMs;
            }
            break;
        case MSP_ANALOG:
            if (len >= 7) {
                lastFcVbatDeciV = payload[0];
                lastFcAmperageCentiA = readS16(payload + 5);
                lastFcDiagMask |= bit;
                lastFcDiagMs = nowMs;
            }
            break;
        case MSP_RC:
            if (len >= 12) {
                lastFcRcThrottle = readU16(payload + CH_THROTTLE * 2);
                lastFcRcArm = readU16(payload + CH_ARM * 2);
                lastFcRcAngle = readU16(payload + CH_ANGLE * 2);
                lastFcDiagMask |= bit;
                lastFcDiagMs = nowMs;
            }
            break;
    }

#if SERIAL_FLIGHT_DEBUG
    static uint32_t lastDiagLogMs = 0;
    if (nowMs - lastDiagLogMs >= 500) {
        lastDiagLogMs = nowMs;
        Serial.printf("[FC_DIAG] mask=0x%02X acc=%d,%d,%d gyro=%d,%d,%d att=%.1f,%.1f,%d cycle=%u sensors=0x%04X rcT=%u rcA=%u rcAng=%u vbat=%.1fV curr=%.2fA\n",
            lastFcDiagMask,
            lastFcAccX, lastFcAccY, lastFcAccZ,
            lastFcGyroX, lastFcGyroY, lastFcGyroZ,
            lastFcRollDeciDeg / 10.0f, lastFcPitchDeciDeg / 10.0f, lastFcYawDeg,
            lastFcCycleTimeUs, lastFcSensorsMask,
            lastFcRcThrottle, lastFcRcArm, lastFcRcAngle,
            lastFcVbatDeciV / 10.0f, lastFcAmperageCentiA / 100.0f);
    }
#endif
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
            uint32_t nowMs = millis();
#if SERIAL_FLIGHT_DEBUG
            static uint32_t mspRawLogMs = 0;
            if (nowMs - mspRawLogMs >= 500) {
                mspRawLogMs = nowMs;
                Serial.printf("[MSP_ALTITUDE] len=%u raw=", len);
                for (uint8_t i = 0; i < len; i++) {
                    Serial.printf("%02X ", payload[i]);
                }
                Serial.printf("alt=%ld fcVario=%d\n", (long)alt_cm, vario);
            }
#endif

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

            int16_t usedVario = selectVarioForControl(vario, heldDerivedVario);

            lastAlt     = fusedAltM;
            lastVario   = usedVario;
            lastFcVario = vario;
            lastDerivedVario = heldDerivedVario;
            lastVarioMs = nowMs;
#if SERIAL_FLIGHT_DEBUG
            static uint32_t varioLogMs = 0;
            if (nowMs - varioLogMs >= 500) {
                varioLogMs = nowMs;
                Serial.printf("[ALT] fused=%.2f baro=%.2f cbaro=%.2f src=%u tof=%s%.2f w=%u%% bf_vario=%d derived=%d used=%d vsrc=%u cm/s\n",
                    lastAlt, baroAltM, lastCorrectedBaroAltM, lastAltitudeSource,
                    lastTofValid ? "" : "invalid:",
                    lastTofAltM, lastTofWeightPct, vario, heldDerivedVario, usedVario, lastVarioSource);
            }
#endif
            pollFcDiagnostics(nowMs);
            return lastAlt;
        }
#if SERIAL_FLIGHT_DEBUG
        static uint32_t mspTimeoutLogMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - mspTimeoutLogMs >= 500) {
            mspTimeoutLogMs = nowMs;
            Serial.printf("[MSP_ALTITUDE] no valid response rxBytes=%u last=0x%02X lastAlt=%.2f\n",
                rxBytes, lastByte, lastAlt);
        }
#endif
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
            lastVarioSource = 0;
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
