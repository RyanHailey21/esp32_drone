#include "Msp.h"

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
        uint32_t timeout = millis() + 25;
        while (millis() < timeout) {
            uint8_t b = 0;
            if (!readByteUntil(b, timeout)) break;
            if (b != '$') continue;
            if (!readByteUntil(b, timeout) || b != 'M') continue;
            if (!readByteUntil(b, timeout) || b != '>') continue;

            uint8_t len = 0, cmd = 0;
            if (!readByteUntil(len, timeout)) break;
            if (!readByteUntil(cmd, timeout)) break;

            uint8_t checksum = len ^ cmd;
            uint8_t payload[16];
            bool keepPayload = (cmd == MSP_ALTITUDE && len == 6);
            for (uint8_t i = 0; i < len; i++) {
                if (!readByteUntil(b, timeout)) return lastAlt;
                checksum ^= b;
                if (keepPayload && i < sizeof(payload)) payload[i] = b;
            }
            if (!readByteUntil(b, timeout)) break;
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

            // If Betaflight sends 0 vario, derive speed from a longer altitude
            // window and smooth it. MSP altitude is quantized, so short-window
            // derivatives flicker between 0 and large pulses.
            static int32_t derivBaseAltCm = 0;
            static uint32_t derivBaseMs = 0;
            static int16_t heldDerivedVario = 0;
            static float filteredDerivedCms = 0.0f;
            if (derivBaseMs == 0) {
                derivBaseAltCm = alt_cm;
                derivBaseMs = nowMs;
            } else if (nowMs - derivBaseMs >= 400) {
                float dtSec = (nowMs - derivBaseMs) / 1000.0f;
                float rawDerivedCms = (alt_cm - derivBaseAltCm) / dtSec;
                constexpr float DERIVED_ALPHA = 0.25f;
                filteredDerivedCms += DERIVED_ALPHA * (rawDerivedCms - filteredDerivedCms);
                heldDerivedVario = constrainVario(filteredDerivedCms);
                derivBaseAltCm = alt_cm;
                derivBaseMs = nowMs;
            }

            // Betaflight 4.4.x MSP_ALTITUDE often reports a zero vario field
            // unless firmware is built with USE_VARIO. Keep FC vario for
            // diagnostics, but use the derived/smoothed estimate for control.
            int16_t usedVario = heldDerivedVario;

            lastAlt     = alt_cm / 100.0f;
            lastVario   = usedVario;
            lastFcVario = vario;
            lastDerivedVario = heldDerivedVario;
            lastVarioMs = nowMs;
            static uint32_t varioLogMs = 0;
            if (nowMs - varioLogMs >= 500) {
                varioLogMs = nowMs;
                Serial.printf("[MSP] alt=%.2f fc_vario=%d derived=%d used=%d cm/s\n",
                    lastAlt, vario, heldDerivedVario, usedVario);
            }
            return lastAlt;
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

    return benchAlt;
}
