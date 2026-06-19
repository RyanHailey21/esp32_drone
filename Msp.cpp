#include "Msp.h"

void sendMSP(uint8_t cmd, uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    fcSerial.write('$');
    fcSerial.write('M');
    fcSerial.write('<');
    fcSerial.write(len); cs ^= len;
    fcSerial.write(cmd); cs ^= cmd;
    for (int i = 0; i < len; i++) { fcSerial.write(data[i]); cs ^= data[i]; }
    fcSerial.write(cs);
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
        uint8_t empty = 0;
        sendMSP(MSP_ALTITUDE, &empty, 0);
        uint32_t timeout = millis() + 50;
        while (fcSerial.available() < 12 && millis() < timeout);
        if (fcSerial.available() < 12) return lastAlt;
        while (fcSerial.available() >= 12) {
            if (fcSerial.read() != '$') continue;
            if (fcSerial.read() != 'M') continue;
            if (fcSerial.read() != '>') continue;
            fcSerial.read(); fcSerial.read();  // len, cmd
            int32_t alt_cm = 0;
            alt_cm  = (uint32_t)fcSerial.read();
            alt_cm |= (uint32_t)fcSerial.read() << 8;
            alt_cm |= (uint32_t)fcSerial.read() << 16;
            alt_cm |= (uint32_t)fcSerial.read() << 24;
            int16_t vario = 0;
            vario  = (uint16_t)fcSerial.read();
            vario |= (uint16_t)fcSerial.read() << 8;
            fcSerial.read();  // checksum
            lastAlt   = alt_cm / 100.0f;
            lastVario = vario;
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
            benchAlt = launchAlt + TARGET_ALT_M;
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
