#include "FlightLog.h"
#include "Config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char flightLogBuf[FLIGHT_LOG_BYTES];
static uint16_t flightLogLen = 0;
static uint16_t flightLogReadOffset = 0;
static bool flightLogTruncated = false;

void flightLogReset() {
    flightLogLen = 0;
    flightLogReadOffset = 0;
    flightLogTruncated = false;
    flightLogBuf[0] = '\0';
}

void flightLogAppend(const char* line) {
    if (!line || flightLogTruncated) return;

    size_t lineLen = strlen(line);
    if (lineLen == 0) return;

    if (flightLogLen + lineLen >= FLIGHT_LOG_BYTES) {
        static const char truncMsg[] = "[LOG] truncated\n";
        size_t room = FLIGHT_LOG_BYTES - flightLogLen - 1;
        size_t copyLen = min(room, sizeof(truncMsg) - 1);
        if (copyLen > 0) {
            memcpy(flightLogBuf + flightLogLen, truncMsg, copyLen);
            flightLogLen += copyLen;
            flightLogBuf[flightLogLen] = '\0';
        }
        flightLogTruncated = true;
        return;
    }

    memcpy(flightLogBuf + flightLogLen, line, lineLen);
    flightLogLen += (uint16_t)lineLen;
    flightLogBuf[flightLogLen] = '\0';
}

void flightLogAppendf(const char* fmt, ...) {
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n <= 0) return;
    line[sizeof(line) - 1] = '\0';
    flightLogAppend(line);
}

uint16_t flightLogSize() {
    return flightLogLen;
}

void flightLogSetReadOffset(uint16_t offset) {
    flightLogReadOffset = min(offset, flightLogLen);
}

uint16_t flightLogReadChunk(uint8_t* out, uint16_t maxLen) {
    if (!out || maxLen == 0) return 0;
    if (flightLogReadOffset >= flightLogLen) return 0;

    uint16_t n = min(maxLen, (uint16_t)(flightLogLen - flightLogReadOffset));
    memcpy(out, flightLogBuf + flightLogReadOffset, n);
    return n;
}
