#pragma once

#include <Arduino.h>

void flightLogReset();
void flightLogAppend(const char* line);
void flightLogAppendf(const char* fmt, ...);
uint16_t flightLogSize();
void flightLogSetReadOffset(uint16_t offset);
uint16_t flightLogReadChunk(uint8_t* out, uint16_t maxLen);
