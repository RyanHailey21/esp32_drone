#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"

// ── State Machine ────────────────────────────────────────────
enum MissionState {
    IDLE,
    ARMING,
    SPRINTING,     // full throttle to SPRINT_CUTOFF_M
    HOLDING,       // PID holds TARGET_ALT_M (competition)
    PUNCHING,      // max throttle final burst
    CUT,
    HOVER_TEST,
    AUTO_HOVER_CAL,
    LANDING,
    DONE,
    ALT_HOLD       // PID holds TARGET_ALT_M (test mode, BLE-safe)
};

// ── Runtime State ────────────────────────────────────────────
extern MissionState state;
extern uint32_t     launchTime;
extern uint32_t     armTime;
extern bool         armingForHover;
extern bool         armingForAutoCal;
extern bool         armingForAltHold;
extern uint32_t     calTime;
extern uint32_t     calStepTime;
extern uint16_t     calThrottle;
extern uint8_t      calLiftoffCount;
extern float        launchAlt;
extern float        currentRelAlt;
extern int16_t      lastVario;
extern float        holdIntegral;
extern uint32_t     holdLastMs;
extern bool         prespunUp;
extern float        benchAlt;
extern uint32_t     benchLastMs;
extern uint32_t     landingStartMs;
extern float        landingStartAlt;
extern volatile bool bleSafetyLand;

// ── Tunable Parameters (BLE-writable) ────────────────────────
extern volatile uint16_t HOVER_THROTTLE;
extern volatile uint16_t SPRINT_THROTTLE;
extern volatile float    SPRINT_CUTOFF_M;
extern volatile float    TARGET_ALT_M;
extern volatile float    HOLD_KP;
extern volatile float    HOLD_KI;
extern volatile float    HOLD_KD;
extern volatile uint32_t PUNCH_START_MS;
extern volatile uint16_t PUNCH_THROTTLE;

// Runtime BLE-controlled bench mode
extern volatile uint8_t  BENCH_MODE_ENABLED;

// ── RC Channels ──────────────────────────────────────────────
extern uint16_t channels[8];

// ── Flight Controller Serial ─────────────────────────────────
extern HardwareSerial fcSerial;

// ── BLE Characteristic Pointers ──────────────────────────────
extern NimBLECharacteristic* hoverChar;
extern NimBLECharacteristic* telemetryChar;
