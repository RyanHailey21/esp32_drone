#pragma once

#include <Arduino.h>
#include "Config.h"

class NimBLECharacteristic;  // forward declaration — includers that call methods must include NimBLEDevice.h

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

// ── Arm Target ───────────────────────────────────────────────
enum ArmTarget { ARM_MISSION, ARM_HOVER_TEST, ARM_AUTO_HOVER_CAL, ARM_ALT_HOLD };

// ── Runtime State ────────────────────────────────────────────
extern MissionState state;
extern ArmTarget    armTarget;
extern uint32_t     launchTime;
extern uint32_t     armTime;
extern uint32_t     calTime;
extern uint32_t     calStepTime;
extern uint16_t     calThrottle;
extern uint8_t      calLiftoffCount;
extern float        launchAlt;
extern float        currentRelAlt;
extern int16_t      lastVario;
extern uint32_t     lastVarioMs;
extern float        internalSetpoint;
extern float        filteredVario;
extern float        vspeedIntegral;
extern uint32_t     vspeedLastMs;
extern float        benchAlt;
extern uint32_t     benchLastMs;
extern uint32_t     landingStartMs;
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
