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
    ALT_HOLD       // PID holds ALT_HOLD_TARGET_M (test mode, BLE-safe)
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
extern int16_t      lastFcVario;
extern int16_t      lastDerivedVario;
extern uint8_t      lastVarioSource;
extern uint8_t      lastMspAltitudePayload[6];
extern int16_t      lastFcAccX;
extern int16_t      lastFcAccY;
extern int16_t      lastFcAccZ;
extern int16_t      lastFcGyroX;
extern int16_t      lastFcGyroY;
extern int16_t      lastFcGyroZ;
extern int16_t      lastFcMagX;
extern int16_t      lastFcMagY;
extern int16_t      lastFcMagZ;
extern int16_t      lastFcRollDeciDeg;
extern int16_t      lastFcPitchDeciDeg;
extern int16_t      lastFcYawDeg;
extern uint32_t     lastFcAttitudeMs;
extern uint16_t     lastFcCycleTimeUs;
extern uint16_t     lastFcI2cErrors;
extern uint16_t     lastFcSensorsMask;
extern uint16_t     lastFcRcThrottle;
extern uint16_t     lastFcRcArm;
extern uint16_t     lastFcRcAngle;
extern uint8_t      lastFcVbatDeciV;
extern int16_t      lastFcAmperageCentiA;
extern uint16_t     lastFcDiagMask;
extern uint32_t     lastFcDiagMs;
extern bool         lastTofValid;
extern float        lastTofAltM;
extern uint8_t      lastTofWeightPct;
extern bool         lastTofReadOk;
extern float        lastTofRawM;
extern uint8_t      lastTofRejectReason;
extern uint8_t      lastTofRangeStatus;
extern uint8_t      lastTofI2cStatus;
extern uint16_t     lastTofReadDtMs;
extern float        lastBaroAltM;
extern float        lastCorrectedBaroAltM;
extern float        lastFusedAltM;
extern uint8_t      lastAltitudeSource;
extern uint32_t     lastVarioMs;
extern float        internalSetpoint;
extern float        filteredVario;
extern float        vspeedIntegral;
extern uint32_t     vspeedLastMs;
extern float        lastAltError;
extern float        lastDesiredVspeed;
extern float        lastVspeedError;
extern float        lastControlPUs;
extern float        lastControlIUs;
extern float        lastRawThrottle;
extern float        lastThrMin;
extern float        lastThrMax;
extern uint16_t     lastClampedThrottle;
extern int8_t       lastThrottleSat;
extern float        benchAlt;
extern uint32_t     benchLastMs;
extern uint32_t     landingStartMs;
extern float        landingStartAlt;
extern volatile bool bleSafetyLand;
extern volatile bool bleRequestedLand;

// ── Tunable Parameters (BLE-writable) ────────────────────────
extern volatile uint16_t HOVER_THROTTLE;
extern volatile uint16_t SPRINT_THROTTLE;
extern volatile uint16_t SPRINT_YAW;
extern volatile float    SPRINT_CUTOFF_M;
extern volatile float    TARGET_ALT_M;
extern volatile float    ALT_HOLD_TARGET_M;
extern volatile float    HOLD_KP;
extern volatile float    HOLD_KI;
extern volatile float    HOLD_KD;
extern volatile float    ALT_RAMP_RATE_MPS;
extern volatile float    MAX_CLIMB_MPS_TEST;
extern volatile float    MAX_DESCENT_MPS_TEST;
extern volatile float    BF_VARIO_GROUND_EFFECT_M;
extern volatile uint32_t PUNCH_START_MS;
extern volatile uint16_t PUNCH_THROTTLE;

// Runtime BLE-controlled bench mode
extern volatile uint8_t  BENCH_MODE_ENABLED;
extern volatile uint8_t  ANGLE_MODE_ENABLED;
extern volatile uint8_t  MISSION_TYPE;

// ── RC Channels ──────────────────────────────────────────────
extern uint16_t channels[8];

// ── Flight Controller Serial ─────────────────────────────────
extern HardwareSerial fcSerial;

// ── BLE Characteristic Pointers ──────────────────────────────
extern NimBLECharacteristic* hoverChar;
extern NimBLECharacteristic* telemetryChar;
