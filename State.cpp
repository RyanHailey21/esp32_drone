#include "State.h"
#include "altitude_gains.h"

// ── State Machine ────────────────────────────────────────────
MissionState state     = IDLE;
ArmTarget    armTarget = ARM_MISSION;

// ── Runtime State ────────────────────────────────────────────
uint32_t launchTime      = 0;
uint32_t armTime         = 0;
float    launchAlt       = 0;
float    currentRelAlt   = 0;
int16_t  lastVario       = 0;
int16_t  lastFcVario     = 0;
int16_t  lastDerivedVario = 0;
uint8_t  lastVarioSource = 0;  // 0=derived fallback, 1=Betaflight MSP vario, 2=KF velocity
uint8_t  lastMspAltitudePayload[6] = {0};
int16_t  lastFcAccX      = 0;
int16_t  lastFcAccY      = 0;
int16_t  lastFcAccZ      = 0;
int16_t  lastFcGyroX     = 0;
int16_t  lastFcGyroY     = 0;
int16_t  lastFcGyroZ     = 0;
int16_t  lastFcMagX      = 0;
int16_t  lastFcMagY      = 0;
int16_t  lastFcMagZ      = 0;
int16_t  lastFcRollDeciDeg = 0;
int16_t  lastFcPitchDeciDeg = 0;
int16_t  lastFcYawDeg    = 0;
uint32_t lastFcAttitudeMs = 0;
uint16_t lastFcCycleTimeUs = 0;
uint16_t lastFcI2cErrors = 0;
uint16_t lastFcSensorsMask = 0;
uint32_t lastFcFlightModeFlags = 0;
uint32_t lastFcArmingDisableFlags = 0;
uint32_t lastFcStatusMs = 0;
bool     lastFcArmed = false;
uint16_t lastFcRcThrottle = 0;
uint16_t lastFcRcArm     = 0;
uint16_t lastFcRcAngle   = 0;
uint8_t  lastFcVbatDeciV = 0;
int16_t  lastFcAmperageCentiA = 0;
uint16_t lastFcDiagMask  = 0;
uint32_t lastFcDiagMs    = 0;
bool     lastTofValid    = false;
float    lastTofAltM     = 0.0f;
uint8_t  lastTofWeightPct = 0;
bool     lastTofReadOk   = false;
float    lastTofRawM     = 0.0f;
uint8_t  lastTofRejectReason = 0;
uint8_t  lastTofRangeStatus = 255;
uint8_t  lastTofI2cStatus = 0;
uint16_t lastTofReadDtMs = 0;
float    lastBaroAltM    = 0.0f;
float    lastCorrectedBaroAltM = 0.0f;
float    lastFusedAltM   = 0.0f;
uint8_t  lastAltitudeSource = 0;  // 0=baro, 1=tof, 2=blend, 3=tof holdover
uint32_t lastVarioMs     = 0;
float    internalSetpoint = 0;
float    filteredVario   = 0;
float    vspeedIntegral  = 0;
uint32_t vspeedLastMs    = 0;
float    lastAltError    = 0.0f;
float    lastDesiredVspeed = 0.0f;
float    lastVspeedError = 0.0f;
float    lastControlPUs  = 0.0f;
float    lastControlIUs  = 0.0f;
float    lastRawThrottle = 0.0f;
float    lastThrMin      = 0.0f;
float    lastThrMax      = 0.0f;
uint16_t lastClampedThrottle = 1000;
int8_t   lastThrottleSat = 0;
float    benchAlt        = 0;
uint32_t benchLastMs     = 0;
uint32_t landingStartMs  = 0;
float    landingStartAlt = 0;
volatile bool bleSafetyLand = false;
volatile bool bleRequestedLand = false;
volatile uint8_t pendingBleCommand = 0;

// ── Tunable Parameters (BLE-writable) ────────────────────────
volatile uint16_t HOVER_THROTTLE  = (uint16_t)(AltitudeParameters::HOVER_THROTTLE + 0.5f);
volatile uint16_t SPRINT_THROTTLE = 2000;
volatile uint16_t SPRINT_YAW      = DEFAULT_SPRINT_YAW_US;
volatile uint16_t PUNCH_YAW       = DEFAULT_PUNCH_YAW_US;
volatile uint16_t HOVER_TEST_YAW  = DEFAULT_HOVER_TEST_YAW_US;
volatile float    SPRINT_CUTOFF_M = 15.8f;
volatile float    TARGET_ALT_M    = 18.3f;
volatile float    ALT_HOLD_TARGET_M = 1.5f;
volatile float    HOLD_KP         = AltitudeParameters::HOLD_KP;
volatile float    HOLD_KI         = AltitudeParameters::HOLD_KI;
volatile float    HOLD_KD         = AltitudeParameters::HOLD_KD;
volatile float    ALT_RAMP_RATE_MPS = DEFAULT_ALT_RAMP_RATE_MPS;
volatile float    MAX_CLIMB_MPS_TEST = DEFAULT_MAX_CLIMB_MPS_TEST;
volatile float    MAX_DESCENT_MPS_TEST = DEFAULT_MAX_DESCENT_MPS_TEST;
volatile float    BF_VARIO_GROUND_EFFECT_M = DEFAULT_BF_VARIO_GROUND_EFFECT_M;
volatile uint32_t PUNCH_START_MS  = 7000;
volatile uint16_t PUNCH_THROTTLE  = 2000;

// Runtime BLE-controlled bench mode
volatile uint8_t  BENCH_MODE_ENABLED = 0;
volatile uint8_t  ANGLE_MODE_ENABLED = DEFAULT_ANGLE_MODE;
volatile uint8_t  MISSION_TYPE = DEFAULT_MISSION_TYPE;

// ── RC Channels ──────────────────────────────────────────────
// CH1=Roll CH2=Pitch CH3=Throttle CH4=Yaw CH5=AUX1(Arm) CH6=AUX2(Angle)
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};

// ── Flight Controller Serial ─────────────────────────────────
HardwareSerial fcSerial(1);

// ── BLE Characteristic Pointers ──────────────────────────────
NimBLECharacteristic* hoverChar     = nullptr;
NimBLECharacteristic* telemetryChar = nullptr;
