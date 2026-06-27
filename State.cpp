#include "State.h"
#include "altitude_gains.h"

// ── State Machine ────────────────────────────────────────────
MissionState state     = IDLE;
ArmTarget    armTarget = ARM_MISSION;

// ── Runtime State ────────────────────────────────────────────
uint32_t launchTime      = 0;
uint32_t armTime         = 0;
uint32_t calTime         = 0;
uint32_t calStepTime     = 0;
uint16_t calThrottle     = CAL_START_THROTTLE;
uint8_t  calLiftoffCount = 0;
float    launchAlt       = 0;
float    currentRelAlt   = 0;
int16_t  lastVario       = 0;
uint32_t lastVarioMs     = 0;
float    internalSetpoint = 0;
float    filteredVario   = 0;
float    vspeedIntegral  = 0;
uint32_t vspeedLastMs    = 0;
float    benchAlt        = 0;
uint32_t benchLastMs     = 0;
uint32_t landingStartMs  = 0;
volatile bool bleSafetyLand = false;

// ── Tunable Parameters (BLE-writable) ────────────────────────
volatile uint16_t HOVER_THROTTLE  = (uint16_t)(AltitudeParameters::HOVER_THROTTLE + 0.5f);
volatile uint16_t SPRINT_THROTTLE = 1850;
volatile float    SPRINT_CUTOFF_M = 17.0f;
volatile float    TARGET_ALT_M    = 18.3f;
volatile float    ALT_HOLD_TARGET_M = 1.5f;
volatile float    HOLD_KP         = AltitudeParameters::HOLD_KP;
volatile float    HOLD_KI         = AltitudeParameters::HOLD_KI;
volatile float    HOLD_KD         = AltitudeParameters::HOLD_KD;
volatile uint32_t PUNCH_START_MS  = 7500;
volatile uint16_t PUNCH_THROTTLE  = 2000;

// Runtime BLE-controlled bench mode
volatile uint8_t  BENCH_MODE_ENABLED = 0;

// ── RC Channels ──────────────────────────────────────────────
// CH1=Roll CH2=Pitch CH3=Throttle CH4=Yaw CH5=AUX1(Arm) CH6=AUX2(Angle)
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};

// ── Flight Controller Serial ─────────────────────────────────
HardwareSerial fcSerial(1);

// ── BLE Characteristic Pointers ──────────────────────────────
NimBLECharacteristic* hoverChar     = nullptr;
NimBLECharacteristic* telemetryChar = nullptr;
