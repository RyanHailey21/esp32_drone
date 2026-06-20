#pragma once

// ── Pins ─────────────────────────────────────────────────────
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define MOTOR_PWM_PIN   6
#define STATUS_LED      8

// ── Brushed Motor PWM ────────────────────────────────────────
#define PWM_FREQ        25000
#define PWM_RESOLUTION  8
#define MOTOR_DUTY      255

// ── MSP Commands ─────────────────────────────────────────────
#define MSP_SET_RAW_RC  200
#define MSP_ALTITUDE    109

// ── BLE UUIDs ────────────────────────────────────────────────
#define SERVICE_UUID        "ab0828b1-198e-4351-b779-901fa0e0371e"
#define HOVER_UUID          "ab0828b2-198e-4351-b779-901fa0e0371e"
#define CEILING_UUID        "ab0828b3-198e-4351-b779-901fa0e0371e"  // unused now, kept for compat
#define PRESPIN_UUID        "ab0828b4-198e-4351-b779-901fa0e0371e"  // unused now, kept for compat
#define SPRINT_THROT_UUID   "ab0828b5-198e-4351-b779-901fa0e0371e"
#define SPRINT_CUTOFF_UUID  "ab0828b6-198e-4351-b779-901fa0e0371e"
#define HOLD_KP_UUID        "ab0828b7-198e-4351-b779-901fa0e0371e"
#define HOLD_KI_UUID        "ab0828bc-198e-4351-b779-901fa0e0371e"
#define HOLD_KD_UUID        "ab0828be-198e-4351-b779-901fa0e0371e"
#define PUNCH_START_UUID    "ab0828b8-198e-4351-b779-901fa0e0371e"
#define PUNCH_THROT_UUID    "ab0828b9-198e-4351-b779-901fa0e0371e"
#define COMMAND_UUID        "ab0828ba-198e-4351-b779-901fa0e0371e"
#define BENCH_MODE_UUID     "ab0828bb-198e-4351-b779-901fa0e0371e"
#define TELEMETRY_UUID      "ab0828bd-198e-4351-b779-901fa0e0371e"
#define TARGET_ALT_UUID     "ab0828bf-198e-4351-b779-901fa0e0371e"

// ── BLE Command IDs ──────────────────────────────────────────
#define CMD_HOVER_TEST      1
#define CMD_START_MISSION   2
#define CMD_DISARM          3
#define CMD_AUTO_HOVER_CAL  4
#define CMD_ALT_HOLD        5

// ── Auto Hover Calibration Constants ─────────────────────────
#define CAL_START_THROTTLE  1150
#define CAL_MAX_THROTTLE    1650
#define CAL_STEP_US         5
#define CAL_STEP_MS         250
#define CAL_LIFTOFF_M       0.15f
#define CAL_GE_OFFSET_US    50   // ground effect: free-air hover needs more throttle than near-ground liftoff
#define CAL_TIMEOUT_MS      30000

// ── Landing Constants ────────────────────────────────────────
#define LANDING_GROUND_M    0.15f  // altitude threshold → cut motors
#define LANDING_TIMEOUT_MS  30000  // safety: force disarm if landing takes too long
#define DESCENT_RATE_MPS    0.4f   // target descent speed m/s

// ── Bench-Mode Simulation Constants ──────────────────────────
#define BENCH_SPRINT_RATE_MPS     9.0f
#define BENCH_PUNCH_RATE_MPS      7.0f
#define BENCH_HOVER_LIFTOFF_US    1325
#define BENCH_HOVER_CAL_RATE_MPS  0.8f

// ── Arming Settle Time ────────────────────────────────────────
#define ARMING_MS  1500

// ── Cascaded Altitude Hold ────────────────────────────────────
// HOLD_KP = outer P gain: altitude error (m) → desired vertical speed (m/s)
// HOLD_KD = inner P gain: speed error (m/s) → throttle offset (µs)
// HOLD_KI = inner I gain: speed integral    → throttle offset (µs)

// Outer loop: setpoint ramp and speed limits
#define ALT_RAMP_RATE_MPS       1.0f    // m/s — internal setpoint ramp rate
#define MAX_CLIMB_MPS_TEST      1.5f    // max commanded climb, test mode
#define MAX_DESCENT_MPS_TEST    1.0f    // max commanded descent, test mode
#define MAX_CLIMB_MPS_HOLD      3.5f    // max commanded climb, competition HOLDING
#define MAX_DESCENT_MPS_HOLD    2.0f    // max commanded descent, competition HOLDING
#define NEAR_TARGET_M           0.5f    // within this radius, scale down max speed
#define NEAR_TARGET_FACTOR      0.35f   // minimum speed factor within near-target zone

// Inner loop: vario filtering
#define VARIO_ALPHA             0.25f   // low-pass weight (0=max smoothing, 1=raw)
#define VARIO_STALE_MS          500     // ms before vario reading is considered stale
#define VARIO_MAX_PLAUSIBLE_CMS 800     // cm/s — implausible above this (~8 m/s)
#define KI_VSPEED_LIMIT         40.0f   // anti-windup clamp on vspeedIntegral (µs)

// Throttle authority around hover
#define THR_UP_OFFSET_US        200     // max µs above HOVER_THROTTLE
#define THR_DOWN_OFFSET_US      250     // max µs below HOVER_THROTTLE (more for braking)

// Safety
#define ALT_MAX_M               22.0f   // absolute ceiling — triggers landing above this
#define LANDING_KP_VSPEED       30.0f   // fixed P gain for landing velocity controller
