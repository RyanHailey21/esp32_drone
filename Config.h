#pragma once

// ── Pins ─────────────────────────────────────────────────────
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define MOTOR_PWM_PIN   6
#define STATUS_LED      8

// ToF altitude sensor (VL53L1X, I2C address is 0x29 in 7-bit Arduino form;
// some datasheets list the same device as 0x52 in 8-bit form).
#define TOF_ENABLED       1
#define TOF_SDA_PIN       10
#define TOF_SCL_PIN       11
#define TOF_SHUT_PIN      -1   // optional VL53L1X SHUT/XSHUT pin; -1 = not connected
#define TOF_I2C_HZ        400000
#define TOF_TIMEOUT_MS    50
#define TOF_PERIOD_MS     50
#define TOF_TIMING_BUDGET_US 50000
#define TOF_VALID_MIN_M   0.04f
#define TOF_VALID_MAX_M   3.80f
#define TOF_BLEND_FULL_M  3.60f
#define TOF_BLEND_ZERO_M  3.80f
#define TOF_STALE_MS      150
#define TOF_HOLDOVER_MS   300    // bridge brief low-altitude ToF dropouts instead of swapping to baro
#define TOF_MAX_STEP_MIN_M 0.18f // reject sudden single-sample range jumps near the ground
#define TOF_MAX_STEP_MPS   3.0f
#define TOF_OFFSET_ALPHA  0.12f  // align corrected baro quickly while full-weight ToF is available

// ── Brushed Motor PWM ────────────────────────────────────────
#define PWM_FREQ        25000
#define PWM_RESOLUTION  8
#define MOTOR_DUTY      255

// ── MSP Commands ─────────────────────────────────────────────
#define MSP_STATUS      101
#define MSP_RAW_IMU     102
#define MSP_RC          105
#define MSP_ATTITUDE    108
#define MSP_ALTITUDE    109
#define MSP_ANALOG      110
#define MSP_SET_RAW_RC  200

// ── RC Channel Indices ────────────────────────────────────────
#define CH_THROTTLE  2
#define CH_ARM       4
#define CH_ANGLE     5

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
#define ALT_HOLD_TARGET_UUID "ab0828c0-198e-4351-b779-901fa0e0371e"
#define ANGLE_MODE_UUID     "ab0828c1-198e-4351-b779-901fa0e0371e"
#define FLIGHT_LOG_OFFSET_UUID "ab0828c2-198e-4351-b779-901fa0e0371e"
#define FLIGHT_LOG_CHUNK_UUID  "ab0828c3-198e-4351-b779-901fa0e0371e"

// BLE flight-log dump
#define FLIGHT_LOG_BYTES       32768
#define FLIGHT_LOG_CHUNK_BYTES 220

// Serial output is not useful during flight and can add timing jitter. Keep
// flight data in the BLE telemetry packet and downloadable in-memory CSV.
#define SERIAL_FLIGHT_DEBUG    0

// ── BLE Command IDs ──────────────────────────────────────────
#define CMD_HOVER_TEST      1
#define CMD_START_MISSION   2
#define CMD_DISARM          3
#define CMD_AUTO_HOVER_CAL  4
#define CMD_ALT_HOLD        5
#define CMD_KILL            6

// ── Auto Hover Calibration Constants ─────────────────────────
#define CAL_START_THROTTLE  1150
#define CAL_MAX_THROTTLE    1650
#define CAL_STEP_US         5
#define CAL_STEP_MS         250
#define CAL_LIFTOFF_M       0.15f
#define CAL_GE_OFFSET_US    0    // no automatic hover bump; tune free-air offset explicitly from logs
#define CAL_TIMEOUT_MS      30000

// ── Landing Constants ────────────────────────────────────────
#define LANDING_GROUND_M    0.15f  // altitude threshold → cut motors
#define LANDING_TIMEOUT_MS  30000  // safety: force disarm if landing takes too long
#define DESCENT_RATE_MPS    0.4f   // target descent speed m/s
#define LANDING_THROTTLE_OFFSET_US 180  // nominal throttle below hover while landing

// ── Bench-Mode Simulation Constants ──────────────────────────
#define BENCH_SPRINT_RATE_MPS     9.0f
#define BENCH_PUNCH_RATE_MPS      7.0f
#define BENCH_HOVER_LIFTOFF_US    1325
#define BENCH_HOVER_CAL_RATE_MPS  0.8f

// ── Arming Settle Time ────────────────────────────────────────
#define ARMING_MS  1500

// ── Hover Test Throttle Ramp ──────────────────────────────────
// µs added per loop iteration (~20ms) when ramping from arm-throttle to HOVER_THROTTLE.
// 4 µs/iter × 50 Hz ≈ 200 µs/s → 270 µs step takes ~1.3 s, too slow to trigger ANTI_GRAVITY.
#define HOVER_RAMP_STEP_US  4
#define DEFAULT_ANGLE_MODE  1  // 0=acro/rate mode, 1=Angle mode

// ── Cascaded Altitude Hold ────────────────────────────────────
// HOLD_KP = outer P gain: altitude error (m) → desired vertical speed (m/s)
// HOLD_KD = inner P gain: speed error (m/s) → throttle offset (µs)
// HOLD_KI = inner I gain: speed integral    → throttle offset (µs)

// Outer loop: setpoint ramp and speed limits
#define ALT_RAMP_RATE_MPS       1.0f    // m/s internal setpoint ramp rate
#define MAX_CLIMB_MPS_TEST      0.60f   // max commanded climb, test mode
#define MAX_DESCENT_MPS_TEST    0.45f   // max commanded descent, test mode
#define MAX_CLIMB_MPS_HOLD      1.20f   // max commanded climb, competition HOLDING
#define MAX_DESCENT_MPS_HOLD    0.80f   // max commanded descent, competition HOLDING
#define ALT_HOLD_TARGET_MIN_M   0.5f    // BLE Alt Hold test target lower bound
#define ALT_HOLD_TARGET_MAX_M   5.0f    // BLE Alt Hold test target upper bound
#define NEAR_TARGET_M           0.5f    // within this radius, scale down max speed
#define NEAR_TARGET_FACTOR      0.35f   // minimum speed factor within near-target zone

// Inner loop: vario filtering
#define VARIO_TAU_S             0.05f   // seconds, light low-pass; avoid lag on low-alt ToF vario
#define VARIO_STALE_MS          500     // ms before vario reading is considered stale
#define VARIO_MAX_PLAUSIBLE_CMS 800     // cm/s — implausible above this (~8 m/s)
#define USE_BF_VARIO_PRIMARY    1       // use BF vario above ToF range; ToF-derived vario wins near ground
#define VSPEED_I_MAX_US         150.0f  // max integral throttle contribution in us

// Throttle authority around hover
#define THR_UP_OFFSET_US        300     // max µs above HOVER_THROTTLE
#define THR_DOWN_OFFSET_US      300     // max µs below HOVER_THROTTLE (more for braking)
#define MIN_ALT_HOLD_THROTTLE_US 1000   // low-altitude test mode can fully unload for braking
#define MIN_MISSION_THROTTLE_US  1050   // mission hold preserves attitude-control authority

// Safety
#define ALT_MAX_M               22.0f   // absolute ceiling — triggers landing above this
#define LANDING_KP_VSPEED       120.0f  // fixed P gain for landing velocity controller

// Takeoff ground guard — avoids integrator windup and baro-spike at liftoff
#define TAKEOFF_ALT_M            0.12f  // ToF-confirmed liftoff threshold before closed-loop engages
#define TAKEOFF_NUDGE_US         15     // us above HOVER_THROTTLE for ground-phase thrust
#define TAKEOFF_RAMP_US_PER_S    30     // additional takeoff throttle ramp while waiting for liftoff
#define TAKEOFF_MAX_OFFSET_US    140    // max takeoff throttle above HOVER_THROTTLE
#define TAKEOFF_INVALID_TOF_MAX_OFFSET_US 60   // cap takeoff thrust until ToF confirms altitude
#define TAKEOFF_CONFIRM_SAMPLES  3      // consecutive ToF-valid samples before cascade latch
#define BARO_SETTLE_MS           500    // ms after throttle step for baro to settle at hover pressure
#define GROUND_GUARD_TIMEOUT_MS  8000   // ms before aborting if ToF/fused altitude hasn't confirmed liftoff
