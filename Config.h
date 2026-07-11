#pragma once

// ── Pins ─────────────────────────────────────────────────────
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define STATUS_LED      8

// ToF altitude sensor (VL53L1X, I2C address is 0x29 in 7-bit Arduino form;
// some datasheets list the same device as 0x52 in 8-bit form).
#define TOF_ENABLED       1
#define TOF_SDA_PIN       10
#define TOF_SCL_PIN       11
#define TOF_SHUT_PIN      -1   // optional VL53L1X SHUT/XSHUT pin; -1 = not connected
#define TOF_I2C_HZ        400000
#define TOF_TIMEOUT_MS    30
#define TOF_PERIOD_MS     25
#define TOF_TIMING_BUDGET_US 20000
#define TOF_VALID_MIN_M   0.04f
#define TOF_VALID_MAX_M   2.50f
#define TOF_BLEND_FULL_M  1.60f
#define TOF_BLEND_ZERO_M  2.20f
#define TOF_RECENT_VALID_MS 130   // allow guard/landing to use recent ToF when no new sample is ready
#define TOF_HELD_WEIGHT_PCT 80    // visible confidence for a recent held ToF sample
#define TOF_MAX_STEP_MIN_M 0.18f // reject sudden single-sample range jumps near the ground
#define TOF_MAX_STEP_MPS   3.0f
#define TOF_FUSION_MAX_STEP_MPS 6.0f // final gate before ToF can affect fusion or baro offset
#define TOF_OFFSET_ALPHA  0.12f  // align corrected baro quickly while full-weight ToF is available

// Linear altitude KF tuning. These are the only compile-time defaults for the
// filter; AltitudeKF.h consumes them rather than carrying independent values.
#define KF_Q_ACCEL_PSD              1.5f
#define KF_R_BARO                   0.6f
#define KF_R_TOF                    0.02f
#define KF_R_BF_VARIO               0.3f
#define KF_R_BF_VARIO_GROUND_MULT   2.0f
#define KF_R_DER_VARIO              0.6f

// ── MSP Commands ─────────────────────────────────────────────
#define MSP_STATUS      101
#define MSP_RAW_IMU     102
#define MSP_RC          105
#define MSP_ATTITUDE    108
#define MSP_ALTITUDE    109
#define MSP_ANALOG      110
#define MSP_SET_RAW_RC  200
#define MSP_ALTITUDE_PERIOD_MS 25
#define MSP_DIAG_PERIOD_MS     50
#define MSP_ALTITUDE_STALE_MS  250

// ── RC Channel Indices ────────────────────────────────────────
#define CH_THROTTLE  2
#define CH_YAW       3
#define CH_ARM       4
#define CH_ANGLE     5
#define RC_NEUTRAL_US 1500

// ── BLE UUIDs ────────────────────────────────────────────────
#define SERVICE_UUID        "ab0828b1-198e-4351-b779-901fa0e0371e"
#define HOVER_UUID          "ab0828b2-198e-4351-b779-901fa0e0371e"
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
#define ALT_RAMP_RATE_UUID     "ab0828c4-198e-4351-b779-901fa0e0371e"
#define MAX_CLIMB_TEST_UUID    "ab0828c5-198e-4351-b779-901fa0e0371e"
#define MAX_DESCENT_TEST_UUID  "ab0828c6-198e-4351-b779-901fa0e0371e"
#define BF_GROUND_EFFECT_UUID  "ab0828c7-198e-4351-b779-901fa0e0371e"
#define MISSION_TYPE_UUID      "ab0828c8-198e-4351-b779-901fa0e0371e"
#define SPRINT_YAW_UUID        "ab0828c9-198e-4351-b779-901fa0e0371e"
#define PUNCH_YAW_UUID         "ab0828ca-198e-4351-b779-901fa0e0371e"
#define HOVER_TEST_YAW_UUID     "ab0828cb-198e-4351-b779-901fa0e0371e"

// Mission hardware mode default. 0=powered landing, 1=autorotor pre-spin + motor cut.
#define DEFAULT_MISSION_TYPE         0
#define DEFAULT_SPRINT_YAW_US        1700  // right/CW aircraft yaw viewed from above
#define SPRINT_YAW_MIN_US            RC_NEUTRAL_US
#define SPRINT_YAW_MAX_US            1900
#define DEFAULT_PUNCH_YAW_US         2000
#define PUNCH_YAW_MIN_US             RC_NEUTRAL_US
#define PUNCH_YAW_MAX_US             2000
#define DEFAULT_HOVER_TEST_YAW_US    RC_NEUTRAL_US

// BLE flight-log dump
#define FLIGHT_LOG_BYTES       60000
#define FLIGHT_LOG_CHUNK_BYTES 220

// BLE UI notifications are deliberately slower in flight so radio work cannot
// compete with MSP acquisition and the control loop. Idle telemetry remains
// faster for responsive preflight sensor checks.
#define BLE_TELEMETRY_ACTIVE_PERIOD_MS 200
#define BLE_TELEMETRY_IDLE_PERIOD_MS   100
#define BLE_TELEMETRY_CONTROL_BUDGET_US 12000
#define CONTROL_LOOP_PERIOD_US           20000

// Serial output is not useful during flight and can add timing jitter. Keep
// flight data in the BLE telemetry packet and downloadable in-memory CSV.
#define SERIAL_FLIGHT_DEBUG    0

// ── BLE Command IDs ──────────────────────────────────────────
#define CMD_HOVER_TEST      1
#define CMD_START_MISSION   2
#define CMD_DISARM          3
#define CMD_ALT_HOLD        5
#define CMD_KILL            6

// ── Landing Constants ────────────────────────────────────────
#define LANDING_GROUND_M    0.06f  // altitude threshold → cut motors
#define LANDING_TIMEOUT_MS  30000  // safety: force disarm if landing takes too long
#define DESCENT_RATE_MPS    0.35f  // target descent speed m/s above flare
#define LANDING_FLARE_ALT_M 0.45f  // below this, slow descent and raise throttle
#define LANDING_FINAL_ALT_M 0.20f  // below this, use final soft touchdown target
#define LANDING_FLARE_DESCENT_MPS 0.22f
#define LANDING_FINAL_DESCENT_MPS 0.07f
#define LANDING_THROTTLE_OFFSET_US 140  // nominal throttle below hover while landing
#define LANDING_FLARE_OFFSET_US 90
#define LANDING_FINAL_OFFSET_US 40
#define LANDING_ENTRY_RAMP_MS 900       // avoid instant throttle chop when landing is commanded in a climb
#define LANDING_ENTRY_START_OFFSET_US 25
#define LANDING_LOOKAHEAD_S 0.25f
#define LANDING_LOW_ALT_FLOOR_M 0.65f   // below this, keep enough thrust to avoid bounce/rebound
#define LANDING_LOW_ALT_MIN_OFFSET_US 15
#define LANDING_FAST_DESCENT_MPS 0.35f
#define LANDING_FAST_DESCENT_MIN_OFFSET_US 90
#define LANDING_MIN_BELOW_HOVER_US 260

// ── Bench-Mode Simulation Constants ──────────────────────────
#define BENCH_SPRINT_RATE_MPS     9.0f
#define BENCH_PUNCH_RATE_MPS      7.0f
#define BENCH_HOVER_LIFTOFF_US    1325

// ── Arming Settle Time ────────────────────────────────────────
#define ARMING_MS              1500
#define ARM_CONFIRM_TIMEOUT_MS 5000
#define FC_STATUS_FRESH_MS      750

// ── Hover Test Throttle Ramp ──────────────────────────────────
// µs added per loop iteration (~20ms) when ramping from arm-throttle to HOVER_THROTTLE.
// 4 µs/iter × 50 Hz ≈ 200 µs/s → 270 µs step takes ~1.3 s, too slow to trigger ANTI_GRAVITY.
#define HOVER_RAMP_STEP_US  4
#define DEFAULT_ANGLE_MODE  1  // 0=acro/rate mode, 1=Angle mode
#define HOVER_TEST_ANGLE_MODE 0 // fixed-throttle hover tests avoid Angle-mode ground kick

// ── Cascaded Altitude Hold ────────────────────────────────────
// HOLD_KP = outer P gain: altitude error (m) → desired vertical speed (m/s)
// HOLD_KD = inner P gain: speed error (m/s) → throttle offset (µs)
// HOLD_KI = inner I gain: speed integral    → throttle offset (µs)

// Outer loop: setpoint ramp and speed limits
#define DEFAULT_ALT_RAMP_RATE_MPS       0.4f    // m/s internal setpoint ramp rate
#define DEFAULT_MAX_CLIMB_MPS_TEST      0.25f   // max commanded climb, test mode
#define DEFAULT_MAX_DESCENT_MPS_TEST    0.35f   // max commanded descent, test mode
#define MAX_CLIMB_MPS_HOLD      1.20f   // max commanded climb, competition HOLDING
#define MAX_DESCENT_MPS_HOLD    0.80f   // max commanded descent, competition HOLDING
#define ALT_HOLD_TARGET_MIN_M   0.5f    // BLE Alt Hold test target lower bound
#define ALT_HOLD_TARGET_MAX_M   10.0f   // BLE Alt Hold test target upper bound
#define NEAR_TARGET_M           0.5f    // within this radius, scale down max speed
#define NEAR_TARGET_FACTOR      0.35f   // minimum speed factor within near-target zone
#define ALT_HOLD_LOOKAHEAD_S    0.30f   // compensate sensor/control lag by aiming at predicted altitude
#define ALT_HOLD_LOOKAHEAD_MAX_V_MPS 1.0f // clamp prediction velocity so spikes cannot dominate
#define ALT_HOLD_CAPTURE_MARGIN_M 0.25f // below target by this much, keep climbing
#define ALT_HOLD_CAPTURE_MIN_CLIMB_MPS 0.16f
#define ALT_HOLD_CAPTURE_FLOOR_MAX_V_MPS 0.05f // only force throttle floor while barely climbing
#define ALT_HOLD_CAPTURE_MIN_OFFSET_US 10 // small above-hover floor while climb has not established
#define ALT_HOLD_RECOVERY_DESCENT_MPS -0.08f // below this, use stronger recovery floor
#define ALT_HOLD_RECOVERY_MIN_OFFSET_US 70
#define ALT_HOLD_RECOVERY_BF_DESCENT_CMS -80 // raw BF vario can trigger recovery floor before KF catches up
#define HOLD_KD_DOWN_SCALE    0.55f // reduce aggressive throttle pull-down on delayed velocity estimates
#define THROTTLE_SLEW_DOWN_US 25    // max throttle decrease per cascade update
#define THROTTLE_SLEW_UP_US   100   // allow fast recovery when falling

// Inner loop: vario filtering
#define VARIO_TAU_S             0.05f   // seconds, light low-pass; avoid lag on low-alt ToF vario
#define VARIO_STALE_MS          500     // ms before vario reading is considered stale
#define VARIO_MEAS_MAX_CMS       1000    // raw velocity input gate; sprint can legitimately exceed 4 m/s
#define VARIO_CONTROL_MAX_CMS    1000    // fused control-state envelope before declaring sensor failure
#define CASCADE_INVALID_GRACE_MS 300     // tolerate brief stale/out-of-envelope estimates at handoff
#define DEFAULT_BF_VARIO_GROUND_EFFECT_M 1.20f // below this, inflate BF vario covariance
#define VSPEED_I_MAX_US         150.0f  // max integral throttle contribution in us

// Throttle authority around hover
#define THR_UP_OFFSET_US        300     // max µs above HOVER_THROTTLE
#define THR_DOWN_OFFSET_US      300     // max µs below HOVER_THROTTLE (more for braking)
#define THR_DOWN_OFFSET_ALT_HOLD_US 150 // gentler low-alt test braking to avoid delayed throttle chops
#define MIN_ALT_HOLD_THROTTLE_US 1000   // low-altitude test mode can fully unload for braking
#define MIN_MISSION_THROTTLE_US  1050   // mission hold preserves attitude-control authority

// Safety
#define ALT_MAX_M               22.0f   // absolute ceiling — triggers landing above this
#define ATTITUDE_ABORT_DEG      45.0f   // disarm if ALT_HOLD/LANDING gets knocked badly off-level
#define ATTITUDE_ABORT_MAX_AGE_MS 300
#define LANDING_KP_VSPEED       120.0f  // fixed P gain for landing velocity controller

// Alt Hold settle throttle — avoids a hover-throttle jump before closed-loop engages
#define TAKEOFF_NUDGE_US         -80    // start below HOVER_THROTTLE so guard does not launch before cascade
#define TAKEOFF_RAMP_US_PER_S    80     // settle throttle ramp before cascade takes over
#define TAKEOFF_MAX_OFFSET_US    45     // max settle throttle above HOVER_THROTTLE
#define TAKEOFF_INVALID_TOF_MAX_OFFSET_US 60   // slightly higher cap if ToF is unavailable during settle
#define BARO_SETTLE_MS           500    // ms after throttle step for baro to settle at hover pressure
