#pragma once
#include <math.h>
#include <stdint.h>
#include "Config.h"

// ============================================================================
// AltitudeKF
//
// Drop-in replacement for fuseAltitude() + selectVarioForControl() in Msp.cpp.
// State: x = [altitude (m), vertical_velocity (m/s)]
//
// Design goals, mapped directly to the bugs found in flight log analysis:
//   1. Physical-plausibility gate on ToF (updateTof) -- catches sensor spikes
//      like the +0.97m/82ms jump found in the 2026-07-05 log, which the old
//      holdover mechanism did NOT catch (holdover only fires on explicit
//      sensor-invalid, not on an implausible-but-"valid" reading).
//   2. Uses cbaro (already baro-shifted toward ToF reference) as the position
//      measurement -- this eliminates the dual-baseline drift between
//      `lowRel` (fixed baseline) and `altitude` (separately drifting offset)
//      that caused reference-frame step discontinuities in mission.cpp.
//   3. Both vario sources (bfVario, derVario) are offered as independent,
//      covariance-weighted measurements instead of chosen by nested if/else
//      in selectVarioForControl(). Ground-effect zone inflates R on bfVario
//      rather than hard-switching away from it.
//
// Call order per getAltitude() cycle:
//   predict(dt)
//   updateBaro(cbaroM)
//   updateTof(tofM, tofValid)        // internally gated -- may no-op
//   updateVelocity(bfVario, isBfPlausible, nearGround)
//   updateVelocity(derVario, isDerPlausible, /*nearGround=*/false)
//
// altitude/velocity are the outputs -- read directly after each cycle.
// ============================================================================

class AltitudeKF {
public:
    // ---- Tunable noise parameters ------------------------------------------
    float q_accel_psd = KF_Q_ACCEL_PSD; // process noise, (m/s^2)^2 * s

    float r_baro    = KF_R_BARO;       // m^2, corrected-baro position noise
    float r_tof     = KF_R_TOF;        // m^2, ToF position noise (well within range)
    float r_bf_vario  = KF_R_BF_VARIO; // (m/s)^2, Betaflight fused vario
    float r_bf_vario_ground_mult = KF_R_BF_VARIO_GROUND_MULT;
    float r_der_vario = KF_R_DER_VARIO; // (m/s)^2, ESP32-derived vario

    // ToF plausibility gate
    float tof_max_rate_mps = TOF_FUSION_MAX_STEP_MPS;
    float ground_effect_alt_m = DEFAULT_BF_VARIO_GROUND_EFFECT_M;

    // ---- State --------------------------------------------------------------
    float altitude = 0.0f;
    float velocity = 0.0f;
    float p_alt_alt = 1.0f;
    float p_alt_vel = 0.0f;
    float p_vel_vel = 1.0f;

    // Internal: last trusted ToF reading, for the plausibility gate
    float lastTrustedTofM = 0.0f;
    uint32_t lastTrustedTofMs = 0;
    bool haveTrustedTof = false;

    void reset(float alt0 = 0.0f, float vel0 = 0.0f) {
        altitude = alt0;
        velocity = vel0;
        p_alt_alt = 1.0f;
        p_alt_vel = 0.0f;
        p_vel_vel = 1.0f;
        haveTrustedTof = false;
        lastTrustedTofMs = 0;
    }

    void predict(float dt) {
        if (dt <= 0.0f || dt > 0.5f) return;

        altitude += velocity * dt;

        const float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
        const float q = q_accel_psd;
        const float q_aa = q*dt4/4.0f, q_av = q*dt3/2.0f, q_vv = q*dt2;

        const float new_p_alt_alt = p_alt_alt + 2.0f*dt*p_alt_vel + dt2*p_vel_vel + q_aa;
        const float new_p_alt_vel = p_alt_vel + dt*p_vel_vel + q_av;
        const float new_p_vel_vel = p_vel_vel + q_vv;

        p_alt_alt = new_p_alt_alt;
        p_alt_vel = new_p_alt_vel;
        p_vel_vel = new_p_vel_vel;
    }

    // 1D position update, generic (shared by baro and ToF)
    void updatePosition(float measM, float R) {
        const float y = measM - altitude;
        const float s = p_alt_alt + R;
        const float k_alt = p_alt_alt / s;
        const float k_vel = p_alt_vel / s;

        altitude += k_alt * y;
        velocity += k_vel * y;

        const float p_alt_alt_new = p_alt_alt - k_alt * p_alt_alt;
        const float p_alt_vel_new = p_alt_vel - k_alt * p_alt_vel;
        const float p_vel_vel_new = p_vel_vel - k_vel * p_alt_vel;

        p_alt_alt = p_alt_alt_new;
        p_alt_vel = p_alt_vel_new;
        p_vel_vel = p_vel_vel_new;
    }

    // Corrected-baro position update -- always available, replaces `altitude`
    // (the drifting baro/launchAlt-relative value) in the old code.
    void updateBaro(float cbaroM) {
        updatePosition(cbaroM, r_baro);
    }

    // ToF position update, WITH the physical-plausibility gate that was
    // missing before. Returns true if applied, false if rejected.
    bool updateTof(float tofM, bool tofValid, uint32_t nowMs) {
        if (!tofValid) return false;

        if (haveTrustedTof && lastTrustedTofMs != 0) {
            float dt = (nowMs - lastTrustedTofMs) / 1000.0f;
            if (dt > 0.001f) {
                float impliedRate = fabsf(tofM - lastTrustedTofM) / dt;
                if (impliedRate > tof_max_rate_mps) {
                    // Reject: physically implausible jump (e.g. multipath/glitch).
                    // Do NOT update lastTrustedTof -- keep the old trusted value
                    // so a second consecutive glitch doesn't get "grandfathered in".
                    return false;
                }
            }
        }

        lastTrustedTofM = tofM;
        lastTrustedTofMs = nowMs;
        haveTrustedTof = true;

        updatePosition(tofM, r_tof);
        return true;
    }

    // 1D velocity update, generic (shared by both vario sources)
    void updateVelocityMeas(float measMps, float R) {
        const float y = measMps - velocity;
        const float s = p_vel_vel + R;
        const float k_vel = p_vel_vel / s;
        const float k_alt = p_alt_vel / s;

        altitude += k_alt * y;
        velocity += k_vel * y;

        const float p_vel_vel_new = p_vel_vel - k_vel * p_vel_vel;
        const float p_alt_vel_new = p_alt_vel - k_vel * p_alt_vel;
        const float p_alt_alt_new = p_alt_alt - k_alt * p_alt_vel;

        p_alt_alt = p_alt_alt_new;
        p_alt_vel = p_alt_vel_new;
        p_vel_vel = p_vel_vel_new;
    }

    // Betaflight fused vario -- R inflated near ground instead of hard-switched away.
    void updateBfVario(float bfVarioMps, bool plausible) {
        if (!plausible) return;
        float R = r_bf_vario;
        if (altitude < ground_effect_alt_m) {
            R *= r_bf_vario_ground_mult;
        }
        updateVelocityMeas(bfVarioMps, R);
    }

    // ESP32-derived vario fallback -- always offered when plausible, higher R.
    void updateDerivedVario(float derVarioMps, bool plausible) {
        if (!plausible) return;
        updateVelocityMeas(derVarioMps, r_der_vario);
    }
};
