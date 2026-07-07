"""
Python mirror of AltitudeKF.h, used to replay logged flights against the raw
sensor columns recorded by the ESP32.

This lets you validate KF tuning against real sensor sequences before flying
new firmware.
"""
import numpy as np


class AltitudeKF:
    def __init__(self):
        self.q_accel_psd = 0.5
        self.r_baro = 0.6
        self.r_tof = 0.02
        self.r_bf_vario = 0.5
        self.r_bf_vario_ground_mult = 6.0
        self.r_der_vario = 1.2
        self.tof_max_rate_mps = 6.0
        self.ground_effect_alt_m = 0.5

        self.altitude = 0.0
        self.velocity = 0.0
        self.p_alt_alt = 1.0
        self.p_alt_vel = 0.0
        self.p_vel_vel = 1.0

        self.last_trusted_tof_m = 0.0
        self.last_trusted_tof_ms = 0
        self.have_trusted_tof = False

        # diagnostics for the replay harness
        self.tof_rejected_log = []

    def reset(self, alt0=0.0, vel0=0.0):
        self.altitude = alt0
        self.velocity = vel0
        self.p_alt_alt = 1.0
        self.p_alt_vel = 0.0
        self.p_vel_vel = 1.0
        self.have_trusted_tof = False
        self.last_trusted_tof_ms = 0

    def predict(self, dt):
        if dt <= 0.0 or dt > 0.5:
            return
        self.altitude += self.velocity * dt
        dt2, dt3, dt4 = dt*dt, dt**3, dt**4
        q = self.q_accel_psd
        q_aa, q_av, q_vv = q*dt4/4.0, q*dt3/2.0, q*dt2

        new_p_alt_alt = self.p_alt_alt + 2*dt*self.p_alt_vel + dt2*self.p_vel_vel + q_aa
        new_p_alt_vel = self.p_alt_vel + dt*self.p_vel_vel + q_av
        new_p_vel_vel = self.p_vel_vel + q_vv

        self.p_alt_alt, self.p_alt_vel, self.p_vel_vel = new_p_alt_alt, new_p_alt_vel, new_p_vel_vel

    def _update_position(self, meas, R):
        y = meas - self.altitude
        s = self.p_alt_alt + R
        k_alt = self.p_alt_alt / s
        k_vel = self.p_alt_vel / s

        self.altitude += k_alt * y
        self.velocity += k_vel * y

        p_alt_alt_new = self.p_alt_alt - k_alt * self.p_alt_alt
        p_alt_vel_new = self.p_alt_vel - k_alt * self.p_alt_vel
        p_vel_vel_new = self.p_vel_vel - k_vel * self.p_alt_vel

        self.p_alt_alt, self.p_alt_vel, self.p_vel_vel = p_alt_alt_new, p_alt_vel_new, p_vel_vel_new

    def update_baro(self, cbaro_m):
        self._update_position(cbaro_m, self.r_baro)

    def update_tof(self, tof_m, tof_valid, now_ms):
        if not tof_valid:
            return False
        if self.have_trusted_tof and self.last_trusted_tof_ms != 0:
            dt = (now_ms - self.last_trusted_tof_ms) / 1000.0
            if dt > 0.001:
                implied_rate = abs(tof_m - self.last_trusted_tof_m) / dt
                if implied_rate > self.tof_max_rate_mps:
                    self.tof_rejected_log.append((now_ms, tof_m, implied_rate))
                    return False
        self.last_trusted_tof_m = tof_m
        self.last_trusted_tof_ms = now_ms
        self.have_trusted_tof = True
        self._update_position(tof_m, self.r_tof)
        return True

    def _update_velocity(self, meas, R):
        y = meas - self.velocity
        s = self.p_vel_vel + R
        k_vel = self.p_vel_vel / s
        k_alt = self.p_alt_vel / s

        self.altitude += k_alt * y
        self.velocity += k_vel * y

        p_vel_vel_new = self.p_vel_vel - k_vel * self.p_vel_vel
        p_alt_vel_new = self.p_alt_vel - k_vel * self.p_alt_vel
        p_alt_alt_new = self.p_alt_alt - k_alt * self.p_alt_vel

        self.p_alt_alt, self.p_alt_vel, self.p_vel_vel = p_alt_alt_new, p_alt_vel_new, p_vel_vel_new

    def update_bf_vario(self, bf_vario_mps, plausible):
        if not plausible:
            return
        R = self.r_bf_vario
        if self.altitude < self.ground_effect_alt_m:
            R *= self.r_bf_vario_ground_mult
        self._update_velocity(bf_vario_mps, R)

    def update_derived_vario(self, der_vario_mps, plausible):
        if not plausible:
            return
        self._update_velocity(der_vario_mps, self.r_der_vario)


def replay_v1(df, vario_max_plausible_cms=800):
    """
    First-draft replay: uses the logged `cbaro` column directly.
    Kept for comparison -- see replay() below for the fixed version that
    gates the offset tracker too, not just the KF's own ToF update.
    """
    kf = AltitudeKF()
    kf.reset(alt0=df['cbaro'].iloc[0])

    kf_alt = []
    kf_vel = []
    last_ms = None

    for _, row in df.iterrows():
        ms = row['ms']
        if last_ms is not None:
            dt = (ms - last_ms) / 1000.0
            kf.predict(dt)
        last_ms = ms

        kf.update_baro(row['cbaro'])

        tof_valid = row['tof'] > 0
        if tof_valid:
            kf.update_tof(row['tof'], True, ms)

        bf_plausible = abs(row['bfV']) <= vario_max_plausible_cms
        kf.update_bf_vario(row['bfV'] / 100.0, bf_plausible)

        der_plausible = abs(row['derV']) <= vario_max_plausible_cms
        kf.update_derived_vario(row['derV'] / 100.0, der_plausible)

        kf_alt.append(kf.altitude)
        kf_vel.append(kf.velocity)

    df = df.copy()
    df['kf_alt'] = kf_alt
    df['kf_vel'] = kf_vel
    return df, kf


def replay(df, vario_max_plausible_cms=800, tof_offset_alpha=0.1, tof_max_rate_mps=6.0):
    """
    Replay the KF against a parsed flight-log dataframe.
    Expects columns: ms, baro, tof, tofW, bfV, derV
    (NOTE: does NOT use the logged `cbaro` column -- that was computed by the
    original firmware's un-gated offset tracker and is contaminated by the
    same ToF glitches we're trying to filter out. We recompute the
    baro-to-ToF offset here, gated by the SAME plausibility check used for
    the KF's own ToF position update, so a rejected ToF reading is rejected
    everywhere, not just at the KF's front door.)

    Returns the dataframe with kf_alt / kf_vel / cbaro_gated columns appended,
    plus the KF instance (for inspecting tof_rejected_log).
    """
    kf = AltitudeKF()
    kf.reset(alt0=df['baro'].iloc[0])

    offset_initialized = False
    offset_m = 0.0
    last_trusted_tof_for_offset = None
    last_trusted_tof_ms_for_offset = None

    kf_alt, kf_vel, cbaro_gated_list = [], [], []
    last_ms = None

    for _, row in df.iterrows():
        ms = row['ms']
        if last_ms is not None:
            dt = (ms - last_ms) / 1000.0
            kf.predict(dt)
        last_ms = ms

        tof_raw = row['tof']
        tof_valid = tof_raw > 0

        tof_accepted = False
        if tof_valid:
            if last_trusted_tof_for_offset is not None:
                dt_g = (ms - last_trusted_tof_ms_for_offset) / 1000.0
                implied_rate = abs(tof_raw - last_trusted_tof_for_offset) / dt_g if dt_g > 0.001 else 0.0
                tof_accepted = implied_rate <= tof_max_rate_mps
            else:
                tof_accepted = True

        if tof_accepted:
            last_trusted_tof_for_offset = tof_raw
            last_trusted_tof_ms_for_offset = ms
            measured_offset = tof_raw - row['baro']
            if not offset_initialized:
                offset_m = measured_offset
                offset_initialized = True
            else:
                offset_m += tof_offset_alpha * (measured_offset - offset_m)
        elif tof_valid:
            kf.tof_rejected_log.append((ms, tof_raw, None))

        cbaro_gated = row['baro'] + (offset_m if offset_initialized else 0.0)
        cbaro_gated_list.append(cbaro_gated)

        kf.update_baro(cbaro_gated)
        if tof_accepted:
            kf.update_tof(tof_raw, True, ms)

        bf_plausible = abs(row['bfV']) <= vario_max_plausible_cms
        kf.update_bf_vario(row['bfV'] / 100.0, bf_plausible)

        der_plausible = abs(row['derV']) <= vario_max_plausible_cms
        kf.update_derived_vario(row['derV'] / 100.0, der_plausible)

        kf_alt.append(kf.altitude)
        kf_vel.append(kf.velocity)

    df = df.copy()
    df['kf_alt'] = kf_alt
    df['kf_vel'] = kf_vel
    df['cbaro_gated'] = cbaro_gated_list
    return df, kf
