"""Replays the altitude estimator (App/app/alt_est.c + App/lib/alt_kf.c) on logged "A" lines.

Turn on "ALTRAW" in the GCS ("Độ cao" tab), fly or carry the drone, then:
    python tools/replay_kf.py gcs/logs/log_YYYYMMDD_HHMMSS.csv [--sigma-a 0.3] [--sigma-tof 0.05] [--step-th 0.10]

With the firmware's parameters the replay must match the logged z (checks this port); change them to see
what other values would have done. Same formulas and constants as the C code: keep both in step.
Not replayed: the M10b baro fallback (no baro column in the A line), so stretches without ToF can differ.
"""
import argparse
import csv
import math
import sys

import matplotlib.pyplot as plt
import numpy as np

# app/alt_est.h
TOF_LOST_MS, REINIT_MS = 500, 2000
TILT_MAX_RAD = math.radians(30.0)
CONFIRM_N, CONFIRM_BAND, CONFIRM_MS = 3, 0.03, 100
TERRAIN_SNAP, GATE_SQ, RESYNC_N, P0 = 0.03, 9.0, 10, 0.01
SPIKE_M, SPIKE_N = 0.06, 8
STEP_RATE, STEP_RECENT, STEP_AVG = 1.0, 3, 0.5
STEP_TILT_RAD, REST_BAND, REST_MS = math.radians(10.0), 0.03, 500
NORMAL, CONFIRM = 0, 1
SRC_TOF, SRC_PREDICT = 0, 2


class AltKf:
    """lib/alt_kf.c"""

    def __init__(self, z0):
        self.z, self.vz = z0, 0.0
        self.P = [[P0, 0.0], [0.0, P0]]

    def predict(self, a_up, dt, sigma_a):
        g0, g1 = 0.5 * dt * dt, dt
        self.z += self.vz * dt + g0 * a_up
        self.vz += g1 * a_up
        (p00, p01), (p10, p11) = self.P
        q = sigma_a * sigma_a
        self.P = [[p00 + dt * (p10 + p01) + dt * dt * p11 + g0 * g0 * q, p01 + dt * p11 + g0 * g1 * q],
                  [p10 + dt * p11 + g0 * g1 * q, p11 + g1 * g1 * q]]

    def update(self, z_meas, sigma, gate_sq):
        s = self.P[0][0] + sigma * sigma
        innov = z_meas - self.z
        if innov * innov > gate_sq * s:
            return False
        k0, k1 = self.P[0][0] / s, self.P[1][0] / s
        self.z += k0 * innov
        self.vz += k1 * innov
        (p00, p01), (p10, p11) = self.P
        sym = 0.5 * ((1 - k0) * p01 + p10 - k1 * p00)
        self.P = [[(1 - k0) * p00, sym], [sym, p11 - k1 * p01]]
        return True


class AltEst:
    """app/alt_est.c"""

    def __init__(self, sigma_a, sigma_tof, step_th):
        self.cfg = (sigma_a, sigma_tof, step_th)
        self.kf, self.init, self.h = None, False, 0.0
        self.tof_prev, self.tof_prev_ms, self.state = 0.0, 0, NORMAL
        self.new_level, self.confirm_n, self.confirm_start, self.rejects = 0.0, 0, 0, 0
        self.spike_n = 0
        self.jump_sum, self.jump_t = 0.0, 0.0
        self.recent = [(0.0, 0.0)] * STEP_RECENT  # (jump, d_tof), newest first
        self.src, self.events = SRC_PREDICT, []
        self.was_armed, self.rest_tof, self.at_rest, self.rest_start = False, 0.0, False, 0

    def _restart(self, tof, now):
        self.kf, self.init, self.h, self.state, self.rejects = AltKf(tof), True, 0.0, NORMAL, 0
        self.spike_n, self.at_rest = 0, False
        self.jump_sum, self.jump_t = 0.0, 0.0
        self.tof_prev, self.tof_prev_ms = tof, now

    def _track_jumps(self, jump, d_tof, dt):
        """alt_est.c track_jumps(): run of fast ToF change not explained by vz."""
        self.recent = [(jump, d_tof)] + self.recent[:-1]
        lim = STEP_RATE * dt
        if self.jump_t > 0.0:
            rj = sum(j for j, _ in self.recent)
            rt = sum(t for _, t in self.recent)
            need = STEP_AVG * lim * STEP_RECENT
            pos = self.jump_sum > 0.0
            if (rj > 0.0) == pos and (rt > 0.0) == pos and abs(rj) >= need and abs(rt) >= need:
                self.jump_sum += jump
                self.jump_t += dt
                return
            self.jump_sum, self.jump_t = 0.0, 0.0
        if abs(jump) > lim and abs(d_tof) > lim and (jump > 0.0) == (d_tof > 0.0):
            self.jump_sum, self.jump_t = jump, dt

    def _confirm(self, tof, now):
        self.state, self.new_level, self.confirm_n, self.confirm_start = CONFIRM, tof, 1, now

    @staticmethod
    def _snap(h):
        return 0.0 if abs(h) < TERRAIN_SNAP else h

    def _tof(self, tof, level, now):
        """Returns True when it caused an event (as tof_step() != ALT_EV_NONE)."""
        sigma_a, sigma_tof, step_th = self.cfg
        gap = now - self.tof_prev_ms
        innov = tof + self.h - self.kf.z
        d_tof = tof - self.tof_prev
        self._track_jumps(d_tof - self.kf.vz * gap * 0.001, d_tof, gap * 0.001)
        n_ev = len(self.events)
        steps_on = step_th > 0.0
        if steps_on and self.state == NORMAL and (gap > TOF_LOST_MS or abs(self.jump_sum) > step_th):
            if level:
                self._confirm(tof, now)
        elif self.state == NORMAL:
            spike = abs(innov) > SPIKE_M
            if not spike:
                self.spike_n = 0
            if spike and (self.spike_n < SPIKE_N or (steps_on and self.jump_t > 0.0)):
                self.spike_n += 1
            elif self.kf.update(tof + self.h, sigma_tof, GATE_SQ):
                self.rejects, self.src = 0, SRC_TOF
            elif level or not steps_on:
                self.rejects += 1
                if self.rejects >= RESYNC_N:
                    z_meas = tof + self.h
                    self.events.append((now, "resync", z_meas - self.kf.z))
                    self.kf, self.rejects, self.src, self.spike_n = AltKf(z_meas), 0, SRC_TOF, 0
        else:
            near = abs(tof - self.new_level) <= CONFIRM_BAND
            if self.jump_t > 0.0:  # still crossing the edge: confirm from the end of the fast run
                self.new_level, self.confirm_n, self.confirm_start = tof, 1, now
            elif level and near:
                self.confirm_n += 1
                if self.confirm_n >= CONFIRM_N:
                    h = self._snap(self.kf.z - tof)
                    self.state, self.rejects = NORMAL, 0
                    self.jump_sum, self.jump_t = 0.0, 0.0
                    if abs(h - self.h) < max(step_th, 3.0 * math.sqrt(max(self.kf.P[0][0], 0.0))):
                        self.events.append((now, "noise", h))
                    else:
                        self.h = h
                        self.events.append((now, "step", h))
            elif level:
                self.new_level, self.confirm_n = tof, 1
            if self.state == CONFIRM and now - self.confirm_start > CONFIRM_MS:
                self.state = NORMAL
                self.events.append((now, "noise", self.h))
        self.tof_prev, self.tof_prev_ms = tof, now
        return len(self.events) != n_ev

    def _ground(self, tof, now):
        if abs(tof - self.rest_tof) > REST_BAND or self.state != NORMAL:
            self.at_rest = False
        elif not self.at_rest:
            self.at_rest, self.rest_start = True, now
        elif now - self.rest_start >= REST_MS and (self.h != 0.0 or abs(self.kf.z - tof) > REST_BAND):
            self.events.append((now, "ground", self.h))
            self._restart(tof, now)

    def step(self, now, dt, a_up, tof_new, in_range, dist, roll, pitch, armed):
        tilt_cos = math.cos(roll) * math.cos(pitch)
        usable = tof_new and in_range and tilt_cos > math.cos(TILT_MAX_RAD)
        level = tilt_cos > math.cos(STEP_TILT_RAD)
        tof = dist * tilt_cos
        if not armed:
            self.was_armed = False
        if self.init:
            self.kf.predict(a_up, dt, self.cfg[0])
            self.src = SRC_PREDICT
        if usable:
            if not self.init or now - self.tof_prev_ms > REINIT_MS:
                if not self.init:
                    self.rest_tof = tof
                self._restart(tof, now)
                self.src = SRC_TOF
                self.events.append((now, "init", tof))
            elif armed and not self.was_armed:
                self.was_armed, self.rest_tof = True, tof
                self.events.append((now, "arm", self.h))
                self._restart(tof, now)
                self.src = SRC_TOF
            elif not self._tof(tof, level, now) and not armed:
                self._ground(tof, now)
        if not self.init:
            return math.nan, math.nan, math.nan, False
        return self.kf.z, self.kf.vz, self.h, now - self.tof_prev_ms <= TOF_LOST_MS


def read_a_lines(path):
    rows = []
    with open(path, encoding="utf-8", newline="") as f:
        for rec in csv.reader(f):
            if len(rec) < 2 or not rec[1].startswith("A,"):
                continue
            p = rec[1].split(",")
            if len(p) == 15:
                p.append("0")  # older firmware: no armed column
            if len(p) != 16:
                continue
            try:
                rows.append([float(x) for x in p[1:]])
            except ValueError:
                continue
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--sigma-a", type=float, default=0.3)
    ap.add_argument("--sigma-tof", type=float, default=0.05)
    ap.add_argument("--step-th", type=float, default=0.0, help="0 = no step detection (firmware default)")
    args = ap.parse_args()
    a = read_a_lines(args.log)
    if len(a) == 0:
        sys.exit("No A lines in this log (turn on ALTRAW in the GCS 'Độ cao' tab).")
    # Columns: t_ms dt_us a_up tof_new in_range dist roll pitch z vz h src state valid armed (mm, cdeg)
    t_ms, dt, a_up = a[:, 0], a[:, 1] * 1e-6, a[:, 2] * 1e-3
    tof_new, in_range, dist = a[:, 3] > 0, a[:, 4] > 0, a[:, 5] * 1e-3
    roll, pitch = np.radians(a[:, 6] / 100.0), np.radians(a[:, 7] / 100.0)
    fw_z, fw_vz, fw_h = a[:, 8] * 1e-3, a[:, 9] * 1e-3, a[:, 10] * 1e-3

    armed = a[:, 14] > 0
    est = AltEst(args.sigma_a, args.sigma_tof, args.step_th)
    out = np.array([est.step(int(t_ms[i]), dt[i], a_up[i], tof_new[i], in_range[i], dist[i], roll[i], pitch[i],
                             armed[i]) for i in range(len(a))])
    t = (t_ms - t_ms[0]) / 1000.0
    ok = ~np.isnan(out[:, 0])
    print(f"{len(a)} steps, {t[-1]:.1f} s. Max |z replay - z firmware| = "
          f"{np.max(np.abs(out[ok, 0] - fw_z[ok])) * 1000:.1f} mm (≈0 with the firmware's parameters)")
    for when, kind, val in est.events:
        print(f"  {(when - t_ms[0]) / 1000:7.2f} s  {kind:8s} {val:+.3f} m")

    tof_plot = np.where(tof_new & in_range, dist * np.cos(roll) * np.cos(pitch), np.nan)
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    ax[0].plot(t, tof_plot, ".", ms=2, color="0.6", label="ToF (bù nghiêng)")
    ax[0].plot(t, fw_z, label="z firmware")
    ax[0].plot(t, out[:, 0], "--", label=f"z replay σa={args.sigma_a} σtof={args.sigma_tof}")
    ax[0].plot(t, fw_h, label="h_terrain firmware")
    ax[0].plot(t, out[:, 2], "--", label="h_terrain replay")
    ax[0].set_ylabel("m")
    ax[0].legend(loc="upper left", fontsize=8)
    ax[1].plot(t, fw_vz, label="vz firmware")
    ax[1].plot(t, out[:, 1], "--", label="vz replay")
    ax[1].set_ylabel("m/s")
    ax[1].legend(loc="upper left", fontsize=8)
    ax[2].plot(t, a_up, lw=0.6, label="a_up")
    ax[2].set_ylabel("m/s²")
    ax[2].set_xlabel("s")
    ax[2].legend(loc="upper left", fontsize=8)
    for x in ax:
        x.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
