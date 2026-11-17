"""Optical flow calibration of the MTF-01P from FLOWLOG "F" lines (motion_x/y units are not documented).

Hand test, motors off, GCS on USB, "Ghi optical flow (FLOWLOG)" ticked, drone held level ~0.5 m above a textured floor:
  1. rock it about roll a few times in place, then about pitch (no translation)  -> axis map, sign, scale, lag
  2. slide it forward ~0.5 m and stop, then right ~0.5 m and stop (no tilt)       -> check: displacement
    python tools/flow_cal.py gcs/logs/log_YYYYMMDD_HHMMSS.csv [--t0 S --t1 S] [--plot]

Rotation: the floor seen by the camera moves with the body rate, so flow = k * gyro on one axis pair.
Translation: (flow / k - gyro) * height = ground speed along the other body axis.
"""
import argparse
import csv
import sys

import numpy as np

AXES = ("p (roll rate)", "q (pitch rate)")

# Reference: the s450drone code (same MTF-01P, MSP, added to this repo 2026-11-10) flies position hold with
# motion_x paired with the roll rate and motion_y with the pitch rate, speed = raw * K_FLOW * height (K_FLOW 0.2735,
# tuned on translation) and a hand-tuned rotation compensation of only 0.3 x gyro. In this script's units
# (counts per rad/s) that is ~3.66 from their translation tuning but ~1.10 from their rotation weight: the two
# disagree, i.e. they never matched flow to gyro, which the rocking test here measures directly.
S450_K_TRANSLATION = 1.0 / 0.2735
S450_K_ROTATION = 0.3 / 0.2735


def read_f_lines(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace", newline="") as f:
        for rec in csv.reader(f):
            if len(rec) < 2 or not rec[1].startswith("F,"):
                continue
            p = rec[1].split(",")
            if len(p) != 11:
                continue
            try:
                rows.append([float(x) for x in p[1:]])
            except ValueError:
                continue
    # t_ms quality mx my p q r (mrad/s) roll pitch (cdeg) tof (mm)
    return np.array(rows)


def lagged(a, b, lag):
    """a[i] against b[i - lag] (flow lags the gyro by lag frames)."""
    return (a[lag:], b[:len(b) - lag]) if lag > 0 else (a, b)


def best_fit(f, g, max_lag):
    best = None
    for lag in range(max_lag + 1):
        fa, ga = lagged(f, g, lag)
        m = np.abs(ga) > 0.3  # rotating clearly: translation is negligible there in a hand test
        if m.sum() < 30:
            continue
        k = float(np.dot(fa[m], ga[m]) / np.dot(ga[m], ga[m]))
        r = float(np.corrcoef(fa[m], ga[m])[0, 1])
        if best is None or abs(r) > abs(best[2]):
            best = (lag, k, r, int(m.sum()))
    return best


# Firmware (App/app/flow_est.h): counts are mapped back to the 2026-11-10 reference mount (motion_x ~ p and
# motion_y ~ q, both k < 0) by flow_rot quarter turns, then v_fwd = h*(ry/k - q), v_right = -h*(rx/k - p).
ROT_MAPS = {0: lambda x, y: (x, y), 90: lambda x, y: (-y, x), 180: lambda x, y: (-x, -y), 270: lambda x, y: (y, -x)}


def firmware_setting(flow, gyro, h, dt, t, fits, match):
    lag = max(fits[(i, match[i])][0] for i in range(2))
    best = None
    for rot, f in ROT_MAPS.items():
        rx, ry = f(flow[:, 0], flow[:, 1])
        bx, by = best_fit(rx, gyro[:, 0], lag), best_fit(ry, gyro[:, 1], lag)
        if bx and by and bx[1] < 0 and by[1] < 0 and (best is None or bx[2] + by[2] < best[1]):
            best = (rot, bx[2] + by[2], (bx[1] + by[1]) / 2)
    if best is None:
        print("\nFirmware: no flow_rot gives the reference pairing (repeat the rocking)")
        return
    rot, _, k = best
    rx, ry = ROT_MAPS[rot](flow[:, 0], flow[:, 1])
    shift = lambda g: np.concatenate([np.zeros(lag), g[:len(g) - lag]]) if lag else g
    vf = h * (ry / k - shift(gyro[:, 1]))
    vr = -h * (rx / k - shift(gyro[:, 0]))
    xf, xr = np.concatenate([[0], np.cumsum(vf[1:] * dt)]), np.concatenate([[0], np.cumsum(vr[1:] * dt)])
    print(f"\nFirmware: SET flow_rot {rot}   SET flow_k {k:.2f}   (then SAVE)")
    print("  slide check as the firmware sees it (forward +, right +): min/max forward "
          f"{xf.min():+.2f}/{xf.max():+.2f} m, right {xr.min():+.2f}/{xr.max():+.2f} m")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--t0", type=float, default=0.0, help="s from the first F line")
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    a = read_f_lines(args.log)
    if len(a) == 0:
        sys.exit("No F lines (tick FLOWLOG in the GCS 'Độ cao' tab, connected over USB).")
    t = (a[:, 0] - a[0, 0]) / 1000.0
    a, t = a[(t >= args.t0) & (t <= args.t1)], t[(t >= args.t0) & (t <= args.t1)]
    qual, flow = a[:, 1], a[:, 2:4]
    gyro = a[:, 4:6] / 1000.0
    h = a[:, 9] / 1000.0
    dt = np.diff(t)
    print(f"{len(a)} frames, {t[-1] - t[0]:.1f} s, {1 / np.median(dt):.0f} Hz (max gap {dt.max() * 1000:.0f} ms)")
    print(f"quality: median {np.median(qual):.0f}, min {qual.min():.0f} | ToF median {np.median(h):.2f} m")
    print(f"motion_x {flow[:, 0].min():.0f}..{flow[:, 0].max():.0f}   motion_y {flow[:, 1].min():.0f}..{flow[:, 1].max():.0f}")

    max_lag = 15  # frames (~150 ms; Betaflight notes ~65 ms of flow latency)
    fits = {(i, j): best_fit(flow[:, i], gyro[:, j], max_lag) for i in range(2) for j in range(2)}
    print("\nflow axis vs gyro axis: lag, scale k (units per rad/s), correlation, samples")
    for (i, j), b in fits.items():
        if b:
            print(f"  motion_{'xy'[i]} ~ {AXES[j]:15s} lag {b[0] * 10:3d} ms  k {b[1]:9.2f}  r {b[2]:+.2f}  n {b[3]}")
    pair_a = abs(fits[(0, 0)][2] if fits[(0, 0)] else 0) + abs(fits[(1, 1)][2] if fits[(1, 1)] else 0)
    pair_b = abs(fits[(0, 1)][2] if fits[(0, 1)] else 0) + abs(fits[(1, 0)][2] if fits[(1, 0)] else 0)
    match = {0: 0, 1: 1} if pair_a >= pair_b else {0: 1, 1: 0}
    if not all(fits[(i, match[i])] for i in range(2)):
        sys.exit("\nNot enough rotation in the log: rock the drone about roll and pitch (> ~20 deg/s).")
    print("\nResult (rotation):")
    for i in range(2):
        lag, k, r, _ = fits[(i, match[i])]
        print(f"  motion_{'xy'[i]} follows {AXES[match[i]]}: k = {k:+.2f} per rad/s, lag {lag * 10} ms, r {r:+.2f}"
              + ("   <- weak, repeat the rocking" if abs(r) < 0.8 else ""))
    s450_pairs = match == {0: 0, 1: 1}
    print(f"\ns450drone reference: pairing x~roll, y~pitch -> {'SAME' if s450_pairs else 'DIFFERENT'} here; "
          f"|k| {S450_K_TRANSLATION:.2f} (their translation tuning) / {S450_K_ROTATION:.2f} (their rotation weight)"
          f" vs {abs(fits[(0, match[0])][1]):.2f} / {abs(fits[(1, match[1])][1]):.2f} measured here")

    firmware_setting(flow, gyro, h, dt, t, fits, match)

    # Translation: compensated angular flow times height, integrated. Rate about x (p) <-> motion along y (right).
    print("\nDisplacement from flow (body frame, sign to be checked against the slide test):")
    disp = np.zeros((len(a), 2))
    for i in range(2):
        lag, k, _, _ = fits[(i, match[i])]
        g = np.concatenate([np.zeros(lag), gyro[:len(gyro) - lag, match[i]]]) if lag else gyro[:, match[i]]
        v = (flow[:, i] / k - g) * h  # m/s along the axis normal to that gyro axis
        v[qual < 1] = 0.0
        disp[1:, i] = np.cumsum(v[1:] * dt)
    names = {0: "right (from p)", 1: "forward (from q)"}
    for s in np.arange(0.0, t[-1] - t[0] + 1e-9, 1.0):
        n = np.searchsorted(t - t[0], s)
        n = min(n, len(t) - 1)
        print(f"  {s:5.1f} s: " + "   ".join(f"motion_{'xy'[i]} -> {names[match[i]]:16s} {disp[n, i]:+.2f} m"
                                           for i in range(2)))
    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
        for i in range(2):
            lag, k, _, _ = fits[(i, match[i])]
            ax[i].plot(t, flow[:, i] / k, label=f"motion_{'xy'[i]} / k")
            ax[i].plot(t, gyro[:, match[i]], label=AXES[match[i]] + " rad/s")
            ax[i].legend(loc="upper left", fontsize=8)
            ax[i].grid(alpha=0.3)
        ax[2].plot(t, disp[:, 0], label="disp motion_x (m)")
        ax[2].plot(t, disp[:, 1], label="disp motion_y (m)")
        ax[2].legend(loc="upper left", fontsize=8)
        ax[2].grid(alpha=0.3)
        ax[2].set_xlabel("s")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
