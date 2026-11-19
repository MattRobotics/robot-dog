#!/usr/bin/env python3
"""M6: is the swing foot off the floor? A load proxy, checked on the twin before it is read on the robot.

Input: the <episode>__<variant>_legs.csv of m6_dev7 (one row per walking snapshot and leg, at the read
instant of the leg's LOWER). The proxy is the signed joint-space duty of the LOWER (the knee carries the
body in stance). On the twin the truth is known (normal force of the foot): the proxy's threshold is
chosen there (best balanced accuracy on the twin's own swing samples) and its accuracy reported; the same
threshold is then applied to the hardware's swing samples. Only swing samples away from lift-off and
touchdown (25-75 % of the law swing) are used, where a lifted foot is unambiguous.

Usage: m6_legs.py <run dir> <variant> [--swing-s S] (law swing duration per mode is taken from the samples)
"""
import pathlib
import sys

import numpy as np


def load(run_dir, variant):
    rows = []
    for f in sorted(pathlib.Path(run_dir).glob(f"*__{variant}_legs.csv")):
        a = np.genfromtxt(f, delimiter=",", names=True)
        if a.size:
            rows.append((f.name.split("__")[0], np.atleast_1d(a)))
    return rows


def main():
    run_dir, variant = sys.argv[1], sys.argv[2]
    data = load(run_dir, variant)
    if not data:
        raise SystemExit("no samples")
    a = np.concatenate([d for _, d in data])
    swing = a["stance"] == 0
    sw_s = a["swing_s"]
    # law swing length per episode/mode is not in the file: keep samples 0.15-0.45 s after lift-off for
    # STANDARD/FAST (0.53/0.44 s swings) and 0.25-0.75 s for SLOW (1.0 s): mid-swing in both
    mid = swing & (sw_s >= 0.12) & (sw_s <= 0.40)
    stance = a["stance"] == 1
    lifted = a["sim_fz_n"] <= 0.5
    print(f"{variant}: {len(a)} leg samples, swing {swing.sum()}, mid-swing {mid.sum()}, stance {stance.sum()}")
    for who in ("hw", "sim"):
        print(f"  {who:3s} LOWER duty (signed, permille) stance p10/50/90 {np.percentile(a[f'{who}_lower'][stance], [10, 50, 90]).round(0)}"
              f" | mid-swing p10/50/90 {np.percentile(a[f'{who}_lower'][mid], [10, 50, 90]).round(0)}")
    s = a[mid]
    truth = s["sim_fz_n"] <= 0.5
    print(f"  twin mid-swing samples with the foot off the floor (fz <= 0.5 N): {truth.mean():.2f}; foot height p50 {np.median(s['sim_foot_z_mm']):.2f} mm")
    best = None
    for thr in np.arange(-100, 201, 4):
        pred = s["sim_lower"] < thr
        tpr = np.mean(pred[truth]) if truth.any() else np.nan
        tnr = np.mean(~pred[~truth]) if (~truth).any() else np.nan
        bal = np.nanmean([tpr, tnr])
        if best is None or bal > best[1]:
            best = (thr, bal, tpr, tnr)
    thr, bal, tpr, tnr = best
    print(f"  proxy on the twin: 'lifted' if LOWER duty < {thr:.0f} permille: balanced accuracy {bal:.2f} (lifted {tpr:.2f}, on floor {tnr:.2f})")
    print(f"  proxy applied: twin mid-swing 'lifted' {np.mean(s['sim_lower'] < thr):.2f} (truth {truth.mean():.2f}); "
          f"hardware mid-swing 'lifted' {np.mean(s['hw_lower'] < thr):.2f} (n {len(s)})")
    for name in sorted({n for n, _ in data}):
        d = next(x for n, x in data if n == name)
        m = (d["stance"] == 0) & (d["swing_s"] >= 0.12) & (d["swing_s"] <= 0.40)
        if m.sum() >= 3:
            print(f"    {name:28s} n {m.sum():3d} hw 'lifted' {np.mean(d['hw_lower'][m] < thr):.2f} twin 'lifted' {np.mean(d['sim_lower'][m] < thr):.2f} truth {np.mean(d['sim_fz_n'][m] <= 0.5):.2f}")


if __name__ == "__main__":
    main()
