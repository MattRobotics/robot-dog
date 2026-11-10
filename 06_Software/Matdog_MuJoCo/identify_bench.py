#!/usr/bin/env python3
"""Identify the unloaded ST3215 dynamics of servo.py on the bench QC V6.1 RAW
captures (500 Hz, 11.1 V, no load, 2026-08-24/25).

Usage: identify_bench.py <dir with the QC *.bin> <RAW_BIN_SHA256SUMS.txt> <out dir>

In units of the stall torque an unloaded joint obeys
    (tau_m / omega0) qdd = duty(sp - q) - qd / omega0 - f sign(qd)
so no absolute torque enters: k (MEASURED under load), omega0 (MEASURED) and
the dead band (FIRMWARE) stay fixed; the mechanical time constant tau_m, the
Coulomb friction f (fraction of the stall torque) and the speed feedforward
fraction c are fitted on the measured
position of every move (RMSE in ticks, grid search), with servo.py's profile
and P loop. TorqueLimit of each capture: from the data (full-speed PresentLoad saturates
at 500 on the six units captured before @NORMALIZE_MATDOG, at 1000 after). Moves: MAX_PRECISION
(phase 8, ACC 50, speed 0) and TRANSFER (phase 10, speed 600, ACC 25/30) are
fitted; MAX_DOWN/UP (phases 6/7, full speed) are only predicted, against the
arrival braking the bench analysis reports (st3215_bench_accel.py).
"""
import hashlib
import json
import pathlib
import struct
import sys

import numpy as np

import servo

RECORD = np.dtype([("t", "<u4"), ("gp", "<u2"), ("gs", "<u2"), ("ga", "u1"), ("ph", "u1"), ("pr", "u1"), ("fl", "u1"),
                   ("pos", "<u2"), ("sp", "<u2"), ("ld", "<u2"), ("v", "u1"), ("T", "u1"), ("r64", "u1"), ("st", "u1"),
                   ("mv", "u1"), ("r67", "u1"), ("r68", "u1"), ("cur", "<u2")])
DT = 0.0005
TAU_M_GRID = (0.010, 0.015, 0.025, 0.035, 0.050)
FRICTION_GRID = (0.02, 0.03, 0.04, 0.05)
FF_GRID = (0.45, 0.6, 0.7, 0.8)


def signed(x, bit):
    x = x.astype(int)
    return np.where(x & (1 << bit), -(x & ((1 << bit) - 1)), x)


def moves(path, phases, per_capture=12):
    """Constant (goal, speed, acc) segments from rest, at least 20 ticks long."""
    b = path.read_bytes()
    n = struct.unpack_from("<8s7I48s", b, 0)[5]
    r = np.frombuffer(b, dtype=RECORD, count=n, offset=84)
    # RAM TorqueLimit of the capture: full-speed moves saturate PresentLoad at it
    # (500 on the six units measured before @NORMALIZE_MATDOG raised MaxTorque, else 1000).
    full = (r["ph"] == 6) | (r["ph"] == 7)
    limit = 0.5 if np.abs(signed(r["ld"][full], 10)).max() <= 520 else 1.0
    out, i = [], 0
    while i < n:
        j = i
        key = (r["gp"][i], r["gs"][i], r["ga"][i], r["ph"][i])
        while j + 1 < n and (r["gp"][j + 1], r["gs"][j + 1], r["ga"][j + 1], r["ph"][j + 1]) == key:
            j += 1
        if key[3] in phases and key[0] != 0xFFFF and j - i > 60 and abs(int(key[0]) - int(r["pos"][i])) >= 20 \
                and abs(signed(r["sp"][i:i + 1], 15)[0]) == 0:
            t = (r["t"][i:j + 1] - r["t"][i]) / 1e6
            out.append({"capture": path.name.split("__")[0], "limit": limit, "phase": int(key[3]), "goal": int(key[0]), "speed": int(key[1]),
                        "acc": int(key[2]), "t": t, "pos": r["pos"][i:j + 1].astype(float),
                        "load": signed(r["ld"][i:j + 1], 10), "speed_meas": signed(r["sp"][i:j + 1], 15)})
        i = j + 1
    by_phase = {}
    for m in out:
        by_phase.setdefault(m["phase"], []).append(m)
    return [m for ms in by_phase.values() for m in ms[:per_capture]]


def simulate(group, params, horizon):
    """All moves of one drive at once; positions in ticks on the sample times."""
    k = len(group)
    q = np.array([m["pos"][0] for m in group]) * servo.TICK
    params = servo.Params(**{**params.__dict__, "torque_limit": group[0]["limit"]})
    s = servo.Servos(params, q)
    s.command(0.0, np.array([m["goal"] for m in group]) * servo.TICK, group[0]["speed"], group[0]["acc"])
    qd = np.zeros(k)
    inertia = params.tau_m_s / params.omega0
    f = params.friction_frac
    steps = int(horizon / DT) + 1
    trace = np.empty((steps, k))
    for n in range(steps):
        now = n * DT
        s.step(now, DT)
        drive = s.duty(q) - qd / params.omega0
        moving = np.abs(qd) > 1e-9
        stuck = ~moving & (np.abs(drive) <= f)
        friction = np.where(moving, f * np.sign(qd), f * np.sign(drive))
        acc = np.where(stuck, 0.0, (drive - friction) / inertia)
        new = qd + acc * DT
        crossed = moving & (np.sign(new) != np.sign(qd)) & (np.abs(drive) <= f)
        qd = np.where(stuck | crossed, 0.0, new)
        q = q + qd * DT
        trace[n] = q / servo.TICK
    return trace


def sampled(m, trace, i):
    return trace[np.minimum((m["t"] / DT).round().astype(int), len(trace) - 1), i]


def rmse(group, trace):
    return np.array([np.sqrt(np.mean((sampled(m, trace, i) - m["pos"]) ** 2)) for i, m in enumerate(group)])


def arrival_braking(t, pos):
    """st3215_bench_accel.py's arrival deceleration (last >= 80 % of the peak speed
    to the first <= 10 % after it), on the speed derived from the position."""
    sp = np.abs(np.gradient(pos, t))
    peak = sp.max()
    kl = int(np.max(np.nonzero(sp >= .8 * peak)[0]))
    ke = next((k for k in range(kl, len(sp)) if sp[k] <= .1 * peak), None)
    return (sp[kl] - sp[ke]) / (t[ke] - t[kl]) if ke is not None and t[ke] > t[kl] else np.nan


def main():
    bins, sums, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
    out_dir.mkdir(parents=True, exist_ok=True)
    expected = dict(reversed(line.split()) for line in sums.read_text().splitlines() if line.strip())
    captures = sorted(bins.glob("*.bin"))
    for path in captures:
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected.get(path.name):
            sys.exit(f"REFUSED: {path.name} does not match RAW_BIN_SHA256SUMS.txt")
    fit, check = [], []
    for path in captures:
        fit += moves(path, {8, 10})
        check += moves(path, {6, 7}, per_capture=4)
    groups = {}
    for m in fit:
        groups.setdefault((m["speed"], m["acc"], m["limit"]), []).append(m)
    horizon = max(m["t"][-1] for m in fit)
    bench = servo.Params(torque_limit=1.0)
    table = []
    for c in FF_GRID:
        for tau_m in TAU_M_GRID:
            for f in FRICTION_GRID:
                p = servo.Params(**{**bench.__dict__, "tau_m_s": tau_m, "friction_frac": f, "ff_frac": c})
                errors = {key: rmse(g, simulate(g, p, horizon)) for key, g in groups.items()}
                every = np.concatenate(list(errors.values()))
                table.append({"ff_frac": c, "tau_m_s": tau_m, "friction_frac": f, "rmse_median_ticks": float(np.median(every)),
                              "rmse_p90_ticks": float(np.percentile(every, 90)),
                              "rmse_median_by_drive": {f"speed {k[0]} acc {k[1]} limit {k[2]}": float(np.median(v)) for k, v in errors.items()}})
                print(f"c {c:.2f} tau_m {tau_m * 1e3:4.0f} ms  f {f:.2f}  RMSE median {table[-1]['rmse_median_ticks']:6.2f}  p90 {table[-1]['rmse_p90_ticks']:6.2f} ticks", flush=True)
    best = min(table, key=lambda r: r["rmse_median_ticks"])
    # The robot's units carry the normalized EEPROM profile (P 32, D 32, MaxTorque 1000); the six
    # captures taken before @NORMALIZE_MATDOG (P 16, D 0, MaxTorque 500) are another controller.
    for r in table:
        normalized = [v for k, v in r["rmse_median_by_drive"].items() if k.endswith("limit 1.0")]
        r["rmse_normalized_units_mean_ticks"] = float(np.mean(normalized))
    best_normalized = min(table, key=lambda r: r["rmse_normalized_units_mean_ticks"])
    p = servo.Params(**{**bench.__dict__, "tau_m_s": best["tau_m_s"], "friction_frac": best["friction_frac"], "ff_frac": best["ff_frac"]})
    full = {}
    for m in check:
        full.setdefault((m["speed"], m["acc"], m["limit"]), []).append(m)
    profile_limited = {}
    for key, g in groups.items():
        if g[0]["phase"] != 10:
            continue
        trace = simulate(g, p, max(m["t"][-1] for m in g))
        model_b = np.array([arrival_braking(m["t"], sampled(m, trace, i)) for i, m in enumerate(g)])
        bench_b = np.array([arrival_braking(m["t"], m["pos"]) for m in g])
        profile_limited[f"speed {key[0]} acc {key[1]} limit {key[2]}"] = {
            "moves": len(g), "register_step_s2": key[1] * 100,
            "model_median": float(np.nanmedian(model_b)), "bench_median": float(np.nanmedian(bench_b))}
    model_braking, bench_braking, errors = [], [], []
    for g in full.values():
        trace = simulate(g, p, max(m["t"][-1] for m in g))
        errors.append(rmse(g, trace))
        for i, m in enumerate(g):
            model_braking.append(arrival_braking(m["t"], sampled(m, trace, i)))
            bench_braking.append(arrival_braking(m["t"], m["pos"]))
    braking, measured = np.array(model_braking), np.array(bench_braking)
    braking, measured = braking[np.isfinite(braking)], measured[np.isfinite(measured)]
    result = {
        "data": {"captures": len(captures), "fitted_moves": len(fit), "drives": [list(k) for k in groups], "full_speed_moves": len(check)},
        "fixed": {"k_duty_per_tick": bench.k_duty_per_tick, "no_load_speed_step_s": bench.no_load_speed_step_s,
                  "deadband_ticks": bench.deadband_ticks, "latency_s": bench.latency_s, "torque_limit": 1.0},
        "grid": table,
        "best": {**best, "armature_kgm2_at_vendor_stall": p.armature, "friction_nm_at_vendor_stall": p.friction},
        "best_normalized_units": {k: best_normalized[k] for k in ("ff_frac", "tau_m_s", "friction_frac", "rmse_normalized_units_mean_ticks", "rmse_median_by_drive")},
        "arrival_braking_profile_limited_phase10": profile_limited,
        "within_2pct_of_best": [{k: r[k] for k in ("ff_frac", "tau_m_s", "friction_frac", "rmse_median_ticks")}
                                for r in table if r["rmse_median_ticks"] <= 1.02 * best["rmse_median_ticks"]],
        "full_speed_prediction": {
            "rmse_median_ticks": float(np.median(np.concatenate(errors))),
            "arrival_braking_step_s2_model": {"median": float(np.median(braking)), "p10": float(np.percentile(braking, 10))},
            "arrival_braking_step_s2_bench_same_moves": {"median": float(np.median(measured)), "p10": float(np.percentile(measured, 10))},
        },
    }
    (out_dir / "identify_bench.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["best"], indent=1))
    print("best on the normalized units:", json.dumps(result["best_normalized_units"]))
    print("within 2 % of the best RMSE:", json.dumps(result["within_2pct_of_best"]))
    print(json.dumps(profile_limited, indent=1))
    print(json.dumps(result["full_speed_prediction"], indent=1))


if __name__ == "__main__":
    main()
