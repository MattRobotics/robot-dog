#!/usr/bin/env python3
"""FAST 40 phase 2 (2026-11-13): dev.7 / F40 / F40T_JC in the same twin scenarios (m6_gait JSON).

Walking scenarios: useful speed along the demand (mm/s; fast_yaw: deg/s) and its share of the law's,
heading drift (deg/cycle; straight walks), lateral body excursion p2p (mm), roll/pitch p2p (mrad), real swing
clearance p50 (mm) and dragging share p50, slip max (mm), UPPER/LOWER p95 joint
speed (step/s) and duty (per mille). (The twin's support margin of the loaded feet is left out: a
dragging swing foot counts as loaded and the polygon degenerates.) STOP scenarios: verified count, faults, slowest verification.

Usage: compare_p2.py <label>=<dir> ... [--sens <name>=<dir>,<dir> ...] [--markdown out.md]
"""
import json
import math
import pathlib
import sys

WALKS = ("fast_fwd", "fast_fwd_32", "fast_back", "fast_left", "fast_right", "fast_yaw", "fast_diag", "fast_diag_yaw", "fast_sweep", "fast_modes_cycle")


def load(d, label):
    out = {}
    for f in pathlib.Path(d).glob(f"*__{label}.json"):
        r = json.loads(f.read_text())
        out[r["scenario"]] = r
    return out


def useful(r):
    tr = r["travel"]
    if r["scenario"] == "fast_yaw":
        return tr["twin_yaw_deg_s"], 100 * tr["twin_yaw_deg_per_cycle"] / tr["law_yaw_deg_per_cycle"], "deg/s"
    law, tw, sp = tr["law_mm_per_cycle"], tr["twin_mm_per_cycle"], tr["twin_speed_mm_s"]
    n = math.hypot(*law)
    return (sp[0] * law[0] + sp[1] * law[1]) / n, 100 * (tw[0] * law[0] + tw[1] * law[1]) / n / n, "mm/s"


def walk_row(r):
    if "travel" not in r:
        return f"FAULT {r['faults']}" if r["faults"] else "no travel"
    v, eff, unit = useful(r)
    tr, ft, j = r["travel"], r["feet"], r["joints"]
    drift = "" if r["scenario"] not in ("fast_fwd", "fast_fwd_32", "fast_back") else f"drift {tr['twin_yaw_deg_per_cycle']:+.2f}°/c "
    rp = r.get("roll_pitch_p2p_mrad", [float("nan")] * 2)
    return (f"{v:6.2f} {unit} ({eff:3.0f} %) {drift}| lat p2p {tr['lateral_excursion_p2p_mm'][0]:4.1f} | r/p {rp[0]:4.0f}/{rp[1]:4.0f} | clear {ft['clearance_mm_p10_p50_p90'][1]:.2f} "
            f"drag {ft['dragging_share_p50_p90'][0]:.2f} slip {ft['slip_max_mm']:4.1f} | "
            f"U {j['UPPER']['speed_p95']:4.0f}/{j['UPPER']['duty_p95']:3.0f} L {j['LOWER']['speed_p95']:4.0f}/{j['LOWER']['duty_p95']:3.0f}" + (f" FAULT {r['faults']}" if r["faults"] else ""))


def stops(runs):
    s = [r for n, r in runs.items() if "stop" in r and r["stop"].get("t_s") is not None]
    ok = [r for r in s if r["stop"]["verified_s"] is not None and not r["stop"]["fault"]]
    bad = [(r["scenario"], r["stop"]["fault"]) for r in s if r not in ok]
    slow = max((r["stop"]["verified_s"] for r in ok), default=float("nan"))
    return f"{len(ok)}/{len(s)} verified, slowest {slow:.2f} s" + (f", faults {bad}" if bad else "")


def main():
    argv = sys.argv[1:]
    md = None
    if "--markdown" in argv:
        i = argv.index("--markdown")
        md = pathlib.Path(argv[i + 1])
        del argv[i:i + 2]
    sens = []
    while "--sens" in argv:
        i = argv.index("--sens")
        sens.append(argv[i + 1])
        del argv[i:i + 2]
    builds = [(a.split("=", 1)[0], a.split("=", 1)[1]) for a in argv]
    data = {lab: load(d, lab) for lab, d in builds}
    lines = []
    for sc in WALKS:
        lines.append(f"### {sc}")
        for lab, _ in builds:
            if sc in data[lab]:
                lines.append(f"- {lab:8s} {walk_row(data[lab][sc])}")
    lines.append("### STOP (every fast_* STOP scenario)")
    for lab, _ in builds:
        lines.append(f"- {lab:8s} {stops(data[lab])}")
    for spec in sens:  # name=label:dir,label:dir
        name, rest = spec.split("=", 1)
        lines.append(f"### perturbation {name}")
        for part in rest.split(","):
            lab, d = part.split(":", 1)
            runs = load(d, lab)
            for sc in WALKS:
                if sc in runs:
                    lines.append(f"- {lab:8s} {sc:12s} {walk_row(runs[sc])}")
            if any("stop" in r for r in runs.values()):
                lines.append(f"- {lab:8s} STOP {stops(runs)}")
    text = "\n".join(lines)
    print(text)
    if md:
        md.write_text(text + "\n")


if __name__ == "__main__":
    main()
