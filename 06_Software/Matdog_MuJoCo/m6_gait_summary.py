#!/usr/bin/env python3
"""M6 gait study table: every scenario, the dev.7 build and each candidate, from m6_gait JSON.

Columns (twin physics, never the commanded law): useful travel per cycle along the demand
(mm; yaw scenarios: deg) and its efficiency against the law, lateral body excursion p2p (mm),
real swing clearance p50 (mm) and dragging share p50, support margin p05/min (mm), slip (mm),
hip margin from kHeight (ticks, fore/aft walks only), p95 |error| / |speed| / |duty| of UPPER and
LOWER, and for STOP scenarios the verification time or the fault.

Usage: m6_gait_summary.py <gait run dir> [--ref dev7] [--markdown out.md]
"""
import collections
import json
import math
import pathlib
import sys

FOREAFT = ("fwd_", "back_", "fast_fwd", "fast_back")


def is_yaw(name):
    """Pure rotation scenarios (yaw_*, fast_yaw): travel in degrees."""
    return name.startswith("yaw") or name in ("fast_yaw",)


def load(d):
    out = collections.defaultdict(dict)
    for f in sorted(pathlib.Path(d).glob("*.json")):
        r = json.loads(f.read_text())
        out[r["scenario"]][r["label"]] = r
    return out


def useful(r):
    """(useful travel per cycle, law, unit) along the demanded direction."""
    tr = r.get("travel")
    if not tr:
        return None
    if is_yaw(r["scenario"]):
        return abs(tr["twin_yaw_deg_per_cycle"]), abs(tr["law_yaw_deg_per_cycle"]), "deg"
    lx, ly = tr["law_mm_per_cycle"]
    n = math.hypot(lx, ly) or 1.0
    tx, ty = tr["twin_mm_per_cycle"]
    return (tx * lx + ty * ly) / n, n, "mm"


def row(r):
    cells = {}
    u = useful(r)
    if u:
        cells["travel"] = f"{u[0]:.1f}/{u[1]:.1f} {u[2]} ({100 * u[0] / u[1]:.0f}%)" if u[1] else "-"
        # per second (laws of different periods): the same useful travel over the law period
        period = r.get("fast_law", {}).get("period") if r["scenario"].startswith("fast_") and r.get("mode") == "FAST" else None
        if period:
            cells["travel"] += f" = {u[0] / period:.2f} {u[2]}/s"
        cells["eff"] = u[0] / u[1] if u[1] else None
        cells["lat"] = r["travel"]["lateral_excursion_p2p_mm"][0]
    f = r.get("feet")
    if f and f["clearance_mm_p10_p50_p90"]:
        cells["clear"] = f["clearance_mm_p10_p50_p90"][1]
        cells["drag"] = f["dragging_share_p50_p90"][0]
        cells["support"] = f["support_margin_mm_p05_min"]
        cells["slip"] = f["slip_max_mm"]
        cells["hip"] = r.get("hip_margin_kheight_min_ticks") if r["scenario"].startswith(FOREAFT) else None
        j = r["joints"]
        cells["joints"] = {g: (j[g]["err_p95"], j[g]["speed_p95"], j[g]["duty_p95"]) for g in ("UPPER", "LOWER")}
    if "stop" in r:
        s = r["stop"]
        cells["stop"] = f"FAULT {s['fault']} +{s['fault_s']:.2f}s" if s.get("fault") else (f"verified +{s['verified_s']:.2f}s" if s.get("verified_s") is not None else "not verified")
    cells["fault"] = r["faults"][0][1] if r["faults"] and "stop" not in r else None
    pre = [x for x in r["refusals"] if not str(x[2]).startswith("OK STOP")]
    cells["refused"] = sorted({x[2] for x in pre})
    return cells


def fmt(c):
    parts = []
    if "travel" in c:
        parts.append(f"travel {c['travel']} lat p2p {c['lat']:.1f}")
    if "clear" in c:
        parts.append(f"clear {c['clear']:.2f} drag {c['drag']:.2f} support {c['support'][0]:.1f}/{c['support'][1]:.1f} slip {c['slip']:.1f}"
                     + (f" hip {c['hip']:.1f}" if c['hip'] is not None else ""))
        parts.append("U " + "/".join(f"{v:.0f}" for v in c["joints"]["UPPER"]) + " L " + "/".join(f"{v:.0f}" for v in c["joints"]["LOWER"]))
    if "stop" in c:
        parts.append(f"STOP {c['stop']}")
    if c["fault"]:
        parts.append(f"FAULT {c['fault']}")
    if c["refused"]:
        parts.append(f"refused {c['refused']}")
    return " | ".join(parts)


def main():
    argv = sys.argv[1:]
    ref = "dev7"
    md = None
    if "--ref" in argv:
        i = argv.index("--ref"); ref = argv[i + 1]; del argv[i:i + 2]
    if "--markdown" in argv:
        i = argv.index("--markdown"); md = argv[i + 1]; del argv[i:i + 2]
    data = load(argv[0])
    labels = sorted({l for s in data.values() for l in s}, key=lambda l: (l != ref, l))
    lines = []
    for scen in sorted(data):
        print(f"\n## {scen}")
        for lab in labels:
            if lab in data[scen]:
                c = row(data[scen][lab])
                print(f"  {lab:28s} {fmt(c)}")
                lines.append((scen, lab, c))
    # aggregate per label over walking scenarios: travel efficiency, clearance, drag, support p05, slip
    print("\n## aggregate over walking scenarios (median)")
    agg = collections.defaultdict(lambda: collections.defaultdict(list))
    for scen, lab, c in lines:
        for k in ("eff", "clear", "drag", "slip", "lat"):
            if c.get(k) is not None:
                agg[lab][k].append(c[k])
        if c.get("support"):
            agg[lab]["support_p05"].append(c["support"][0])
            agg[lab]["support_min"].append(c["support"][1])
        if c.get("hip") is not None:
            agg[lab]["hip"].append(c["hip"])
    for lab in labels:
        a = agg[lab]
        med = lambda k: sorted(a[k])[len(a[k]) // 2] if a[k] else float("nan")
        print(f"  {lab:28s} travel eff {med('eff'):.2f} clear {med('clear'):.2f} drag {med('drag'):.2f} slip {med('slip'):.1f} "
              f"support p05 {med('support_p05'):.1f} min-of-min {min(a['support_min']) if a['support_min'] else float('nan'):.1f} "
              f"hip min {min(a['hip']) if a['hip'] else float('nan'):.1f}")
    if md:
        with open(md, "w") as f:
            f.write("| scenario | build | result |\n|---|---|---|\n")
            for scen, lab, c in lines:
                f.write(f"| {scen} | {lab} | {fmt(c)} |\n")


if __name__ == "__main__":
    main()
