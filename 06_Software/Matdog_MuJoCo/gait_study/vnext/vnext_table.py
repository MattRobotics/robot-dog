#!/usr/bin/env python3
"""Locomotion vNext: one comparison table of two scenario sets (vnext_eval.py / m6_gait.py JSON reports).

Per scenario: achieved travel per cycle against the law (x, y), achieved speed, yaw per cycle, lateral excursion,
effective clearance (vnext_eval, front / hind peak median), share of the swing with the foot loaded, weight on
law-swing feet, samples with no loaded foot, slip, STOP (verified time or fault), faults and refusals.

Usage: vnext_table.py <dir A> <label A> <dir B> <label B> [scenario ...]   (default: every scenario of dir A)
"""
import json
import pathlib
import sys


def load(d, name):
    f = sorted(pathlib.Path(d).glob(f"{name}__*.json"))
    return json.loads(f[0].read_text()) if f else None


def cells(r):
    if r is None:
        return "-"
    out = []
    if r.get("faults"):
        out.append(f"FAULT {r['faults'][0][1]}")
    if "travel" in r:
        t = r["travel"]
        law = t["law_mm_per_cycle"]
        tw = t["twin_mm_per_cycle"]
        lead = 0 if abs(law[0]) >= abs(law[1]) else 1
        pct = 100 * tw[lead] / law[lead] if law[lead] else float("nan")
        out.append(f"{tw[0]:6.1f},{tw[1]:5.1f}/{law[0]:6.1f},{law[1]:5.1f} mm ({pct:4.0f}%) {t['twin_speed_mm_s'][0]:5.1f},{t['twin_speed_mm_s'][1]:5.1f} mm/s "
                   f"yaw {t['twin_yaw_deg_per_cycle']:5.2f}/{t['law_yaw_deg_per_cycle']:5.2f} lat {t['lateral_excursion_p2p_mm'][0]:4.1f}")
    if "clearance" in r and r["clearance"]["by_group"]:
        g = r["clearance"]["by_group"]
        out.append("clr " + "/".join(f"{g[k]['gap_at_law_peak_mm']:.1f}" for k in ("front", "hind") if k in g)
                   + " load " + "/".join(f"{g[k]['loaded_share']:.2f}" for k in ("front", "hind") if k in g))
    if "feet" in r:
        f = r["feet"]
        extra = f" swingW {f['weight_on_law_swing_feet_mean']:.2f} noload {f['no_loaded_foot_share']:.3f}" if "weight_on_law_swing_feet_mean" in f else ""
        out.append(f"slip {f['slip_max_mm']:4.1f}{extra}")
    if "stop" in r:
        s = r["stop"]
        out.append(f"STOP {'FAULT ' + str(s['fault']) if s.get('fault') else ('ok %.2fs' % s['verified_s'] if s.get('verified_s') is not None else str(s))}")
    if r.get("refusals"):
        out.append(f"refusals {len(r['refusals'])}")
    return " | ".join(out)


def main():
    a, la, b, lb = sys.argv[1:5]
    names = sys.argv[5:] or sorted({p.name.split("__")[0] for p in pathlib.Path(a).glob("*__*.json")})
    for n in names:
        print(f"{n}\n  {la:10s} {cells(load(a, n))}\n  {lb:10s} {cells(load(b, n))}")


if __name__ == "__main__":
    main()
