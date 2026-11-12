#!/usr/bin/env python3
"""FAST 40 study: grid of the FAST law's own parameters at a fixed body speed, screened by
the firmware's own offline checks (unmodified binaries of walk_support_check and
walk_tilt_envelope_check built from the dev.7 tree), never by new criteria.

Screen (walk_support_check, 0.1 s each), the selection rule written in SimpleMotion.h for the
walk modes: static support and STOP capture point not below SLOW (1.70 / 1.41 mm), HIP margin
to kHeight >= 8.7 ticks, and the drive rule "RAM drive = law peak / 0.8" inside the FAST drive
{1950, 220} (register 85 target kAccCeilingTarget 220 unchanged): law peak speed <= 0.8 x 2.991
rad/s and acceleration <= 0.8 x 33.75 rad/s^2. Survivors then run walk_tilt_envelope_check in
its default check mode (kFastTilt rays, 8 release-grid phases, T2 chords, StopPlan at
kStopBrakeAcc): PASS needs 0 failures of every kind.

Usage: grid.py <bin dir> <out.jsonl> [--speed 40] [--jobs 8] [--tilt-top N] [--dims name=v1,v2,... ...]
"""
import concurrent.futures as cf
import itertools
import json
import re
import subprocess
import sys

SPEED = 40.0
STRIDES = (90, 95, 100, 105, 110, 115, 120)
DUTIES = (.78, .79, .80, .8125, .825)
RAMPS = (.20, .25, .30, .35, .40)
RAMP_ENDS = (.35, .45, .55, .60, .65)
HEIGHTS = (100, 105, 110, 115, 120)
LIFTS = (5, 6, 8)
SWAYX = (3.0,)  # kWalkSwayXM (shared by the three modes in dev.7)
COMS = (11.0,)  # kWalkComShiftXM (shared as well)
SWAYY = (1.0,)  # FAST swayYM (dev.7)
# Front-leg swing ramps (study tree only, STUDY_FRONT_RAMP[_END]; -1: the law's own ramps on every leg)
FRONT_RAMPS, FRONT_RAMP_ENDS = (-1.0,), (0.0,)
# SimpleMotion.h walk modes selection rule and the FAST drive
MIN_SUPPORT, MIN_CAPTURE, MIN_HIP = 1.70, 1.41, 8.7
VMAX, AMAX = .8 * 1950 * 6.283185307179586 / 4096, .8 * 220 * 100 * 6.283185307179586 / 4096

NUM = r"(-?[\d.]+)"


def parse_support(line):
    g = lambda pat: float(re.search(pat, line).group(1))
    return {
        "solve_fails": int(g(r"solve fails=(\d+)")),
        "support": g(r"static support min " + NUM),
        "zmp": g(r"\| ZMP min " + NUM),
        "capture": g(r"STOP capture min " + NUM),
        "stop_zmp": g(r"STOP ZMP min " + NUM),
        "vmax": g(r"joint vmax " + NUM),
        "amax": g(r"amax " + NUM),
        "stance_vmax": g(r"stance \(loaded\) vmax " + NUM),
        "hip": g(r"HIP " + NUM),
        "upper": g(r"UPPER " + NUM + " LOWER"),
        "lower": g(r"LOWER " + NUM + r" \|"),
        "support_verdict": line.rsplit("|", 1)[1].strip(),
    }


def args_of(c):
    return ["--mode", "2", "--stride", "2", "--stride-mm", str(c["stride"]), "--period", f"{c['stride'] / c['speed']:.6f}",
            "--duty", str(c["duty"]), "--ramp", str(c["ramp"]), "--ramp-end", str(c["ramp_end"]),
            "--height-mm", str(c["height"]), "--lift-mm", str(c["lift"]), "--swayx-mm", str(c["swayx"]), "--com-mm", str(c["com"]), "--swayy-mm", str(c["swayy"])]


def env_of(c):
    import os
    e = dict(os.environ)
    if c["fr"] >= 0:
        e.update(STUDY_FRONT_RAMP=str(c["fr"]), STUDY_FRONT_RAMP_END=str(c["fre"]))
    return e


def support(bindir, c):
    out = subprocess.run([f"{bindir}/walk_support_check", *args_of(c)], capture_output=True, text=True, env=env_of(c))
    if out.returncode not in (0, 1) or "mode=" not in out.stdout:
        return {**c, "error": f"rc {out.returncode}"}
    r = {**c, **parse_support(out.stdout.strip().splitlines()[-1])}
    r["screen"] = (r["solve_fails"] == 0 and r["support"] >= MIN_SUPPORT and r["capture"] >= MIN_CAPTURE and r["hip"] >= MIN_HIP
                   and r["vmax"] <= VMAX and r["amax"] <= AMAX)
    return r


def tilt(bindir, r):
    out = subprocess.run([f"{bindir}/walk_tilt_envelope_check", *args_of(r)], capture_output=True, text=True, env=env_of(r))
    lines = out.stdout.strip().splitlines()
    head = next((l for l in lines if l.startswith("mode=")), "")
    m = re.search(r"fail nominal/ik/box/stop=(\d+)/(\d+)/(\d+)/(\d+)", head)
    step = re.search(r"step_ticks/release nominal/compensated=" + NUM + "/" + NUM, head)
    box = re.search(r"box_margin_ticks nominal/compensated=" + NUM + "/" + NUM, head)
    r = dict(r)
    r["tilt_fail"] = [int(x) for x in m.groups()] if m else None
    r["tilt_pass"] = bool(m) and not any(int(x) for x in m.groups())
    r["step_ticks"] = [float(step.group(1)), float(step.group(2))] if step else None
    r["tilt_box_margin"] = [float(box.group(1)), float(box.group(2))] if box else None
    r["tilt_first"] = [l.strip() for l in lines if l.strip().startswith("first")]
    return r


def main():
    bindir, out = sys.argv[1], sys.argv[2]
    argv = sys.argv[3:]
    opt = lambda k, d: type(d)(argv[argv.index(k) + 1]) if k in argv else d
    speed, jobs, top = opt("--speed", SPEED), opt("--jobs", 8), opt("--tilt-top", 400)
    # --dims name=v1,v2,... replaces one dimension (stride, duty, ramp, ramp_end, height, lift, swayx, com, swayy, fr, fre)
    dims = {"stride": STRIDES, "duty": DUTIES, "ramp": RAMPS, "ramp_end": RAMP_ENDS, "height": HEIGHTS, "lift": LIFTS, "swayx": SWAYX, "com": COMS, "swayy": SWAYY,
            "fr": FRONT_RAMPS, "fre": FRONT_RAMP_ENDS}
    for i, a in enumerate(argv):
        if a == "--dims":
            k, v = argv[i + 1].split("=")
            dims[k] = tuple(float(x) for x in v.split(","))
    grid = [dict(speed=speed, **dict(zip(dims, vals))) for vals in itertools.product(*dims.values())]
    grid = [c for c in grid if c["ramp"] + c["ramp_end"] < 1 and (c["fr"] < 0 or c["fr"] + c["fre"] < 1)]
    with cf.ThreadPoolExecutor(jobs) as ex:
        rows = list(ex.map(lambda c: support(bindir, c), grid))
    ok = [r for r in rows if r.get("screen")]
    print(f"{len(rows)} configurations, {len(ok)} pass the support screen", file=sys.stderr)
    ok.sort(key=lambda r: -min(r["capture"] - MIN_CAPTURE, r["support"] - MIN_SUPPORT))
    with cf.ThreadPoolExecutor(jobs) as ex:
        tilted = {id(r): t for r, t in zip(ok[:top], ex.map(lambda r: tilt(bindir, r), ok[:top]))}
    with open(out, "w") as f:
        for r in rows:
            f.write(json.dumps(tilted.get(id(r), r)) + "\n")
    print(f"tilt check on {min(top, len(ok))}: {sum(t['tilt_pass'] for t in tilted.values())} PASS", file=sys.stderr)


if __name__ == "__main__":
    main()
