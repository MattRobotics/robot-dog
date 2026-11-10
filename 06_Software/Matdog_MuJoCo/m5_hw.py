#!/usr/bin/env python3
"""Hardware logs of a MATDOG session for the twin comparison (M5), read only.

Two sources carry the same /api/motion/status snapshot: the dashboard trace export
(`samples[].status`, one every ~0.35 s, with the commands the page sent) and the
session recorder's http.jsonl (1 Hz, /status every 5 s). In a snapshot each joint
holds its last FAST read (raw, speed, load), taken `age_ms` before `now_ms`: two
servos per 20 ms release, so each servo every 120 ms (SimpleMotion.cpp pollNext_).
`commanded_raw` is the frame being written at `now_ms`; the IMU quaternion has its
own `sampled_at_ms`. The 12 readings of a snapshot are not simultaneous and a joint
is never sampled faster than the snapshots: rates() measures it per joint.

Usage: m5_hw.py <session dir | dashboard trace .json | http.jsonl> [...] [--out file.json]
"""
import json
import pathlib
import re
import sys

import numpy as np

SLOT_NAMES = ("LF_HIP", "LF_UPPER", "LF_LOWER", "RF_HIP", "RF_UPPER", "RF_LOWER",
              "RH_HIP", "RH_UPPER", "RH_LOWER", "LH_HIP", "LH_UPPER", "LH_LOWER")
GROUPS = {"HIP": [0, 3, 6, 9], "UPPER": [1, 4, 7, 10], "LOWER": [2, 5, 8, 11]}
CONTROLLER_READ_MS = 120  # each servo's FAST read period in the controller (2 of 12 per 20 ms release)


def signed(v, bit):  # ST sign-magnitude registers: load bit 10, speed bit 15
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v & ((1 << bit) - 1)


def snapshot(s, host, source):
    j = s["joints"][:12]
    tel = s.get("telemetry") or {}
    mode = s.get("walk_mode") or {}
    return {"now_ms": float(s["now_ms"]), "host": host, "source": source,
            "state": s["state"], "phase": s["phase"], "reason": s["reason"], "walk_cycles": float(s.get("walk_cycles") or 0.0),
            "mode": mode.get("active"), "target": mode.get("target"), "selected": mode.get("selected"), "servo_read": mode.get("servo_read"),
            "raw": np.array([x["raw"] for x in j], float), "cmd": np.array([x["commanded_raw"] for x in j], float),
            "load": np.array([signed(x["load"], 10) for x in j], float), "speed": np.array([signed(x["speed"], 15) for x in j], float),
            "age_ms": np.array([x["age_ms"] for x in j], float), "valid": all(x["valid"] for x in j),
            "temp": [x["temperature"] for x in j], "voltage_dv": [x["voltage_dv"] for x in j],
            "quat": tel.get("quaternion"), "quat_ms": tel.get("sampled_at_ms"),
            "imu": s.get("imu_correction") or {}, "vel": s.get("walk_velocity") or {},
            "height_mm": s.get("requested_height_mm"), "height_est_mm": s.get("height_estimate_mm"),
            "torque": bool(s.get("torque_active")), "fault": bool(s.get("fault")), "stop": s.get("stop") or {},
            "command": s.get("active_command") or {}, "missed": s.get("missed_releases"), "overruns": s.get("overrun_releases")}


def read(path):
    """(snapshots, dashboard commands, /status records) of one trace export or http.jsonl."""
    path = pathlib.Path(path)
    if path.suffix == ".jsonl":
        snaps, status = [], []
        for line in path.read_text().splitlines():
            r = json.loads(line) if line.strip() else {}
            if r.get("code") != 200 or "s" not in r:
                continue
            if r["path"] == "/status":
                status.append(dict(r["s"], host_utc=r["host_utc"]))
            elif "joints" in r["s"]:
                snaps.append(snapshot(r["s"], r["host_utc"], path.name))
        return snaps, [], status
    d = json.loads(path.read_text())
    snaps = [snapshot(x["status"], x.get("received_ms"), path.name) for x in d["samples"] if "joints" in x.get("status", {})]
    for c in d.get("commands", []):
        c["source"] = path.name
    return snaps, d.get("commands", []), []


def boots(snaps):
    """Split a time-ordered stream where now_ms goes backwards (a controller reset)."""
    out, cur = [], []
    for s in snaps:
        if cur and s["now_ms"] < cur[-1]["now_ms"] - 1000:
            out.append(cur)
            cur = []
        cur.append(s)
    return out + ([cur] if cur else [])


def merge(snaps):
    """One snapshot per controller instant (a trace and http.jsonl overlap), ordered."""
    seen, out = set(), []
    for s in sorted(snaps, key=lambda s: (s["now_ms"], s["source"].endswith(".jsonl"))):
        if s["now_ms"] not in seen:
            seen.add(s["now_ms"])
            out.append(s)
    return out


def label(s):
    if s["state"] == "WALKING":
        a, b = s["mode"], s["target"]
        return f"WALKING {a}" if a == b or not b else f"CHANGE {a}->{b}"
    return f"{s['state']} {s['phase']}"


def rates(snaps, where=lambda s: True):
    """Per joint: distinct FAST readings (now_ms - age_ms) and their spacing over the selected snapshots,
    counted inside each uninterrupted run of them (a gap in the selection is not a sampling interval)."""
    idx = [i for i, s in enumerate(snaps) if where(s)]
    runs = [r for r in np.split(np.array(idx), np.where(np.diff(idx) > 1)[0] + 1) if len(r) >= 2] if idx else []
    if not runs:
        return None
    readings, intervals, gaps, span_s = np.zeros(12), [[] for _ in range(12)], [], 0.0
    for run in runs:
        sel = [snaps[i] for i in run]
        now = np.array([s["now_ms"] for s in sel])
        stamps = now[:, None] - np.array([s["age_ms"] for s in sel])
        span_s += (now[-1] - now[0]) / 1e3
        gaps += list(np.diff(now))
        for i in range(12):
            u = np.unique(np.round(stamps[:, i]))
            readings[i] += len(u) - 1  # readings after the first one of the run, over its span
            intervals[i] += list(np.diff(u))
    age = np.array([snaps[i]["age_ms"] for run in runs for i in run])
    gaps = np.array(gaps)
    per_joint = [{"readings": int(readings[i] + len(runs)), "hz": float(readings[i] / span_s) if span_s else None,
                  "interval_ms_p50": float(np.median(intervals[i])) if intervals[i] else None,
                  "interval_ms_max": float(np.max(intervals[i])) if intervals[i] else None} for i in range(12)]
    return {"snapshots": int(sum(len(r) for r in runs)), "runs": len(runs), "span_s": float(span_s),
            "snapshot_interval_ms_p50_p95_max": [float(np.median(gaps)), float(np.percentile(gaps, 95)), float(gaps.max())],
            "age_ms_p50_p95_max": [float(np.median(age)), float(np.percentile(age, 95)), float(age.max())],
            "readings_hz_min_max": [min(p["hz"] for p in per_joint), max(p["hz"] for p in per_joint)],
            "controller_hz": 1e3 / CONTROLLER_READ_MS, "per_joint": dict(zip(SLOT_NAMES, per_joint))}


WALK_PRESET = re.compile(r"^WALK (SLOW|STANDARD|FAST)?\s*(\d+)?")
JOY = re.compile(r"JOYSTICK (avanti|indietro) ([\d.]+) mm/s, (destra|sinistra) ([\d.]+) mm/s")


def command_kind(c):
    """(kind, argument) of a dashboard command: STAND/PRONE/STOP/SELECT/WALK/WALKSTOP/DRIVE, or None."""
    t, lab = c.get("type"), c.get("label", "")
    if t == "POSTURE.STAND":
        return "STAND", float(lab.split()[1]) / 1000 if lab.startswith("STAND ") else None
    if t == "POSTURE.LIE_DOWN":
        return "PRONE", None
    if t == "MOTION.STOP":
        return "STOP", None
    if t == "GAIT.SELECT":
        return "SELECT", lab.split()[-1]
    if t == "BASE_VELOCITY":
        if lab in ("WALK STOP", "JOYSTICK neutro", "JOYSTICK rilascio"):
            return "WALKSTOP", None
        if (m := JOY.search(lab)):
            vx = float(m[2]) / 1000 * (1 if m[1] == "avanti" else -1)
            vy = float(m[4]) / 1000 * (-1 if m[3] == "destra" else 1)
            return "DRIVE", (vx, vy)
        if lab.startswith("WALK"):
            return "WALK", lab
    return None, None


def commands(trace_commands, snaps):
    """Accepted commands in controller time: [{t_ms, kind, arg, label, exact}].

    A dashboard command takes the Controller's `accepted_ms` when a snapshot saw it
    active, else `issued_at_ms` (the page's estimate of the Controller clock). GAIT.SELECT
    never becomes the active command: its time is the page's. Without a dashboard
    trace the active_command changes of the snapshots give the time and the kind is
    read from what followed (`exact` False for a mode selection: +-1 snapshot)."""
    out = []
    if trace_commands:
        accepted = {}
        for s in snaps:
            a = s["command"]
            if a.get("id") and a.get("accepted_ms"):
                accepted[a["id"]] = a["accepted_ms"]
        for c in trace_commands:
            ack = c.get("ack") or {}
            if c.get("http") != 200 or ack.get("outcome") != "ACCEPTED":
                continue
            kind, arg = command_kind(c)
            if kind is None:
                continue
            obs = (c.get("observed") or {}).get("accepted_ms")
            t = accepted.get(c["id"], obs) or c.get("issued_at_ms")
            out.append({"t_ms": float(t), "kind": kind, "arg": arg, "label": c.get("label"), "exact": c["id"] in accepted or bool(obs),
                        "ack_reason": ack.get("reason")})
        return sorted(out, key=lambda c: c["t_ms"])
    prev = None
    for k, s in enumerate(snaps):
        a = s["command"]
        if a.get("accepted_ms") and (prev is None or a.get("sequence") != prev.get("sequence")):
            nxt = snaps[min(k + 1, len(snaps) - 1)]
            kind = arg = None
            if a.get("reason", "").startswith("STOP_REQUESTED") or s["state"] == "STOPPED" or s["stop"].get("active"):
                kind = "STOP"
            elif s["vel"].get("joystick") and (s["vel"].get("vx_m_s") or nxt["vel"].get("vx_m_s")):
                kind, arg = "DRIVE", (s["vel"].get("vx_m_s") or nxt["vel"].get("vx_m_s"), s["vel"].get("vy_m_s") or 0.0)
            elif s["reason"] in ("WALK_STOPPING", "WALK_SETTLING") or s["phase"] == "WALK_SETTLE":
                kind = "WALKSTOP"
            elif s["state"] == "WALKING" or nxt["state"] == "WALKING":
                kind, arg = "WALK", s["selected"]
            elif s["phase"].startswith("PRONE"):
                kind = "PRONE"
            elif s["state"] in ("MOVING", "STANDING"):
                kind, arg = "STAND", (s["height_mm"] or 0) / 1000
            out.append({"t_ms": float(a["accepted_ms"]), "kind": kind, "arg": arg, "label": None, "exact": True, "ack_reason": a.get("reason")})
        prev = a if a.get("accepted_ms") else prev
    for a, b in zip(snaps, snaps[1:]):  # a selection between two snapshots
        if a["selected"] and b["selected"] and b["selected"] != a["selected"]:
            out.append({"t_ms": (a["now_ms"] + b["now_ms"]) / 2, "kind": "SELECT", "arg": b["selected"], "label": None, "exact": False,
                        "uncertainty_ms": (b["now_ms"] - a["now_ms"]) / 2, "ack_reason": None})
    return sorted(out, key=lambda c: c["t_ms"])


def episodes(snaps, cmds):
    """Replayable stretches (t_from_s, t_to_s, first command): a WALK, joystick DRIVE or SELECT given from
    STANDING HOLD, up to the next STAND or PRONE command, the next STANDING HOLD after the walk ended, a FAULT
    (+3 s) or the end of the data."""
    out, k = [], 0
    while k < len(cmds):
        c = cmds[k]
        before = [s for s in snaps if s["now_ms"] <= c["t_ms"]]
        if c["kind"] in ("WALK", "DRIVE", "SELECT") and before and before[-1]["state"] == "STANDING" and before[-1]["torque"]:
            nxt = next((d for d in cmds[k + 1:] if d["kind"] in ("STAND", "PRONE")), None)
            end = nxt["t_ms"] - 1 if nxt else snaps[-1]["now_ms"]
            later = [s for s in snaps if c["t_ms"] < s["now_ms"] <= end]
            walked = False
            for s in later:
                walked |= s["state"] == "WALKING"
                if s["state"] == "FAULT":
                    end = min(end, s["now_ms"] + 3000)
                    break
                if walked and s["state"] == "STANDING" and s["phase"] == "HOLD":
                    end = min(end, s["now_ms"] + 2000)
                    break
            out.append((c["t_ms"] / 1e3 - 0.5, end / 1e3, c["kind"]))
            while k < len(cmds) and cmds[k]["t_ms"] <= end:
                k += 1
            continue
        k += 1
    return out


def transitions(snaps):
    """(now_ms, state, phase, reason, mode) at each change, with the previous sample's time (the change is between)."""
    out, prev = [], None
    for s in snaps:
        key = (s["state"], s["phase"], s["reason"], s["mode"], s["target"])
        if key != (prev and (prev["state"], prev["phase"], prev["reason"], prev["mode"], prev["target"])):
            out.append({"now_ms": s["now_ms"], "after_ms": prev["now_ms"] if prev else None, "state": s["state"], "phase": s["phase"],
                        "reason": s["reason"], "mode": s["mode"], "target": s["target"]})
        prev = s
    return out


def loaded_legs(snaps):
    """Mean |load| of UPPER+LOWER in STANDING HOLD: the legs carry the body (ground) or hang (fixture)."""
    hold = [s for s in snaps if s["state"] == "STANDING" and s["phase"] == "HOLD" and s["torque"]]
    if not hold:
        return None
    ul = np.abs(np.array([s["load"][GROUPS["UPPER"] + GROUPS["LOWER"]] for s in hold]))
    return {"hold_snapshots": len(hold), "upper_lower_abs_load_mean_permille": float(ul.mean()), "max": float(ul.max())}


def provenance(root):
    """Identity, power and conditions of a session folder (READY_REPORT.md layout), as found."""
    root = pathlib.Path(root)
    p = {"folder": str(root)}
    for name in ("h0/preflight_h0.json", "pre/preflight.json"):
        f = root / name
        if f.exists():
            d = json.loads(f.read_text())
            st = d.get("status", {})
            p[name] = {k: st.get(k) for k in ("firmware_version", "build_id", "ota_running_build_id", "system_health", "bms_pack_v", "bms_soc_percent", "uptime_ms")}
            p[name]["utc"] = d.get("utc")
    flash = root / "flash/flash.log"
    if flash.exists():
        text = flash.read_text(errors="replace")
        p["flash"] = {"verified": "Verification successful" in text, "application_only_flash_pass": "APPLICATION_ONLY_FLASH = PASS" in text,
                      "refuse": "REFUSE" in text, "sha256": sorted(set(re.findall(r"\b[0-9a-f]{64}\b", text)))[:4]}
    serial = sorted(root.glob("*/serial.log"))
    p["serial"] = {}
    for f in serial:
        log = f.read_text(errors="replace").splitlines()
        p["serial"][f.parent.name] = {
            "lines": len(log), "first_utc": log[0][:23] if log else None, "last_utc": log[-1][:23] if log else None,
            "boots": sum("SYSTEM_BOOT_COMPLETE" in l for l in log),
            "build_ids": sorted(set(re.findall(r"build_id[=: ]+([0-9a-f]{12})", "\n".join(log)))),
            "panic": sum(bool(re.search(r"Guru Meditation|abort\(\)|panic|Backtrace:", l)) for l in log),
            "acc_ceiling": [l[:23] + " " + m[0] for l in log if (m := re.search(r"MOTION_ACC_CEILING verified=\d+ target=\d+", l))],
            "reg85": {m[1]: int(m[2]) for l in log if (m := re.search(r"SERVO_ACC id=(\d+) reg=85 status=OK value=(\d+)", l))},
            "motion_diag_exit": [l[:23] + " " + l[l.index("MOTION_DIAG exit"):][:160] for l in log if "MOTION_DIAG exit" in l][-4:]}
    status = []
    for f in sorted(root.glob("*/http.jsonl")):
        status += read(f)[2]
    if status:
        v = [float(s["bms_pack_v"]) for s in status if s.get("bms_pack_v")]
        cur = [float(s["bms_current_a"]) for s in status if s.get("bms_current_a")]
        p["status"] = {"records": len(status), "identity": sorted({(s.get("firmware_version"), s.get("build_id"), s.get("ota_running_build_id")) for s in status}),
                       "bms_pack_v_min_max": [min(v), max(v)] if v else None, "bms_current_a_min_max": [min(cur), max(cur)] if cur else None,
                       "bms_soc_first_last": [status[0].get("bms_soc_percent"), status[-1].get("bms_soc_percent")],
                       "charger_or_state": sorted({s.get("bms_state") for s in status if s.get("bms_state")}),
                       "uptime_monotonic": all(int(b.get("uptime_ms", 0)) >= int(a.get("uptime_ms", 0)) for a, b in zip(status, status[1:]))}
    p["files"] = sorted(str(f.relative_to(root)) for f in root.rglob("*") if f.is_file() and f.suffix in (".json", ".jsonl", ".log", ".txt", ".sha256", ".md"))
    return p


def load(paths):
    """Snapshots (merged, last boot of each stream kept separately) and commands of the given files/folders."""
    files = []
    for p in map(pathlib.Path, paths):
        files += sorted(p.rglob("http.jsonl")) + sorted(p.rglob("*trace*.json")) if p.is_dir() else [p]
    snaps, cmds = [], []
    for f in files:
        s, c, _ = read(f)
        snaps += s
        cmds += c
    return files, merge(snaps), cmds


def main():
    args = [a for a in sys.argv[1:] if a != "--out"]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None
    args = [a for a in args if a != out]
    files, snaps, cmds = load(args)
    report = {"files": [str(f) for f in files], "provenance": {}, "boots": []}
    for a in args:
        if pathlib.Path(a).is_dir():
            report["provenance"][a] = provenance(a)
    for b in boots(snaps):
        walking = rates(b, lambda s: s["state"] == "WALKING")
        report["boots"].append({
            "now_ms": [b[0]["now_ms"], b[-1]["now_ms"]], "snapshots": len(b), "rates_all": rates(b), "rates_walking": walking,
            "labels": {k: sum(label(s) == k for s in b) for k in dict.fromkeys(label(s) for s in b)},
            "loaded_legs": loaded_legs(b), "commands": commands([c for c in cmds], b), "transitions": transitions(b)})
        r = report["boots"][-1]
        r["episodes"] = episodes(b, r["commands"])
        print(f"boot now {b[0]['now_ms'] / 1e3:.1f}-{b[-1]['now_ms'] / 1e3:.1f} s: {len(b)} snapshots, labels {r['labels']}")
        for name, rr in (("all", r["rates_all"]), ("WALKING", walking)):
            if rr:
                nominal = 1e3 / max(p["interval_ms_p50"] for p in rr["per_joint"].values())
                print(f"  {name:8s} snapshot every {rr['snapshot_interval_ms_p50_p95_max'][0]:.0f} ms (p95 {rr['snapshot_interval_ms_p50_p95_max'][1]:.0f}, "
                      f"longest gap {rr['snapshot_interval_ms_p50_p95_max'][2] / 1e3:.1f} s), age p50 {rr['age_ms_p50_p95_max'][0]:.0f} max {rr['age_ms_p50_p95_max'][2]:.0f} ms; "
                      f"per joint {nominal:.2f} Hz while recording, {rr['readings_hz_min_max'][0]:.2f} Hz on average (controller {rr['controller_hz']:.2f} Hz)")
        print(f"  legs: {r['loaded_legs']}")
        for e in r["episodes"]:
            print(f"  episode --from {e[0]:.2f} --to {e[1]:.2f}  ({e[2]} from STANDING)")
        for c in r["commands"]:
            print(f"  cmd {c['t_ms'] / 1e3:9.3f} s {c['kind']:8s} {c['arg']} {'' if c['exact'] else '(+-' + format(c.get('uncertainty_ms', 0), '.0f') + ' ms)'} {c['label'] or ''}")
    for a, p in report["provenance"].items():
        print(json.dumps(p, indent=1, default=str)[:3000])
    if out:
        pathlib.Path(out).write_text(json.dumps(report, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


if __name__ == "__main__":
    main()
