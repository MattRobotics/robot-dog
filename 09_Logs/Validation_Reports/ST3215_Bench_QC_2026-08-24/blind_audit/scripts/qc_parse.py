"""FASE A — integrity audit + move segmentation cross-validated against the log."""
import csv, json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

RE_BEGIN = re.compile(
    r"MEASURE_BEGIN phase=(\d+) start=(-?\d+) target=(-?\d+) "
    r"speed=(\d+) acc=(\d+) window_ms=(\d+)")
RE_END = re.compile(
    r"MEASURE_END phase=(\d+) outcome=(\w+) start=(-?\d+) target=(-?\d+) "
    r"final=(-?\d+) error=(-?\d+) range=(-?\d+)\.\.(-?\d+) peak_speed=(-?\d+) "
    r"peak_load=(-?\d+) peak_current=(-?\d+) first_motion_ms=(-?[\d.]+) "
    r"elapsed_ms=([\d.]+)")
RE_PHASE = re.compile(r"^PHASE (\d+): n=(\d+) ")
RE_MINRES = re.compile(
    r"MIN_RESULT zone=(\d+) dir=([+-]\d+) cmd_speed=(\d+) travel=(-?\d+) "
    r"first_motion_ms=(-?[\d.]+) peak_present_speed=(-?\d+) outcome=(\w+)")


def parse_log(path):
    txt = Path(path).read_text(errors="replace")
    ends = [m.groups() for m in RE_END.finditer(txt)]
    begins = [m.groups() for m in RE_BEGIN.finditer(txt)]
    phases = {int(a): int(b) for a, b in RE_PHASE.findall(txt)}
    mins = [m.groups() for m in RE_MINRES.finditer(txt)]
    summ = {}
    for k in ("THERMAL_TRANSIENTS", "THERMAL_CONFIRMED", "PERFORMANCE_EVENTS",
              "PROTECTIVE_STOPS", "NO_MOTION_EVENTS", "NO_PROGRESS_EVENTS",
              "SETTLED_RESIDUALS", "WINDOW_END_EVENTS", "SKIPPED_TESTS",
              "MIN_ATTEMPTS", "MIN_RESPONSES", "RAW_SAMPLES"):
        m = re.search(rf"^{k}\s*:\s*(-?\d+)", txt, re.M)
        summ[k] = int(m.group(1)) if m else None
    m = re.search(r"^OBSERVED_ENVELOPE\s*:\s*(-?\d+) \.\. (-?\d+)", txt, re.M)
    summ["ENV_LO"], summ["ENV_HI"] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    summ["CHECKSUM"] = "PASS" if re.search(r"^CHECKSUM\s*:\s*PASS", txt, re.M) else "NOT_PASS"
    summ["EXECUTION"] = (re.search(r"^QC_EXECUTION\s*:\s*(\w+)", txt, re.M) or [None, "?"])[1]
    summ["START_TEMP"] = int(re.search(r"^START TEMP\s*:\s*(\d+)", txt, re.M).group(1))
    summ["START_VOLT"] = int(re.search(r"^START VOLTAGE\s*:\s*(\d+)", txt, re.M).group(1))
    return dict(begins=begins, ends=ends, phase_counts=phases, mins=mins, summary=summ)


def main():
    mapping = q.load_mapping()["mapping"]
    manifest_sha = {}
    for line in (q.CAMPAIGN / "manifest.tsv").read_text().splitlines()[1:]:
        f = line.split("\t")
        manifest_sha[f[8]] = f[4]          # sha256 -> execution

    integrity, moverows, minrows = [], [], []
    for sc in sorted(mapping):
        d = q.load_raw(sc)
        h = d["header"]
        lg = parse_log(mapping[sc]["log"])

        segs = q.segment_moves(d)
        meas = [(s, e) for (s, e) in segs
                if d["goal_pos"][s] != q.GOAL_SENTINEL_POS
                and not (d["goal_spd"][s] == 100 and d["goal_acc"][s] == 10)]

        dt = np.diff(d["t_us"])
        good = q.clean_position(d["pos"], d["t_us"])

        # telemetry holes strictly INSIDE a measurement move (not printf overhead)
        inmove = np.zeros(d["n"] - 1, dtype=bool)
        for s_, e_ in segs:
            if e_ - s_ > 1:
                inmove[s_:e_ - 1] = True
        midmove_gap = (dt > 6000) & inmove

        # per-phase counts vs log
        pc_ok = all(int((d["phase"] == p).sum()) == c for p, c in lg["phase_counts"].items())

        # recompute firmware-reported per-move statistics
        mism = {"final": 0, "peak_speed": 0, "peak_load": 0, "peak_current": 0,
                "range": 0, "first_motion": 0}
        n_cmp = min(len(meas), len(lg["ends"]))
        for i in range(n_cmp):
            s, e = meas[i]
            g = lg["ends"][i]
            pos = d["pos"][s:e]
            recomputed_final = int(pos[-1])
            if recomputed_final != int(g[4]):
                mism["final"] += 1
            if int(np.abs(d["speed"][s + 1:e]).max(initial=0)) != int(g[8]):
                mism["peak_speed"] += 1
            if int(np.abs(d["load"][s + 1:e]).max(initial=0)) != int(g[9]):
                mism["peak_load"] += 1
            if int(np.abs(d["current"][s + 1:e]).max(initial=0)) != int(g[10]):
                mism["peak_current"] += 1
            if (int(pos.min()), int(pos.max())) != (int(g[6]), int(g[7])):
                mism["range"] += 1
            moverows.append(dict(
                scode=sc, move_idx=i, phase=int(d["phase"][s]),
                phase_name=q.PHASE[int(d["phase"][s])],
                idx_start=s, idx_end=e, n=e - s,
                t_start_us=int(d["t_us"][s]), t_end_us=int(d["t_us"][e - 1]),
                goal_pos=int(d["goal_pos"][s]), goal_spd=int(d["goal_spd"][s]),
                goal_acc=int(d["goal_acc"][s]),
                start_pos=int(pos[0]), final_pos=recomputed_final,
                min_pos=int(pos.min()), max_pos=int(pos.max()),
                log_outcome=g[1], log_final=int(g[4]),
                log_elapsed_ms=float(g[12]),
                elapsed_ms=(int(d["t_us"][e - 1]) - int(d["t_us"][s])) / 1000.0))

        for z, dr, cs, tr, fm, ps, oc in lg["mins"]:
            minrows.append(dict(scode=sc, zone=int(z), direction=int(dr),
                                cmd_speed=int(cs), log_travel=int(tr),
                                log_first_motion_ms=float(fm),
                                log_peak_present_speed=int(ps), log_outcome=oc))

        integrity.append(dict(
            scode=sc, magic=h["magic"], version=h["version"],
            sample_hz=h["sample_hz"], sample_size=h["sample_size"],
            sample_count=h["sample_count"], n_parsed=d["n"],
            size_exact=(h["file_bytes"] == q.HDR_SIZE + h["sample_count"] * h["sample_size"]),
            result_code=h["result_code"], abort_reason=h["abort_reason"],
            sha_matches_manifest=mapping[sc]["sha256"] in manifest_sha,
            manifest_execution=manifest_sha.get(mapping[sc]["sha256"]),
            log_checksum=lg["summary"]["CHECKSUM"],
            elapsed_s=round(h["elapsed_us"] / 1e6, 3),
            t_monotonic=bool((dt > 0).all()),
            dt_median_us=int(np.median(dt)), dt_p99_9_us=int(np.percentile(dt, 99.9)),
            dt_max_us=int(dt.max()),
            gaps_gt_10ms=int((dt > 10000).sum()),
            gaps_at_move_boundary=int((dt > 6000).sum() - midmove_gap.sum()),
            midmove_gaps_gt_6ms=int(midmove_gap.sum()),
            midmove_gap_max_us=int(dt[midmove_gap].max()) if midmove_gap.any() else 0,
            midmove_lost_samples=int(np.round(dt[midmove_gap] / 2000.0 - 1).sum())
            if midmove_gap.any() else 0,
            dup_timestamps=int((dt == 0).sum()),
            proto_status_nonzero=int((d["proto"] != 0).sum()),
            servo_status_nonzero=int((d["status"] != 0).sum()),
            flags_nonzero=int((d["flags"] != 0).sum()),
            pos_out_of_domain=int(((d["pos"] < 0) | (d["pos"] > 4095)).sum()),
            pos_impossible_jumps=int((~good).sum()),
            n_segments=len(segs), n_measurement_moves=len(meas),
            log_measure_end=len(lg["ends"]), log_measure_begin=len(lg["begins"]),
            move_count_match=(len(meas) == len(lg["ends"])),
            phase_counts_match=pc_ok,
            mismatch_final=mism["final"], mismatch_peak_speed=mism["peak_speed"],
            mismatch_peak_load=mism["peak_load"],
            mismatch_peak_current=mism["peak_current"],
            mismatch_range=mism["range"],
            log_perf_events=lg["summary"]["PERFORMANCE_EVENTS"],
            log_protective=lg["summary"]["PROTECTIVE_STOPS"],
            log_thermal_transients=lg["summary"]["THERMAL_TRANSIENTS"],
            log_thermal_confirmed=lg["summary"]["THERMAL_CONFIRMED"],
            log_skipped=lg["summary"]["SKIPPED_TESTS"],
            log_min_attempts=lg["summary"]["MIN_ATTEMPTS"],
            log_min_responses=lg["summary"]["MIN_RESPONSES"],
            env_lo=lg["summary"]["ENV_LO"], env_hi=lg["summary"]["ENV_HI"],
            start_temp=lg["summary"]["START_TEMP"],
            start_volt=lg["summary"]["START_VOLT"],
        ))
        print(f"{sc} n={d['n']} segs={len(segs)} meas={len(meas)} "
              f"log_end={len(lg['ends'])} match={len(meas)==len(lg['ends'])} "
              f"mism={mism}", flush=True)

    def dump(rows, name):
        with open(q.AUDIT / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)

    dump(integrity, "blind_integrity.csv")
    dump(moverows, "blind_moves.csv")
    dump(minrows, "blind_min_log.csv")
    print("\nwrote blind_integrity.csv, blind_moves.csv, blind_min_log.csv")


if __name__ == "__main__":
    main()
