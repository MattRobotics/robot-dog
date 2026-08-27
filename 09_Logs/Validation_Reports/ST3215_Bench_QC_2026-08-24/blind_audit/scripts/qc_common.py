"""
MATDOG blind QC audit — shared parsing layer.

BLIND RULE: this module never exposes filename, label, path, mtime or
header.servo_id to any caller. Only S-codes leave this file.
Format is derived from matdog_servo_commissioning.ino, not assumed:
  QCFileHeader  = '<8s7I48s'  (84 B)
  QCFullSample  = 27 B packed, raw[15] = verbatim SRAM 56..70
"""
import hashlib
import json
import struct
from pathlib import Path

import numpy as np

CAMPAIGN = Path("/home/matteo-manicardi/MATDOG/runtime/esp32/qc_campaign")
AUDIT = Path("/home/matteo-manicardi/MATDOG/runtime/esp32/qc_campaign_blind_audit_claude")

HDR_FMT = "<8s7I48s"
HDR_SIZE = struct.calcsize(HDR_FMT)          # 84
SAMPLE_SIZE = 27
SAMPLE_HZ = 500

PHASE = {
    1: "STATIC_PRE", 2: "PREPOSITION", 3: "SLOW_DOWN", 4: "SLOW_UP",
    5: "MIN_PROBE", 6: "MAX_DOWN", 7: "MAX_UP", 8: "MAX_PRECISION",
    9: "STATIC_POST", 10: "TRANSFER", 11: "STEP45_MEDIUM",
}

# Sentinels written by qcReadAndStore for non-commanded reads.
GOAL_SENTINEL_POS = 0xFFFF
GOAL_SENTINEL_SPD = 0xFFFF
GOAL_SENTINEL_ACC = 0xFF


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.digest()


def build_mapping():
    """Deterministic S-code assignment, independent of label/ID/time."""
    bins = sorted(CAMPAIGN.glob("*.bin"), key=lambda p: p.name)
    if len(bins) != 18:
        raise SystemExit(f"expected 18 RAW files, found {len(bins)}")

    digests = {p: _sha256(p) for p in bins}

    key_material = b"".join(
        digests[p].hex().encode() + b"  " + p.name.encode() + b"\n" for p in bins
    )
    campaign_key = hashlib.sha256(key_material).digest()

    ranked = sorted(
        bins, key=lambda p: hashlib.sha256(campaign_key + digests[p]).hexdigest()
    )
    mapping = {}
    for i, p in enumerate(ranked, start=1):
        mapping[f"S{i:02d}"] = {
            "bin": str(p),
            "log": str(p.with_suffix(".log")),
            "sha256": digests[p].hex(),
            "rank_digest": hashlib.sha256(campaign_key + digests[p]).hexdigest(),
        }
    return campaign_key.hex(), mapping


def load_mapping():
    f = AUDIT / "anonymous_mapping_PRIVATE_until_freeze.json"
    return json.loads(f.read_text())


def scodes():
    return sorted(load_mapping()["mapping"].keys())


def load_raw(scode):
    """Return decoded arrays for one unit. servo_id is deliberately dropped."""
    m = load_mapping()["mapping"][scode]
    data = Path(m["bin"]).read_bytes()

    magic, version, servo_id, hz, ss, sc, elapsed_us, rc, abort = struct.unpack_from(
        HDR_FMT, data, 0
    )
    body = data[HDR_SIZE:]
    n = len(body) // SAMPLE_SIZE

    a = np.frombuffer(body, dtype=np.uint8).reshape(n, SAMPLE_SIZE)

    t_us = a[:, 0:4].copy().view(np.uint32).ravel().astype(np.int64)
    goal_pos = a[:, 4:6].copy().view(np.uint16).ravel().astype(np.int32)
    goal_spd = a[:, 6:8].copy().view(np.uint16).ravel().astype(np.int32)
    goal_acc = a[:, 8].astype(np.int32)
    phase = a[:, 9].astype(np.int32)
    proto = a[:, 10].astype(np.int32)
    flags = a[:, 11].astype(np.int32)
    r = a[:, 12:27]

    def u16(lo):
        return r[:, lo].astype(np.int32) | (r[:, lo + 1].astype(np.int32) << 8)

    def sm15(v):
        return np.where(v & 0x8000, -(v & 0x7FFF), v)

    pos = u16(0)
    speed = sm15(u16(2))
    lv = u16(4)
    load = np.where(lv & 1024, -(lv & ~1024), lv)      # bit10 sign, per firmware
    volt = r[:, 6].astype(np.int32)
    temp = r[:, 7].astype(np.int32)
    reg64 = r[:, 8].astype(np.int32)
    status = r[:, 9].astype(np.int32)
    moving = r[:, 10].astype(np.int32)
    reg67 = r[:, 11].astype(np.int32)
    reg68 = r[:, 12].astype(np.int32)
    current = sm15(u16(13))

    return dict(
        scode=scode, n=n,
        header=dict(magic=magic.rstrip(b"\0").decode(), version=version,
                    sample_hz=hz, sample_size=ss, sample_count=sc,
                    elapsed_us=elapsed_us, result_code=rc,
                    abort_reason=abort.split(b"\0")[0].decode(),
                    file_bytes=len(data)),
        _servo_id=servo_id,          # underscore: for the post-freeze batch check only
        t_us=t_us, goal_pos=goal_pos, goal_spd=goal_spd, goal_acc=goal_acc,
        phase=phase, proto=proto, flags=flags,
        pos=pos, speed=speed, load=load, volt=volt, temp=temp,
        reg64=reg64, status=status, moving=moving, reg67=reg67, reg68=reg68,
        current=current,
    )


def segment_moves(d, gap_us=None):
    """
    A move = maximal run of consecutive samples sharing
    (phase, goal_pos, goal_spd, goal_acc).

    NOTE: an earlier revision also split on dt > 15 ms. That was WRONG: the
    firmware's thermal-confirmation path (3 x delay(5 ms) + direct reads,
    qcThermalGuard) injects a ~17 ms hole *inside* a move, which produced a
    spurious extra segment in 3 of 18 units. Segmentation is therefore on the
    command tuple alone, and is validated 1:1 against the log's
    MEASURE_BEGIN/MEASURE_END sequence. Mid-move holes are recorded separately
    as telemetry dropouts. Pass gap_us explicitly only for diagnostics.
    Returns list of (start_idx, end_idx_exclusive).
    """
    n = d["n"]
    key_change = np.zeros(n, dtype=bool)
    for f in ("phase", "goal_pos", "goal_spd", "goal_acc"):
        v = d[f]
        key_change[1:] |= v[1:] != v[:-1]

    if gap_us is not None:
        key_change[1:] |= np.diff(d["t_us"]) > gap_us
    key_change[0] = True

    starts = np.flatnonzero(key_change)
    ends = np.append(starts[1:], n)
    return list(zip(starts.tolist(), ends.tolist()))


def clean_position(pos, t_us, max_tps=6000):
    """
    Flag physically impossible encoder jumps (byte corruption), the same class
    of artifact the firmware itself documents for temperature.
    Returns boolean mask of GOOD samples.
    """
    good = np.ones(len(pos), dtype=bool)
    dt = np.diff(t_us).astype(np.float64) / 1e6
    dt[dt <= 0] = 1e-9
    v = np.abs(np.diff(pos.astype(np.float64))) / dt
    # a sample is suspect only if BOTH the step in and the step out are impossible
    bad = np.zeros(len(pos), dtype=bool)
    bad[1:-1] = (v[:-1] > max_tps) & (v[1:] > max_tps)
    good[bad] = False
    return good


def robust_z(x, med=None, mad=None, floor=1e-9):
    x = np.asarray(x, dtype=np.float64)
    if med is None:
        med = np.nanmedian(x)
    if mad is None:
        mad = np.nanmedian(np.abs(x - med))
    scale = max(mad, floor)
    return 0.6745 * (x - med) / scale
