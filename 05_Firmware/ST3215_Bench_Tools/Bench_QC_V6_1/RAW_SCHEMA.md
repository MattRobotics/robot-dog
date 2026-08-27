# Bench QC V6.1 — RAW `.bin` capture schema

Binary layout of the QC RAW captures produced by
`matdog_servo_commissioning.ino` (SHA256 `74656fb9…`) at `QC_PROTOCOL_VERSION = 61`.

Verified against real campaign captures during the 2026-08-27 audit: for every file,
`84 + sample_size × sample_count` equals the exact file size.

```text
file = header (84 bytes) || sample[0] || sample[1] || … || sample[n-1]
n    = (file_size - 84) / 27
```

All multi-byte integers are **little-endian**.

---

## Header — 84 bytes

Python `struct` format: `<8s7I48s`

| Offset | Size | Field | Type | Notes |
|---:|---:|---|---|---|
| 0 | 8 | `magic` | `char[8]` | `"MATQC01\0"` |
| 8 | 4 | `version` | `uint32` | QC protocol version — **61** for V6.1 |
| 12 | 4 | `servo_id` | `uint32` | bus ID at capture time |
| 16 | 4 | `sample_hz` | `uint32` | nominal rate — **500** |
| 20 | 4 | `sample_size` | `uint32` | bytes per sample — **27** |
| 24 | 4 | `sample_count` | `uint32` | number of samples |
| 28 | 4 | `elapsed_us` | `uint32` | total capture duration (µs) |
| 32 | 4 | `result_code` | `uint32` | 0 = clean completion |
| 36 | 48 | `abort_reason` | `char[48]` | NUL-terminated; empty on success |

Reference values from `M22__id22__20260824_122509.bin`:

```text
magic MATQC01  version 61  servo_id 22  sample_hz 500
sample_size 27  sample_count 68514  elapsed_us 139325053  result_code 0
file size 1849962 == 84 + 27 × 68514
```

---

## Sample record — 27 bytes

Each record is a 12-byte **command/context prefix** followed by a 15-byte **raw ST3215 register
block** copied verbatim from the servo response.

### Prefix — bytes 0–11

| Offset | Size | Field | Type | Notes |
|---:|---:|---|---|---|
| 0 | 4 | `t_us` | `uint32` | timestamp, monotonic within a capture |
| 4 | 2 | `goal_pos` | `uint16` | commanded position; `0xFFFF` = no command active |
| 6 | 2 | `goal_spd` | `uint16` | commanded speed; `0xFFFF` = sentinel |
| 8 | 1 | `goal_acc` | `uint8` | commanded acceleration; `0xFF` = sentinel |
| 9 | 1 | `phase` | `uint8` | QC phase index |
| 10 | 1 | `proto` | `uint8` | protocol/context marker |
| 11 | 1 | `flags` | `uint8` | per-sample flags |

The three sentinels (`GOAL_SENTINEL_POS/SPD/ACC`) mark samples taken while no motion command was
outstanding, and are how measurement moves are segmented from idle telemetry.

### Raw register block — bytes 12–26 (`r[0..14]`)

| `r` offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 2 | `position` | `uint16` LE, 0…4095 |
| 2 | 2 | `speed` | `uint16` LE, **sign-magnitude, bit 15 = sign** |
| 4 | 2 | `load` | `uint16` LE, **bit 10 = sign** (firmware-specific) |
| 6 | 1 | `voltage` | raw decivolts |
| 7 | 1 | `temperature` | raw °C |
| 8 | 1 | `reg64` | raw register passthrough |
| 9 | 1 | `status` | ST3215 status byte |
| 10 | 1 | `moving` | moving flag |
| 11 | 1 | `reg67` | raw register passthrough |
| 12 | 1 | `reg68` | raw register passthrough |
| 13 | 2 | `current` | `uint16` LE, **sign-magnitude, bit 15 = sign** |

### Sign decoding

Two different sign conventions appear and must not be confused:

```python
def sign_magnitude_15(v):        # speed, current
    return -(v & 0x7FFF) if v & 0x8000 else v

def load_sign(v):                # load — bit 10 is the sign bit
    return -(v & ~1024) if v & 1024 else v
```

---

## Reference decoder

The authoritative reader is `qc_common.load_raw()`, committed at
[`09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/blind_audit/scripts/qc_common.py`](../../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/blind_audit/scripts/qc_common.py).

Minimal standalone decode:

```python
import struct, numpy as np

HDR_FMT, HDR_SIZE, SAMPLE_SIZE = "<8s7I48s", 84, 27

def load(path):
    data = open(path, "rb").read()
    magic, version, servo_id, hz, ss, sc, elapsed_us, rc, abort = \
        struct.unpack_from(HDR_FMT, data, 0)
    assert magic.rstrip(b"\0") == b"MATQC01"
    body = data[HDR_SIZE:]
    n = len(body) // SAMPLE_SIZE
    assert n * SAMPLE_SIZE == len(body), "trailing bytes: truncated capture"
    a = np.frombuffer(body, dtype=np.uint8).reshape(n, SAMPLE_SIZE)

    t_us     = a[:, 0:4].copy().view(np.uint32).ravel()
    goal_pos = a[:, 4:6].copy().view(np.uint16).ravel()
    goal_spd = a[:, 6:8].copy().view(np.uint16).ravel()
    goal_acc, phase, proto, flags = a[:, 8], a[:, 9], a[:, 10], a[:, 11]

    r = a[:, 12:27].astype(np.int32)
    u16 = lambda lo: r[:, lo] | (r[:, lo + 1] << 8)
    sm15 = lambda v: np.where(v & 0x8000, -(v & 0x7FFF), v)

    lv = u16(4)
    return dict(
        n=n, version=version, sample_hz=hz,
        t_us=t_us, goal_pos=goal_pos, goal_spd=goal_spd, goal_acc=goal_acc,
        phase=phase, proto=proto, flags=flags,
        position=u16(0),
        speed=sm15(u16(2)),
        load=np.where(lv & 1024, -(lv & ~1024), lv),
        voltage=r[:, 6], temperature=r[:, 7],
        status=r[:, 9], moving=r[:, 10],
        current=sm15(u16(13)),
    )
```

---

## Integrity expectations

A well-formed capture satisfies all of:

| Check | Expectation |
|---|---|
| `magic` | `MATQC01` |
| `version` | 61 |
| `sample_size` | 27 |
| body length | exactly `sample_size × sample_count` — no trailing bytes |
| `t_us` | non-decreasing |
| `position` | within 0…4095 |
| `result_code` | 0, with empty `abort_reason` |

---

## Note on the firmware `QCSample` struct

The firmware also declares a `struct QCSample` (line 200 of the sketch) whose fields sum to 22
bytes. **That struct does not describe the on-disk record.** The captured record is the 27-byte
layout above — a command/context prefix plus the raw 15-byte servo register block — and
`sample_size` in the header (27) is authoritative. Decode against this document, not against the
struct declaration.

---

## Where the RAW lives

The 26 campaign RAW captures (~49 MB) are **not** committed. They are preserved on the ASUS
canonical archive at `~/MATDOG/runtime/esp32/qc_campaign/*.bin`, with the hash manifest committed
at
[`09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt`](../../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt).
