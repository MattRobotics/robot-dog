# MATDOG — Independent Blind QC Audit of 18 × Feetech ST-3215-C018

**Anonymous report — frozen before any label was revealed.**  
Protocol: MATDOG QC V6.1 (binary version 61), 500 Hz RAW telemetry.  
Scope: offline, read-only. No serial port opened, no torque enabled, no register written.

Units are identified only by S-code. The S-code is a deterministic function of the file
content (`sha256(campaign_key || file_sha256)`, ascending), so it carries no information
about label, servo ID or acquisition order.

---

## 1. Integrity and tooling audit (FASE A)

### 1.1 Everything that checks out

| Check | Result |
|---|---|
| SHA256 of all 18 RAW vs `manifest.tsv` | **18/18 match** |
| Header magic / version / sample_hz / sample_size | `MATQC01` / 61 / 500 / 27 — uniform |
| `filesize == 84 + sample_count × 27` | exact, zero remainder, 18/18 |
| `result_code` / `abort_reason` | 0 / empty, 18/18 |
| On-device FNV-1a checksum at transfer (from logs) | PASS, 18/18 |
| `t_us` strictly monotonic | 18/18, no duplicates, no overflow |
| Protocol status byte (`st.Error`) non-zero | **0 samples out of 1,301,333** |
| Servo status byte (reg 65) non-zero | 0 samples (but see 1.3) |
| `flags` byte | 0 everywhere (never written by this firmware) |
| Position outside 0..4095 | 0 samples |
| Physically impossible encoder jumps | 0 samples |
| All 11 phases present | 18/18 |

### 1.2 Independent re-derivation of the firmware's own numbers

Move segmentation was derived from the RAW alone (maximal runs of constant
`phase, goal_position, goal_speed, goal_acc`) and then checked 1:1 against each log's
`MEASURE_BEGIN`/`MEASURE_END` sequence.

- **1,672 measurement moves** reconstructed; move count matches the log for **18/18** units.
- For every one of those moves, recomputing `final position`, `position range`,
  `peak |speed|`, `peak |load|` and `peak |current|` from the RAW reproduces the
  firmware-printed value **exactly — 0 mismatches in 8,360 comparisons**.
- Per-phase sample counts match the log's `PHASE n:` lines for 18/18.

This validates both my decoder and the firmware's own reporting path.

### 1.3 Bugs and weaknesses found (documented, not corrected)

**B1 — `matdog_qc_unit.sh:202-204`, header fields swapped.** The inline Python assigns
`samples = vals[4]` and `sample_size = vals[5]`, but in `<8s7I48s>` `vals[4]` *is*
`sample_size` and `vals[5]` *is* `sample_count`. The wrapper consequently prints
`SAMPLES : 27` for every unit. *Impact: cosmetic only* — neither value is validated nor
written to the manifest. **Does not alter any data or any ranking.**
`matdog_qc_runner.py:191-201` unpacks correctly.

**B2 — the hardware status register address is not backed by the installed driver.**
The firmware treats `raw[9]` (SRAM 65) as the servo hardware-error byte and *aborts QC*
if it is non-zero. The installed `SCServo` driver (`SMS_STS.h`) defines 56, 58, 60, 62,
63, 66, 69 — **but not 64, 65, 67 or 68**. Community register tables do document 65 as
"Servo Status", and the value read 0 in all 1.3 M samples, but *0 is also what an
unimplemented register returns*. **Consequence: the statement "no unit reported a
hardware fault" is not verifiable from this dataset.** The packet-level status
(`st.Error`, register-independent) *is* verified and is clean. See §10 for the read-only
check that would settle it.

**B3 — failed feedback reads are dropped, not recorded** (`qcReadAndStore` returns before
storing). A dropout is invisible except as a hole in `t_us`. Quantified below; benign here.

**B4 — `qcPrintPhaseSummary` reports raw min/max temperature including known-corrupt
samples**, which is why the logs show impossible ranges such as `temp=30..130`. The log's
per-phase temperature range must not be used as evidence.

**B5 — min-probe targets chase a moving reference** (`probeTarget = current ± 64`, with
`current` re-read after each probe), so repeated probes in one direction are not against a
fixed goal. Affects interpretation only; all 18 responded on the first probe (speed 1).

**B6 (my own, found and fixed mid-audit, recorded for transparency).** My first
segmentation also split on `dt > 15 ms`, which produced a spurious extra segment in 3 of
18 units. Cause: the firmware's thermal-confirmation path (`qcThermalGuard`, 3 × `delay(5)`
plus direct reads) injects a ~17 ms hole *inside* a move. Fixed by segmenting on the
command tuple alone. Separately, my first MAX-speed estimator used central differences over
8 ms, which quantizes to whole encoder ticks and **fabricated ~4 % direction asymmetries**
and three spurious speed clusters; replaced by span/duration over the whole cruise. Both
errors were mine, not the firmware's, and both are corrected in the numbers below.

### 1.4 Sampling quality

| | min | median | max |
|---|---|---|---|
| inter-sample interval (µs) | 1998 | 1998 | 2000 |
| gaps at move boundaries (printf overhead) | 91 | 93 | 93 |
| holes *inside* a move | 1 | 1 | 2 |
| samples lost to those holes | 31 | 31 | 38 |

Effective rate 500 Hz (median 2000 µs). Every unit has exactly one ~64 ms hole in the
2-sample PREPOSITION prime segment — that is the torque-on handshake, not a dropout. The
3 units with an extra hole are the ones that triggered a *transient* thermal confirmation.
**No unit shows a telemetry-integrity problem.**

---

## 2. Blind methodology and freeze proof

1. `RAW_SHA256_BEFORE.txt` captured before any analysis.
2. `campaign_key = sha256` over the basename-sorted `(sha256, basename)` list of the 18 RAW.
3. `S-code` = rank of `sha256(campaign_key || file_sha256)`, ascending hex.
4. The loader reads bytes only; filename, path, mtime and `header.servo_id` are dropped at
   parse time and never reach a metric, a table or a plot.
5. This report is hashed into `blind_report_ANONYMOUS.sha256` **before** the mapping file is
   read back. The labelled report is a separate file.

**Disclosed limit of the blind:** `servo_id` is inside the RAW header, and it partitions the
set into 12 distinct IDs plus 6 files sharing one ID. Anyone re-running the parser can
recover that grouping. The blind here is *procedural* (the pipeline never reads it), not
information-theoretic. Stating this is more useful than claiming a guarantee the data
cannot support.

**Baselines are the population itself** — median/MAD over the 18, computed separately for
each (phase, direction, encoder-bin) cell so that protocol artifacts common to all units
cancel. No "known-good" subset was used. Every spatial conclusion was re-run with a
**leave-one-out** baseline (a unit never contributes to the population it is judged
against): identical result, same regions, same peaks.

---

## 3. The 18 units

| S-code | samples | elapsed s | moves | reach | MAX tick/s | RPM | s/60° | load ceiling | tier |
|---|---|---|---|---|---|---|---|---|---|
| S01 | 69060 | 140.417 | 93 | 1–4093 | 2817 | 41.3 | 0.2424 | 1000 | CLEAN |
| S02 | 70886 | 144.069 | 93 | 1–4094 | 2848 | 41.7 | 0.2397 | 1000 | CLEAN WITH MINOR NOTE |
| S03 | 75464 | 153.224 | 93 | 3–4095 | 2833 | 41.5 | 0.2410 | 1000 | CLEAN |
| S04 | 77348 | 156.993 | 93 | 2–4094 | 1495 | 21.9 | 0.4567 | 500 | STRONG MULTI-DOMAIN SUSPECT + CONFIGURATION-CONFOUNDED |
| S05 | 77340 | 156.929 | 91 | 3–4090 | 1496 | 21.9 | 0.4562 | 500 | INCONCLUSIVE - CONFIGURATION-CONFOUNDED |
| S06 | 75461 | 153.219 | 93 | 2–4095 | 2816 | 41.3 | 0.2424 | 1000 | CLEAN |
| S07 | 75565 | 153.427 | 93 | 2–4093 | 2839 | 41.6 | 0.2405 | 1000 | CLEAN |
| S08 | 68514 | 139.325 | 93 | 2–4093 | 2816 | 41.3 | 0.2424 | 1000 | CLEAN |
| S09 | 75267 | 152.831 | 93 | 2–4093 | 2830 | 41.5 | 0.2413 | 1000 | CLEAN WITH MINOR NOTE |
| S10 | 68571 | 139.438 | 93 | 1–4093 | 2830 | 41.5 | 0.2412 | 1000 | CLEAN |
| S11 | 71993 | 146.283 | 93 | 1–4093 | 2844 | 41.7 | 0.2401 | 1000 | CLEAN |
| S12 | 75618 | 153.533 | 93 | 2–4092 | 2840 | 41.6 | 0.2404 | 1000 | CLEAN WITH MINOR NOTE |
| S13 | 75414 | 153.124 | 93 | 3–4093 | 2813 | 41.2 | 0.2427 | 1000 | CLEAN |
| S14 | 70291 | 142.879 | 93 | 2–4093 | 2846 | 41.7 | 0.2399 | 1000 | CLEAN |
| S15 | 68953 | 140.203 | 93 | 2–4092 | 2834 | 41.5 | 0.2409 | 1000 | CLEAN WITH MINOR NOTE |
| S16 | 72279 | 146.855 | 93 | 1–4093 | 2821 | 41.3 | 0.2420 | 1000 | STRONG MULTI-DOMAIN SUSPECT |
| S17 | 68637 | 139.571 | 93 | 2–4093 | 2828 | 41.4 | 0.2414 | 1000 | CLEAN WITH MINOR NOTE |
| S18 | 68672 | 139.641 | 93 | 2–4094 | 2814 | 41.2 | 0.2426 | 1000 | CLEAN |

---

## 4. Confirmed focal anomalies, with RAW indices and timestamps

Detector: mean `Present Load` excess over the population **at the same 16-tick encoder
position**, requiring ≥4 independent traversals. Threshold = **p99.5 of the pooled
population distribution of that statistic (15.1 load units ≈ 6.1 robust SD)** — derived
from the data, not chosen by hand. A region is CONFIRMED only if ≥3 adjacent positions
(≥48 ticks) clear it.

| S-code | encoder range | degrees | width | passes | peak excess | mean excess | speed vs pop | verdict |
|---|---|---|---|---|---|---|---|---|
| S02 | 2016–2031 | 177.2°–178.6° | 16 | 4 | +22.0 | +22.0 | 1.0% | **FOCAL_WEAK** |
| S04 | 2144–2271 | 188.4°–199.7° | 128 | 6 | +31.0 | +24.1 | -1.4% | **FOCAL_CONFIRMED** |
| S05 | 2016–2031 | 177.2°–178.6° | 16 | 5 | +16.0 | +16.0 | -0.2% | **FOCAL_WEAK** |
| S09 | 528–543 | 46.4°–47.8° | 16 | 5 | +16.8 | +16.8 | -0.5% | **FOCAL_WEAK** |
| S16 | 2304–2319 | 202.5°–203.9° | 16 | 4 | +23.0 | +23.0 | -6.3% | **FOCAL_WEAK** |
| S16 | 2560–2607 | 225.0°–229.2° | 48 | 4 | +26.4 | +20.8 | 2.3% | **FOCAL_CONFIRMED** |
| S16 | 2624–2751 | 230.6°–241.9° | 128 | 6 | +42.0 | +25.8 | 2.4% | **FOCAL_CONFIRMED** |

Only **S04** and **S16** have CONFIRMED regions. The four `FOCAL_WEAK` entries are single
16-tick bins; two of them (S02 and S05) sit at the *same* position 2016, immediately below
the 2048 slow-segment boundary, which marks them as residual common-mode rather than unit
properties. Per the frozen rule, single-bin flags are notes, not findings.

### S04 — supporting spatial clusters (speed-led detector, independent of the load detector)

| group | dir | encoder | RAW idx | t start–end (s) | speed | baseline | deficit | load | load base | conf |
|---|---|---|---|---|---|---|---|---|---|---|
| MEDIUM | -1 | 32–63 | 50171–50207 | 101.195–101.267 | 430.7 | 469.2 | -8.2% | 144.0 | 152.0 | LOW |
| MEDIUM | 1 | 960–991 | 41131–41167 | 82.779–82.851 | 430.5 | 469.6 | -8.3% | 144.0 | 152.0 | LOW |
| SLOW | 1 | 2080–2143 | 14575–14854 | 29.331–29.889 | 111.7 | 125.5 | -11.0% | 64.0 | 56.0 | HIGH |
| SLOW | 1 | 2112–2175 | 14710–14979 | 29.601–30.139 | 117.1 | 127.5 | -8.2% | 80.0 | 56.0 | LOW |
| MEDIUM | 1 | 2112–2175 | 42739–42803 | 86.067–86.195 | 492.1 | 594.3 | -17.2% | 208.0 | 208.0 | LOW |
| MEDIUM | 1 | 2112–2175 | 42739–42803 | 86.067–86.195 | 493.2 | 592.2 | -16.8% | 204.0 | 206.0 | HIGH |
| TRANSFER | 1 | 2112–2239 | 57475–57594 | 116.107–116.345 | 525.5 | 595.3 | -11.7% | 216.0 | 207.0 | HIGH |
| TRANSFER | 1 | 2112–2175 | 57475–57535 | 116.107–116.227 | 517.3 | 596.2 | -13.2% | 210.0 | 208.0 | HIGH |
| SLOW | 1 | 2208–2239 | 15104–15239 | 30.389–30.659 | 114.8 | 126.0 | -8.9% | 92.0 | 56.0 | LOW |
| TRANSFER | 1 | 2208–2271 | 57563–57631 | 116.283–116.419 | 465.3 | 521.9 | -10.6% | 184.0 | 176.0 | HIGH |
| SLOW | -1 | 2240–2303 | 30315–30602 | 60.979–61.553 | 109.8 | 127.0 | -13.6% | 64.0 | 56.0 | LOW |
| SLOW | -1 | 2240–2271 | 30437–30602 | 61.223–61.553 | 93.9 | 125.0 | -24.8% | 92.0 | 56.0 | LOW |
| MEDIUM | -1 | 2240–2303 | 47449–47507 | 95.655–95.771 | 543.0 | 605.8 | -10.4% | 212.0 | 208.0 | LOW |
| MEDIUM | -1 | 2240–2271 | 47476–47507 | 95.709–95.771 | 500.1 | 596.3 | -16.1% | 224.0 | 208.0 | LOW |
| TRANSFER | -1 | 2368–2399 | 71481–71516 | 144.759–144.829 | 442.9 | 499.9 | -11.4% | 154.0 | 162.0 | LOW |
| TRANSFER | -1 | 3904–3935 | 67923–67957 | 137.355–137.423 | 455.9 | 500.0 | -8.8% | 156.0 | 164.0 | LOW |

### S16 — supporting spatial clusters (speed-led detector, independent of the load detector)

| group | dir | encoder | RAW idx | t start–end (s) | speed | baseline | deficit | load | load base | conf |
|---|---|---|---|---|---|---|---|---|---|---|
| MEDIUM | 1 | 256–287 | 38986–39014 | 78.465–78.521 | 535.7 | 596.2 | -10.2% | 200.0 | 208.0 | LOW |
| MEDIUM | 1 | 1344–1375 | 40257–40285 | 81.055–81.111 | 535.8 | 596.3 | -10.1% | 228.0 | 208.0 | LOW |
| MEDIUM | -1 | 2176–2207 | 45731–45760 | 92.219–92.277 | 534.6 | 600.0 | -10.9% | 208.0 | 208.0 | LOW |
| TRANSFER | -1 | 2176–2207 | 67862–67891 | 137.617–137.675 | 517.2 | 600.0 | -13.8% | 212.0 | 208.0 | LOW |
| MEDIUM | -1 | 2304–2335 | 45625–45656 | 92.007–92.069 | 500.0 | 596.2 | -16.1% | 220.0 | 208.0 | LOW |
| MEDIUM | -1 | 2400–2463 | 45519–45574 | 91.795–91.905 | 555.6 | 596.1 | -6.8% | 207.0 | 206.0 | LOW |
| MEDIUM | -1 | 2688–2751 | 45113–45170 | 90.959–91.073 | 552.6 | 601.9 | -8.2% | 252.0 | 208.0 | LOW |
| TRANSFER | 1 | 2688–2719 | 56784–56813 | 114.795–114.853 | 534.5 | 596.3 | -10.4% | 224.0 | 208.0 | LOW |
| SLOW | -1 | 2720–2751 | 27296–27433 | 54.917–55.191 | 113.1 | 125.0 | -9.5% | 62.0 | 56.0 | LOW |
| TRANSFER | 1 | 3488–3519 | 59910–59938 | 121.187–121.243 | 553.6 | 598.1 | -7.4% | 212.0 | 208.0 | LOW |

---

## 5. Minimum-speed probes — 8 zones × 2 directions

Directional travel in ticks during a 900 ms observation at `GoalSpeed=1`. All 18 units
responded on the first probe, so speeds 2/4/8 were never needed (`MIN_ATTEMPTS=16`,
`MIN_RESPONSES=16` for all 18).

| S-code | 256+ / 256− | 768+ / 768− | 1280+ / 1280− | 1792+ / 1792− | 2304+ / 2304− | 2816+ / 2816− | 3328+ / 3328− | 3840+ / 3840− | asym |
|---|---|---|---|---|---|---|---|---|---|
| S01 | 39 / 43 | 41 / 41 | 37 / 41 | 39 / 41 | 36 / 42 | 36 / 43 | 34 / 44 | 42 / 40 | 3.5 |
| S02 | 42 / 42 | 41 / 45 | 42 / 43 | 41 / 44 | 41 / 44 | 40 / 44 | 40 / 44 | 41 / 45 | 3.0 |
| S03 | 37 / 44 | 21 / 41 | 41 / 41 | 40 / 42 | 41 / 42 | 42 / 41 | 41 / 42 | 38 / 41 | 1.0 |
| S04 | 32 / 42 | **7** / 44 | **18** / 36 | 35 / 42 | **14** / 40 | 36 / 37 | 35 / 35 | 36 / 35 | 5.0 |
| S05 | **10** / 41 | **15** / 42 | **19** / 41 | **17** / 42 | 35 / 41 | **18** / 37 | 35 / 41 | **11** / 41 | 23.5 |
| S06 | 38 / 43 | 42 / 43 | 40 / 44 | 40 / 43 | 42 / 43 | 41 / 43 | 38 / 43 | 41 / 44 | 2.5 |
| S07 | 36 / 44 | 41 / 41 | 22 / 42 | 41 / 42 | 40 / 45 | 41 / 44 | 38 / 42 | 40 / 44 | 3.0 |
| S08 | 40 / 44 | 42 / 41 | 41 / 44 | 38 / 44 | 42 / 42 | 41 / 42 | 37 / 44 | 41 / 42 | 2.0 |
| S09 | 20 / 44 | 41 / 42 | 40 / 42 | 40 / 44 | 39 / 41 | 42 / 43 | 40 / 44 | 38 / 43 | 3.0 |
| S10 | 40 / 44 | 41 / 45 | 42 / 42 | 42 / 42 | 38 / 44 | 41 / 42 | 41 / 43 | 39 / 42 | 1.5 |
| S11 | 42 / 41 | 41 / 42 | 42 / 42 | 41 / 42 | 39 / 42 | 42 / 42 | 42 / 42 | 39 / 42 | 0.5 |
| S12 | **19** / 44 | 40 / 42 | 38 / 41 | 41 / 41 | 38 / 42 | 39 / 44 | 39 / 43 | 36 / 43 | 4.0 |
| S13 | 42 / 42 | 38 / 41 | 42 / 42 | 40 / 44 | 40 / 43 | 36 / 45 | 42 / 42 | 38 / 43 | 2.5 |
| S14 | 42 / 42 | 42 / 44 | 42 / 42 | 42 / 42 | 40 / 44 | 42 / 43 | 41 / 42 | 42 / 41 | 0.0 |
| S15 | 41 / 44 | 42 / 41 | 38 / 41 | 40 / 44 | 40 / 45 | 40 / 44 | 41 / 41 | 41 / 44 | 3.5 |
| S16 | 41 / 43 | 38 / 43 | 42 / 43 | 42 / 42 | 38 / 42 | 41 / 41 | 41 / 43 | 40 / 43 | 2.0 |
| S17 | **18** / 44 | 42 / 42 | 42 / 42 | 38 / 41 | 36 / 44 | 41 / 43 | 41 / 42 | 41 / 41 | 1.0 |
| S18 | 41 / 42 | 40 / 44 | 42 / 41 | 41 / 42 | 41 / 42 | 40 / 43 | 41 / 41 | 37 / 44 | 1.0 |
| **pop median** | 40 / 43 | 41 / 42 | 41 / 42 | 40 / 42 | 40 / 42 | 41 / 43 | 40 / 42 | 40 / 42 | — |

Bold = below 50 % of the population at that zone and direction.

- **S05 is the only systematic case**: the **+ direction collapses in 6 of 8 zones**
  (10, 15, 19, 17, 18, 11 ticks against a population of 40–41) while the − direction is
  entirely normal. Directional asymmetry 23.5 ticks vs 0.0–5.0 for every other unit.
- **S04**: 3 of 8 zones below half, same + direction bias, asymmetry 5.0.
- **S03, S07, S09, S12, S17**: one isolated low zone each. Four of the five are at the same
  zone 256 (the first zone probed after a transfer), so this is partly a protocol position
  effect. Low confidence by construction — a single probe is a note, not a defect.
- A population-wide +/− asymmetry of ~2 ticks exists in *all* units and is removed by the
  per-cell baseline.

**Caveat that limits this section:** S04 and S05 both carry the 50 % output cap (§7).
Breaking static friction at `GoalSpeed=1` is exactly what a torque cap impairs, so their
min-speed deficits **cannot be attributed to mechanics** from this dataset.

---

## 6. Precision, hysteresis and MAX

| S-code | prec MAE | max abs err | bias | hyst median | hyst max | settle ms | MAX tick/s | MAX dev from pop | MAX asym | median current | peak current |
|---|---|---|---|---|---|---|---|---|---|---|---|
| S01 | 2.75 | 4 | -0.12 | 6.0 | 7 | 138 | 2817 | -0.4% | 0.72% | 18.0 | 33 |
| S02 | 2.25 | 3 | +0.12 | 5.0 | 6 | 136 | 2848 | +0.7% | 1.27% | 19.0 | 32 |
| S03 | 2.75 | 4 | +0.25 | 5.0 | 7 | 142 | 2833 | +0.2% | 0.13% | 22.0 | 39 |
| S04 | 4.69 | 8 | -0.44 | 8.5 | 13 | 153 | 1495 | -47.2% | 0.33% | 7.0 | 13 |
| S05 | 4.50 | 7 | -1.00 | 9.0 | 11 | 151 | 1496 | -47.1% | 0.33% | 7.0 | 13 |
| S06 | 2.38 | 3 | -0.12 | 5.0 | 6 | 140 | 2816 | -0.4% | 0.25% | 24.5 | 41 |
| S07 | 2.44 | 4 | -0.31 | 5.0 | 6 | 140 | 2839 | +0.4% | 0.40% | 21.5 | 38 |
| S08 | 2.75 | 4 | -0.25 | 5.5 | 6 | 140 | 2816 | -0.4% | 0.53% | 22.5 | 41 |
| S09 | 2.69 | 4 | -0.31 | 5.0 | 7 | 149 | 2830 | +0.0% | 0.52% | 23.0 | 37 |
| S10 | 2.50 | 3 | +0.00 | 5.0 | 6 | 145 | 2830 | +0.1% | 0.73% | 20.5 | 34 |
| S11 | 2.44 | 3 | +0.06 | 5.0 | 6 | 143 | 2844 | +0.5% | 1.35% | 19.0 | 34 |
| S12 | 2.44 | 4 | -0.19 | 5.5 | 6 | 139 | 2840 | +0.4% | 0.33% | 21.0 | 38 |
| S13 | 2.38 | 3 | -0.12 | 5.0 | 6 | 150 | 2813 | -0.5% | 0.18% | 22.0 | 42 |
| S14 | 2.00 | 3 | -0.12 | 4.0 | 6 | 141 | 2846 | +0.6% | 0.41% | 19.5 | 35 |
| S15 | 2.56 | 4 | -0.06 | 5.5 | 6 | 136 | 2834 | +0.2% | 0.48% | 20.5 | 37 |
| S16 | 2.69 | 3 | +0.19 | 5.5 | 6 | 141 | 2821 | -0.3% | 0.48% | 20.0 | 39 |
| S17 | 2.44 | 4 | +0.06 | 5.0 | 6 | 149 | 2828 | -0.0% | 0.14% | 23.0 | 38 |
| S18 | 2.38 | 3 | -0.12 | 5.0 | 6 | 141 | 2814 | -0.5% | 1.84% | 21.5 | 36 |

- Precision MAE is 2.00–2.75 ticks for 16 units and **4.50 / 4.69 for S05 / S04**.
- Directional hysteresis is 4–6 ticks for 16 units and **9.0 / 8.5 for S05 / S04**.
  At 0.088°/tick that is 0.35–0.53° for the population and 0.75–0.79° for the two.
  This is *measured directional hysteresis* — control deadband + friction + backlash
  combined — and must **not** be equated with the datasheet's ≤0.5° gear backlash.
- MAX direction asymmetry is ≤1.9 % for every unit including the two capped ones.
- Overshoot past target: **0 ticks for all 18** in every MAX and precision move.

---

## 7. Configuration confound (FASE E) — the single most important finding

**S04 and S05 run at almost exactly half the population's maximum speed, and their
`Present Load` register never exceeds exactly 500 while all 16 others reach exactly 1000.**

| | 16 units | S04 | S05 |
|---|---|---|---|
| MAX speed (tick/s) | 2829 (2813–2848) | 1495 | 1496 |
| as % of population | 100 % | 53.1 % | 53.2 % |
| `max |load|` ever observed | **1000** | **500** | **500** |
| samples pinned at that ceiling | ~913–982 | 2491 | 2494 |
| median current at MAX (raw) | 18–24.5 | 7 | 7 |
| bus voltage (raw) | 110–111 | 111 | 111 |

The ceiling is *hard* — neither unit ever produced a single sample above 500 in 1.3 M
samples, and both sat pinned at exactly 500 for ~5 s of MAX travel. Per the installed
driver, `SMS_STS_TORQUE_LIMIT_L/H = 48/49` is a 16-bit SRAM value where 1000 = 100 % of
locked-rotor torque; `Max Torque` at EEPROM 16 has the same scale. **A value of 500 in
either register reproduces every observation**: half the output ceiling, ~53 % of no-load
speed, ~⅓ of the current, worse steady-state error, longer first motion, more dwell.

Because `Present Load` truncates at 500 rather than rescaling to 1000, the register is an
**absolute** duty scale clipped by the limit, not a percentage of the limit. That is what
makes the load-excess comparison in §4 valid across capped and uncapped units.

**Consequence for the verdicts:** every S04/S05 metric that a torque cap would degrade —
MAX speed, precision MAE, hysteresis, min-speed travel, first-motion latency, off-target
settling — is *unusable* as evidence of mechanical damage. The one thing the cap cannot
explain is a **localized** load excess at a fixed encoder angle, because S05 carries the
identical cap and is completely flat through S04's anomalous region (load 56–60 against
S04's 76–104 at the same positions, same passes). That is the discriminator this audit
rests on for S04.

**No configuration register is present in the RAW** (only SRAM 56..70 is captured), so the
cap cannot be confirmed from this dataset — only inferred. §10 gives the read-only check.

---

## 8. Telemetry artifacts (FASE B)

| S-code | temp single-sample spikes | temp runs 2–24 | temp runs ≥25 | max clean temp | temp rise | volt spikes | volt runs ≥25 | min clean volt | load=1000 at low speed |
|---|---|---|---|---|---|---|---|---|---|
| S01 | 439 | 5 | 0 | 39 | 5 | 0 | 0 | 75 | 0 |
| S02 | 430 | 2 | 0 | 39 | 3 | 1 | 0 | 77 | 0 |
| S03 | 690 | 18 | 0 | 38 | 4 | 0 | 0 | 109 | 0 |
| S04 | 619 | 5 | 0 | 38 | 5 | 0 | 0 | 109 | 0 |
| S05 | 687 | 3 | 0 | 37 | 5 | 0 | 0 | 110 | 0 |
| S06 | 610 | 14 | 0 | 39 | 4 | 0 | 0 | 110 | 0 |
| S07 | 612 | 8 | 0 | 37 | 3 | 1 | 0 | 83 | 0 |
| S08 | 515 | 11 | 0 | 37 | 5 | 0 | 0 | 64 | 0 |
| S09 | 734 | 10 | 0 | 37 | 4 | 0 | 0 | 74 | 0 |
| S10 | 437 | 4 | 0 | 35 | 4 | 1 | 0 | 73 | 0 |
| S11 | 517 | 9 | 0 | 37 | 4 | 0 | 0 | 84 | 0 |
| S12 | 619 | 12 | 0 | 39 | 4 | 0 | 0 | 74 | 0 |
| S13 | 620 | 11 | 0 | 38 | 4 | 0 | 0 | 85 | 0 |
| S14 | 321 | 7 | 0 | 37 | 4 | 0 | 0 | 109 | 0 |
| S15 | 435 | 5 | 0 | 41 | 0 | 0 | 0 | 87 | 0 |
| S16 | 568 | 13 | 0 | 38 | 5 | 1 | 0 | 76 | 0 |
| S17 | 535 | 10 | 0 | 36 | 3 | 0 | 0 | 66 | 0 |
| S18 | 448 | 10 | 0 | 39 | 5 | 0 | 0 | 64 | 0 |

- **Temperature corruption is real, firmware-acknowledged, and present in all 18.** Logs
  show `THERMAL_CANDIDATE initial=83 → confirmations [32,32,32] → THERMAL_TRANSIENT`.
  321–734 isolated corrupt samples per unit. **Not one unit has a single run of ≥25
  consecutive anomalous samples**, and after filtering, the maximum real temperature is
  35–41 °C against a 70 °C limit. Total rise over a ~150 s run: 0–5 °C.
  No thermal finding for any unit.
- **Voltage:** at most 1 isolated spike per unit, no persistent run in any unit. Raw
  extrema down to 64 (6.4 V) are single samples during braking, never sustained; the
  firmware's own 25/50-sample persistence guards never fired.
- **`load = 1000` is never a stall here.** It occurs only in the MAX phases, concurrent
  with ~2820–2850 tick/s. Samples with load saturated *and* speed below 100 tick/s:
  **0 in all 18 units**.
- Encoder and `PresentSpeed` agree: median ratio 1.000 across slow/medium/transfer/MAX,
  confirming `PresentSpeed` LSB = 1 tick/s and giving two independent motion channels.

---

## 9. Robust outlier table and ranking stability (FASE D)

Features, weights and sensitivity are in `ranking_sensitivity.csv`. Four weightings were
run: spatial-led, uniform, precision-led, and mechanical-only (which zeroes every feature
a torque cap would move).

| S-code | cross-speed bins | focal width | focal peak | min zones <50% | min asym | prec MAE | hyst | off-target | MAX deficit | Borda | rank spread | stable |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S04 | 12 | 128 | 31 | 3 | 5.0 | 4.69 | 8.5 | 7 | 47.1% | 1 | 1 | yes |
| S05 | 4 | 0 | 0 | 6 | 23.5 | 4.50 | 9.0 | 9 | 47.1% | 2 | 2 | yes |
| S16 | 6 | 176 | 42 | 0 | 2.0 | 2.69 | 5.5 | 1 | 0.3% | 3 | 2 | yes |
| S01 | 0 | 0 | 0 | 0 | 3.5 | 2.75 | 6.0 | 0 | 0.4% | 4 | 2 | yes |
| S15 | 2 | 0 | 0 | 0 | 3.5 | 2.56 | 5.5 | 0 | -0.2% | 5 | 2 | yes |
| S12 | 0 | 0 | 0 | 1 | 4.0 | 2.44 | 5.5 | 1 | -0.4% | 6 | 10 | no |
| S03 | 0 | 0 | 0 | 0 | 1.0 | 2.75 | 5.0 | 1 | -0.2% | 7 | 2 | yes |
| S09 | 0 | 0 | 0 | 0 | 3.0 | 2.69 | 5.0 | 1 | -0.0% | 8 | 4 | no |
| S08 | 0 | 0 | 0 | 0 | 2.0 | 2.75 | 5.5 | 0 | 0.4% | 9 | 3 | yes |
| S02 | 0 | 0 | 0 | 0 | 3.0 | 2.25 | 5.0 | 0 | -0.7% | 10 | 5 | no |
| S17 | 0 | 0 | 0 | 1 | 1.0 | 2.44 | 5.0 | 0 | 0.0% | 11 | 10 | no |
| S07 | 0 | 0 | 0 | 0 | 3.0 | 2.44 | 5.0 | 0 | -0.4% | 12 | 3 | yes |
| S10 | 0 | 0 | 0 | 0 | 1.5 | 2.50 | 5.0 | 0 | -0.1% | 13 | 1 | yes |
| S06 | 0 | 0 | 0 | 0 | 2.5 | 2.38 | 5.0 | 0 | 0.4% | 14 | 8 | no |
| S13 | 0 | 0 | 0 | 0 | 2.5 | 2.38 | 5.0 | 0 | 0.6% | 15 | 1 | yes |
| S18 | 0 | 0 | 0 | 0 | 1.0 | 2.38 | 5.0 | 0 | 0.5% | 16 | 3 | yes |
| S11 | 0 | 0 | 0 | 0 | 0.5 | 2.44 | 5.0 | 0 | -0.5% | 17 | 4 | no |
| S14 | 0 | 0 | 0 | 0 | 0.0 | 2.00 | 4.0 | 0 | -0.6% | 18 | 2 | yes |

Kendall τ between weightings: +0.88 to +0.96 among the three that include cap-sensitive
features, and +0.52 to +0.53 against the mechanical-only view — which is the point of
running it: **the ordering below the top 3 is driven by the cap features, not by mechanics.**

**Stability statement, stated plainly:** only the top three positions are meaningful.
S04, S05 and S16 occupy ranks 1–3 under *every* weighting. Ranks 4–18 all have composite
scores ≤0.23 on a scale where S04 scores 2.5–3.1; their ordering changes with the weights
(spread up to 10 places) and **should not be read as a quality ordering**. Those 15 units
are statistically indistinguishable from one another.

---

## 10. Read-only checks still required

None of these were performed. Each needs explicit authorization; all are pure reads.

**Priority 1 — settle the configuration confound on S04 and S05.** Read and compare across
all 18 (addresses from the installed `SCServo/SMS_STS.h` where defined, otherwise from the
register table and to be verified on-device):

| addr | register | why |
|---|---|---|
| 48–49 | Torque Limit (SRAM, 16-bit, 0–1000) | **the prime suspect** — 500 here explains everything |
| 16 | Max Torque (EEPROM, 0–1000) | same scale, applied at power-up |
| 21 / 22 / 23 | P / D / I | steady-state error and hysteresis |
| 24 | Minimum startup force / punch | first-motion latency, min-speed travel |
| 26 / 27 | CW / CCW dead zone | directional hysteresis, min-speed asymmetry (S05) |
| 28 | Protection current | protection behaviour |
| 34 / 35 / 36 | Protection torque / time / overload torque | protection behaviour |
| 33 | Mode | must be 0; firmware already gates on this |
| 55 | Lock | whether EEPROM was left unlocked |
| 31–32 | Position offset | affects the *absolute* angle of the §4 regions |

**Priority 2 — resolve B2.** Read registers 64, 65, 67, 68 on a unit while deliberately
provoking a benign, self-clearing condition (e.g. read during a normal overload the
firmware already tolerates) to establish whether 65 is really the status byte. Until then,
QC's hardware-fault channel is unproven.

**Priority 3 — characterize registers 67–68.** They are not constant (256 and 16 distinct
values in every unit, consistent with a 12-bit quantity) and are undocumented in the
installed driver. They were not used in this audit.

**Priority 4 — re-run S04 after clearing the cap.** If Torque Limit / Max Torque is indeed
500, restoring 1000 and repeating QC is the only way to separate S04's focal anomaly from
its reduced torque headroom. That is a *write*, so it needs its own authorization.

---

## 11. Feetech official cross-check (FASE F)

Source: **SHENZHEN FEETECH RC MODEL CO., LTD — "ST-3215-C018 PRODUCT SPECIFICATION",
Edition A/0, dated 2023-07-20** (§5 Electrical, §6 Mechanical, §7 Control).

| Spec | Official | Measured here | Assessment |
|---|---|---|---|
| Operating voltage | 12 V (range 4–14 V) | 11.0–11.1 V measured | in range, below nominal |
| No-load speed ±10 % | 0.222 s/60° = 45 RPM @12 V | 41.2–41.7 RPM @~11.05 V | voltage-scaled expectation 41.4 RPM — **population is exactly at spec** |
| No-load running current ±10 % | 180 mA | median 117–159 mA, peak 208–273 mA @ MAX | consistent |
| Idle current | 30 mA | ~6.5 mA at 1.8 RPM (1 LSB) | below idle spec; see note |
| Rated / stall current | 900 mA / 2.7 A | never approached (max 273 mA) | no unit loaded near rating |
| Encoder | 12-bit magnetic, 0.088°/pulse | 4096 ticks/rev confirmed | matches |
| Backlash | ≤0.5° | not separately measurable | see §6 caveat |
| Over-hot protection | >70 °C torque off | max clean temp 41 °C | never approached |
| Over-current protection | >2 A for 2 s | never approached | — |
| Over-load protection | >80 % stall for 2 s | load 1000 reached at full speed only | not a stall |

Firmware constants are all corroborated: `6.5 mA/raw`, `no_load≈28 raw` (=182 mA),
`rated≈139` (=904 mA), `overcurrent≈308` (=2.00 A), `stall≈416` (=2.70 A), overload 80 %.

**Discrepancy to record:** the installed `SCServo` driver documents neither register 65 nor
67/68, while community register tables document 65 as "Servo Status". The official
specification sheet above is a product datasheet and contains no memory table at all, so it
cannot arbitrate. Reported as an open discrepancy rather than resolved by preference.

**Resolution of a concern I raised during planning:** I flagged the measured currents as
implausibly low. Against the official 180 mA no-load figure they are correct — 117–159 mA
median at 92 % of nominal voltage. The real limitation is *resolution*: at 6.5 mA/LSB, slow
moves read 1–3 LSB, so **current is not a usable diagnostic channel below MAX speed**.
`Present Load` is, and that is what the §4 detector uses.

---

## 12. Tiers

### STRONG MULTI-DOMAIN SUSPECT + CONFIGURATION-CONFOUNDED — 1 unit(s)

- **S04** — confirmed focal load anomaly repeatable across slow+medium+transfer in both directions, AND a 50% output cap that independently degrades MAX speed, precision and min-speed behaviour

### STRONG MULTI-DOMAIN SUSPECT — 1 unit(s)

- **S16** — confirmed focal load anomaly repeatable across slow+medium (frozen criterion)

### INCONCLUSIVE - CONFIGURATION-CONFOUNDED — 1 unit(s)

- **S05** — 50% output cap; every degraded metric is explainable by the cap and no localized mechanical signature survives

### CLEAN WITH MINOR NOTE — 5 unit(s)

- **S02** — 1 single-bin focal load flag(s)
- **S09** — 1 single-bin focal load flag(s)
- **S12** — 1 min-speed zone(s) below 50% of population
- **S15** — 2 cross-speed speed-bin flag(s), no focal load
- **S17** — 1 min-speed zone(s) below 50% of population

### CLEAN — 10 unit(s)

- **S01** — no metric outside the robust population envelope
- **S03** — no metric outside the robust population envelope
- **S06** — no metric outside the robust population envelope
- **S07** — no metric outside the robust population envelope
- **S08** — no metric outside the robust population envelope
- **S10** — no metric outside the robust population envelope
- **S11** — no metric outside the robust population envelope
- **S13** — no metric outside the robust population envelope
- **S14** — no metric outside the robust population envelope
- **S18** — no metric outside the robust population envelope

---

## 13. If 12 units must be chosen

**High-confidence exclusions (3):**

- **S04** — confirmed focal load anomaly at encoder 2144–2271 (188.4°–199.7°), peak +31
  load units over population, present in all 6 independent traversals in both directions,
  *plus* a 50 % output cap. Exclude on either ground.
- **S05** — 50 % output cap, and the only unit with a systematic directional min-speed
  collapse (6 of 8 zones in the + direction). Exclude, but note the cause may be entirely
  configuration; it is a candidate for recovery after a register check.
- **S16** — confirmed focal load anomaly at encoder 2560–2751 (225°–242°), peak +42 load
  units, all 6 traversals. No configuration confound, so this one is *clean mechanical
  evidence*. Its speed, precision, MAX and telemetry are all population-normal, so the
  practical severity is lower than S04's — but the localization is the most certain of the three.

**That leaves 15 candidates for 12 slots, and the data does not support choosing among
them.** The 15 are statistically indistinguishable: composite scores ≤0.23 versus 2.5–3.1
for the excluded three, and their rank order is unstable under reweighting (spread up to 10
places). Stating a "best 12" would be inventing precision that is not in the measurements.

If 12 must be picked, the only defensible tie-break is to prefer units with no minor note
at all, then accept that the remainder is arbitrary:

- **No note at all (10):** S01, S03, S06, S07, S08, S10, S11, S13, S14, S18
- **Minor note only (5):** S02 (1 single-bin focal load flag(s)); S09 (1 single-bin focal load flag(s)); S12 (1 min-speed zone(s) below 50% of population); S15 (2 cross-speed speed-bin flag(s), no focal load); S17 (1 min-speed zone(s) below 50% of population)

Only 10 units are note-free, so 2 must come from the
minor-note group. Those notes are single isolated observations and, per the frozen
criteria, carry low confidence — they are tie-breakers, not defects.

**Confidence:** exclusion of S04 and S16 on focal-anomaly grounds — **high** (survives a
leave-one-out baseline, 6 independent passes each, ≥6 robust SD, two independent detectors).
Exclusion of S05 — **high as a screening decision, low as a mechanical diagnosis**: the cap
alone may explain all of it. Inclusion of any specific 12 of the remaining 15 — **low**,
because the units are indistinguishable, not because they are doubtful.

---

## 14. Results that contradict expectations, stated explicitly

- `QC_EXECUTION: COMPLETE`, `CHECKSUM: PASS`, `PERFORMANCE_EVENTS: 0`,
  `PROTECTIVE_STOPS: 0` for **all 18** — and yet three units carry real, repeatable
  anomalies. The pass flags describe the *test*, not the servo, exactly as the protocol says.
- The two slowest units (S04, S05, at 53 % of population speed) are **not** the two with the
  clearest mechanical evidence. S16 runs at full population speed and has the largest focal
  load anomaly in the campaign.
- Conversely S05, which looks bad on five separate summary metrics, has **no** localized
  mechanical signature at all once its co-capped twin S04 is used as the control.
- No unit had a restricted travel envelope; all 18 reached 1–3 at the bottom and 4090–4095
  at the top.
- Two of my own analysis methods produced artifacts that would have created false findings
  (a fabricated 4 % MAX direction asymmetry across the whole population, and 3 spurious
  move splits). Both are documented in §1.3 rather than quietly corrected.

