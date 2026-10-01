# TRUE Full Calibration 24/24 — hardware validation report (2026-10-01)

**Verdict: TRUE FULL CALIBRATION 24/24 — `all_contact_calibrated=1`.**
- All four legs reached `HARDWARE_CONTACT_CALIBRATED`, 6/6 each, with accepted V25 diagnostics.
- Each leg ran in its own session and attempt, with no retry. Run 18:08:41 → 18:17:12.
- Final state: SAFE_OFF 13/13 verified, authority released, permit revoked, session completed for
  every leg.
- `legs_envelope_accepted=0`: no operational envelope is approved yet; none exist in this build.
- **The calibration is RAM-only.** It is lost at the next power cycle. The export below is
  evidence. It is not an implemented restore. No EEPROM, NVS, `PositionOffset`, servo-ID or
  `CalibrationOfs` write took place.

## 1. Provenance

| Item | Value |
|---|---|
| Firmware validated on hardware | `dfcecb670d0565d2db1a8152b6cd7ad230bdb87d` (BUILD_ID `dfcecb670d05`, `SOURCE_STATE=CLEAN`) |
| Profile | `ROBOT_POWERED`, `OTA_INGEST_ENABLED=0` |
| Application | `MATDOG_Controller.ino.bin`, 1,100,432 B, SHA256 `5fe625cd061b203f4067acec7e6a28ab2582449c74b611f4f66607da11c503da` |
| Flash | app-only, 2026-10-01 18:02 (`hw_session_prepare_20261001_180250.log`); source signature `dfcecb670d05 ROBOT_POWERED` verified at run start |
| Geometry V5 | bundle `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4`; URDF SHA256 `3890a3f0…1e59`; firmware geometry tag `3713f4ddc43b204e` (in every export record) |
| Installation record | `MATDOG_SERVO_ALLOCATION.yaml` SHA256 `d9c66134357acbc95c13aae81180f50843cef5cef2613a2ede7927691884e769` (= `kProvenance.allocation_sha256`, current-installation `encoder_direction`) |
| Fresh q0 (18:07:26 capture, promoted 12/12) | 11:2104 12:2078 13:1975 21:1995 22:2108 23:2030 31:2034 32:2061 33:2081 41:2073 42:2098 43:2026 (`q0_promoted.json` = export `CALIBRATION_EVIDENCE_Q0`) |

Any commit after `dfcecb6` on this branch is documentation only. The hardware-validated firmware
is `dfcecb6`, not a later commit.

**Evidence.** The originals are in `~/MATDOG/evidence/full_cal_24contact_hw_20260930/`. They are
unmodified and byte-identical to the read-only session backup in
`FINAL_24of24_20261001_180841_dfcecb670d05/` (hashes in `BACKUP_SHA256SUMS.txt`):
- export: SHA256 `d4e1f4df6dd7ea8d8873b6bcfc7e5afa98739ef6f39e06762950f88a736df1c8`, also copied here;
- full hardware log `hw_session_legs_20261001_180841.log` (2.3 MB): SHA256
  `b1d6f60eb8cb21ded1a65ce3dfb99c5624836a334d3f3d1f2cd7a7b923bf5aa3`, kept off-repo;
- the backup also holds the q0, recover and prepare logs, the manifest, the `.bin` and `.elf`, the
  allocation record, the generated profile and the encoder-direction table;
- the failed sessions of 2026-09-30/10-01 stay beside them as diagnostic evidence.

**Persistence V1 inputs.** The backup holds everything Calibration Persistence V1 needs:
- per-joint q0 with identity, unit and bus;
- the 24 contacts (scout and two fine passes), plus repeatability and witness status;
- diagnostics (span, scale, affine zero);
- the geometry tag and the source hashes;
- the firmware manifest.

## 2. The 24 contacts

Conventions:
- contact = midpoint of the two fine passes (the finalizer's `min_contact`/`max_contact`); the
  scout is reference evidence only;
- measured q = `encoder_direction · (contact − q0) / (4096 / 2π)`;
- V5 ref = the bundle's geometric contact angle. Where the bundle's q=0-context result is a
  **path-obstruction bracket**, the clear angle is shown: it is the q=0 obstruction that the
  sequence's prerequisite pose or parking removes (geometry-validated), not a contact prediction;
- margins are in ticks along the probe direction: scout past the corridor entry, and contact to
  the guard.

| leg | joint | side | unit/bus | dir | q0 | scout | fine1 | fine2 | contact (mid) | rep | meas q (°) | URDF (°) | V5 ref (°) [bracket] | Δ URDF (°) | Δ V5 (°) | entry / guard | scout past entry | contact to guard |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| LF | HIP | MIN | M22/13 | -1 | 1975 | 2511 | 2506 | 2503 | 2504 | 3 ✓ | -46.49 | -45.00 | -46.01 [contact] | -1.49 | -0.48 | 2423 / 2551 | +88 | +47 |
| LF | HIP | MAX | M22/13 | -1 | 1975 | 1489 | 1476 | 1472 | 1474 | 4 ✓ | +44.03 | +45.00 | +45.22 [contact] | -0.97 | -1.19 | 1527 / 1399 | +38 | +75 |
| LF | UPPER | MIN | ELR01/12 | +1 | 2078 | 1460 | 1463 | 1459 | 1461 | 4 ✓ | -54.23 | -52.50 | -52.13 [contact] | -1.73 | -2.10 | 1545 / 1417 | +85 | +44 |
| LF | UPPER | MAX | ELR01/12 | +1 | 2078 | 3479 | 3476 | 3478 | 3477 | 2 ✓ | +122.96 | +122.50 | +121.87 [obstruction@q=0 ctx, clear +73.28] | +0.46 | +1.08 | 3408 / 3536 | +71 | +59 |
| LF | LOWER | MIN | M33/11 | -1 | 2104 | 3152 | 3149 | 3149 | 3149 | 0 ✓ | -91.85 | -92.00 | -92.07 [obstruction@q=0 ctx, clear -84.27] | +0.15 | +0.23 | 3087 / 3215 | +65 | +66 |
| LF | LOWER | MAX | M33/11 | -1 | 2104 | 1706 | 1712 | 1712 | 1712 | 0 ✓ | +34.45 | +37.50 | +38.18 [contact] | -3.05 | -3.73 | 1741 / 1613 | +35 | +99 |
| RF | HIP | MIN | NEW01/23 | -1 | 2030 | 2551 | 2545 | 2543 | 2544 | 2 ✓ | -45.18 | -45.00 | -45.22 [contact] | -0.18 | +0.05 | 2478 / 2606 | +73 | +62 |
| RF | HIP | MAX | NEW01/23 | -1 | 2030 | 1492 | 1498 | 1497 | 1497 | 1 ✓ | +46.85 | +45.00 | +46.01 [contact] | +1.85 | +0.83 | 1582 / 1454 | +90 | +43 |
| RF | UPPER | MIN | ELR03/22 | -1 | 2108 | 2726 | 2730 | 2726 | 2728 | 4 ✓ | -54.49 | -52.50 | -52.13 [contact] | -1.99 | -2.36 | 2641 / 2769 | +85 | +41 |
| RF | UPPER | MAX | ELR03/22 | -1 | 2108 | 717 | 721 | 722 | 721 | 1 ✓ | +121.90 | +122.50 | +121.87 [obstruction@q=0 ctx, clear +73.28] | -0.60 | +0.03 | 778 / 650 | +61 | +71 |
| RF | LOWER | MIN | NEW03/21 | +1 | 1995 | 936 | 938 | 937 | 937 | 1 ✓ | -92.99 | -92.00 | -92.07 [obstruction@q=0 ctx, clear -84.27] | -0.99 | -0.91 | 1012 / 884 | +76 | +53 |
| RF | LOWER | MAX | NEW03/21 | +1 | 1995 | 2364 | 2359 | 2356 | 2357 | 3 ✓ | +31.82 | +37.50 | +38.18 [contact] | -5.68 | -6.36 | 2358 / 2486 | +6 | +129 |
| RH | HIP | MIN | NEW06/33 | +1 | 2081 | 1557 | 1563 | 1565 | 1564 | 2 ✓ | -45.44 | -45.00 | -45.16 [contact] | -0.44 | -0.28 | 1633 / 1505 | +76 | +59 |
| RH | HIP | MAX | NEW06/33 | +1 | 2081 | 2603 | 2595 | 2595 | 2595 | 0 ✓ | +45.18 | +45.00 | +46.01 [contact] | +0.18 | -0.84 | 2529 / 2657 | +74 | +62 |
| RH | UPPER | MIN | ELR02/32 | -1 | 2061 | 2672 | 2668 | 2670 | 2669 | 2 ✓ | -53.44 | -52.50 | -52.13 [contact] | -0.94 | -1.30 | 2594 / 2722 | +78 | +53 |
| RH | UPPER | MAX | ELR02/32 | -1 | 2061 | 655 | 659 | 659 | 659 | 0 ✓ | +123.22 | +122.50 | +121.87 [contact] | +0.72 | +1.35 | 731 / 603 | +76 | +56 |
| RH | LOWER | MIN | NEW05/31 | +1 | 2034 | 987 | 992 | 991 | 991 | 1 ✓ | -91.67 | -92.00 | -92.07 [obstruction@q=0 ctx, clear -84.27] | +0.33 | +0.40 | 1051 / 923 | +64 | +68 |
| RH | LOWER | MAX | NEW05/31 | +1 | 2034 | 2429 | 2424 | 2421 | 2422 | 3 ✓ | +34.10 | +37.50 | +38.18 [contact] | -3.40 | -4.08 | 2397 / 2525 | +32 | +103 |
| LH | HIP | MIN | M43/43 | +1 | 2026 | 1494 | 1502 | 1502 | 1502 | 0 ✓ | -46.05 | -45.00 | -46.01 [contact] | -1.05 | -0.04 | 1578 / 1450 | +84 | +52 |
| LH | HIP | MAX | M43/43 | +1 | 2026 | 2546 | 2541 | 2543 | 2542 | 2 ✓ | +45.35 | +45.00 | +45.16 [contact] | +0.35 | +0.20 | 2474 / 2602 | +72 | +60 |
| LH | UPPER | MIN | M42/42 | +1 | 2098 | 1482 | 1486 | 1488 | 1487 | 2 ✓ | -53.70 | -52.50 | -52.13 [contact] | -1.20 | -1.57 | 1565 / 1437 | +83 | +50 |
| LH | UPPER | MAX | M42/42 | +1 | 2098 | 3489 | 3484 | 3485 | 3484 | 1 ✓ | +121.82 | +122.50 | +121.87 [contact] | -0.68 | -0.06 | 3428 / 3556 | +61 | +72 |
| LH | LOWER | MIN | M41/41 | -1 | 2073 | 3133 | 3133 | 3133 | 3133 | 0 ✓ | -93.16 | -92.00 | -92.07 [obstruction@q=0 ctx, clear -84.27] | -1.16 | -1.09 | 3056 / 3184 | +77 | +51 |
| LH | LOWER | MAX | M41/41 | -1 | 2073 | 1697 | 1702 | 1705 | 1703 | 3 ✓ | +32.52 | +37.50 | +38.18 [contact] | -4.98 | -5.66 | 1710 / 1582 | +13 | +121 |

**Repeatability:** fine 1 vs fine 2 ≤ 4 ticks on all 24 (limit 16). Witness accepted 24/24.

## 3. Spans per joint (from the fine-pass midpoints)

| leg | HIP span (°) | UPPER span (°) | LOWER span (°) | HIP scale ‰ | UPPER scale ‰ | LOWER scale ‰ |
|---|---|---|---|---|---|---|
| LF | 90.53 | 177.19 | 126.30 | 1006 | 1013 | 975 |
| RF | 92.02 | 176.40 | 124.80 | 1022 | 1008 | 963 |
| RH | 90.62 | 176.66 | 125.77 | 1007 | 1010 | 971 |
| LH | 91.41 | 175.52 | 125.68 | 1016 | 1003 | 970 |
| URDF | 90.00 | 175.00 | 129.50 | | | |

**Findings:**
- **HIP:** within ±1.9° of the URDF ±45°, and ±1.2° of the V5 contacts.
- **UPPER MIN:** 0.9–2.0° past the URDF −52.5° on every leg, as V25 also measured (−53.53°).
  UPPER MAX lies within ±0.7° of the URDF limit.
- **LOWER MIN:** matches the CAD within 1.2° (−91.7° to −93.2° against V5 −92.07°).
- **LOWER MAX is systematically short.**
  - Measured +31.8° to +34.5° on all four legs: 3.0–5.7° before the URDF +37.5° and 3.7–6.4°
    before the V5 contact +38.18°.
  - V25 measured the same on its own installation (+34.28°).
  - So the deficit is a property of the mechanism or its CAD, not of q0 placement. A q0 offset
    would move MIN and MAX by the same amount, whereas MIN agrees with the CAD.
  - LOWER spans are 124.8–126.3° against the URDF's 129.5° (scale 963–975 ‰, inside the V25 gate
    of 850–1150 ‰).
  - Nothing is changed: no URDF, Geometry V5, corridor or detector change is authorized.
- **Reduced scout margins**, a direct consequence of the LOWER MAX deficit (corridor entry =
  URDF − 64 ticks = +31.9°):
  - **RF LOWER MAX: the scout was only +6 ticks past the entry.** The fine midpoint 2357 is 1 tick
    shallower than the entry (2358) and was accepted by V25's adaptive scout − 32 rule.
  - **LH LOWER MAX: +13 ticks.**
  - LF +35, RH +32.
  - This is the R1 risk class. A slightly shallower stop or a different q0 placement can fail
    RF LOWER MAX with `EARLY_STALL_OUTSIDE_CORRIDOR` on a repeat run (fail closed). A corridor
    decision needs its own reviewed change; none is made here.

## 4. Historical comparison — LF V25 (previous installation)

- **What is compared.** V25 ran on the **previous** installation (units M11/M12/M13, frozen
  `PositionOffset`, displayed home 2048, directions HIP −1 / UPPER +1 / LOWER −1). Its raw ticks
  are **not** compared with today's raw ticks; only angles and spans are. The V25 angles
  reproduce the archive README exactly (−42.803 / +39.375, −53.525 / +122.607, −91.846 / +34.277).
- **Units.** Today's LF units are M22 (HIP), ELR01 (UPPER) and M33 (LOWER), installed after
  2026-08-27.

| joint | V25 MIN raw | V25 MAX raw | V25 dir | V25 MIN (°) | V25 MAX (°) | V25 span (°) | current LF MIN (°) | current LF MAX (°) | current span (°) | Δ span (°) | Δ MIN (°) | Δ MAX (°) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| HIP | 2535 | 1600 | -1 | -42.80 | +39.38 | 82.18 | -46.49 | +44.03 | 90.53 | +8.35 | -3.69 | +4.66 |
| UPPER | 1439 | 3443 | +1 | -53.53 | +122.61 | 176.13 | -54.23 | +122.96 | 177.19 | +1.05 | -0.70 | +0.35 |
| LOWER | 3093 | 1658 | -1 | -91.85 | +34.28 | 126.12 | -91.85 | +34.45 | 126.30 | +0.18 | +0.00 | +0.18 |


- **UPPER and LOWER** reproduce V25 within 1.1° on both endpoints, on different units, a
  different installation and a fresh q0.
- **HIP gained +8.35° of span, on BOTH endpoints:** MIN by −3.69° and MAX by +4.66°.
  - A q0 difference shifts both endpoints the same way. The opposite-signed shifts are therefore
    a genuine range increase of about 4.2° per side, plus about 0.5° that q0 placement can explain.
  - The principal identified explanation is a mechanical revision, confirmed by the operator
    (Matteo, 2026-10-01). After LF V25 a new upper cover was fitted. It defines the upper
    mechanical end-stop and allows more HIP travel than the previous main-base cover.
  - The operator also confirms that the current URDF, visual and collision meshes used by
    Geometry V5 represent this revision.
  - Consistent with that, today's LF HIP (−46.49° / +44.03°, span 90.53°) matches the URDF ±45°
    and the V5 contacts (−46.01° / +45.22°) within 1.2°.
  - This report does not establish which cover feature stops which side. No CAD, URDF or
    Geometry change is warranted by this comparison.

## 5. Return to q0 and the observed LOWER pose

| leg | joint | fresh q0 | return target | after SAFE_OFF (runner snapshot after the LH leg) | err ticks | err ° | affine zero | affine − q0 | fixed-endpoint disagreement |
|---|---|---|---|---|---|---|---|---|---|
| LF | HIP | 1975 | 1975 | 1975 | +0 | +0.00 | 1989 | +14 (+1.23°) | 6 |
| LF | UPPER | 2078 | 2078 | 2080 | +2 | +0.18 | 2065 | -13 (-1.14°) | 25 |
| LF | LOWER | 2104 | 2104 | 2102 | -2 | -0.18 | 2128 | +24 (+2.11°) | 37 |
| RF | HIP | 2030 | 2030 | 2024 | -6 | -0.53 | 2020 | -10 (-0.88°) | 23 |
| RF | UPPER | 2108 | 2108 | 2106 | -2 | -0.18 | 2126 | +18 (+1.58°) | 16 |
| RF | LOWER | 1995 | 1995 | 1997 | +2 | +0.18 | 1946 | -49 (-4.31°) | 54 |
| RH | HIP | 2081 | 2081 | 2086 | +5 | +0.44 | 2080 | -1 (-0.09°) | 7 |
| RH | UPPER | 2061 | 2061 | 2058 | -3 | -0.26 | 2066 | +5 (+0.44°) | 19 |
| RH | LOWER | 2034 | 2034 | 2036 | +2 | +0.18 | 2007 | -27 (-2.37°) | 43 |
| LH | HIP | 2026 | 2026 | 2026 | +0 | +0.00 | 2022 | -4 (-0.35°) | 16 |
| LH | UPPER | 2098 | 2098 | 2101 | +3 | +0.26 | 2086 | -12 (-1.05°) | 6 |
| LH | LOWER | 2073 | 2073 | 2072 | -1 | -0.09 | 2117 | +44 (+3.87°) | 44 |

**How the return works.** Every RETURN/RESTORE target in the log is the **promoted fresh q0**
(for example LF RETURN_HIP 1975, RETURN_LOWER 2104 and RETURN_UPPER 2078, and the park back to
2098). Affine zeros are diagnostics only: they are never promoted and never commanded. After
SAFE_OFF every joint read within **6 ticks (0.53°)** of q0, inside the 16-tick rest gate. The
largest offsets are on the HIPs that carried load: RF HIP −6, RH HIP +5.

**The affine zero is not a q0-placement signal for LOWER.** The LOWER affine zeros differ from q0
by −2.1° to −4.3° in joint-q terms, on all four legs and in the same sense. That is the systematic
LOWER MAX deficit (§3) pulling the endpoint-midpoint model; it is not a manual-q0 error. LOWER MIN
alone agrees with the CAD within 1.2°.

**The operator's observation** (some legs, especially the LOWERs, looked slightly off their manual
q0 pose after the run). It is **not** an encoder positioning error: the servos returned within
≤ 6 ticks. Compatible explanations are:
- backlash and compliance of the horn/linkage chain beyond the servo encoder, relaxing under
  gravity once torque is off;
- the accuracy of the original hand placement of q0 itself, which no encoder can reveal.

The data cannot separate these. The current tolerances are met. Before operational motion,
consider a **separate, reviewed zero-refinement procedure** (for example a LOWER-MIN-referenced
zero, since MIN matches the CAD). Nothing was re-promoted and no contact evidence was invalidated.

## 6. Telemetry observations (two separate findings)

**A. 23 isolated over-limit PresentTemperature readings**
- **Values:** single readings of 71–130 °C on 10 of the 12 leg buses: 13×4, 32×5, 21/31/33/41/42/43×2,
  and 22/23×1.
- **Confirmation:** every one was followed by two direct reads at 26–32 °C and classified
  TRANSIENT by the V25 confirmation. None was CONFIRMED.
- **Interpretation:** the values are not physical. This is a telemetry-integrity question about
  the bulk feedback read. The confirmation stays as is.

**B. 39 `CALIBRATION_HELD_SPEED_TRANSIENT` (diagnostic only)**
- **Where:** held joints during the neighbour's UPPER MAX / RETURN motions. RF LOWER 9, LH LOWER 8,
  LF LOWER 6, RH LOWER 6, LH HIP 5, LF park 2, and four buses with one each.
- **Values:** all 39 read exactly `speed=50`, with position error 0–5 ticks inside the hold. No
  held-role failure occurred.
- **Interpretation:** `speed=50` is the ST3215 speed register's smallest step, consistent with a
  single encoder tick of dither on a loaded hold. The retired D5 rule would have aborted on these.

**Relation between A and B:** not established. A is single implausible values on any bus. B is a
quantized, physically plausible reading on held joints during the neighbour's motion. Neither
should be suppressed. Every fail-closed check stays as it is.

**Bounded follow-up (before any hardware gait authorization):**
- record, for every out-of-range bulk frame, the raw feedback bytes and read status;
- compare against a direct read of the same registers;
- quantify the rate at rest and during motion;
- confirm the speed-register quantum on the bench;
- decide whether a frame-level integrity check belongs in `readControlFeedback`.

The calibration engine is not weakened.

## 7. Limitations carried forward

- **Operational envelopes:** not approved (`parameters_approved=0`, `envelope_accepted=0`); no
  stand or gait authorization exists.
- **Calibration Persistence:** not implemented; the result is RAM-only.
- **LOWER MAX:** a systematic deficit of 3.0–5.7° against the URDF, and a thin RF LOWER MAX scout
  margin (+6 ticks).
- **Telemetry integrity:** the §6 follow-up is open.
