# Motion Performance (post v1.0.1) — checkpoint recuperabile

Sessione autonoma Claude Code, 2026-11-07 notte. Worktree
`~/MATDOG/worktrees/matdog-motion-perf-next-20261107`, branch
`feat/motion-perf-next-20261107` da `main` 40728b7 (Stable v1.0.1, firmware
installato dev.17 `17db0ef`, APP `14e055a0…`). **Nessuna azione hardware.**
Kit di ingresso estratto in `~/MATDOG/verification-artifacts/MATDOG_MOTION_PERF_KIT_20261107/`
(SHA256 verificati).

## Stato (aggiornato a ogni fase)

| Fase | Stato | Commit |
|---|---|---|
| Verifica remoto/kit/worktree | FATTA | — |
| RCA tracce dev.16/dev.17 per fase | FATTA (`tools/trace_phase_analysis.py`) | 7029e17 |
| A2 IK analitica esatta (remainder/atan2/costanti) | FATTA, bit-identica | 7029e17 |
| Probe costi esteso (hypot/remainder/floor) | FATTA | 99329c5 |
| A1 campionamento gait scalare | FATTA, bit-identica | 52e2865 |
| T2 nominale a release alterne + interpolazione | FATTA, suite PASS, errore interp < 1 tick | d7ed9da |
| T1b riuso T1 tollerante ai retarget joystick + test T2 | FATTA | affa60c |
| Checker inviluppo tilt WALK / cadenza | FATTA (baseline PASS; stop 0.5 FAIL; cadenze >1,25× FAIL) | d890da7 |
| Fase B gait: nessuna cadenza ammissibile senza nuovo profilo servo; stop 0,5 e ramp 1 bocciati; sway documentato | FATTA (REPORT §6) | — |
| T2 frame di stack (`solveGaitInto`, in place) | FATTA | 2569401, e01e95a, 2d95b99 |
| Build ROBOT_POWERED CLEAN + manifest/SHA | FATTA: **128c480**, APP 1.718.304 B, SHA256 e3e69368… (sostituisce 131ca53) | — |
| Mandato integrato A–D: rate STOP su storia, una risoluzione/release, sway joystick proporzionale (gate PASS), mappa cadenza/legge (nessun preset ammissibile senza qualifica drive), WALK_SETTLE decomposto, istogrammi D11 | FATTA (REPORT §13) | 172726d, 05e8423, 128c480 |
| Revisione T2 retarget/jitter: difetto confermato e corretto (riferimento a `now`, rate STOP ≥ 1 periodo) | FATTA, 36 semi 0 fault (Stable 3) | 131ca53 |
| Static audit completo | PASS | 27c409f |
| Draft PR #67 + commento #52 | FATTA; commento di revisione in aggiunta | — |
| Revisione P1 rate STOP: invariante su griglia, fallback solo con passo 0, casi diretti; nessuna patch | FATTA (REPORT §14) | 7c8ea01 |
| Revisione P2 sway: buco fisico della mappa lineare (contatto sul raccordo a ~5,5 mm, −0,08 mm al 3,3 %); nuova mappa 2–5 mm / 14 mm con isteresi, cambi solo a 4 appoggi; gate 32 scenari | FATTA | 7c8ea01 |
| Difetto preesistente inviluppo steered in rotazione > 4 cicli (anche v1.0.1): proiezione nel frame heading; gate con giri completi | FATTA | 4c897ed |
| P3: audit configurazione ESP32-S3, -Os/-O2 selettivo e double/float (entrambi respinti con misure), scheduling | FATTA (REPORT §14) | — |
| P4: vincolo di cadenza attivo (StopPlan bordo UPPER kHeight), 6,2 s unico periodo robusto (+5 %), periodo invariato | FATTA | 4c897ed |
| P5: costo istogrammi misurato su ELF (+156 B codice, +72 B RAM) | FATTA | — |
| Build ROBOT_POWERED CLEAN finale | FATTA: **4c897ed**, APP 1.718.736 B, SHA256 03c84606… (sostituisce 128c480) | — |

## Risultati chiave finora

- Hardware dev.17 (VERIFIED HARDWARE dalle tracce): WALK STANDARD release
  tipica piena ~17–18 ms, leggera (T1) ~11–12 ms; overrun 9,4 % da inflazione
  ×1,7–2 dello stesso lavoro (max 33 ms). JOYSTICK: piena tipica ~29,5 ms
  (strutturalmente oltre budget), leggera ~17,5 ms; 20 % overrun, 1,3 perse/s.
  HOLD: 0 overrun.
- Host (proxy libm/release): WALK45 383 → 321 (−16 %), JOYSTICK 484 → 410
  (−15 %) con A1+A2, senza alcun cambiamento di target (digest identici).

## Baseline riproducibili

- `scripts/offline_motion/run_motion_frame_digest.sh` (digest frame RAW):
  pristine = `/tmp/mp-baseline/digest_pristine.txt` (ricalcolabile da main).
- `scripts/offline_motion/run_motion_cost_probe.sh` (libm/release).
- `scripts/tests/run_simple_motion_tests.sh` PASS su baseline.

## Risultati Fase B (modello, nessun hardware)

- `kWalkStopCycles` 0,5: la legge manda i touchdown dell'ultimo ciclo al punto
  terminale mentre il corpo è a piena velocità → nominale fuori kHeight (−59/−139
  tick). Non praticabile senza riscrivere la regola di ancoraggio.
- Cadenza (periodo) con legge e drive attuali {1000,50}: 1,25× passa solo a
  30/45 mm; 60 mm fallisce StopPlan (6/44396) già a 1,25×; ≥1,5× fallisce per
  accelerazione articolare (9,8→17,5 rad/s² contro 7,67) e StopPlan. In
  firmware un fallimento di compensationSafe in WALK è `WALK_IK_OR_WORKSPACE`
  → SAFE_OFF: nessuna cadenza con fallimenti è ammissibile.

## Prossimo passo

Collaudo hardware supervisionato secondo `HW_TEST_PLAN.md` (decisione
dell'operatore), dopo accordo sul nome `1.0.2-dev.1`.
