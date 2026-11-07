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
| Fase B gait: cadenza selezionabile se ammissibile, sway documentato | IN CORSO | — |
| Build ROBOT_POWERED CLEAN + manifest/SHA | DA FARE | — |
| Draft PR + commento #52 | DA FARE | — |

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

T2 in `SimpleMotion::generateWalk`: soluzione nominale esatta a release
alterne (look-ahead di un periodo), interpolazione lineare in spazio giunti
sulle altre, tutte le prove (pointSafe, box, compensationSafe/StopPlan) a ogni
release; T1 leggera solo sulle release con nominale esatto; shadow test
dell'errore di interpolazione.
