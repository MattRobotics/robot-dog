> **Stable corrente (19/11/2026) — `v1.0.2`: firmware `1.0.2-dev.7` / `1314bc411c58` (APP `dd16bd62…`), Motion fixture-qualified.** [Indice della qualifica](Motion_1314bc4_HW_2026-11-19/README.md) · [rapporto del collaudo](Motion_1314bc4_HW_2026-11-19/qualification/REPORT_HW.md) · [blocco D e S9](Motion_1314bc4_HW_2026-11-19/completion/REPORT_HW_COMPLETION.md) · [manifest della release](Motion_1314bc4_HW_2026-11-19/RELEASE_MANIFEST.md) · [evidenze](Motion_1314bc4_HW_2026-11-19/EVIDENCE_MANIFEST.md).
> - **Prove:** catena 8/8, D1/D2 con STOP HTTP, blocco D 27/27, S9 15/15.
> - **Motion Timing P0 CLOSED — VERIFIED HARDWARE nel perimetro delle prove:** massimo 12 729 µs, 0 overrun, 0 perse ([ADR-008](../Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md)). La locomozione a terra non è qualificata.
> - **Prima della qualifica:**
>   - build e verifiche offline: [Motion_Integrated_1314bc4_2026-11-19](Motion_Integrated_1314bc4_2026-11-19/REPORT.md);
>   - FP32 e P1: [Motion_FP32_2026-11-18](Motion_FP32_2026-11-18/REPORT.md);
>   - `kSteer`: [Motion_SteerSlow_Recert_2026-11-18](Motion_SteerSlow_Recert_2026-11-18/README.md);
>   - campagna P0 del 17/11: [rapporto](MATDOG_1_0_2_P0_2026-11-17/REPORT.md).
> - **Rollback:** [`30c894a`, qualifica del 18/11](Motion_30c894a_HW_2026-11-18/README.md).
> - **Nel firmware anche:** [Ground Contact Fase 1](Ground_Contact_Phase1_2026-11-13/REPORT.md) (13/11), diagnostica passiva VERIFIED OFFLINE, mai provata sul robot; nessuna misura assoluta del terreno.
> - **DALY:** [A/B del 18/11](BMS_DALY_AB_2026-11-18/REPORT_AB.md) INCONCLUSIVE, [B2 del 19/11](BMS_DALY_B2_2026-11-19/README.md) CLEAN; RCA congelata, causa UNKNOWN.

> **Storico — branch operativo unico (14/11/2026) — `integ/matdog-unified-operational-20261114`:** [handoff](Unified_Operational_2026-11-14/HANDOFF.md) — Battery P1 + Motion dev.7, Ground Contact Fase 1 e ricerca MuJoCo vNext in un solo branch; consolidamento solo Git, nessun flash.

> **Storico — robot installato (16/11/2026 sera) — diagnostica `6be46906d069` (`a86c680` + F3 + IRAM G2s, APP `be678d83…3b68`):** [rapporto P0 F3 G2s](Unified_Operational_2026-11-14/P0_F3_G2s_2026-11-16/REPORT.md).
> - **Validazione su fixture:** mirata 12/12, A/B 18/18, FAST E–G 20/20 (29,6 mm/s, STOP nel FAST 2,2 s), H 20/20, nessun fault.
> - **Rispetto ad `a86c680` nella stessa sessione:** −6…−10 % per release, release perse 5 → 0, overrun 6 → 0.
> - **Memoria:** heap minimo 25 672 B, margine 1 096 B con 2 client HTTP.
> - **Timing P0 non chiuso:** 2 overrun, 20,8 ms.
> - **Nello stesso giorno:** P5 su `a86c680` (FAULT F3 nel FAST 32 mm/s) e MEMORY_STOP di `cb9ed85` (G2) con rollback.
> - **Branch:** la legge Motion qualificata è integrata qui senza strumentazione diagnostica ([ADR-007](../Architecture_Decisions/ADR-007_Motion_Timing_Integration_and_Heap_Margin.md)); build di verifica `33134ae28260` non installata.

> **Storico — robot installato (14/11/2026 sera) — `1.0.2-dev.8` diagnostica / `8530d3a11ac9` (APP SHA256 `75fdd3ac…0c4d4b`):** [installazione APP-only e boot](Unified_Operational_2026-11-14/DIAG_INSTALL_8530d3a.md) — VERIFIED HARDWARE PASSIVE (boot, NVS gen-1, census 13/13, IMU, DALY + D1, SAFE_OFF, 8 min stabili) · [validazione Motion su fixture del 15/11](Unified_Operational_2026-11-14/HW_VALIDATION_8530d3a_20261115.md): riarmo STOP → STAND e ripresa dopo yaw (Patch A) VERIFIED HARDWARE; suite A–G 65/65 PASS; Timing: ≤ 7 perse in 30 s ma 30 perse e 677 overrun, picco 39,67 ms, stallo istruzioni dominante → **Timing P0 non chiuso**; interruzione DALY di 440 s prima della suite, causa UNKNOWN. Sorgente = branch operativo `539b646` + strumentazione Timing, non pubblicata.

> **Storico — robot installato (15/11/2026 pomeriggio) — diagnostica IRAM `e321e8668139` (APP `b88c448d…`):** [validazione hardware e RCA DALY](Unified_Operational_2026-11-14/HW_VALIDATION_e321e86_20261115.md) — flash APP-only PASS; suite A–G 65/65 in un'unica esecuzione; Timing −8/−11 % per release (stalli istruzione 0,47–0,50 → 0,37–0,45), perse 28 → 10, overrun 648 → 323, picco 39,67 → 36,11 ms, heap −4,6 KB → **Timing P0 non chiuso**. DALY: guasto a freddo con risposte troncate a metà frame (98/129, 61/129), rientro spontaneo, BMS vivo (SOC continuo); **causa UNKNOWN circoscritta**, nessun difetto software dimostrato, nessuna patch.

> **Sviluppo offline (15/11/2026):** [rapporto](Unified_Operational_2026-11-14/Offline_Dev_2026-11-15/REPORT.md) — candidato IRAM G1 (miss I-cache simulati −36/−43 % medi, heap −4,6 KB, frame identici, non pubblicato), causa del gap di 147 ms della prima STAND (soglia ACC, assestamento di 100 ms), RCA DALY (`8e20c63` equivalente sul filo, causa UNKNOWN, [procedura con analizzatore logico](Unified_Operational_2026-11-14/Offline_Dev_2026-11-15/LOGIC_ANALYZER_PROCEDURE.md)), baseline della locomozione nel gemello sulla legge attuale. FAST 40 non fattibile nelle condizioni attuali.

> **Chiusura strutturale del Motion Timing P0 (15/11/2026 sera, offline):** [rapporto](Unified_Operational_2026-11-14/P0_Structural_2026-11-15/REPORT.md) — IRAM G1 integrato in questo branch (`f3b0d48`, non flashato). RCA dei picchi: soluzione nominale + compensazione piena con IK sul bordo del piede nella stessa release. Candidato di ricerca T2R1M `0fb22b0` (branch locale, 4 istruzioni, nessuno stato): QEMU WALK max −37 %, RECOVERY media −40 %, stesse sequenze su 64 scenari, errore IMU medio in WALK +18 %. **16 ms non raggiungibili con T2/T1** (IK di bordo, resync, ALIGN/DEPLOY). D2 read-only UART2 `4e3a740` e diagnostica integrata `46b3175` per una sola sessione fixture, non flashati. **Timing P0 non chiuso.**

> **Storico — robot installato (16/11/2026 mattina) — diagnostica `46b31757eb81` (`e321e86` + T2R1M + D2, APP `57508746…`):** [validazione hardware](Unified_Operational_2026-11-14/HW_VALIDATION_46b3175_20261116.md) — flash APP-only PASS; suite A–G 65/65 in un'unica esecuzione + blocco H (STOP durante la ripresa, a ~50 e ~15 release) 19/19; RECOVERY 15,1–16,4 → 10,9–12,0 ms per release, WALK invariato, picco 36,1 → 27,0 ms, overrun 323 → 157, 0 perse in WALK/RECOVERY (2 all'attivazione da PRONE_REST) → **Timing P0 non chiuso**. Errore IMU sterzato +16 %. DALY: nessun silenzio (freddo 601/601, D2 attiva con baseline): **causa UNKNOWN, non risolta**.

> **Storico — robot installato (16/11/2026 sera) — diagnostica `2638204fb4ef` (`82c1af9` + P4):** [validazione hardware](Unified_Operational_2026-11-14/HW_VALIDATION_2638204_20261116.md) — A–G 65/65 + H 19/19, 0 perse; overrun 29 → 47 spiegati da un fattore di sessione (+5–7 % per istruzione su entrambe le build), non da P4, che però non riduce le release in overrun (esclusa). **Chiusura P0 offline** ([rapporto](Unified_Operational_2026-11-14/P0_Closure_2026-11-16/REPORT.md)): picco WALK_SETTLE = prova STOP rifiutata dopo la ricerca completa (riprodotto offline, ipotesi forte sul chip); **P5** esatta (equivalenza bit a bit), candidato `a86c680` = `82c1af9` + P5 non flashato; P0 non chiudibile con P5 (4–7 % delle release fra 15 e 20 ms, ~90 % nei cammini). Silenzio DALY del 16/11 catturato con D2 ([analisi](Unified_Operational_2026-11-14/P0_Closure_2026-11-16/DALY_EPISODE_20261116.md)).

> **Storico — robot installato (16/11/2026 pomeriggio) — diagnostica `82c1af95e0a5` (`46b3175` + E1/E1b/P3a/P3b + D3, APP `93c61562…`):** [validazione hardware](Unified_Operational_2026-11-14/HW_VALIDATION_82c1af9_20261116.md) — flash APP-only PASS; A–G 65/65 + H 19/19; WALK 9,8–10,7 ms (−10/−12 %), RECOVERY 9,8–10,6 ms; overrun 161 → 31; 0 perse operative; picchi WALK 23,7, RECOVERY 19,8, OTHER 26,8 ms (WALK_SETTLE dopo un rilascio sterzato, causa da misurare) → **Timing P0 non chiuso**. D3: dump diagnostica 20,7–21,4 ms, registro 85 ~4 ms, 0 perse a riposo. Criterio IMU sterzato non soddisfatto sul massimo (13,87 contro 13,30 mrad a 1 Hz). Prima causa residua (18/31 overrun, release attenuate a 2 soluzioni) → **P4**, candidato `2638204` verificato offline, non flashato.

> **Picchi residui dopo T2R1M (16/11/2026, offline):** [rapporto](Unified_Operational_2026-11-14/P0_Peaks_2026-11-16/REPORT.md) — branch locale `research/p0-peaks-20261116`: E1 (seme dell'IK di compensazione), E1b (anche nella ripresa), P3a (`ViaPlan::plan`, identico bit per bit), P3b (release del piano). QEMU WALK max 1 430 → 1 172 k, RECOVERY 2 156 → 1 562 k, OTHER 2 045 → 1 376 k; non flashati. Prossimo candidato diagnostico `82c1af9` (+ D3), costruito e auditato, [piano della sessione](Unified_Operational_2026-11-14/P0_Peaks_2026-11-16/NEXT_SESSION_PLAN_82c1af9.md). Le 2 perse «da PRONE_REST» sono probabilmente un artefatto della dump diagnostica (da confermare con D3). Cattura a freddo B8 con D2: 601/601 poll OK, causa DALY UNKNOWN. [Valutazione della locomozione](Unified_Operational_2026-11-14/P0_Peaks_2026-11-16/LOCO_EVAL.md) e [piano delle misure a terra](Unified_Operational_2026-11-14/P0_Peaks_2026-11-16/GROUND_MEASUREMENT_PLAN.md): FAST 40 non certificabile con i box e la frenata attuali, TROT non supportato dalle evidenze; dato mancante principale = C per giunto.

> **Storico — robot installato l’11/11/2026, `1.0.2-dev.7` / `b5610620d0ce` (APP SHA256 `28744982…6053d4`), Motion vNext + Battery P1:** [VALIDATED HARDWARE nel perimetro provato, NOT STABLE](MATDOG_DEV7_HW_2026-11-11/HW_REPORT.md) — fixture e terra senza Jetson, 14/14 STOP verificati (4 in avanti a terra nei tre modi), cedimento statico HIP ≤ 4 tick; aperti: timing a terra (fino a 43 release perse in 30 s), cambio da FAST verso il basso, STANDARD 25 mm/s, sway laterale, letture di temperatura · [evidenze e SHA256](MATDOG_DEV7_HW_2026-11-11/EVIDENCE_INDEX.md) · [punti aperti](MATDOG_DEV7_HW_2026-11-11/OPEN_ITEMS.md) · [ripresa](MATDOG_DEV7_HW_2026-11-11/NEXT_SESSION_HANDOFF.md). Rollback `450d2c7`. La Stable resta `v1.0.1`.

> **Storico — robot installato il 10/11/2026, `1.0.2-dev.6` ACC 220 / `450d2c793dcb` (APP SHA256 `82122e06…6256`):** [validazione hardware](Dev6_ACC220_HW_2026-11-10/REPORT.md) · [manifest delle evidenze](Dev6_ACC220_HW_2026-11-10/EVIDENCE_MANIFEST.md) · [rapporto originale](Dev6_ACC220_HW_2026-11-10/REPORT_HW_SESSION.md).
> - **Esito:** fixture Q1–Q8 qualificata; 14/14 STOP verificati; 0 fault.
> - **A terra:** verifica positiva ma fuori protocollo G0–G5.
> - **Stato:** build di sviluppo, non una release; sorgente in `main` dalla PR #67.
>
> **Gemello MuJoCo V1 (offline, 10/11/2026):** [rapporto M5](../../06_Software/Matdog_MuJoCo/REPORT_M5.md), confronto HW/SIM sulla sessione del 10/11. Non è un'evidenza hardware.

> **Storico — candidata offline per il prossimo flash (09/11/2026) — `1.0.2-dev.6` / `2c53ecfad664` (APP SHA256 `eef32e61…14c1`):** dev.5 più la correzione emersa dalla [review pre-flash](Night_2026-11-09/REVIEW_JOYSTICK_LAW.md) (banda frontale applicata anche nella camminata sterzata). IMPLEMENTED NOT VALIDATED HARDWARE.

> **Candidata offline (09/11/2026) — `1.0.2-dev.5` / `3afe6ce10a2e` (APP SHA256 `da6d0ad6…8cc0`), NON flashata:** [rapporto della notte](Night_2026-11-09/REPORT.md) — recupero della posa d'ingresso senza riavvio, joystick avanti/indietro sulla legge WALK STANDARD (20/10 mm/s), stack della loop task −4,9 KB nel caso peggiore statico, diagnostica della prima causa. IMPLEMENTED NOT VALIDATED HARDWARE.

> **Storico — robot installato l'08/11/2026, `1.0.2-dev.4` / `4358f13234f9` (APP SHA256 `3e7ee77d…9c66`), PR #67 Draft:** [VALIDATED HARDWARE nel perimetro provato, NOT STABLE](Dev4_HW_2026-11-08/REPORT.md) — fix H1-RCA verificato sulla fixture, WALK STANDARD 100 a terra, STOP/STAND/PRONE nelle finestre osservate; difetti aperti: rifiuto della posa iniziale non recuperabile senza riavvio, sway e lentezza del joystick, budget temporale del joystick · [evidenze e SHA256](Dev4_HW_2026-11-08/EVIDENCE_MANIFEST.md) · [metriche](Dev4_HW_2026-11-08/TRACE_METRICS.csv). La Stable resta `v1.0.1` (dev.17), rollback qui sotto.

> **Stable storica (07/11/2026) — prima Stable GitHub `v1.0.1`, superata dalla `v1.0.2` il 19/11:** [dev.17 VERIFIED HARDWARE — QUALIFIED FUNCTIONAL SCOPE, installata fino all'08/11](Dev17_HW_2026-11-07/REPORT.md) · [manifest firmware/release](Dev17_HW_2026-11-07/RELEASE_MANIFEST.md) · [evidenze e SHA256](Dev17_HW_2026-11-07/EVIDENCE_MANIFEST.md) · [metriche tracce](Dev17_HW_2026-11-07/TRACE_METRICS.csv). APP `1.0.1-dev.17` / `17db0efe2324` (SHA256 `14e055a0…4a83`) installata e promossa senza ricompilazione; PR #64 integrata in `main`. Candidata e regressioni offline: [Dev17_Candidate_2026-11-07](Dev17_Candidate_2026-11-07/REPORT.md) · [flash e collaudo](Dev17_Candidate_2026-11-07/FLASH_AND_TEST.md). Rollback: dev.16 qui sotto.

> **Storico (07/11/2026, prima della dev.17):** [dev.16 VALIDATED HARDWARE nel perimetro provato, NOT STABLE](Dev16_HW_2026-11-07/REPORT.md) · [metriche per finestra](Dev16_HW_2026-11-07/TRACE_WINDOW_METRICS.csv) · [manifest originali](Dev16_HW_2026-11-07/EVIDENCE_MANIFEST.md). Robot allora installato con dev.16 `e0ddbb1791d0` (ora rollback); PR #64 allora draft; restano HTTP/UX, STOP→WALK e timing del gait. Le righe più sotto sono fotografie storiche, non lo stato attuale.

> **Storico pre-flash:** [candidata dev.16](Dev16_Candidate_2026-11-07/REPORT.md) e [procedura originale](Dev16_Candidate_2026-11-07/FLASH_AND_TEST.md). In seguito è stata installata, collaudata e conservata come rollback della dev.17.
> **Storico dev.15:** [validazione parziale del 06/11](Dev15_HW_2026-11-06/REPORT.md), [manifest](Dev15_HW_2026-11-06/EVIDENCE_MANIFEST.md), [analisi video](Dev15_HW_2026-11-06/VIDEO_AUDIT.md) e [RCA timing](Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md); difetti PRONE di quella build non rappresentano la stable corrente.

# Validation reports — index

**Latest hardware checkpoint: 19/11/2026 — [Stable `v1.0.2`, `1314bc4`](Motion_1314bc4_HW_2026-11-19/README.md)** (fixture: catena 8/8, D1/D2, blocco D, S9; P0 CLOSED nel perimetro delle prove; terra non qualificata). Checkpoint precedenti: 10/11/2026 [`1.0.2-dev.6` ACC 220](Dev6_ACC220_HW_2026-11-10/REPORT.md). **Stable storica:** 07/11/2026, [dev.17, prima Stable v1.0.1](Dev17_HW_2026-11-07/REPORT.md). Per la sequenza precedente: [dev.16](Dev16_HW_2026-11-07/REPORT.md) → [dev.15](Dev15_HW_2026-11-06/REPORT.md) → [dev.14](Dev14_HW_2026-11-05/REPORT.md). Il [tag storico dev.14](https://github.com/MattRobotics/robot-dog/tree/checkpoint/matdog-dev14-hw-20261105) è soltanto un checkpoint passato.

Historical scoped closure (11/10/2026): [MATDOG hardware non-motion 2026-10-11](MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md) — baseline validated, motion not validated; final flash/NVS/STA traced to original operator-pasted output recovered from conversation; [privacy-audited appendix](HW_Closeout_2026-10-11/MATDOG_2026-10-11_EVIDENCE_APPENDIX.md).

Each entry is evidence for its recorded date and scope only. A PASS here does not carry over to a
later firmware, a later mechanical state or a later power architecture, and none of the historical diagnostics authorizes motion on its own. The build flag `MOTION_AUTHORIZED=0` is also present in the v1.0.1 manifest (with `BOOT_MOTION_AUTHORIZED=0` and `MOTION_AUTHORITY=RUNTIME_GOVERNED`): no binary authorizes motion at build time or at boot; the Controller admits each motion command at runtime. Recorded PASS/FAIL results are not edited by this index.
Current state: [root README](../../README.md). Chronology: [`HISTORY_INDEX.md`](../HISTORY_INDEX.md).

Legend — **Era**: *Station* = Station-mediated, 12-servo phase (historical); *Rebuild* = after the
2026-08-27 rebuild (17 allocated servo slots, of which 13 are physically installed today: 12 leg + neck ID 51; ESP32-S3 Controller). **Kind**: HW = hardware evidence, OFFLINE =
software/model only.

## Model, kinematics and early hardware (June–July 2026)

| Report | Date | Kind | Era | Scope as recorded |
|---|---|---|---|---|
| [ST3215 Bus Validation](2026-06-17_ST3215_Bus_Validation.md) | 2026-06-17 | HW | Station, SUPERSEDED banner | Station-mediated ST3215 bus validation |
| [URDF REV00 Kinematic Validation](2026-06-30_URDF_REV00_Kinematic_Validation.md) | 2026-06-30 | OFFLINE | model | URDF topology, frames, axes, limits (see [ADR-003](../Architecture_Decisions/ADR-003_URDF_REV00_Kinematic_Baseline.md)) |
| [Visual-Zero and Static Actuation Validation](2026-07-06_Visual_Zero_and_Static_Actuation_Validation.md) | 2026-07-06 | HW | Station, SUPERSEDED banner | Visual-zero calibration and static actuation, 12-servo installation |
| [C4-A offline safe-stand candidate](2026-07-08_175245_C4A_offline_safe_stand_candidate.json) | 2026-07-08 | OFFLINE | model | Offline candidate; `command_eligible=false`. Path is globbed by kinematics tools: do not rename |
| C4 offline policy set — [collision/contact](C4_collision_contact_policy/), [rest-to-stand trajectory](C4_rest_to_stand_trajectory/), [timing envelope](C4_trajectory_timing_envelope/), [static stability](C4_static_stability_support_polygon/), [hardware safe-mode preflight](C4_hardware_safe_mode_preflight/) | 2026-07-08 | OFFLINE | model | C4-B…C4-F offline artifacts |
| [Geometry Compiler artifacts](Geometry_Compiler/README.md) | 2026-08-07 – 2026-08-11 | OFFLINE | model | v1–v5 geometry runs with per-run status and SHA-256 (index inside) |

## ST3215 bench phase (August 2026)

| Report | Date | Kind | Scope as recorded |
|---|---|---|---|
| [ST3215 Bench QC](ST3215_Bench_QC_2026-08-24/README.md) | 2026-08-24/25 | HW (bench) | 26/26 runs COMPLETE, protocol V6.1 |
| [ST3215 Provisioning](ST3215_Provisioning_2026-08-27/README.md) | 2026-08-27 | HW (bench) | Historical bench provisioning of 17 allocated units, 17/17 PASS. It does not imply that 17 servos are installed: 13 are installed today (12 leg + neck ID 51) |

Navigation for the whole ST3215 evidence set: [`ST3215_EVIDENCE_INDEX.md`](../ST3215_EVIDENCE_INDEX.md).

## Controller on the rebuilt robot (September–October 2026)

| Report | Date | Kind | Scope as recorded |
|---|---|---|---|
| [CR2-C current-installation q0 capture](Calibration_Q0_CR2C_2026-09-27/README.md) | 2026-09-27 | HW (read-only) | Read-only q0 capture, "CR2-C PASS"; does not accept or promote calibration |
| [Full Calibration sequence geometry](Full_Calibration_Sequence_Geometry_2026-09-30/README.md) | 2026-09-30 | OFFLINE | Nominal CAD mesh non-intersection, 24-contact sequence; no hardware |
| [TRUE Full Calibration 24/24](Full_Calibration_24_Contact_Hardware_2026-10-01/README.md) | 2026-10-01 | HW | 24/24 contacts, `all_contact_calibrated=1`, no operational envelope approved. Its statement "RAM-only" is correct for that session and firmware; for the later persistence work see the dev.3 audit below and [`OPEN_ITEMS.md`](../OPEN_ITEMS.md) |
| [Wi-Fi / provisioning / OTA V3 offline](2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md) | 2026-10-04 | OFFLINE | Offline qualification only. It names a local worktree and a branch that no longer exist; read it as historical |
| [dev.2 boot self-test delta audit](MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md) | 2026-10-06 | HW + audit | Boot census 13/13; calibration stopped at LF under dev.2 (block-read temperature artifact) |
| [dev.3 thermal delta audit](MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md) | 2026-10-06 | HW + audit | 24/24 under dev.3; SAVE/ACK and LOAD across a reset observed; automatic recovery not triggered; no real power cycle |
| [LED RMT DMA + dirty-frame hardware validation](MATDOG_LED_RMT_DMA_DIRTY_FRAME_HW_VALIDATION_2026-10-10.md) | 2026-10-10 | HW (LED only) | Installed `fee9383`: active RMT DMA, 10 min SoftAP-on steady LED without observed anomalies, servo timing unchanged; **no release or motion approval** |

## REV01-FIX geometry, physical properties and handoff (October 2026, added 2026-10-31)

| Report | Date | Kind | Scope as recorded |
|---|---|---|---|
| [REV01-FIX integration](REV01_FIX_Integration_2026-10-30/REPORT.md) | 2026-10-30/31 | OFFLINE | New `base_link`, HIP Z +2.2 mm, unified visual/collision STL; REV00 calibration basis preserved ([decision](REV01_FIX_Integration_2026-10-30/calibration_chain/CALIBRATION_BASIS_DECISION.md)); no hardware |
| [Physical properties](REV01_FIX_Physical_Properties_2026-10-31/REPORT.md) | 2026-10-31 | OFFLINE | Solid Edge masses, CoM, inertias for 17 links (nominal configuration with Jetson); 9 g residual documented; no dynamic validation |
| [Alignment and Astra handoff](REV01_FIX_Alignment_2026-10-31/ASTRA_HANDOFF.md) | 2026-10-31 | OFFLINE | Approved 7-sheet workbook, dependency map for 29 vs 40 RPM, Motion certifications still bound to REV00, dev5 vs installed firmware, work list for STAND / return-to-PRONE / WALK STANDARD |

Merged by PR #57 (`887103e`). Local `dev5` hardware logs (30/10) are not in this directory.

## Not in this directory

- **Hardware Validation 2026-10-07 (dev.3):** execution COMPLETE, acceptance BLOCKED, as stated by
  the project owner. The full report is **not in the repository** and is not reconstructed here.
  See [`EXTERNAL_ARCHIVES.md`](../EXTERNAL_ARCHIVES.md).
- **Firmware offline test results** (PR-1, PR-2, PR-3 gates) are recorded in
  [`05_Firmware/MATDOG_Controller/VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md).
- **Controller V0.1, G2, G3 sessions** (2026-09-15, 2026-09-18) are recorded in the same
  `VALIDATION.md`, not as separate reports.
- **Motion G1–G5-A evidence** (G4, G4.1, G5-A, G35 pose audit) is deferred by
  [ADR-004](../Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)
  and is not on `main`.
