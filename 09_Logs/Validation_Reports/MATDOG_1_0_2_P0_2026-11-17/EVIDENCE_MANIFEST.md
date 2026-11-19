# Manifest delle evidenze — P0 Motion/HTTP, 17/11/2026

Registro: [issue #70](https://github.com/MattRobotics/robot-dog/issues/70). Rapporto: [`REPORT.md`](REPORT.md).

**Le cartelle elencate sono LOCALI e NON pubblicate**: stanno sulla macchina di sviluppo in `~/MATDOG/verification-artifacts/`, con log grezzi, banchi e alberi copiati (migliaia di file).
- **Pubblicate in questo commit:** soltanto le patch di [`patches/`](patches/).
- **Sigillo:** `SHA256SUMS.txt` della cartella, che elenca lo SHA256 di ogni file. La colonna «Sigillo» riporta lo SHA256 di quel file. Ogni sigillo è stato **riverificato il 17/11 sera** (`sha256sum -c`: 0 differenze).

## Esperimenti

| # | Identificativo | Risultato | Percorso locale (non pubblicato) | File principali | Sigillo (SHA256 di `SHA256SUMS.txt`), file | Classificazione | Decisione |
|---|---|---|---|---|---|---|---|
| 1 | validDefinition | `StandTrajectory::validDefinition()` −95 % (35 446 → 1 708 istruzioni), F6/H18 −2,8…−3,3 %; uscite identiche | `MATDOG_1_0_2_P0_VALIDDEF_20261117/` | `REPORT.md`, `PATCH_validDefinition_202ecbc.diff` (`0d4627e3…`) | `a964c48779e5907a9378e05d2496b8ae1f4355a2fc66e31b062e012e868e0a0d`, 173 | VERIFIED OFFLINE; IMPLEMENTED NOT VALIDATED | **KEEP** (nel worktree) |
| 2 | REST attribution | REST di F6/H18 = 4 IK di contatto (80–86 %); target F6 = duplicato di `pendingGoal_` (≈ 3,8 ms, stima); nessuna patch | `MATDOG_1_0_2_P0_REST_ATTRIB_20261117/` | `REPORT.md` | `f0e776f27c57bc5818eed5c1434f91cf54462e0151b479488d6f2c51de661d10`, 153 (+ uno zip di condivisione creato dopo il sigillo: integro, `REPORT.md` identico, non coperto) | VERIFIED OFFLINE; risparmio HYPOTHESIS | **DEFER** (con V2 il lavoro è già fuori dalla release di avvio) |
| 3 | Preparazione della qualifica operativa | test dashboard PASS anche in Chrome; immagini APP installate salvate; procedura proposta | `MATDOG_1_0_2_OPQUAL_PREP_20261117/` | `REPORT.md` | `d879c0cb55e53d34d3ae4c9004eb06f64a4de104630d0c3adec75636f07b0b55`, 3 | VERIFIED OFFLINE | **REEVALUATE**: la composizione è superata da §7–8 |
| 4 | A. Riuso trigonometrico FK → Jacobiano | bit per bit esatto; Jacobiano −25 %, IK −9,6 %, COMPENSATE −6 %; picchi −0,4…−0,6 ms (stima) | `MATDOG_1_0_2_P0_KERNEL_20261117/` | `REPORT.md`, `trees/CANDIDATE.diff` (`fdacecf0…`) | `9b896d8577430b49ab24413e986d814d22a12b282fb2fe36cc4baeb08e309df5`, 6 511 | VERIFIED OFFLINE; **NON INTEGRATO** | **DEFER** |
| 5 | B. Jacobiano X/Y/Y | non bit per bit (≈ 75 % dei casi diversi internamente; RAW, stati, STOP identici nei banchi); Jacobiano −39 %, COMPENSATE −8,9…−10,7 %; −0,56…−1,14 ms (stima) | `MATDOG_1_0_2_P0_IK_MATH_20261117/` | `REPORT.md`, `FEASIBILITY.md`, `trees/CANDIDATE.diff` (`9ccc2d02…`) | `0f71472654162a915b039f4f1f8f307ace173c903ea2d312f9e7870b570bbc2f`, 2 839 | VERIFIED OFFLINE; **NON INTEGRATO**; esclude A | **DEFER** |
| 6 | PlanSplit V2 | piano HEIGHT/PRONE_HEIGHT preparato in k−1; RAW, piano e STOP identici; avvio F6/H18 ≈ 15,4–15,7 ms (stima) | `MATDOG_1_0_2_P0_PLANSPLIT_20261117/` | `REPORT.md`, `trees/CANDIDATE.diff` (`ed112a90…`, V2) | `95a59cbd2a79934b7aa6d00ad44d644bf0293086e7223e99782162f6da1dc99a`, 6 753 | VERIFIED OFFLINE; IMPLEMENTED NOT VALIDATED | **KEEP** (nel worktree) |
| 7 | STOP → STAND dopo un frame annullato | difetto preesistente (salti fino a 555 tick, 2 fault spuri), corretto con 6 righe in `beginStop()`; i casi corretti sono identici | `MATDOG_1_0_2_STOP_PREEMPT_RESUME_20261117/` | `REPORT.md`, `trees/FIX.diff` (`f7b0a7ae…`), `review_original/` | `f777335b35fb8ecfbd7eeb4f777789bb76a45bcdea8fae4fc19ef8230a3a3350`, 6 321 | VERIFIED OFFLINE; IMPLEMENTED NOT VALIDATED; difetto presente nel firmware installato | **KEEP** (nel worktree) |
| 8 | Consolidamento operativo | worktree = `202ecbc` + `FINAL.diff`; box STOP 5 208 configurazioni, 0 rifiuti; gate PASS; build DIRTY APP 1 776 416 B | `MATDOG_1_0_2_OPCAND_CONSOLIDATION_20261117/` | `REPORT.md`, `FINAL.diff` (`7e2d3d70…`) | `82d1430674aca9ea2ba10994d2de140d2cacb8fb4d85bc63f32618d59a99ca50`, 2 166 | VERIFIED OFFLINE | **KEEP** |
| 9 | Decisione temporale Motion/HTTP | 22 coppie a parità di lavoro, COMPENSATE ×1,94; PMU 54–55 % di stall nelle più pesanti; 24/87 picchi subito dopo una lettura di stato | `MATDOG_1_0_2_P0_TEMPORAL_DECISION_20261117/` | `REPORT.md`, `tools/coincidence.py`, `out/` | `bf99ae0cf5bf61e473872ab625aaa684cd1da1479cd89daf4e8753b0f574a6f2`, 7 | VERIFIED OFFLINE (dati hardware storici); meccanismo HYPOTHESIS | **KEEP** (evidenza) |
| 10 | A/B HTTP e DALY sul robot | 140/140 passi; release ≥ 15 ms 168 → 25 (−85 %); massimo 20 449 → 19 786 µs; overrun 1 → 0; DALY senza silenzi dopo 282 s | `MATDOG_1_0_2_HTTP_AB_BMS_HW_20261117/` | `REPORT.md`, `WORKLOG.md`, `analysis/AB_TABLES.md`, log seriali e HTTP grezzi | `1cd5d95a3db90eb8a55c2e029e593eb2872122ee4c129afb0df29bfb3594e9a6`, 227 | **VERIFIED HARDWARE** (`6be4690`) | **KEEP** (evidenza) |
| 11 | Scrittore HTTP di stato | identico byte per byte; `formatStatus` −51 % istruzioni; codice raggiungibile −66 %; heap, IRAM, DRAM invariati; APP +2 800 B; stack −585 B | `MATDOG_1_0_2_HTTP_OPT_CANDIDATE_20261117/` | `REPORT.md`, `INVENTORY.md`, `HW_TEST_PLAN.md`, `HTTP_CANDIDATE.diff`, `COMPOSED_vs_202ecbc.diff` | `fafbabb053bce50198c907af67d6cc920e8ce9eb716ad4c597ac172dadaeacc2`, 4 996 | VERIFIED OFFLINE; non installato; effetto sul chip UNKNOWN | **KEEP** (nella prossima APP, con prova HW) |
| 12 | Dashboard a 1 Hz a Motion ingaggiato | misurata come profilo B nella #10 | – | – | – | VERIFIED HARDWARE come carico; non applicata | **DEFER** |

## Patch pubblicate in [`patches/`](patches/)

Copie byte per byte degli originali locali, non applicate. Il dettaglio di applicazione è in [`patches/README.md`](patches/README.md).

| File pubblicato | Originale locale | SHA256 (originale = copia) | Stato |
|---|---|---|---|
| `FINAL.diff` | `MATDOG_1_0_2_OPCAND_CONSOLIDATION_20261117/FINAL.diff` | `7e2d3d7045f88d056d7e56be6f2e1d8412200706bcff14095151ea8b69c9804d` | candidato consolidato (nel worktree, non committato) |
| `HTTP_CANDIDATE.diff` | `MATDOG_1_0_2_HTTP_OPT_CANDIDATE_20261117/HTTP_CANDIDATE.diff` | `22188f112085dd50c9cfd1872ec0f0cbba45360817578539c61c6faff76aa501` | patch HTTP isolata |
| `COMPOSED_vs_202ecbc.diff` | `MATDOG_1_0_2_HTTP_OPT_CANDIDATE_20261117/COMPOSED_vs_202ecbc.diff` | `c24bac1005eaff4c8446f50098be488e55b5583bd6fd567f5ae156a15e55ea95` | composizione proposta per la prossima APP |
| `component_validDefinition_202ecbc.diff` | `MATDOG_1_0_2_P0_VALIDDEF_20261117/PATCH_validDefinition_202ecbc.diff` | `0d4627e3e28f2b545c4dc005106b6f7caea09b45e2a7828a1781ec7e83b07ad9` | componente, già contenuto in `FINAL.diff` |
| `component_PlanSplit_V2.diff` | `MATDOG_1_0_2_P0_PLANSPLIT_20261117/trees/CANDIDATE.diff` | `ed112a907e9f74afecddb5560205d35fea9d357a1d55fcb7a7b5ef07ee87b76f` | componente, già contenuto in `FINAL.diff` |
| `component_STOP_STAND_fix.diff` | `MATDOG_1_0_2_STOP_PREEMPT_RESUME_20261117/trees/FIX.diff` | `f7b0a7aee5cf5199fefed1ba5f6740daa11a163759125995e2be2945a743b6e7` | componente, già contenuto in `FINAL.diff` |
| `NOT_INTEGRATED_A_kernel_trig_reuse.diff` | `MATDOG_1_0_2_P0_KERNEL_20261117/trees/CANDIDATE.diff` | `fdacecf05b1905a91b0dc4599fda4deef382fb8a1cd0c0898501b709d88165f6` | **NON INTEGRATO** |
| `NOT_INTEGRATED_B_jacobian_xyy.diff` | `MATDOG_1_0_2_P0_IK_MATH_20261117/trees/CANDIDATE.diff` | `9ccc2d02512aa8dd8ae33e3a80a4041bb51d8bac2d8900980953b45cf32970da` | **NON INTEGRATO** |

**Ricostruzione verificata il 17/11 sera** su copie di `git archive 202ecbc`:
- `FINAL.diff` + `HTTP_CANDIDATE.diff`, oppure `COMPOSED_vs_202ecbc.diff` da solo, riproducono **identico** il candidato sigillato della #11;
- i tre componenti in sequenza riproducono **identico** `FINAL.diff`;
- A e B si applicano su `FINAL.diff` (solo prova a vuoto).

## Firmware e backup (locali, non pubblicati)

| Oggetto | Percorso locale | Verifica del 17/11 sera |
|---|---|---|
| APP installata `6be4690` (immagine, manifest, `SHA256SUMS`) | `~/MATDOG/backups/esp32/app_images/6be4690/` | `sha256sum -c` OK; APP `be678d83037451d7937783e70d443a83612cbc992005a9f0cac9561ad0a13b68`, partizioni `8f756ecb…` |
| Secondo livello di rollback `a86c680` | `~/MATDOG/backups/esp32/app_images/a86c680/` | `sha256sum -c` OK; APP `f7bc8b62…` |
| Script di flash pinnato `6be4690` (rollback esatto) | `MATDOG_1_0_2_G2S_HW_20261116/preflash/` | SHA `6f369ae117e4946077ed…` |
| Full-flash da 16 MiB richiesto dal gate degli script | `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin` | `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`, 16 777 216 B |
| Full-flash più recente, NVS compresa (10/10) | `~/MATDOG/backups/esp32/led-ap-preflight-20261010T133829Z-kOK0Bl/current-fullflash-16m.bin` | `154a6131ca0088b50b836a46ca52a6eef2b1ceefb39fd3e8dcfb7fd8a9454eb0`, uguale al file `.sha256` |

**NVS:** al preflight del 17/11 risulta gen 1 `VALID_ACKNOWLEDGED`; nessuna scrittura NVS il 17/11. L'uguaglianza byte per byte fra la NVS attuale e il backup del 10/10 è UNKNOWN: servirebbe una lettura hardware, non autorizzata.
