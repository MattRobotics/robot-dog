# MATDOG 19/11/2026 — qualifica hardware di `1314bc4` e Stable `v1.0.2`

**Firmware qualificato e installato:**

| Campo | Valore |
|---|---|
| Versione interna | `MATDOG Controller 1.0.2-dev.7` |
| Build | `1314bc411c58` |
| Commit | `1314bc411c589c4216e95629f4665d9d01f05781` |
| APP | 1 777 952 B, SHA256 `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a` |
| Profilo | `ROBOT_POWERED` |

**Esito:** KEEP. Promossa senza ricompilazione come Stable [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2) («Motion fixture-qualified»).

**Perimetro:** tutte le prove sono sulla fixture, con le zampe libere. **Non è una qualifica della locomozione a terra.**

## Risultati (VERIFIED HARDWARE, 19/11/2026)

| Voce | Esito |
|---|---|
| Flash APP-only, digest e boot | PASS: `digest matched`, nessun panic né eccezione FPU, uptime continuo |
| NVS e calibrazione | PASS: gen 1 `VALID_ACKNOWLEDGED`, `be8d5902bb4e37da` 12/12, nessuna scrittura |
| Catena A/B sulla fixture | 8/8 prove, 140/140 passi PASS |
| Dashboard D1 e D2 | PASS. STOP HTTP verificato 2/2, ripresa STAND |
| Blocco D | 27/27 PASS: joystick indietro e laterale, rotazione, STOP in due fasi, riprese |
| S9, SLOW sterzato | 15/15 PASS: nessun `WALK_IK_OR_WORKSPACE` né arresto di box |
| STOP USB | 14/14 in catena, 3 nel completamento: tutti verificati in 2,2–3,2 s |
| Motion Timing P0 | **CLOSED — VERIFIED HARDWARE nel perimetro delle prove del 19/11.** Massimo 12 729 µs contro il budget di 16 000 µs (margine 3 271 µs); 0 overrun; 0 release perse |
| Memoria | `heap_min_free` 43 368 B (soglia 24 576), nessuna allocazione fallita |
| DALY | Nessun errore nelle sessioni Motion. La causa dei silenzi precedenti resta UNKNOWN ([B2](../BMS_DALY_B2_2026-11-19/README.md)) |

**Contro `30c894a` (18/11), stessa catena:**
- release più lenta da 20 761 a 11 012 µs (nel blocco D 12 729 µs);
- release ≥ 15 ms da 217 a 0;
- overrun da 2 a 0.

## Contenuto

| Percorso | Cosa contiene |
|---|---|
| [`qualification/REPORT_HW.md`](qualification/REPORT_HW.md) | Collaudo del mattino: identità, flash, boot, catena A/B, D1/D2, confronto P0 con `30c894a`, regressioni, KEEP |
| [`completion/REPORT_HW_COMPLETION.md`](completion/REPORT_HW_COMPLETION.md) | Blocco D e S9 |
| [`completion/PLAN.md`](completion/PLAN.md) | Criteri del completamento, scritti prima delle prove |
| `completion/fixture_block_d.py`, `completion/fixture_s9_slow_steer.py` | Selettori di passi usati: stessi comandi, controlli e limiti della suite qualificata |
| `qualification/analysis/`, `completion/analysis/` | Uscite testuali degli strumenti di analisi, byte-identiche agli originali |
| `*/SHA256SUMS.local.txt` | Sigillo di ciascuna cartella originale sull'Asus, con l'hash di ogni file, compresi log seriali e JSONL non pubblicati |
| [`RELEASE_MANIFEST.md`](RELEASE_MANIFEST.md) | Catena di identità della Stable `v1.0.2`, installazione e rollback |
| [`EVIDENCE_MANIFEST.md`](EVIDENCE_MANIFEST.md) | Archivi locali (percorsi, sigilli, verifiche), seconda copia e cosa non è nel Git |
| `LOCAL_SEALS_VERIFIED_20261119.txt` | Verifica del 19/11 dei sigilli delle cartelle del 17–19/11 |

**Altri riferimenti:**
- build e verifiche offline: [Motion_Integrated_1314bc4_2026-11-19](../Motion_Integrated_1314bc4_2026-11-19/REPORT.md);
- procedura HW: [HW_PROCEDURE.md](../Motion_Integrated_1314bc4_2026-11-19/HW_PROCEDURE.md);
- certificato `kSteer`: [Motion_SteerSlow_Recert_2026-11-18](../Motion_SteerSlow_Recert_2026-11-18/README.md);
- FP32 e P1: [Motion_FP32_2026-11-18](../Motion_FP32_2026-11-18/REPORT.md);
- qualifica del rollback `30c894a`: [Motion_30c894a_HW_2026-11-18](../Motion_30c894a_HW_2026-11-18/README.md);
- congelamento P0: [ADR-008](../../Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md).

## Limiti della qualifica

- **Sulla fixture, non a terra.** STANDARD e FAST hanno `hardware_qualified=false` nella tabella firmware (costante dal 09/11).
- **Non è un WCET matematico.** Il massimo è il valore osservato nelle prove eseguite, con il carico HTTP operativo: profili A e B, dashboard con 1 e 2 client.
- **Condizioni IMU:** non tutte sono provate. La condizione `random_3` di `kSteer` (beccheggio ≈ +55 mrad, passo 40 mm) non è stata riprodotta sul robot: resta certificata solo offline. L'angolo di RF UPPER non è leggibile in rad dallo stato.
- **Non eseguiti:** ciclo a freddo S7 e joystick dalla dashboard (richiede di tenere premuto).

## Errata

- **`HW_PROCEDURE.md`:** dichiarava che la catena A/B copre il joystick D2/D4/D6. Non è così: il blocco D non è in `run_ab_chain.sh`, ed è stato eseguito a parte, 27/27. Il documento originale resta invariato.
- **`completion/PLAN.md`:** indicava «28/28» passi per il blocco D; la sequenza stampata da `--plan` prima della prova ne aveva 27. Il piano non è stato corretto dopo la prova.
