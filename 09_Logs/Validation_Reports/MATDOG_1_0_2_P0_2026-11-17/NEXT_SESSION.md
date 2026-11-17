# Handoff per la prossima sessione MATDOG (scritto il 17/11/2026 sera)

Rapporto: [`REPORT.md`](REPORT.md). Evidenze: [`EVIDENCE_MANIFEST.md`](EVIDENCE_MANIFEST.md). Registro: [issue #70](https://github.com/MattRobotics/robot-dog/issues/70).

## 1. Stato esatto

| Oggetto | Stato |
|---|---|
| Branch remoto `integ/matdog-unified-operational-20261114` | sorgente firmware a `202ecbc`. **Unica aggiunta:** il commit documentale che contiene questa cartella (padre `202ecbc`). `main` invariato (`6838e4d`) |
| Worktree operativo `~/MATDOG/worktrees/matdog-unified-operational-20261114/` (locale) | HEAD = quel commit documentale. **4 file firmware modificati e NON committati**: `StandTrajectory.cpp`, `motion_final_release_sources.json`, `SimpleMotion.h`, `SimpleMotion.cpp`. `git diff` = `FINAL.diff`, SHA256 `7e2d3d7045f88d056d7e56be6f2e1d8412200706bcff14095151ea8b69c9804d`. Nessuno stash |
| **Firmware installato sul robot** (spento) | `6be46906d069` (`1.0.2-dev.8`, diagnostica F3 + IRAM G2s), APP `be678d83…3b68`. **Non** contiene `validDefinition`, PlanSplit V2, la correzione STOP → STAND né lo scrittore HTTP |
| Candidato consolidato | `202ecbc` + `FINAL.diff`. Build DIRTY APP 1 776 416 B, mai installata |
| Patch HTTP | `patches/HTTP_CANDIDATE.diff` (`22188f11…`), **isolata, non nel worktree**. VERIFIED OFFLINE |
| Composizione proposta | `patches/COMPOSED_vs_202ecbc.diff` (`c24bac10…`, 5 file). Riproduce esattamente il candidato verificato |

**Cautela sul robot finché gira `6be4690`:** dopo uno STOP vicino a un comando STAND/WALK/PRONE, **non riprendere con STAND**; passare da SAFE_OFF.

## 2. Provenienza delle evidenze

- **Locali, non pubblicate:** `~/MATDOG/verification-artifacts/MATDOG_1_0_2_*_20261117/`, con sigilli `SHA256SUMS.txt` riverificati il 17/11 sera (0 differenze). Elenco e sigilli nel manifest.
- **Pubblicate:** solo le patch di [`patches/`](patches/), con `SHA256SUMS`.
- **Strumenti delle sessioni hardware:** locali, nelle cartelle `MATDOG_1_0_2_HTTP_AB_BMS_HW_20261117/` (catena A/B, profili HTTP, `session_proc.py`, analisi) e `MATDOG_1_0_2_G2S_HW_20261116/`.
- **Backup** (locali): APP installata `~/MATDOG/backups/esp32/app_images/6be4690/`, rollback `a86c680`, full-flash da 16 MiB del 10/09 (`5cbba0b9…`, gate degli script) e del 10/10 (`154a6131…`, con NVS).

## 3. Completato il 17/11

- [x] `validDefinition()`, PlanSplit V2 e correzione STOP → STAND verificati offline e consolidati nel worktree (non committati).
- [x] Analisi temporale Motion/HTTP: caso B, i picchi coincidono con le risposte di stato.
- [x] **Prova A/B sul robot** (VERIFIED HARDWARE): release ≥ 15 ms 168 → 25 riducendo le letture di stato; massimo 20 449 → 19 786 µs; 140/140 passi.
- [x] Scrittore di stato HTTP: identico byte per byte, −51 % istruzioni, memoria interna invariata (VERIFIED OFFLINE).
- [x] Inventario delle ottimizzazioni: A e B del Jacobiano, `pendingGoal_` e dashboard a 1 Hz rimandati.
- [x] Questa documentazione e le patch pubblicate.

## 4. Ancora da fare

- [ ] **P0 aperto.** Misurare lo scrittore sul chip: nessun effetto sui massimi è dimostrato.
- [ ] Prima ACQUIRE dopo il boot (123 ms, 6 release perse): problema distinto, da analizzare e qualificare. Da solo impedisce di dichiarare zero release perse.
- [ ] Qualifica sul robot di STOP e ripresa dalla dashboard con la correzione installata.
- [ ] DALY BMS: causa dei silenzi UNKNOWN. Manca la misura sul bus RS485 durante un silenzio; serve un mandato.
- [ ] Rivalutare A o B (mai insieme) solo se, dopo la prova, restano release > 16 ms non sovrapposte a risposte HTTP.

## 5. Sequenza minima verso la build CLEAN e il collaudo APP-only

Ogni passo richiede la relativa autorizzazione: commit dei sorgenti, flash, sessione hardware.

1. **Verifica iniziale** nel worktree:
   - `git status --short`: solo i 4 file;
   - `git diff | sha256sum` = `7e2d3d70…`;
   - `git ls-remote origin` sul branch uguale all'HEAD locale.
2. **Composizione:** applicare la patch HTTP **dalla radice** del worktree.
   - Comando: `patch -p1 < 09_Logs/Validation_Reports/MATDOG_1_0_2_P0_2026-11-17/patches/HTTP_CANDIDATE.diff`.
   - Controllo: il diff completo rispetto a `202ecbc` deve coincidere con `COMPOSED_vs_202ecbc.diff`, cioè 5 file.
3. **Commit locale dei 5 file** (autorizzazione esplicita), per ottenere l'identità CLEAN. Push solo se richiesto.
4. **Build ROBOT_POWERED** con `scripts/build.sh`, poi `python3 scripts/static_audit.py "$(pwd)"` dalla cartella firmware. Il gate delle sorgenti deve passare: `MotionService.cpp` non è pinnato.
5. **Pacchetto di flash APP-only** con lo script qualificato, le nuove pin e l'SHA della nuova APP. Rollback pronto: script `6be4690` (`6f369ae1…`), immagini in `app_images/`. Prova a vuoto dei gate.
6. **Sessione sul robot:**
   - QON e registratore USB passivo dal boot;
   - gate DALY: QON + 6 min e 150 poll validi;
   - preflight;
   - un solo flash APP-only;
   - controlli di boot: identità, restore 12/12, NVS gen 1, census 13/13;
   - controllo di memoria M0 con al massimo 2 client HTTP.
7. **Collaudo**, con i criteri fissati prima e da non modificare:
   - stessa catena A/B del 17/11 (profili A e B, ABBA);
   - poi un blocco con la dashboard reale: STAND, WALK STANDARD, RECOVERY, STOP e ripresa;
   - P0 chiuso solo con massimo ≤ 16 000 µs, 0 overrun e 0 release operative perse in ogni prova, compresa la prima STAND;
   - effetto dello scrittore: release ≥ 15 ms nel profilo A contro le 168 del 17/11.
8. **Chiusura:** PRONE_REST, SAFE_OFF, coppie OFF, stream BMS OFF, nessun processo di sessione. Rapporto VERIFIED HARDWARE separato dalle evidenze offline.

**Vincoli permanenti:**
- Nessuna modifica a NVS gen 1, calibrazioni 12/12, mapping, geometria o limiti.
- Nessun criterio rilassato dopo la prova.
- Repository pubblico: niente credenziali né nomi di reti locali nei documenti.
