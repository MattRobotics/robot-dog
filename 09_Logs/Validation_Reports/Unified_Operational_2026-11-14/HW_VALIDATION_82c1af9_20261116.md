> Copia del rapporto di sessione. I percorsi relativi (`cold/`, `suite_20261116_0924/`, `h_20261116_0932/`, `preflash/`, …) si riferiscono alla cartella delle evidenze `verification-artifacts/MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116/` (sigillata, `SHA256SUMS.txt`), non a questo repository.

# Validazione hardware del candidato `82c1af9` (E1/E1b/P3a/P3b + D3) — 16/11/2026, pomeriggio

Ore locali CET; nei log orari UTC (un'ora in meno). Mandato dell'operatore del 16/11: un'unica sessione supervisionata sulla fixture, un unico flash APP-only di `82c1af9`, suite A–G e blocco H, confronto a parità di blocchi con `46b3175`. Operatore presente, robot sostenuto, zampe libere, STOP accessibile.

**Criteri:** fissati prima della prova in `PLAN_FROM_CANDIDATE.md` §4 (= `candidate_82c1af9/NEXT_SESSION_PLAN.md`), invariati.

## Conclusione tecnica

**Che cosa è migliorato (VERIFIED HARDWARE, stessi blocchi del 16/11 mattina):**
- **WALK** 11,0–12,0 → **9,8–10,7 ms** per release (−10/−12 %); **RECOVERY** 10,9–12,0 → **9,8–10,6 ms** (−10/−11 %).
- **Overrun** 157 → **29** nella suite (blocco H 4 → 2); release ≥ 20 ms 1,03 % → **0,12 %**.
- **Picchi:** WALK 27,0 → **23,7 ms**; RECOVERY 22,5 → **19,8 ms**.
- **0 release perse operative:** solo le 6 dell'attesa ACC alla prima STAND dopo il boot, come in tutte le sessioni.
- **Gli stalli di istruzioni (cache) scendono in ogni classe:** `istall_mem` 0,40–0,46 → 0,31–0,41.
- **Effetto di layout:** anche le release OTHER, con istruzioni invariate, sono più veloci del 7–9 %. Una parte del guadagno viene quindi da un diverso layout della cache, non solo da E1/E1b.
- **Funzioni:** STOP, riprese, cambi di modo e FAST 32 regolari; 65/65 + 19/19; NVS e calibrazione invariate.

**Che cosa resta sopra budget:**
- **P0 NON chiuso:** 31 overrun, massimo 26,8 ms; 6 perse all'attivazione dopo il boot (attesa ACC).
- **Distribuzione degli overrun:**
  - **18 su 31** (58 %): release di STAND, PRONE e HEIGHT con **compensazione attenuata a 2 soluzioni** (sonda + proiezione): 20–22 ms per release, 13,5–16 ms di compensazione;
  - **8:** **WALK_SETTLE dopo il rilascio di un cammino sterzato** (D5, D9, D11): fino a 26,8 ms. La compensazione dura 21 ms con sole 4 IK, mentre QEMU, per lo stesso lavoro, dà ~0,7 M istruzioni (~7 ms): l'eccesso sul chip non è ancora spiegato;
  - **4:** cammini sterzati (B3, D10); **1:** WALK STOP (E8, soluzione e compensazione nella stessa release).

**Perché:**
- Le soluzioni IK di compensazione costano 6–8 ms ciascuna sul chip: due nella stessa release superano il budget anche senza soluzione nominale.
- Lo stallo di istruzioni resta il componente più grande.

**Prossimo intervento concreto:**
- **P4** (§6): una sola soluzione per release attenuata quando la scala accettata non supera la proiezione.
- Sviluppata e verificata offline, non installata: elimina l'83 % delle release a 2 soluzioni.

## Esito per punto

| Punto | Esito | Stato |
|---|---|---|
| Verifica di `46b3175` (USB e HTTP) | build `46b31757eb81` = `ota_running`, NVS gen-1 `LOAD_COUNT=4` `VALID_ACKNOWLEDGED`, restore 12/12 `be8d5902bb4e37da`, census, IMU, SAFE_OFF 8191, coppie OFF; preflight 8/8 | VERIFIED HARDWARE |
| DALY prima del flash (D1+D2) | 160/160 poll OK; gate 150 superato | VERIFIED HARDWARE |
| ROM | ESP32-S3 v0.2, MAC corretto; layout app0, tabella `8f756ecb…`; **installato `46b3175` con digest uguale** (rollback esatto) | VERIFIED HARDWARE |
| Flash unico APP-only `82c1af9` | `0x010000..0x1c37c0`, 40 s, hash e `verify-flash` con digest uguale; NVS, bootloader, tabella, otadata non toccati | VERIFIED HARDWARE |
| Boot | `82c1af95e0a5` = `ota_running`; NVS `LOAD_COUNT=4` gen-1 invariata; restore 12/12 `UNCHANGED_OFF`; census PASS; IMU PASS; D2 e **D3 presenti**; gate 150/0; preflight 8/8 | VERIFIED HARDWARE |
| Suite A–G | **65/65 in un'unica esecuzione** | VERIFIED HARDWARE |
| Blocco H (STOP durante la ripresa) | **19/19**; STOP a 1,001 s e 0,305 s nella ripresa, verificati in 2,4 e 2,2 s con coppia ON, ripresa fino a STAND_REACHED | VERIFIED HARDWARE |
| DALY durante la prova | 100/100 campioni HTTP OK; nessun silenzio in tutta la sessione | VERIFIED HARDWARE |
| Stato finale | READY / PRONE_REST, SAFE_OFF 13/13 (8191), coppia 0 su 12, nessun fault, stream BMS OFF, porta libera; poi SAFE_TO_POWER_OFF e **MATDOG spento dall'operatore** | VERIFIED HARDWARE |

## Criteri del piano (§4), valutati come scritti

| Criterio | Risultato | Esito |
|---|---|---|
| Regressione funzionale: 65/65 + 19/19, nessun fault, passi come il 16/11 | 65/65 + 19/19, nessun fault; esiti per passo uguali (`compare_steps_vs_46b3175.txt`) | **soddisfatto** |
| STOP: verifica entro 2,2–2,6 s; ripresa a STAND_REACHED | B4 2,2, D14 2,6, D18 2,2, F4 2,4, H7 2,4, H14 2,2 s; riprese 4,8–6,4 s, tutte STAND_REACHED o WALKING | **soddisfatto** |
| IMU sterzato: media e massimo non peggiori del 16/11 (1,39 / 13,30 mrad, 1 Hz) | media 1,39 → **1,39**; massimo 13,30 → **13,87** (+0,57 mrad su un campione a 1 Hz; p90 3,19 → 1,22) | **non soddisfatto sul massimo**. Offline E1 non cambia la statistica (0,926 → 0,922); il campionamento a 1 Hz non permette di separare l'effetto dal rumore. Segnalato senza rilassare il criterio |
| Timing E1/E1b/P3a/P3b: misurati e riportati, nessuna soglia nuova | §3 | riportato |
| D3: ipotesi «perse a riposo = dump diagnostica» confermata solo se ogni persa a riposo è attribuita a una dump e nessuna cade nelle attivazioni | **0 perse a riposo, 0 perse all'attivazione** (salvo l'attesa ACC) in tutta la sessione: nessun evento da attribuire | **non decidibile:** né confermata né smentita (§4) |
| P0: 0 perse operative, 0 overrun, ≤ 16 ms | 0 perse operative; 31 overrun; massimo 26,8 ms | **non chiuso** |
| Memoria | heap −32 B atteso. `heap_free` minimo nella suite 66 468 B (mattina 65 564); `heap_min_free` 41 604 B (40 440); stack minimo del loop 6 288 B (6 112) | nessun peggioramento |

## 1. Cronologia

| Ora CET | Evento |
|---|---|
| 10:05–10:10 | verifiche offline; READY_FOR_POWER_ON; registratore armato |
| 10:12:38 | QON: `POWERON`, `46b31757eb81`; DALY e D2 OK |
| 10:14:22 | l'host entra nell'AP (riconnessione automatica) |
| 10:17:46 | gate BMS 152/0; preflight `46b3175` 8/8 |
| 10:18 | ROM, `rom_checks` PASS; **flash** `82c1af9` (40 s) |
| 10:19:21 | boot con watchdog-reset, registratore aperto 1,5 s dopo |
| 10:24:38 | gate 150/0; preflight `82c1af9` 8/8 |
| 10:24:50–10:31:54 | **suite A–G 65/65** |
| 10:32–10:33 | **blocco H 19/19** |
| 10:33:48 | stato finale in sola lettura, stream BMS OFF; SAFE_TO_POWER_OFF |

## 2. Identità

| Voce | Valore |
|---|---|
| Prima | `46b31757eb81` (APP `57508746…`), verificato via USB e con `verify-flash` in ROM |
| Ora | **`82c1af95e0a5`**, branch locale `cand/diag-p0-peaks-20261116` = `46b3175` + E1 `ae9b6d1` + P3a `db6d621` + E1b `447082c` + P3b `e6ee46b` + D3 `82c1af9`. APP `93c61562fdcf1d79206565a9dc440debd2bc8dbd7e2fa91176d7aefd2f966ace`, 1 783 744 B, app0 |
| Script | `preflash/flash-qualified-app0-1.0.2-dev.8-82c1af9.sh` (`aa8e16db…`) |
| Rollback | `46b3175`: script identico a quello che l'ha installato; dry gates PASS; **non eseguito** |
| Immagine nelle evidenze | `MATDOG_1_0_2_P0_PEAKS_20261116/candidate_82c1af9/image/` |

## 3. Timing a parità di blocchi (`analyze_diag.py` invariato; `compare_vs_46b3175.txt`)

| Blocco | Classe | ms per release `46b3175` → `82c1af9` | N+C (ms) | istruzioni (k) | IPC | `istall_mem` |
|---|---|---|---|---|---|---|
| A | OTHER | 9,39 → 8,55 | 5,85 → 5,19 | 617 → 614 | 0,46 → 0,48 | 0,44 → 0,33 |
| B | WALK | 11,98 → **10,72** | 8,28 → 7,20 | 795 → 738 | 0,42 → 0,43 | 0,46 → 0,40 |
| B | RECOVERY | 11,95 → **10,60** | 8,24 → 7,07 | 740 → 729 | 0,38 → 0,44 | 0,45 → 0,38 |
| C | WALK | 11,02 → **9,76** | 7,34 → 6,27 | 688 → 639 | 0,39 → 0,42 | 0,46 → 0,41 |
| D | WALK | 11,67 → **10,33** | 7,98 → 6,83 | 749 → 702 | 0,40 → 0,43 | 0,44 → 0,40 |
| D | RECOVERY | 10,86 → **9,79** | 7,22 → 6,30 | 701 → 647 | 0,40 → 0,43 | 0,45 → 0,40 |
| E–F | WALK | 11,01 → **9,79** | 7,34 → 6,30 | 685 → 644 | 0,37 → 0,42 | 0,43 → 0,41 |
| E–F | RECOVERY | 11,31 → **10,03** | 7,62 → 6,51 | 696 → 635 | 0,35 → 0,43 | 0,47 → 0,41 |
| B–F | OTHER | 7,92–9,29 → 7,43–8,55 | −0,4…−0,7 | invariate | +0,02…+0,06 | −0,05…−0,11 |
| H | WALK / RECOVERY / OTHER | 11,37 / 11,07 / 7,37 → 10,11 / 9,98 / 7,19 | | 723 / 704 / 438 → 677 / 658 / 469 | | |

**Picco per classe dal boot** (wall, NOMINAL / COMPENSATE):
- WALK: 27,02 ms (11,2 / 11,7) → **23,70 ms** (8,2 / 9,1, E8 WALK STOP);
- RECOVERY: 22,51 → **19,84 ms** (13,1 / 0,6);
- OTHER: 25,52 → **26,82 ms** (0,9 / 21,0, WALK_SETTLE dopo il rilascio dello yaw D11).

**Il picco sterzato da 27 ms è stato eliminato?**
- In parte. I cammini sterzati (B3, D2–D10) hanno ora un massimo di 18,8–20,9 ms (mattina 25,7–27,0 ms).
- Il picco assoluto WALK (23,7 ms) è un WALK STOP frontale (E8), con soluzione e compensazione nella stessa release.
- Il caso sterzato più pesante si è spostato **dopo** il cammino: WALK_SETTLE dopo il rilascio (fino a 26,8 ms).

**Distribuzione:**
- release ≥ 20 ms nei comandi: 1,03 % → 0,12 %;
- p99 sotto 20 ms in tutte le categorie (istogramma del firmware per comando, `compare_vs_46b3175.txt` §3).

**Confronto omogeneo degli overrun** (suite A–G + blocco H, stessi passi, causa dalla fase della release massima di ogni passo; `overrun_causes.py`, `overrun_causes_vs_46b3175.txt`):

| Causa | `46b3175` (16/11 mattina) | `82c1af9` |
|---|---|---|
| STAND, PRONE e HEIGHT attenuati | 78 | 18 |
| Cammini sterzati, rilasci e WALK_SETTLE | 73 | 12 |
| Riprese dopo STOP | 9 | 0 |
| Altro (WALK STOP) | 1 | 1 |
| **Totale** | **161** (157 + 4) | **31** (29 + 2) |

**Overrun per causa nella sessione di oggi** (`residual_by_step.txt`):

| Causa | Overrun | Release massima |
|---|---|---|
| STAND, PRONE e HEIGHT con compensazione attenuata a 2 soluzioni | 18 | 20,0–21,9 ms |
| WALK_SETTLE dopo un rilascio sterzato | 8 | 20,6–26,8 ms |
| Cammini sterzati | 4 | 20,8–20,9 ms |
| WALK STOP | 1 | 23,7 ms |

## 4. D3 e perse a riposo

| Misura (sul chip) | Valore |
|---|---|
| Perse a Motion non attivo in tutta la sessione | **0** (`LAST_IDLE_SKIP count=0` all'ultima dump) |
| Perse all'attivazione da PRONE_REST | **0** (A8, C1, D1, E1, H1) |
| Durata della dump `@MOTION TIMING` | 6,1–6,8 ms (piccola), 11,0 ms (media), **20,7–21,4 ms** (grande, dopo blocchi con WALK/RECOVERY) |
| Stessa dump vista dall'host | 24–26 ms (oggi pomeriggio); 26–36 ms il 16/11 mattina, con le perse dopo le dump di 28 e 36 ms |
| Passaggio del registro 85 (`configureAccCeiling`) | **3,9–4,1 ms** a ogni attivazione; 127,8 ms alla prima dopo il boot (scrittura + 100 ms di assestamento) |

**Lettura:**
- La dump grande tiene il loop per **più di un periodo** di Motion (20,7–21,4 ms > 20 ms), quindi può far perdere una release a riposo.
- Il passaggio del registro 85 da solo (~4 ms) **non può**.
- Oggi non si è persa alcuna release a riposo. Le dump grandi hanno superato il periodo di soli 0,7–1,4 ms, e una persa richiede che il blocco inizi in quella frazione del periodo.
- Le perse della mattina seguivano dump viste dall'host come più lunghe (28 e 36 ms).
- **L'ipotesi resta compatibile ma non confermata:** manca un evento da attribuire.
- In ogni caso è un'attività **diagnostica**, accettata solo a coppia OFF e assente da una build operativa.

## 5. IMU, STOP, memoria, DALY

- **IMU** (|voluto − applicato|, campioni HTTP 1 Hz):
  - frontale: media 0,62 → 0,51 mrad, massimo 15,9 → **4,1**;
  - sterzato: media 1,39 → 1,39, p90 3,19 → 1,22, massimo 13,30 → **13,87** (criterio del piano non soddisfatto sul massimo, §Criteri);
  - ripresa: media 0,42 → 0,55, massimo 0,82 → 1,02.
- **STOP:** tutti verificati con coppia ON in 2,2–2,6 s; riprese in 4,8–6,4 s; STOP in ripresa (H7, H14) come il 16/11.
- **Memoria:** `heap_free` 70 124 B dopo il boot; `heap_min_free` 41 604 B nella suite; stack del loop 6 288 B. Nessun peggioramento contro il 16/11.
- **DALY:**
  - 160/160 poll prima del flash, 157/157 dopo, 100/100 campioni durante la suite e il blocco H;
  - D2 con baseline invariata (`d2_session_analysis.txt`);
  - **nessun silenzio, causa UNKNOWN, non dichiarata risolta**.

## 6. Prima causa residua misurata e patch proposta (offline)

**Causa misurata sul chip** (18 overrun su 31, il gruppo più grande):
- nelle fasi STAND, PRONE e HEIGHT la compensazione attenuata (`compensate`, ramo non WALK) risolve due volte quando la proiezione si sposta di oltre il 2 % dalla scala accettata: una sonda alla scala precedente, poi la proiezione;
- `max_work_solves = 2`, 13,5–16 ms di compensazione, 20–22 ms per release;
- lo stesso caso offline: 5 376 release a 2 soluzioni nei 64 scenari host.

**Patch P4** (`research/p0-peaks-20261116` `e57dd89`, offline, non installata):
- in ogni release attenuata la sonda alla scala accettata è tenuta quando quella scala non supera la proiezione, cioè una correzione non più grande di quanto l'attenuazione consente;
- la proiezione diventa la scala della sonda della release successiva;
- una proiezione che si restringe risolve subito come prima;
- nessuno stato nuovo; generalizza P3b.

Verifiche offline: `MATDOG_1_0_2_P0_PEAKS_20261116/REPORT.md` §9.

## File

- **Piano e strumenti:** `README.md`, `PLAN_FROM_CANDIDATE.md`, `TOOLS.sha256`, `preflash/`, `session_compare.py` (con il proprio autotest `selftest_compare_e321e86_vs_46b3175.txt`).
- **Acquisizioni:** `cold/` (D1+D2 prima del flash), `pre/`, `flashlog/`, `usb_boot/`, `suite_20261116_0924/`, `h_20261116_0932/`, `final_usb/`, `final_state_readonly.json`.
- **Analisi:** `overrun_causes_vs_46b3175.txt` (`overrun_causes.py`), `compare_vs_46b3175.txt`, `compare_steps_vs_46b3175.txt`, `residual_by_step.txt`, `d3_analysis.txt`, `d3_all_serial.log`, `d2_session_analysis.txt`, `*/analyze_diag.txt`.
- **Registro:** `WORKLOG.md`.
