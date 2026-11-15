# Validazione hardware Motion P0 su fixture — 15/11/2026 (diagnostica `8530d3a11ac9`)

**Esito:**
- riarmo STOP → STAND e ripresa dopo yaw: **VERIFIED HARDWARE**;
- suite fixture A–G: **65/65 passi PASS**;
- Timing: criterio storico rispettato, **Timing P0 non chiuso**;
- DALY: interruzione di 440 s prima della suite con **causa UNKNOWN**; 188/188 letture OK durante il movimento;
- robot lasciato in **PRONE_REST**, SAFE_OFF 13/13, coppie OFF.

Contesto:
- build installata `8530d3a11ac9` (`1.0.2-dev.8` diagnostica, APP SHA256 `75fdd3ac…0c4d4b`) = `539b646` di questo branch + strumentazione Timing;
- robot sulla fixture, operatore presente, esecuzione automatica autorizzata;
- nessun flash, reset, scrittura NVS, EEPROM, BMS o calibrazione; nessuna modifica al firmware o ai limiti;
- evidenze complete e sigillate: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DIAG8530_MOTION_HW_20261115/` (`REPORT.md`, `MANIFEST.md`, `SHA256SUMS.txt`).

## Risultati

| Punto | Esito |
|---|---|
| **A. Riarmo dopo STOP/SAFE_OFF** (`7878be0`) | **PASS.** STOP a riposo → STOPPED, SAFE_OFF 8191, coppie OFF in 0,2 s. Poi STAND 120 tramite ACQUIRE/REFERENCE/ENABLE fino a `STAND_REACHED` a 120,1 mm |
| **B. WALK_SETTLE dopo yaw** (Patch A `4d4b797`) | **PASS** sullo scenario del FAULT del 14/11: STANDARD yaw orario a fondo scala per 22 s, STOP, 4 s, STAND 120. La correzione −8,3 / −33,2 mrad portata in WALK_SETTLE (14/11: −10,4 / −32,6) è stata riportata a zero senza rifiuti; `STAND_REACHED` a 119,8 mm in 6,2 s, 6 perse (14/11: FAULT `NATIVE_TARGET_NOT_REACHED`, 24 perse) |
| **Quote e PRONE** | STAND 80 / 110 / 120 / 150 raggiunte; PRONE_REST ↔ STAND quattro volte |
| **WALK STANDARD in avanti** | 30 s, 0 perse |
| **Joystick STANDARD** | indietro −16,7 mm/s; laterale ±6,7 mm/s; yaw ±0,04 rad/s. 12 s ciascuno, rilascio fino a `STAND_REACHED` |
| **STOP** | 5 STOP in camminata, alle fasi 0,37 / 0,42 / 0,69 / 0,81 / 0,94, verificati in ≤ 2,6 s con coppia ON (`STAND_RECOVERY_AVAILABLE`). Riprese STAND 5,2–6,2 s; WALK dopo STOP in 4,6 s |
| **Cambi di modo in cammino** | STANDARD → SLOW completato in circa 7 s. SLOW → STANDARD accettato (`WALK_MODE_CHANGING`) ma **non completato** nei 12 s previsti, per progetto all'orologio della legge più lenta: durata da qualificare |
| **FAST 32 mm/s** (preset 2, 105 mm) | 15 s e 10 s, 0 perse; STOP in fase 0,94; ripresa STAND con 1 persa |
| **DALY in movimento** | 188/188 OK, 129 B, `stale` 0, `fail_streak` 0. Primo byte fino a 84 ms in movimento, contro ≤ 24 ms a riposo |

Durante la sessione ci sono state **due interruzioni dello strumento**, non del robot. In entrambe la suite ha inviato un solo `@STOP` e la ripresa è partita dallo STOP mantenuto con una STAND.

1. L'assert del joystick usava `walk_mode.active` (SLOW) invece della modalità di `joystickMode()`, cioè `walk_mode.joystick` oppure `walk_mode.selected`. Nessun comando inviato.
2. La ripresa del WALK non ammetteva lo HOLD `WALK_IMU_SETTLING` previsto da `startWalk`.

## Timing (`analyze_diag.py`, tre registrazioni concatenate)

| Classe | Voce | 14/11 (`0526cf6`) | 15/11 B (stesso scenario) | C (avanti) | D (multidirezionale) | E–F | Proiezione B2 |
|---|---|---:|---:|---:|---:|---:|---:|
| WALK | muro / N+C (ms) | 14,79 / 10,51 | 13,53 / 10,01 | 11,78 / 8,26 | 12,92 / 9,40 | 11,85 / 8,35 | ~11,8 |
| WALK | istruzioni N+C / release | 1 005 k | 863 k | 696 k | 807 k | 715 k | 864 k |
| RECOVERY | muro / N+C (ms) | 21,46 / 17,26 | 18,46 / 14,97 | — | 16,99 / 13,49 | 17,13 / 13,65 | ~17,0 |
| RECOVERY | istruzioni N+C / release | 1 723 k | 1 285 k | — | 1 157 k | 1 200 k | 1 349 k |
| — | perse | 31 (24 nella ripresa) | 7 | **0** | 16 | 1 | — |

- **Perse:** al massimo **7 in 30 s**, quindi il criterio storico (≤ 16) è rispettato. In totale 30 in movimento, più 1 a riposo causata dall'ultimo dump.
- **Overrun (> 20 ms):** 677. Valgono il 5–16 % delle release nei cammini sterzati e nelle riprese, lo 0,8–2,8 % in avanti.
- **Picchi:** massimo 39,67 ms, nessuna release ≥ 40 ms.
- **Gap:** uno solo di 147 ms, nella prima STAND dopo l'avvio. È maturato prima della prima release del comando; causa UNKNOWN.
- **Causa prima, misurata dal PMU:** stallo istruzioni su memoria 0,47–0,50 dei cicli N+C in ogni classe (IPC 0,35–0,38), `DOMINANT istall_mem` anche per RECOVERY. I picchi uniscono NOMINAL (10–18 ms) e COMPENSATE (14–22 ms) nella stessa release.
- **Conclusione: Timing P0 non chiuso.** Il criterio §8.3 della dev.8 richiede 0 perse, 0 overrun e un massimo ≤ 16 ms per comando.

## RCA DALY (riassunto)

- **Il guasto:** 08:37:41–08:45:02, robot fermo. 440 s con `rx_bytes=0` e `stale_bytes=0` mentre i poll continuavano ogni 2 s; rientro con una risposta 128/129 e poi 129/129, senza reset. Il fail-closed del firmware ha funzionato: Motion STOPPED, SAFE_OFF.
- **Esclusi:** driver, timeout e buffer (replay offline sul driver di produzione); cambi di framework (core 3.3.11 dal 22/08); altri utilizzatori di UART2 o GPIO15/16. Le stesse build hanno sessioni pulite e sessioni guaste.
- **Commit candidato:** `8e20c63`, per sola correlazione temporale.
  - È lo snapshot rc.11 r2 del 30/10, già sull'hardware dalle build rc di fine ottobre: rimozione del `flush()`, drenaggio limitato, gate `availableForWrite`.
  - È l'unica modifica del percorso UART DALY prima del primo guasto (06/11, dev.14); tutti i guasti sono successivi.
  - Il meccanismo non è dimostrato e mal si accorda con la firma: durante il guasto il TX si svuotava (218 poll) e lo stale restava 0.
- **Patch:** nessuna.
- **Ipotesi:**
  - H1, il lato DALY o RS485 non risponde: la più coerente;
  - H2, accoppiamento elettrico o RF;
  - H3, firmware: solo la correlazione con `8e20c63`.
- **Prossimo passo:** osservazione passiva di GPIO15/16 e delle linee RS485 A/B durante un guasto, a 2–5 min da un'accensione a freddo, con `@BMS STREAM ON` e una prova A/B del traffico Wi-Fi. Discrimina le classi 1–4 e la correlazione con `8e20c63`.

## Prossimo sviluppo

Candidato IRAM secondo la tabella del §6 del rapporto dev.8 (`MATDOG_DEV8_P0_TIMING_2026-11-12/REPORT.md`, branch `diag/dev8-p0-timing-20261112`):
- funzioni: `solveGaitInto`, `contactForwardKinematics`, `contactInverseKinematicsGroundNormal`, `inverseWithModel`, `forwardWithModel`, `pointToWorld`, 14,9 KB;
- prima la verifica di spazio: heap minimo libero osservato 38 136 B;
- poi equivalenza offline dei frame e un solo passaggio hardware con la stessa suite (`timing_tools/fixture_suite.py`).

Senza un nuovo mandato non si fanno flash né altre prove.
