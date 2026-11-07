# Stato operativo corrente — 07/11/2026, validazione hardware dev.16

**Installata e validata sul robot nel perimetro provato:** `1.0.1-dev.16`, build `e0ddbb1791d0`, APP SHA256 `73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`. NVS gen-1/census 13/13 preservati; STAND, WALK STANDARD45/60, PRONE con SAFE_OFF e STOP da movimenti sono riusciti nei casi registrati. Ultimo stato `READY/PRONE_REST`, `safe_off_mask=8191`, 0 fault; robot spento via KEY OFF (discharge MOS BMS alimenta/disalimenta anche ESP32-S3 in configurazione attuale). **NON dichiarata release stabile**. Dossier [HW 07/11](Validation_Reports/Dev16_HW_2026-11-07/REPORT.md) · [manifest](Validation_Reports/Dev16_HW_2026-11-07/EVIDENCE_MANIFEST.md) · [metriche](Validation_Reports/Dev16_HW_2026-11-07/TRACE_WINDOW_METRICS.csv). La PR #64 resta DRAFT; main non integrato, APP/manifest binari su ASUS.

## Candidata dev.17 — IMPLEMENTATA, NON VALIDATA HARDWARE (07/11 sera)

`1.0.1-dev.17`, commit `7503bfc`, APP `f49b531a5826d5e2a22a70ee17d5d8976a1075d1e604cb44c5b2803566213c5e`, non flashata. [Dossier](Validation_Reports/Dev17_Candidate_2026-11-07/REPORT.md) · [flash APP-only e collaudo minimo](Validation_Reports/Dev17_Candidate_2026-11-07/FLASH_AND_TEST.md).

| Voce della tabella sotto | Stato dev.17 |
|---|---|
| P0 Dashboard / `/status` 503 / letture | Cause dimostrate (ammissione slack per le pagine di stato; abort browser a 800 ms < RTO lwIP); corrette. **Causa delle perdite TCP solo con coppia ON: UNKNOWN** (serve cattura pacchetti HOLD vs IDLE) |
| P0 Reflow joystick | Corretto (overlay, slot fissi, centro pad ancorato al `pointerdown`, rilascio ovunque); Chrome: 0 px contro 89–202 px di dev.16 |
| P0 Twin cold refresh | Corretto (Twin a robot fermo anche con coppia ON, 304 dalla cache; mai in moto o con joystick premuto; un retry a torque OFF) |
| P1 STOP→WALK | Implementato con l'ammissione e le rotte di ripresa di STAND; WALK STOP, joystick e PRONE restano `STOP_LATCHED` |
| P1 Timing | Non toccato; `-O2` selettivo valutato offline (APP +0,9 %, codice Motion +18 %, nessuna misura ESP32-S3) e **non incluso**; variabilità HOLD UNKNOWN |
| P2 Cockpit / ViewCube | Implementati (Twin in alto a tutta larghezza, quota 50–150 passo 5, 🐢30/45/🐇60, ViewCube CAD) |
| Nuovo, HYPOTHESIS | Il Controller non ordina i comandi per `sequence`: un refresh joystick riordinato dopo il neutro sarebbe accettato entro 1 s (non osservato nelle tracce dev.16) |

## Richieste dalla validazione dev.16 (07/11 pomeriggio; stato aggiornato sopra)

| Priorità | Difetto residuo / richiesta operativa |
|---|---|
| **P0 – Dashboard controllo** | Alert e stati dinamici causano **reflow** verticale del joystick durante pressione; risolvere layout e cattura pointer senza alterare STOP/deadman. `/status` espone 503 sotto admission slack 5 ms, HTTP 200 talvolta incompleto/lento; `/api/motion/status` 20/20 HTTP200 da terminale. Separare cause di trasporto, server e polling. [#62](https://github.com/MattRobotics/robot-dog/issues/62), [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| **P0 – Twin cold refresh** | Refresh browser durante torque ON cancella iframe; `prepareTwin` blocca reload fino a torque OFF. Recuperare visualizzazione senza torque OFF e senza carico pesante durante Motion. Reset vista +X già verificato. [#62](https://github.com/MattRobotics/robot-dog/issues/62) |
| **P1 – STOP→WALK** | Da STOP concluso, WALK richiede STAND; se campo STAND=150, robot sale a 150 e ridiscende alla quota WALK nominale ~120. Controller deve gestire automaticamente ENTRY e transizione da posa certificata a WALK con comando unico, rifiutando pose non ammissibili, preservando STOP. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| **P1 – Timing sotto gait/joystick** | Trace(32): +7 missed e +154 overrun nella sua finestra, non da sommare alle altre; HOLD100 regolare, HOLD150 +2 overrun senza missed. T1 funziona ma non risolve 20 ms per tutte le traiettorie; indagare variabilità HOLD senza assegnare causa non provata. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| **P2 – Cockpit Motion** | Twin e comandi unico pannello superiore full-width; joystick/rotazioni + STAND/PRONE/WALK/WALK STOP; quota STAND step 5 mm (50–150); stride 30/45/60 mm/ciclo con 🐢→🐇; ViewCube/triade CAD cliccabile e trascinabile; rimuovere descrizione joystick dev.15; STOP sempre visibile. [#62](https://github.com/MattRobotics/robot-dog/issues/62) |
| **P2 – Prestazioni** | Gait ancora lenta, sway joystick, WALK_SETTLE ~7–9 s; obiettivo fino a 5× solo dopo profilo temporale e margini di stabilità verificati. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| **Monitorare** | Picchi termici isolati ID21/31, LED intermittenti (issue #62), RESTORE manuale/gen gestione (issue #48) non qualificato. Protezioni sempre attive. |

**Non eseguire ora nuovi flash, merge, release o modifiche alla safety.** Il report dev.16 fotografa risultati reali. Gli elementi sotto sono lo snapshot storico della dev.15, preservato per tracciabilità.

---

## Snapshot storico — 06/11/2026 / RCA 07/11 dev.15

# Stato operativo e problemi aperti — 06/11/2026 (agg. 07/11: RCA timing, sorgente dev.15 nella PR #64)

**Installed on robot: `MATDOG Controller 1.0.1-dev.15`, build `7997ad4d4497`**, flash APP-only `ROBOT_POWERED` verificato; sorgente esatto `7997ad4` pubblicato il 07/11 sul branch della [PR #64](https://github.com/MattRobotics/robot-dog/pull/64) (merge `e5b0644`, draft), **non su `main`**. **PARTIAL HW VALIDATION — NOT STABLE**. [Report dev.15](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [Manifest raw locali](Validation_Reports/Dev15_HW_2026-11-06/EVIDENCE_MANIFEST.md) · [Audit video](Validation_Reports/Dev15_HW_2026-11-06/VIDEO_AUDIT.md) · [README corrente](../README.md).

## Funzioni verificate sul robot reale

STAND50–150, WALK STANDARD 45/60 mm per ciclo, WALK STOP, sei versi del joystick + combinazioni proporzionali, ritorno a STAND, BNO085 pitch/roll correttivi, Digital Twin realtime e Reset vista frontale +X. STOP è sempre accessibile; semantiche di coppia diverse in `STOPPED_SUPPORT_UNVERIFIED` e `SAFE_OFF_VERIFIED_FAULT_LATCHED`. NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13 e boot READY. PRONE ha completato due volte con torque OFF13/13 e recupero PRONE→STAND100, **ma è fallito due volte con log e un'altra volta secondo osservazione operatore**: la funzione non è affidabile in tutte le condizioni.

## Blocchi reali da correggere, nell'ordine

| Area | Evidenza e decisione |
|---|---|
| **PRONE fallimento / rilascio da quota intermedia** | `PRONE_HEIGHT` si blocca attorno a 52–53 mm, poi `NATIVE_TARGET_NOT_REACHED` e SAFE_OFF con caduta o sostegno manuale. Due report JSON negativi e due `PRONE_REST` positivi su stessa dev.15; terzo negativo riferito senza JSON. Prima verificare arrived/settledUnderLoad, freshness, margini successivi e first divergence; poi patch minima. Proteggere la condizione di appoggio e consentire recovery sicuro da fault latched. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** **Candidata dev.16 (07/11, non flashata):** causa = consegna a 53 mm con target 2,2 tick dentro `kLift` e cedimento sotto carico fino a −0,8 tick fuori; consegna a 52 mm e sostegno con coppia invece di SAFE_OFF se il segmento non si completa. [Report](Validation_Reports/Dev16_Candidate_2026-11-07/REPORT.md) |
| **Motion timing** | 50 Hz (20 ms) non mantenuti in molte prove joystick: max lavoro ~49,59 ms e centinaia di release perse per finestra; `COMPENSATE` ~31,24 ms perfino in HOLD. Indagine costo per-release e regressione mirata, senza eliminare STOP/feedback/limiti. **RCA 07/11** ([TIMING_RCA](Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md)): NOMINAL+COMPENSATE (double software) oltre 20 ms anche nel caso migliore (WALK45 18,4 ms, joystick 28,2 ms); regressione `2a630a0`; patch T0+T1 raccomandata, non implementata. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** **dev.16 (non flashata):** T0 + T1 implementati, attesa verifica hardware; joystick previsto ancora oltre 20 ms. |
| **HTTP admission** | A STAND150, `WALK45` e `STAND100` rifiutati `503 MOTION_BUDGET_UNAVAILABLE` dopo 101–102 ms. Comando NON arrivato all'attuazione. Verificare handler, lock, scheduling prima di incolpare servo. **RCA 07/11:** conseguenza dell'overrun (nessuno slack di 5 ms entro 100 ms); non lock, non starvation HTTPD, non servo/IK. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** **dev.16 (non flashata):** T0 + T1 implementati, attesa verifica hardware; joystick previsto ancora oltre 20 ms. |
| **Velocità e sway WALK** | Sway laterale marcato, passo utile ridotto, ciclo visivo marker ~6,5 s; STOP via joystick rapido nel comando ma assestamento `WALK_SETTLE→STAND` ~7,3–8,7 s. Obiettivo prestazionale anche 5× richiesto dall'operatore, non provato. Ottimizzare cadenza e sequenza solo dopo timing e supporto verificati. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **IMU pitch/roll** | Reattività migliore secondo operatore; roll applicato fino ~1,9° contro ~4,8° richiesti, limitazione `WORKSPACE`; pitch applicato ~13,5°. Margini reali da ricostruire, non disabilitare workspace. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **Termica e servo feedback** | Spike termici isolati ID21/ID31 e altri; letture periodiche normalmente ~26–34°C. Non provato né escluso difetto servo/protocollo; mantenere protezioni. Rumore storico LF sulla fixture non osservato a terra, RCA non chiusa. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **BMS / LED** | BMS dev.14 `BMS_TELEMETRY_STALE` rientrato dopo riavvio, causa UNKNOWN. LED fisici ancora soggetti a sfarfallio policromatico intermittente pur con `LED PASS`. **[#62](https://github.com/MattRobotics/robot-dog/issues/62)** |
| **Dashboard / Twin** | **Reset vista +X è VERIFIED HARDWARE e può essere depennato**; altre UX, floor reference, badge LIVE e LED non tutti qualificati. Issue #62 resta aperta. |
| **RESTORE manuale** | NVS gen-1 e startup automatico verificati; mancano prova RESTORE manuale dashboard, sospensione RAM per manutenzione e specifici rifiuti. **[#48](https://github.com/MattRobotics/robot-dog/issues/48)** |
| **HostLink / Jetson** | Bridge USB no-reset e candidate offline, ma controllo Motion da USB non qualificato hardware (helper BrokenPipe, nessun ACK `STAND`). Jetson non ancora integrata. Riprendere soltanto con Controller Motion robusto. |
| **Firmware / stable** | Sorgente dev.15 esatto ora nella PR #64 (tree firmware = `7997ad4`, manifest CLEAN, APP SHA coincidente), non su main; servono review, regressioni e fix prima di integrare; nessuna release stable prima dei fix e qualifica. **[#46](https://github.com/MattRobotics/robot-dog/issues/46)** |

## Sequenza raccomandata

1. RCA mirata PRONE fallito e **prevenzione caduta**; recupero supervisionato senza power cycle solo se sicuro.
2. Eliminare il collo di bottiglia del periodo 20 ms e i falsi rifiuti HTTP: priorità STOP, deadman, limiti e servo watchdog restano invariati.
3. Profilare performance fisiche e incrementare gait, ridurre sway superfluo e settle; obiettivo 5× eventuale, progressivo e verificato.
4. Estendere ROLL entro workspace realmente utilizzabile.
5. ~~Pubblicare sorgente dev.15~~ fatto il 07/11 nella PR #64 (draft, non su main); quindi integrare fix e host/Jetson a perimetro controllato.

Questa voce non autorizza flash, reset NVS, modifiche EEPROM o aumento dei limiti. I report sono fotografie delle rispettive date.

## Archivio cronologico

Precedenti stati: [dev.14 05/11](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [dev.7 31/10](Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md) · [snapshot storico](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md). I loro esiti e limiti restano validi **per quelle versioni/date**. Nessun precedente documento viene cancellato o retroattivamente qualificato.
