# Stato operativo e problemi aperti — 06/11/2026

**Installed on robot: `MATDOG Controller 1.0.1-dev.15`, build `7997ad4d4497`**, flash APP-only `ROBOT_POWERED` verificato; sorgente dev.15 commit `7997ad4` **solo su worktree ASUS locale**, non su `main`. **PARTIAL HW VALIDATION — NOT STABLE**. [Report dev.15](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [Manifest raw locali](Validation_Reports/Dev15_HW_2026-11-06/EVIDENCE_MANIFEST.md) · [Audit video](Validation_Reports/Dev15_HW_2026-11-06/VIDEO_AUDIT.md) · [README corrente](../README.md).

## Funzioni verificate sul robot reale

STAND50–150, WALK STANDARD 45/60 mm per ciclo, WALK STOP, sei versi del joystick + combinazioni proporzionali, ritorno a STAND, BNO085 pitch/roll correttivi, Digital Twin realtime e Reset vista frontale +X. STOP è sempre accessibile; semantiche di coppia diverse in `STOPPED_SUPPORT_UNVERIFIED` e `SAFE_OFF_VERIFIED_FAULT_LATCHED`. NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13 e boot READY. PRONE ha completato due volte con torque OFF13/13 e recupero PRONE→STAND100, **ma è fallito due volte con log e un'altra volta secondo osservazione operatore**: la funzione non è affidabile in tutte le condizioni.

## Blocchi reali da correggere, nell'ordine

| Area | Evidenza e decisione |
|---|---|
| **PRONE fallimento / rilascio da quota intermedia** | `PRONE_HEIGHT` si blocca attorno a 52–53 mm, poi `NATIVE_TARGET_NOT_REACHED` e SAFE_OFF con caduta o sostegno manuale. Due report JSON negativi e due `PRONE_REST` positivi su stessa dev.15; terzo negativo riferito senza JSON. Prima verificare arrived/settledUnderLoad, freshness, margini successivi e first divergence; poi patch minima. Proteggere la condizione di appoggio e consentire recovery sicuro da fault latched. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **Motion timing** | 50 Hz (20 ms) non mantenuti in molte prove joystick: max lavoro ~49,59 ms e centinaia di release perse per finestra; `COMPENSATE` ~31,24 ms perfino in HOLD. Indagine costo per-release e regressione mirata, senza eliminare STOP/feedback/limiti. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **HTTP admission** | A STAND150, `WALK45` e `STAND100` rifiutati `503 MOTION_BUDGET_UNAVAILABLE` dopo 101–102 ms. Comando NON arrivato all'attuazione. Verificare handler, lock, scheduling prima di incolpare servo. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **Velocità e sway WALK** | Sway laterale marcato, passo utile ridotto, ciclo visivo marker ~6,5 s; STOP via joystick rapido nel comando ma assestamento `WALK_SETTLE→STAND` ~7,3–8,7 s. Obiettivo prestazionale anche 5× richiesto dall'operatore, non provato. Ottimizzare cadenza e sequenza solo dopo timing e supporto verificati. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **IMU pitch/roll** | Reattività migliore secondo operatore; roll applicato fino ~1,9° contro ~4,8° richiesti, limitazione `WORKSPACE`; pitch applicato ~13,5°. Margini reali da ricostruire, non disabilitare workspace. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **Termica e servo feedback** | Spike termici isolati ID21/ID31 e altri; letture periodiche normalmente ~26–34°C. Non provato né escluso difetto servo/protocollo; mantenere protezioni. Rumore storico LF sulla fixture non osservato a terra, RCA non chiusa. **[#52](https://github.com/MattRobotics/robot-dog/issues/52)** |
| **BMS / LED** | BMS dev.14 `BMS_TELEMETRY_STALE` rientrato dopo riavvio, causa UNKNOWN. LED fisici ancora soggetti a sfarfallio policromatico intermittente pur con `LED PASS`. **[#62](https://github.com/MattRobotics/robot-dog/issues/62)** |
| **Dashboard / Twin** | **Reset vista +X è VERIFIED HARDWARE e può essere depennato**; altre UX, floor reference, badge LIVE e LED non tutti qualificati. Issue #62 resta aperta. |
| **RESTORE manuale** | NVS gen-1 e startup automatico verificati; mancano prova RESTORE manuale dashboard, sospensione RAM per manutenzione e specifici rifiuti. **[#48](https://github.com/MattRobotics/robot-dog/issues/48)** |
| **HostLink / Jetson** | Bridge USB no-reset e candidate offline, ma controllo Motion da USB non qualificato hardware (helper BrokenPipe, nessun ACK `STAND`). Jetson non ancora integrata. Riprendere soltanto con Controller Motion robusto. |
| **Firmware / stable** | Dev.15 effettiva rimane sul device, non su GitHub; serve import sorgente locale a commit identificato, diff, build manifest, regressioni; nessuna release stable prima dei fix e qualifica. **[#46](https://github.com/MattRobotics/robot-dog/issues/46)** |

## Sequenza raccomandata

1. RCA mirata PRONE fallito e **prevenzione caduta**; recupero supervisionato senza power cycle solo se sicuro.
2. Eliminare il collo di bottiglia del periodo 20 ms e i falsi rifiuti HTTP: priorità STOP, deadman, limiti e servo watchdog restano invariati.
3. Profilare performance fisiche e incrementare gait, ridurre sway superfluo e settle; obiettivo 5× eventuale, progressivo e verificato.
4. Estendere ROLL entro workspace realmente utilizzabile.
5. Pubblicare in PR separata sorgente dev.15 dopo recupero da ASUS; quindi integrare fix e host/Jetson a perimetro controllato.

Questa voce non autorizza flash, reset NVS, modifiche EEPROM o aumento dei limiti. I report sono fotografie delle rispettive date.

## Archivio cronologico

Precedenti stati: [dev.14 05/11](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [dev.7 31/10](Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md) · [snapshot storico](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md). I loro esiti e limiti restano validi **per quelle versioni/date**. Nessun precedente documento viene cancellato o retroattivamente qualificato.
