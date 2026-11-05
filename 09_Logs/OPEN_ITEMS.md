# Stato operativo e problemi aperti — 05/11/2026

Firmware installato: **dev.14**, flash APP-only verificato; Motion **PARTIAL HW VALIDATION**, non stable. [Report hardware](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [tag sorgente esatto](https://github.com/MattRobotics/robot-dog/tree/checkpoint/matdog-dev14-hw-20261105) · [README corrente](../README.md).

## Funzioni osservate su hardware

STAND/HOLD, PRONE → STAND a terra, WALK30 e WALK60, STOP con coppia mantenuta, Census13/13, record NVS gen-1 integro e boot READY. RESTORE automatico operativo con evidenze hardware anche nelle sessioni precedenti. Digital Twin segue la posa; compensazione IMU già implementata e attiva. Le condizioni e i limiti di ogni osservazione sono nel report: non sono una qualifica generale di affidabilità.

## Lavoro ancora aperto

| Area | Stato e riferimento autorevole |
|---|---|
| Motion e timing | Picchi WALK oltre20ms e release perse persistenti; ulteriori miss nel recupero. Cause interne da discriminare. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| Stabilizzazione IMU | Pitch/roll attivi, ma ampiezza e velocità insufficienti; pitch quasi assente a150mm, risposta lenta a100mm e roll modesto secondo operatore. Non aumentare dominio o gain per supposizione. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| STOP e recovery | STOP funziona; ritorno WALK lungo e recupero da STOP in PRONE intermedia mancante. PRONE resta a coppiaON con sostegno non verificato; rilascio sicuro da qualificare. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| Feedback e temperatura | Dati FAST/CONTROL di epoche diverse; picchi isolati94/128°C non confermano surriscaldamento reale. Preservare le protezioni termiche. [#52](https://github.com/MattRobotics/robot-dog/issues/52) |
| Dashboard e Twin | Reset vista frontale+X ancora errato; mantenere diagnosi e stato senza effetti sul BNO085. [#62](https://github.com/MattRobotics/robot-dog/issues/62) |
| LED | Difetto intermittente storico aperto, anche se non osservato nell'ultima sessione. [#62](https://github.com/MattRobotics/robot-dog/issues/62) |
| RESTORE manuale/manutenzione | Automatismo non significa percorso manuale qualificato. Restano prove di dashboard, rifiuti e sospensione per manutenzione. [#48](https://github.com/MattRobotics/robot-dog/issues/48) |
| Stable | Dev.7 pre-release storica; dev.14 parziale; nessuna stable qualificata da questo closeout. [#46](https://github.com/MattRobotics/robot-dog/issues/46) |
| Dev.15 | Motion direzionale in sviluppo separato, NON validato hardware; nessun PASS di build o joystick attestato qui. |

Nessuna nuova issue o duplicazione integrale: le discussioni collegate conservano dettagli e criteri. I risultati VERIFIED OFFLINE restano distinti da VERIFIED HARDWARE; spiegazioni non dimostrate sono HYPOTHESIS o UNKNOWN.

## Archivio degli stati precedenti

Il precedente OPEN_ITEMS, inclusi gli stati “Motion non collegato”, “0/12 limiti”, STAND/WALK bloccati e rilievi degli audit storici, è conservato **integralmente** con commit e SHA256 nello [snapshot storico](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md). Non descrive la dev.14 installata. Non viene dichiarato risolto ogni vecchio item: i dettagli restano nelle evidenze datate; questa pagina evita di presentarli come stato corrente.
