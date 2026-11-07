# Stato operativo corrente — 07/11/2026: prima Stable `v1.0.1` (firmware 1.0.1-dev.17)

**Installata, VERIFIED HARDWARE nel perimetro qualificato e rilasciata:** `1.0.1-dev.17`, build `17db0efe2324`, sorgente `17db0ef`, APP SHA256 `14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83`, promossa senza ricompilazione come GitHub Release `v1.0.1`. Rollback: dev.16 `e0ddbb1791d0`. [Report HW](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md) · [manifest release](Validation_Reports/Dev17_HW_2026-11-07/RELEASE_MANIFEST.md).

## Checklist della prima Stable (chiusa con `v1.0.1`)

- [x] APP installata identificata (commit, build ID, SHA256, profilo, slot) e flash APP-only verificato con digest.
- [x] Boot, NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13, SAFE_OFF 8191, audit IDLE.
- [x] STAND100/125, WALK STANDARD 45/60, WALK STOP, STOP e WALK diretto dopo STOP, joystick e rilascio, PRONE con SAFE_OFF, Twin dopo refresh con coppia ON, ViewCube.
- [x] `/status` senza 503 sotto slack Motion, letture oltre il recupero TCP, joystick senza reflow e in ordine (dev.17).
- [x] Sorgente su `main`, tag e Release con APP originale, manifest e checksum; dev.16 conservata come rollback.

## Lavoro residuo per il prossimo ciclo (non prerequisiti della `v1.0.1`)

| Area | Stato | Issue |
|---|---|---|
| Timing Motion | overrun/perse in WALK e joystick (finestra `(36)`: +64 perse, +491 overrun, lavoro max 47,2 ms); variabilità HOLD UNKNOWN | #52 |
| Stack del loop | minimo osservato 2.060 B durante joystick/WALK (dev.16 ≥ 4.208 B): da misurare e ridurre prima di aggiungere lavoro al loop | #52 |
| Velocità e qualità del gait | gait lento (obiettivo fino a 5× non qualificato), sway laterale del joystick, WALK_SETTLE ~8,9 s | #52 |
| HTTP con coppia ON | rari stalli e risposte lunghe (max 1,68 s); causa delle perdite TCP UNKNOWN, serve cattura pacchetti HOLD vs IDLE | #62 |
| STOP durante il caricamento Twin | misurato solo in Chrome (fino a ~0,5 s); misura end-to-end su telefono reale a cache fredda mancante | #62 |
| UX Twin | overlay leggermente trasparente dei comandi dentro il Twin; zampe posteriori sospese sul pavimento grafico in PRONE | #62 |
| LED | verde stabile osservato in dev.17, nessuna qualificazione del sottosistema | #62 |
| RESTORE e manutenzione | RESTORE manuale da dashboard, rifiuti per giunto, multigenerazione non qualificati | #48 |
| Non ricollaudati in dev.17 | rotazioni IMU pitch/roll (dev.15: roll applicato ~1,9° contro ~4,8° richiesti, limite `WORKSPACE`), quote STAND diverse da 100/125, WALK FAST, funzioni avanzate | #52 |
| Termica servo | picchi termici isolati ID21/ID31 (dev.15/dev.16) da discriminare tra dato spurio e fault reale, senza ridurre le protezioni | #52 |
| HostLink / Jetson | controllo Motion via USB (HostLink) non qualificato su hardware; Jetson non ancora integrata | — |

## Archivi storici — per provenienza, non per attività corrente

Le sezioni originali della dev.15 e dev.16 sono preservate nei loro dossier, anziché ripetere qui checklist di una vecchia build che riportavano `PR #64 DRAFT`, `dev.17 non flashata` o `PRONE non qualificato`:

- [dev.16 hardware](Validation_Reports/Dev16_HW_2026-11-07/REPORT.md) · [candidata pre-flash](Validation_Reports/Dev16_Candidate_2026-11-07/REPORT.md).
- [dev.15 hardware](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [RCA timing](Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md).
- [dev.17 candidata offline](Validation_Reports/Dev17_Candidate_2026-11-07/REPORT.md) · [dev.17 realmente validata e promossa a Stable](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md).

Per i dettagli dello stato del repository, del tag e dell'APP installata usa [README corrente](../README.md) e [Release v1.0.1](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1). Il tag è immutabile: i successivi commit di sola documentazione su `main` non alterano il binario originale pubblicato.

Archivio cronologico: [dev.14 05/11](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [dev.7 31/10](Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md) · [snapshot documentale dev.14](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md) · versione completa precedente di questo file, con gli snapshot dev.15/dev.16, al commit della release: [`OPEN_ITEMS.md` @ `35f5ca4`](https://github.com/MattRobotics/robot-dog/blob/35f5ca43e0118cdfc501c14456efb615abb9a8be/09_Logs/OPEN_ITEMS.md).

Prossimi interventi tecnici: issue [#52](https://github.com/MattRobotics/robot-dog/issues/52), [#62](https://github.com/MattRobotics/robot-dog/issues/62) e [#48](https://github.com/MattRobotics/robot-dog/issues/48), tutte da trattare come **lavoro del ciclo successivo**.
