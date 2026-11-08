# RCA — primo FAULT hardware di `1.0.2-dev.3` (`NATIVE_TARGET_NOT_REACHED`) — traccia (38), 2026-11-08

Robot sulla fixture; sequenza joystick in una pressione AVANTI → AVANTI-DESTRA → DESTRA → INDIETRO-DESTRA → INDIETRO; FAULT all'inversione completa. Robot già in KEY OFF: `@MOTION DIAG` non recuperabile. Analisi solo offline, sorgente installato `94a260c`. Evidenze: [`rca_trace38/`](rca_trace38/) (traccia, SHA256 `2e22bf5b…`; due sorgenti di banco).

## 1. Fatti dimostrati dalla traccia

La traccia contiene 30 campioni, tutti successivi al fault, e nessun comando (pagina ricaricata).

| Fatto | Evidenza |
|---|---|
| FAULT `NATIVE_TARGET_NOT_REACHED`, SAFE_OFF 8191, coppia OFF | `state/reason/safe_off_mask` |
| Comando 204 ACCEPTED a 648 156 ms, ABORTED a 659 545 ms (11,389 s) | `active_command` |
| La finestra temporale per comando, riarmata da `stand()` (`timing::arm`), contiene **solo** release `HEIGHT`: lavoro massimo 8,4 ms, 0 overrun, 0 perse | `command_timing.max_work_phase=HEIGHT` |
| L'ultimo passo è un passo **joystick concluso da un WALK STOP completo** | `walk_cycles 7,575` = quarto + 1 ciclo + 0,075 di assestamento (duty 0,825) |
| Posa comandata al fault = **stand canonico 120 mm + correzione di livello roll +0,060 / pitch −0,0675 rad** (3,4° / −3,9°) | `commanded_raw` (lo scarto da `raw` è ≤ 8 tick); fit con [`fit_fault_pose.cpp`](rca_trace38/fit_fault_pose.cpp): rms 4,8 mrad (3 tick), diagonale LF/RH fino a 166 mrad dal canonico |
| Ultima correzione IMU: `reason=WORKSPACE`, `wanted=[0,0]` | `imu_correction` (applied 0 per il reset del fault) |
| 209 overrun / 12 perse sono **cumulativi dal boot**, non nella finestra del fault | `overrun_releases`, `missed_releases` contro `command_timing` |

## 2. Percorso ricostruito e prima divergenza

Ogni deadline `NATIVE_TARGET_NOT_REACHED` vale ≥ 12 s dall'inizio della propria fase. Il fault cade 11,4 s dopo il 204, quindi la fase scaduta era iniziata prima del 204. Nel ramo `HEIGHT` di `update()` (attesa di srotolamento) la deadline è `12 + max(|roll|,|pitch|)/0,020 = 15,4 s`, con `stand()` a ≈ 644,2 s. Il 204 è quindi un refresh accettato durante la salita oppure il rilascio (neutro).

1. Inversione (`JOYSTICK_REVERSING`) → WALK STOP completo → WALK_SETTLE → STANDING: compatibile con `walk_cycles` e con la logica d'inversione verificata al banco.
2. Al primo STANDING la pagina invia l'intento tenuto → `drive()` → `stand(kWalkHeightM)` → `HEIGHT` (riarma la finestra di timing).
3. **Prima divergenza:** prima di muoversi, `HEIGHT` deve riportare a zero la correzione di livello della HOLD (`pendingGoalValid_`, `compensateLevel(..., unwind=true)`). Ogni passo è stato **rifiutato da `compensationSafe`** (`WORKSPACE`, `wanted 0`), la posa comandata è rimasta ferma a correzione piena, nessun progresso → deadline → FAULT.

## 3. Causa, concausa, ipotesi

- **Causa dimostrata del fault:** lo srotolamento pre-movimento di `HEIGHT` resta bloccato su una correzione di livello grande (60/67 mrad) perché ogni passo viene rifiutato; senza progresso, la deadline lo trasforma in `NATIVE_TARGET_NOT_REACHED`. Codice **invariato da v1.0.1**: ramo `HEIGHT`, `compensationSafe`, deadline, srotolamento. Lo stesso percorso lo attraversa qualunque STAND, WALK o avvio joystick da una HOLD.
- **Concausa dimostrata (fit + banco):** la correzione arriva a 3,4° / 3,9°, contro i −24…−29 mrad di dev.17 in HOLD. Il livello si stima dalla FK degli encoder delle zampe in appoggio. Al banco:
  - il solo cedimento servo misurato nella traccia (−8…+6 tick) genera ~30 mrad di correzione apparente;
  - con il corpo trattenuto, perché la fixture non lascia seguire l'assetto, l'anello diventa aperto e la correzione sale fino al limite del box (fino a −132 mrad).
- **Contributo di dev.3:** la ripartenza automatica a STANDING chiama `stand()` subito dopo l'assestamento. Con dev.17 l'operatore doveva premere di nuovo, ma percorreva lo stesso codice.
- **UNKNOWN — quale controllo di `compensationSafe` rifiuta sul robot:** il banco non lo riproduce. Con cedimento misurato, tracking ritardato e inclinazione ±3,4°/±3,9°, lo stesso srotolamento si completa in 1–3 s e il passo all'indietro riparte. Candidati:
  - margine di ritardo pesato sull'età del feedback ai bordi HIP di `kHeight` (~10 tick al fault);
  - residuo di carico;
  - prova dello STOP.
  Serve lo snapshot di uscita (feedback, età, carichi, goal, deadline).
- **Escluso:** logica d'inversione joystick. Al banco inversione → WALK STOP → STANDING → ripartenza funziona, anche con rilascio durante la salita, cambi di direzione e seconda inversione. Esclusi anche il budget di tempo (0/0 nella finestra) e il BMS.

## 4. Patch

**Nessuna.** Il difetto non è riproducibile offline. Correggere `compensationSafe`, la deadline o lo srotolamento senza sapere quale controllo rifiuta vorrebbe dire indebolire una protezione alla cieca. Nessuna build.

## 5. Decisione

`1.0.2-dev.3` resta utilizzabile solo per collaudi **limitati a terra** (anello di livello chiuso, correzione ~25 mrad come dev.17), dove il percorso è identico a v1.0.1. Va controllato `imu_correction.applied` prima di ogni STAND, WALK o joystick partito da una HOLD: se supera ~40 mrad, PRONE oppure STOP e ripresa. Non usarla per cambi di postura **sospesi sulla fixture** dopo HOLD o passi lunghi: lì il rischio di ripetere il fault è concreto.

## 6. Unica prova hardware mirata (dopo autorizzazione)

Sulla fixture, come nell'evento, senza joystick né passo, con l'USB-C collegato e il robot **non** spento al fault:

1. STAND 120 → HOLD 30–60 s; leggere `imu_correction.applied` dalla dashboard o da `/api/motion/status`.
2. Se |applied| ≥ 40 mrad: STAND 120 (stessa quota).
3. Se resta in `HEIGHT` con `WORKSPACE` o va in FAULT: lettura **passiva** USB di `@MOTION DIAG` subito, prima del KEY OFF. Apertura senza reset, nessun comando oltre la lettura.
4. Scaricare la traccia della dashboard.

Lo snapshot `exit` (feedback, età, carichi, `commanded_raw`, deadline) permette di ricalcolare offline ogni controllo di `compensationSafe` e di individuare quale rifiuta. Se invece STAND 120 si completa, il fault richiede anche il passo o l'inversione e va ripetuto il percorso completo con la stessa acquisizione.
