# MATDOG `1.0.2-dev.4` — validazione hardware dell'08/11/2026 (fixture e pavimento)

Firmware installato e provato: **`1.0.2-dev.4`**, build `4358f13234f9`, sorgente CLEAN `4358f13234f99c3c29470cebc381bd95cb822d41`, APP SHA256 `3e7ee77d934bb681efcf4b7ff534447aab9d08db599df2ab45601efb19ed9c66` (1.723.024 B), app0 `0x010000`. **dev.4 è VALIDATED HARDWARE nel perimetro provato qui sotto, NOT STABLE.** La Stable resta `v1.0.1` (dev.17). Evidenze e SHA256: [EVIDENCE_MANIFEST.md](EVIDENCE_MANIFEST.md). Metriche per finestra: [TRACE_METRICS.csv](TRACE_METRICS.csv). Comandi conservati nelle tracce: [COMMANDS_RETAINED.csv](COMMANDS_RETAINED.csv). Note operatore: [OPERATOR_OBSERVATIONS.md](OPERATOR_OBSERVATIONS.md).

Classi: **VH** = verificato su hardware; **VO** = verificato solo offline; **UNKNOWN** = non determinabile dai dati disponibili.

## 1. Flash APP-only e H0 (robot sulla fixture, PRONE) — VH

| Controllo | Esito |
|---|---|
| Preflight su dev.3 | READY/RUN, SAFE_OFF 8191, torque OFF ×12, BMS 11,2 V fresco, NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13, 5 asset di rollback OK |
| Layout in sola lettura | tabella `8f756ecb…` (invariata, `device-layout.log` byte-identico a quello della dev.17), otadata [1,0], app0 attiva |
| Flash | una scrittura 1.723.024 B in `0x010000–0x1b4a90`, hash e `verify-flash` PASS, NVS non toccata |
| Primo boot | `1.0.2-dev.4` / `4358f13234f9`, READY/RUN, IMU PASS, NVS gen-1 valida, census 13/13, SAFE_OFF 8191, pose `PRONE_ENTRY` |
| IDLE 64 s (solo GET) | 50,01 Hz, 0 perse, 0 overrun, max gap 23,4 ms, lavoro max 4,9 ms, stack loop min 5228 B, HTTP 78/78 |

## 2. H1-RCA sulla fixture — fix dello srotolamento IMU — VH

Due sessioni dell'operatore con acquisizione read-only (USB senza reset, HTTP 1 Hz, tracce dashboard 41 e 42). Il report originale con i dettagli è nello ZIP fixture (`H1RCA_RETEST_REPORT.md`).

| Sessione | Sequenza | Correzione all'avvio di HEIGHT/PRONE_HEIGHT | Esito |
|---|---|---|---|
| 1 (19:59–20:03Z, traccia 41) | STAND 120, joystick con 2 inversioni, 2 STOP con ripresa, WALK 75, PRONE | max +28,0/−44,6 mrad, dentro la fascia nominale | nessun FAULT; **non esercita il fix** (solo non-regressione) |
| 2 (20:07–20:09Z, traccia 42) | joystick indietro-sinistra prolungato, diagonali opposte, 2 inversioni, rilascio, PRONE | **+58,3/−72,4, +58,7/−73,0, +55,4/−75,0 mrad** (inclinazione asse piede 0,064/0,065/0,061 rad > fascia 0,0349) | tutte e tre riportate a zero in ~1,8 s (SLEW monotono, mai WORKSPACE), ripartenza joystick dopo ogni inversione, `PRONE_REST`, **nessun FAULT, nessun `NATIVE_TARGET_NOT_REACHED`** |

`@MOTION DIAG` finale (senza reset): exit `PRONE_REST`, `fault=0`. Sul fault dev.3 (traccia 38 e H1) la stessa condizione (+63,7/−66,5 mrad) faceva rifiutare ogni passo da `pointSafe`. **VERIFIED HARDWARE — FIX H1-RCA**, limitatamente a: fixture (corpo trattenuto), STAND 120, joystick a fondo scala, correzioni fino a ~59/−75 mrad. Tempi sessione 2 (joystick + assestamento, 104 s): 11,5 % overrun, 0,54 perse/s, lavoro max 47,8 ms, stack loop min **1696 B**.

## 3. Prova a terra (pavimento) — originali della dashboard del cellulare

Due boot. **Boot 1** (prima di KEY OFF/ON): traccia `matdog-motion-trace.json` + screenshot. **Boot 2**: tracce `(1)`, `(2)`, `(3)`; `(2)` e `(3)` si sovrappongono (comandi 629–669, campioni 367,7–376,1 s): campioni deduplicati per uptime, comandi per sequenza (649 campioni distinti, 58 comandi). I JSON sono osservazioni HTTP campionate (~0,35 s), non un log per release.

### 3.1 Rifiuto iniziale e latch (boot 1) — difetto riprodotto dai dati

| Fatto (traccia/screenshot) | Valore |
|---|---|
| STAND 150 | `ABORTED` · `INITIAL_POSE_OUTSIDE_SAFE_ENTRY` (screenshot, uptime 79 s; SAFE_OFF verificato a 72,34 s) |
| STOP successivo | accettato a 95,33 s, chiuso a 95,331 s con `SAFE_OFF_VERIFIED_FAULT_LATCHED` (motivo prodotto solo da uno STOP ricevuto dopo un fault) |
| 106,4–116,6 s (30 campioni) | `state=FAULT`, motivo invariato; `pose_region=PRONE_ENTRY`, 12 feedback validi e freschi (età ≤ 105 ms), torque 0 ×12, velocità 0, stato servo 0, SAFE_OFF 8191, potenza READY |
| Posa rimessa a mano (116,6 s) contro posa accettata dopo KEY OFF/ON (boot 2, 27,5 s) | **identiche**: 10 giunti stesso RAW, 2 giunti ±1 tick; margine minimo dalla box `kEntry` 10,5 mrad (RH/LH LOWER) |

Causa nel sorgente dev.4 (`SimpleMotion.cpp`): in ACQUIRE e REFERENCE, prima di qualsiasi abilitazione, `fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY")` imposta `faultLatched_`, che nessun percorso azzera; STOP con fault presente si limita a osservare il SAFE_OFF e STAND risponde `FAULT_STOP_REQUIRED`. Lo stesso motivo copre anche un feedback non fresco. **Il power-cycle non ha cambiato la posa: ha solo cancellato il latch.** UNKNOWN: la posa nell'istante del rifiuto (nessun `@MOTION DIAG`, campioni a partire da 34 s dopo). Ipotesi dell'operatore, compatibile ma non dimostrata: posteriore leggermente sollevato.

### 3.2 Boot 2 — funzioni eseguite e metriche

| Finestra (uptime) | Esito | Overrun | Perse | Escursione laterale piedi p-p |
|---|---|---|---|---|
| STAND 150 da PRONE (32,1–45,7 s) | `STAND_REACHED` in 13,8 s | 1,6 % | 0 | 0,5 mm |
| WALK STANDARD 100 #1 (52,4–92,3 s, ~7,9 cicli) | cammina, nessun fault | 2,3 % (46/1994) | 0 | **2,9 mm** |
| WALK STOP + assestamento #1 | `STAND_REACHED` | 1,9 % | 0 | 2,6 mm |
| Joystick laterale (vy 3,0 mm/s, vx 1,0–1,6) | `WALK_JOYSTICK` | 16,2 % | 2 | 30,6 mm |
| Joystick avanti (vx 6,9, **vy 0,1–0,4 mm/s**) | `WALK_JOYSTICK` | 12,4 % | 5 | **28,3 mm** |
| Joystick rotazione antioraria 1,76 °/s | `WALK_JOYSTICK` | 20,9 % | 4 | 31,0 mm |
| Rilascio + assestamento (×3) | `STAND_REACHED` | 9,5–12,6 % | 1–2 | 28–32 mm |
| WALK STANDARD 100 #2 (388,7–439,1 s) | cammina | 0,6 % (14/2521) | 0 | 2,8 mm |
| STOP durante WALK (seq 671) | `STOPPED_SUPPORT_UNVERIFIED` in 2,7 s, `STAND_RECOVERY_AVAILABLE` | 0 % | 0 | — |
| STAND 150 dal WALK arrestato (seq 672) | `STAND_FROM_STOPPED_WALK` → `STAND_REACHED` | 13,3 % | 2 | 2,5 mm |
| STAND 85 → 135 → 80 | tutti `STAND_REACHED` | 0,2 % | 0 | — |
| PRONE (seq 676) | `PRONE_REST` | 0,5 % | 0 | — |

Totali boot 2: **0 fault**, 58/58 comandi HTTP 200, lavoro max per release 43,4 ms, max gap 45,0 ms, late 0, busy 0, heap min 57.128 B, **stack loop min 2064 B** (raggiunto nella ripresa dal WALK arrestato). Joystick complessivo (3 movimenti + 3 assestamenti, 45,8 s): 13,6 % overrun, 15 perse (0,33/s).

L'escursione laterale è la variazione picco-picco della media delle 4 referenze di contatto nel telaio corpo, calcolata con la cinematica diretta del firmware dai 12 angoli misurati ([`tools/`](tools/)). Con il campionamento a ~0,35 s è un limite inferiore del valore reale. Comando di sway della legge: WALK STANDARD 1,5 mm (3 mm p-p), joystick 14 mm (28 mm p-p): **i valori misurati coincidono con le due leggi.**

### 3.3 Video WALK100 — non disponibile

`1000083730.mp4` (~24,9 s, 1920×1080, dichiarato dal README del pacchetto) **non è stato consegnato all'ASUS**: il pacchetto ricevuto è la versione `NO_VIDEO` e il file non esiste sul filesystem. Il suo SHA256 non è elencato in `MANIFEST_SHA256.txt` né in `INDEX.json`. Analisi visiva di appoggi, falcata, oscillazione e trasferimento del peso: **non eseguita (UNKNOWN)**; nessun link pubblico o URL.

## 4. Anomalie

| # | Osservazione | Evidenza | Classe |
|---|---|---|---|
| A1 | Rifiuto della posa iniziale non recuperabile senza riavvio | §3.1: latch permanente con posa poi identica a quella accettata | VH (difetto) |
| A2 | Sway laterale del joystick ~10× WALK STANDARD anche con richiesta quasi solo frontale | 28,3 mm p-p con vy 0,1–0,4 mm/s; WALK 2,8–2,9 mm | VH (misura) + causa nel codice da confermare (Fase B) |
| A3 | Joystick lento | fondo scala osservato: avanti 6,9 mm/s, laterale 3,0–3,1 mm/s, rotazione 1,76 °/s; WALK STANDARD 100: 20 mm/s nominali | VH (valori comandati) |
| A4 | Budget temporale del joystick | 9,5–20,9 % overrun e 0,12–0,65 perse/s nelle finestre joystick; WALK STANDARD 0,6–2,3 % e 0 perse | VH |
| A5 | Ripresa dal WALK arrestato costosa | 13,3 % overrun, 2 perse; stack min 2064 B | VH |
| A6 | Stack loop | 1696 B sulla fixture (joystick), 2064 B a terra; 5228 B in IDLE | VH, causa da analizzare |

## 5. Limiti

- Tracce dashboard: campioni ogni ~0,35 s, buffer di 240 campioni e 48 comandi. I comandi 4–621 del boot 2 (joystick precedenti) non sono nel buffer esportato; i movimenti laterale, avanti e rotazione sono comunque visibili nei campioni (`walk_velocity`).
- Nessun `@MOTION DIAG` / `@MOTION TIMING` acquisito durante la prova a terra.
- Distanza e velocità reali non misurate: i mm/s sono quelli comandati dalla legge.
- Contatto al suolo e supporto passivo in PRONE: non misurati.
