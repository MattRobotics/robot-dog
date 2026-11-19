# MATDOG 18/11/2026: validazione hardware di `30c894a0d430` (ACQUIRE v2) — verdetto finale

**Sessione:** 11:17Z–12:36Z sulla fixture, con l'operatore presente.
- Flash APP-only, catena A/B di 8 prove, ciclo a freddo con la chiave.
- Dashboard D1/D2 non eseguite: Chrome non era controllabile da questa sessione (estensione Claude in Chrome non collegata, nessuno strumento di controllo dello schermo). Il token amministratore non è stato letto né inserito.
- Acquisizione chiusa per decisione dell'operatore. Analisi solo sui log, senza altri comandi al robot.

**Procedura e criteri:** `../MATDOG_1_0_2_ACQV2_CLEAN_20261118/hw_session/PROCEDURE.md`, fissati prima della prova.

## Verdetto

| Voce | Esito |
|---|---|
| Installazione e identità | **VERIFIED HARDWARE, PASS** |
| NVS gen 1, calibrazione 12/12, nessuna scrittura | **VERIFIED HARDWARE, PASS** |
| Catena sulla fixture (140 passi), sicurezza, memoria | **VERIFIED HARDWARE, PASS** |
| STOP USB | **VERIFIED HARDWARE, PASS** (14/14) |
| ACQUIRE (patch v2) dopo il reset del flash | **VERIFIED HARDWARE, PASS** (B, S, W) |
| ACQUIRE dopo il ciclo a freddo | B **FAIL** per il criterio pre-registrato (vedi §3); prima STAND dopo il freddo NOT TESTED |
| Motion Timing P0 | **APERTO** (massimo 20 761 µs, 2 overrun) |
| DALY | silenzio di ≈ 11,5 min dopo il QON su `85bad0e`, ripresa spontanea; poi 214/0 e 212/0 |
| STOP HTTP, dashboard D1/D2, joystick dalla dashboard | **NOT TESTED** |

**Firmware installato:** `30c894a0d430`.
- Commit locale `30c894a0d4303d497d750cccb5fd12f347c61d7d`, non pubblicato: sul remoto `integ/…-20261114` = `06b6db0`.
- APP `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340`.
- Rollback pronto a `85bad0e`: script `1546942f…`, backup `app_images/85bad0e` verificato.

## 1. Controlli hardware PASS

**Flash** (`flashlog/flash.log`):
- `APPLICATION_ONLY_FLASH = PASS`, scrittura `0x010000..0x1c2670`.
- `verify-flash` con digest uguale.
- Prima del flash, `rom_checks.sh` = PASS: layout `8f756ecb…`, app0 attiva, installato `85bad0e` con digest uguale.

**Identità** (`usb_boot/serial.log`, `qon_cold/serial.log`):
- `build_id=30c894a0d430`, `GIT_SHA=30c894a0d4303d49…`, `GIT_DIRTY=NO`;
- `ROBOT_POWERED`, OTA ingest DISABLED.

**NVS e calibrazione:**
- `@CALIBRATION PERSIST STATUS` identico prima del flash (`qon`), dopo il flash (`usb_boot`) e dopo il ciclo a freddo (`qon_cold`):
  - `CLASS=CONSISTENT`, generazione 1, `SLOT_A=VALID`;
  - `LOAD_COUNT=4`, `DEGRADED=0`;
  - `MARKER=VALID`.
- Restore al boot: `generation=1 fingerprint=be8d5902bb4e37da restored=1 mapped=12/12 torque=UNCHANGED_OFF`.
- Nessun comando di scrittura NVS inviato.

**Preflight 9/9 PASS** prima e dopo il flash (`pre/`): census 13/13, IMU valida, SAFE_OFF 8191, coppie OFF, BMS OK, heap ≥ 24 KB.

**Catena A/B** (`chain.log`, `analysis_ab_compare.out`):
- **Passi:** 140/140 PASS, 0 abort, 0 fault.
- **Memoria e stack:** `heap_min_free` minimo 48 200 B (18/11 su `85bad0e`: 34 344 B). Stack minimo libero: loop 6 352 B, httpd 9 976 B.
- **BMS:** 0 campioni non OK durante la catena.

**STOP USB:** 14 `@STOP`, tutti verificati e seguiti dal passo successivo.
- A1, a riposo;
- B4, durante la camminata con rotazione;
- F4, durante il WALK FAST;
- H5 e H12, durante il WALK;
- H7 e H14, durante la RECOVERY, con ripresa STAND fino in fondo.

## 2. Motion Timing P0 e release perse

| Misura | `30c894a` (oggi) | `85bad0e` (18/11) | `6be4690` (17/11) |
|---|---:|---:|---:|
| Massimo per release | **20 761 µs** (ab_A, passo B3, rotazione col joystick) | 20 604 µs | 20 449 µs |
| Overrun (> 20 ms) | **2** (ab_A, B3) | 3 | 1 |
| Prove con massimo ≤ 16 ms | 1/8 (h_B, 15 447 µs) | 0/8 | 0/8 |
| Release perse durante un comando | **0** in 8/8 prove | 5 (prima STAND) | 6 (prima STAND) |
| Release ≥ 15 ms, profilo A / B | 188 / 29 | 147 / 30 | 168 / 25 |

**Release ≥ 15 ms:**
- Gli intervalli di Poisson al 95 % si sovrappongono in tutti i confronti: nessuna differenza dimostrata.
- Anche nel profilo B, a carico HTTP ridotto, 11 finestre superano 16 ms. Il residuo quindi non dipende solo dal carico HTTP.

**`missed_releases` dal boot = 5**, tutte IDLE: sono la preparazione del registro 85 al READY, contata fuori da ogni finestra di comando. Con `85bad0e` le stesse 5 erano dentro la prima STAND.

**P0 resta APERTO:** il massimo supera 16 ms e ci sono 2 overrun. Il criterio non è stato reinterpretato.

## 3. ACQUIRE (`analysis/ACQ_CHECK.md`, `acq_check.py` pre-registrato)

**Dopo il reset del flash:**

| Criterio | Esito | Dato |
|---|---|---|
| B | PASS | riga `MOTION_ACC_CEILING verified=220` alle 11:40:44.808Z, senza comandi dell'host nei 2 s precedenti |
| S | PASS | prima STAND → verifica del registro: **11 ms** (prima 131 ms) |
| W | PASS | prima finestra della STAND con **0** release perse (prima 5); massimo 18 141 µs (STAND_DEPLOY) |

**Dopo il ciclo a freddo: B = FAIL secondo la regola pre-registrata.**
- La riga c'è, alle 12:08:37.729Z (uptime ≈ 4,2 s, al READY).
- Nei 2 s precedenti `boot_reads.sh` aveva inviato letture di stato (`@BMS STREAM ON`, `@SYSTEM SOURCE_SIGNATURE`, `@STATUS`, …).
- Fatto osservato, che non cambia il verdetto: prima della riga il log non contiene alcun comando Motion.
- La prima STAND dopo il ciclo a freddo non è stata eseguita: NOT TESTED.

**Osservazione O1 della revisione** (feedback vecchio per un istante dopo il blocco): non osservata nei campioni disponibili, il primo a ≈ 2,8 s dal blocco (`now_ms` 7 069 contro la riga a uptime ≈ 4 260 ms). A scala di millisecondi resta UNKNOWN.

**Riarmo dopo lo spegnimento del solo rail servo:** non provocabile sul robot (chiave comune a MCU e servo, USB solo dati). Resta VERIFIED OFFLINE.

## 4. DALY BMS (problema separato, nessuna modifica)

**Accensione con `85bad0e`** (`qon/serial.log`):
- ultima risposta valida a uptime 317 290 ms;
- poi silenzio dalle 11:22:41.989Z alle 11:34:12.198Z (≈ 11,5 min);
- ripresa spontanea, gate PASS con 154 letture valide consecutive.

Il controller si è fermato in sicurezza (STOPPED, coppie OFF, `BMS_COMMUNICATION_LOST`). Dopo la ripresa è tornato READY.

**Dato nuovo per l'RCA:** il primo errore è una risposta **troncata, 128 byte su 129** (primo byte a 14 ms), seguita da `rx_bytes=0`.

| Fase | Esito |
|---|---|
| Dopo il reset del flash (`usb_boot`) | 214 letture, 0 errori: PASS |
| Dopo il ciclo a freddo (`qon_cold`) | **212 letture valide, 0 errori**: PASS |

## 5. Non eseguito

| Voce | Stato |
|---|---|
| Dashboard D1/D2 | NOT TESTED: nessun browser con sessione amministratore controllabile |
| STOP HTTP dalla dashboard | NOT TESTED |
| Joystick dall'interfaccia | NOT TESTED (il joystick via USB è nella catena, B3) |
| Prima STAND dopo il freddo | NOT TESTED |

Queste prove mancanti non annullano le qualifiche precedenti e non richiedono una nuova build.

## 6. Stato finale

**Ultima lettura dello stato Motion via USB, 12:15:50Z:**
- READY/IDLE, `ready=true`, nessun fault;
- coppie OFF, SAFE_OFF 8191 verificato;
- `missed_releases` 5, overrun 0.

**Altro:**
- BMS OK alle 12:35:49Z;
- registratori chiusi, `session_proc.py check --expect-none` = PASS;
- scheda della dashboard chiusa.

**SAFE_TO_POWER_OFF: YES.** Nessuna modifica a firmware, NVS, calibrazioni o limiti oltre al flash autorizzato.

## 7. File

- Avvio con `85bad0e`: `qon/`, `pre/`.
- Flash: `flashlog/`.
- Boot del candidato: `usb_boot/`; ciclo a freddo: `qon_cold/`.
- Catena: `*_1148*`, `chain.log`.
- Analisi: `analysis/` (`CAND_VS_REF_1811.md`, `CAND_VS_REF_1711.md`, `ACQ_CHECK.md`), `analysis_ab_compare.out`.
- D1: `dash1/` (solo letture a riposo).
- Sigillo: `SHA256SUMS.txt`.
