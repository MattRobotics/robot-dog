# MATDOG 19/11/2026: completamento hardware di `1314bc411c58` — blocco D e S9

**Sessione:** 07:24Z–07:33Z sulla stessa fixture di stamattina, zampe libere. L'operatore aveva autorizzato il completamento autonomo; nessun intervento gli è stato chiesto.

**Firmware:** `1314bc411c58`, APP `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`. Stesso avvio del flash di stamattina:
- l'uptime cresce senza interruzioni, da 2 606 s a fine D2 a 4 543 s alla ripresa;
- l'assetto IMU differisce di 0,09° dalla fine di D2: il robot non è stato spostato.

**Piano e criteri:** `PLAN.md`, scritto prima delle prove. Non si ripetono catena, D1/D2, STOP né P0, già qualificati (`../MATDOG_1_0_2_MOTION_1314BC4_HW_20261119/REPORT_HW.md`).

## Verdetto

| Voce | Esito |
|---|---|
| Blocco D: joystick indietro (D2) e laterale (D4, D6), rotazione, STOP in due fasi, riprese | **PASS**, 27/27 passi |
| S9: SLOW sterzato, laterale e rotazione ai limiti SLOW, con STOP USB | **PASS**, 15/15 passi |
| P0 | **Confermato** in entrambe le prove: massimo 12 729 µs, 0 overrun, 0 perse. **Nuovo massimo** dal boot (prima 11 012 µs, §3) |
| Stato finale | PRONE_REST, coppie OFF, SAFE_OFF 8191, nessun fault |

## 1. Verifiche prima del movimento

| Voce | Esito |
|---|---|
| Identità | `SOURCE_SIGNATURE build_id=1314bc411c58`, `GIT_DIRTY=NO`, ROBOT_POWERED |
| NVS | `CLASS=CONSISTENT`, generazione 1, `SLOT_A=VALID`, `LOAD_COUNT=4`, `DEGRADED=0`, `MARKER=VALID` |
| Preflight 9/9 PASS | BMS fresco e OK; census 13/13; IMU valida; SAFE_OFF 8191, coppie OFF; READY senza fault; heap ≥ 24 KB |
| Stato | READY/IDLE, PRONE_REST |
| Giunti | 12 con status 0, temperatura massima 25 °C, tensione minima 11,0 V |
| DALY | `comm=OK`, allarmi 0, SoC 52 % |
| STOP | Disponibile: canale USB attivo, nessuno STOP in corso |

## 2. Strumenti

**Comandi:** solo quelli di `fixture_suite.py` pinnato (`EXPECT_BUILD=1314bc411c58`). Nessun browser, nessun joystick manuale.

**Carico HTTP:** profilo A operativo (poller 0,5 s più watcher 1 Hz), tramite `run_acquisition.py`: M0, heap guard, chiusura verificata.

**Selettori di passi.** Il blocco D da solo e S9 non si potevano lanciare con gli strumenti esistenti:
- `fixture_blocks.py` accetta solo `A,B` o `E,F,G`;
- `--start` riparte solo da uno STOP tenuto;
- per S9 c'era solo la descrizione nella procedura.

Ho quindi scritto due selettori di passi sul modello di `fixture_settle.py`. Classe `Suite`, comandi, controlli, limiti e regole di STOP sono invariati.
- **`fixture_block_d.py`:** passi D1–D23 presi per nome dalla suite pinnata. Dopo D1 c'è MODE STANDARD con attesa in sola lettura della pubblicazione, perché nella suite il blocco D gira in STANDARD (B1) con i massimi STANDARD; prima di D22, MODE SLOW come F9.
- **`fixture_s9_slow_steer.py`:** passi come D4–D10 e B4–B6, con le ampiezze massime SLOW pubblicate dal dispositivo; nessun cammino indietro.

**Errore di conteggio nel piano:** `PLAN.md` indica «28/28», ma la sequenza stampata da `--plan` prima della prova aveva 27 passi. Il piano non è stato corretto dopo la prova.

## 3. Blocco D (`blockD_A_0725`), PASS

**Passi:** 27/27 PASS, `missed +0 overruns +0` in ogni passo, `ACQUISITION_CLOSED`.

**Velocità eseguita** (`walk_velocity`, `joystick=true`): sempre solo sull'asse richiesto.

| Passo | Movimento | Velocità |
|---|---|---|
| D2 | indietro | vx −0,0167 m/s |
| D4 | laterale sinistra | vy +0,0067 m/s |
| D6 | laterale destra | vy −0,0067 m/s |
| D8 | rotazione antioraria | yaw +0,040 rad/s |
| D10 | rotazione oraria | yaw −0,040 rad/s |

**Rilasci, STOP e riprese:**
- ogni rilascio porta a STANDING COMPLETED;
- STOP D14 e D18 verificati in 2,41 e 2,8 s, ripresa ammessa (`STAND_RECOVERY_AVAILABLE`);
- D15: STAND_REACHED; D19: ripresa del WALK; D21: WALK STOP;
- D22: PRONE_REST con `safe_off_mask` 8191.

**Sicurezza e feedback:**
- nessun fault né motivo di fault;
- DALY 37/37 OK;
- temperatura massima 47 °C;
- |raw − comandato| p95 ≤ 53 tick su ogni giunto.

**Compensazione IMU** (182 campioni del watcher): ENVELOPE 115, ATTENUATED 49, SLEW 15, WORKSPACE 3. I `WORKSPACE` cadono solo nelle transizioni dopo i rilasci e in un campione dopo MODE STANDARD: è la correzione trattenuta, già vista nella catena.

**P0:**
- 7 979 release in 12 finestre: < 5 ms 1 851, 5–10 ms 6 100, 10–15 ms 28, ≥ 15 ms **0**;
- 0 overrun, 0 perse (+0 su 9 061 release dal boot).
- **Massimo 12 729 µs, il nuovo massimo dal boot**, in D15 (ripresa STAND dopo lo STOP in fase 1, fase `STAND_FROM_STOPPED_WALK`).
  - Sezioni: READ 3 894, NOMINAL 3 593, COMPENSATE 1 778, CONVERT 710, **FRAME 2 548** µs.
  - FRAME vale circa il doppio dei soliti 1,3 ms ed è la causa dell'aumento. Sotto il limite, margine 3,3 ms.

## 4. S9 (`s9_A_0729`), PASS

**Passi:** 15/15 PASS, `missed +0 overruns +0` in ogni passo, `ACQUISITION_CLOSED`.

**Modalità:** SLOW selezionata e attiva in tutti i 126 campioni di cammino. Il nome della fase di cammino resta `WALK_STANDARD` anche in SLOW.

**Velocità eseguita:** sempre solo sull'asse richiesto.

| Passo | Movimento | Velocità |
|---|---|---|
| K2 | laterale sinistra | vy +0,0040 m/s |
| K4 | laterale destra | vy −0,0040 m/s |
| K6 | rotazione antioraria | yaw +0,032 rad/s |
| K8 | rotazione oraria | yaw −0,032 rad/s |

**STOP e chiusura:**
- STOP K9 durante la rotazione verificato in 2,4 s, ripresa ammessa;
- K11: STAND_REACHED; K12: PRONE_REST, 8191.

**`kSteer`:**
- nessun `WALK_IK_OR_WORKSPACE`, `STOP_MEASURED_OUTSIDE_BOX` o `STOP_RECOVERY_PATH_NOT_CLEAR`, nessun fault;
- nei cammini sterzati la compensazione è ENVELOPE 69 e SLEW 4 per ciascun asse (73 campioni in laterale, 73 in rotazione), **nessun WORKSPACE**, con correzioni applicate entro ±31 mrad per componente.

**RF UPPER:**
- comandato fra 1 109 e 1 654 tick;
- |raw − comandato| mediana 5, p95 32, massimo 53 tick (in K8);
- temperatura massima del robot 42 °C; DALY 27/27 OK.

**P0:**
- 6 581 release in 8 finestre, ≥ 15 ms **0**, massimo **10 753 µs**;
- 0 overrun, 0 perse (+0 su 6 658).

**Cosa S9 dimostra e cosa no:**
- **Dimostra** che sul robot, con la patch, lo SLOW sterzato laterale e in rotazione a fondo scala funziona senza rifiuti, fault o arresti di box, con STOP e ripresa.
- **Angolo di RF UPPER:** non è osservabile in rad, perché lo stato pubblica solo tick e la calibrazione non è nello stato. Il margine fra il vecchio limite (0,963456 rad) e il nuovo (1,004874) non è quindi misurato sul robot.
- **Caso `random_3`:** passo 40 mm con beccheggio di circa +55 mrad. **Non è riprodotto**: le correzioni sono rimaste entro ±31 mrad. Resta certificato solo offline (FCL 120/120 + 12/12, inviluppo SLOW PASS).

## 5. Memoria

- `MEMORY_OK` in entrambe le prove: `heap_min_free` 43 368 B, nessuna allocazione fallita.
- Stack minimo libero: loop 7 712 B, httpd 9 972 B.

## 6. Stato finale

**Lettura USB delle 07:32:59Z:**
- READY/IDLE, PRONE_REST, nessun fault;
- coppie OFF su 12/12, `safe_off_verified`, maschera 8191;
- nessuno STOP attivo; modalità SLOW;
- `missed` 0 e overrun 0 su 253 309 release dal boot.

**NVS:** `MARKER_STATE=IDLE`, nessuna generazione in sospeso, quindi nessuna scrittura in corso.

**Chiusura:** `@BMS STREAM OFF`; registratori chiusi; `session_proc.py check --expect-none` = PASS; nessun processo di sessione o di flash.

**Nessuna modifica** a firmware, NVS, EEPROM, partizioni, bootloader, limiti o calibrazione. Nessun flash, reboot né KEY OFF/ON.

**ROBOT PRONTO ALLO SPEGNIMENTO: SÌ**

## 7. Restano non eseguiti

- Ciclo a freddo S7.
- Joystick dalla dashboard.
- Cammino a terra: STANDARD e FAST hanno `hardware_qualified=false`.
- Condizione `random_3` sul robot.

## 8. File

- Piano: `PLAN.md`.
- Selettori: `fixture_block_d.py`, `fixture_s9_slow_steer.py`.
- Verifiche iniziali: `pre_usb/`, `pre/preflight_1314bc4.json`.
- Prove: `blockD_A_0725/`, `s9_A_0729/`, `runs.log`, console.
- Chiusura: `final_usb/`.
- Analisi (`analysis/`):
  - `SUITE_SUMMARY_*`, `AB_ANALYZE_*`, `HEAP_CHECK.txt`;
  - `STEER_CHECK_*`, con lo script `steer_check.py` (sola lettura).
- Sigillo: `SHA256SUMS.txt`.
