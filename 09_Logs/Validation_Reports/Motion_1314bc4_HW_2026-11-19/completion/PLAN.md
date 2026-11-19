# Completamento hardware di `1314bc411c58`: blocco D e S9 (19/11/2026)

Piano scritto **prima** delle prove; i criteri non si modificano dopo.

**Mandato:** «MATDOG — Completamento hardware Motion, 19/11/2026». L'operatore autorizza il completamento autonomo. Si fanno solo le due coperture mancanti; catena, D1/D2, STOP e P0 non si ripetono.

**Condizioni:**
- robot sulla stessa fixture, zampe libere, operatore presente (dichiarazione del mandato);
- nessun flash, reboot, KEY OFF/ON, modifica firmware o test aggiuntivo.

## 1. Strumenti

Copia identica degli strumenti qualificati di `../MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/hw_session`. Sono gli stessi 30 file usati stamattina, `cmp` uguale.

**Perché servono due selettori nuovi:**
- `fixture_blocks.py` accetta solo `A,B` o `E,F,G`;
- `--start` di `fixture_suite.py` riparte solo da uno STOP tenuto;
- S9 non aveva uno strumento.

Per questo ci sono due selettori di passi sul modello di `fixture_settle.py`. Classe `Suite`, comandi, controlli, limiti e regole di STOP restano quelli di `fixture_suite.py` pinnato (`EXPECT_BUILD=1314bc411c58`).

**`fixture_block_d.py`:**
- passi D1–D23 presi per nome dalla sequenza pinnata;
- dopo D1, MODE STANDARD e attesa in sola lettura della pubblicazione, come B1 nella suite e S2/S2b in `fixture_settle.py`. Servono perché le ampiezze del blocco D sono i massimi STANDARD;
- MODE SLOW prima di D22, come F9.

**`fixture_s9_slow_steer.py`:**
- K0: SLOW selezionato, in sola lettura e a riposo;
- K1: STAND 120;
- K2–K7: laterale sinistra e destra, yaw antiorario, con i rilasci (come D4–D9);
- K8–K11: yaw orario terminato da `@STOP`, 4 s tenuto, ripresa STAND (come D10, B4–B6);
- K12: PRONE; K13: dump TIMING.
- Le ampiezze sono i limiti SLOW pubblicati dal dispositivo. Nessun cammino indietro (fuori da S9).

**Carico HTTP:** profilo A, quello operativo: poller 0,5 s più watcher 1 Hz, tramite `run_acquisition.py` (M0, heap guard, chiusura verificata). Una prova per blocco: `blockD_A_<HHMM>`, poi `s9_A_<HHMM>`. S9 parte solo se il blocco D finisce PASS in PRONE_REST.

## 2. Criteri

**Blocco D: PASS solo se tutte le condizioni valgono.**
1. 28/28 passi PASS, uscita 0, `ACQUISITION_CLOSED`.
2. Nessun campione FAULT.
3. Esiti dei passi:
   - D2, D4, D6, D8, D10: WALKING;
   - ogni rilascio: STANDING COMPLETED;
   - D14 e D18: STOP verificato con ripresa ammessa;
   - D15: STAND_REACHED; D19: WALKING;
   - D22: PRONE_REST con `safe_off_mask` 8191.
4. Velocità eseguita (`walk_velocity` a fine passo) nella direzione richiesta e solo sull'asse richiesto:
   - D2: vx < 0;
   - D4: vy > 0; D6: vy < 0;
   - D8: yaw > 0; D10: yaw < 0.

**S9: PASS solo se tutte le condizioni valgono.**
1. 15/15 passi PASS, uscita 0, `ACQUISITION_CLOSED`.
2. Nessun FAULT, in particolare nessun `WALK_IK_OR_WORKSPACE`, `STOP_MEASURED_OUTSIDE_BOX` o `STOP_RECOVERY_PATH_NOT_CLEAR`.
3. Esiti dei passi:
   - K9: STOP verificato con ripresa ammessa;
   - K11: STAND_REACHED;
   - K12: PRONE_REST 8191.
4. Direzioni:
   - K2: vy > 0; K4: vy < 0;
   - K6: yaw > 0; K8: yaw < 0.

**P0, per prova** (criterio della catena): massimo ≤ 16 000 µs, 0 overrun, 0 release perse (`missed` +0 in ogni passo).

**Memoria:** `heap_min_free` ≥ 24 576 B, nessuna allocazione fallita.

**Riportati, non criteri:**
- motivi della compensazione IMU, conteggio `WORKSPACE` compreso;
- RF UPPER: intervallo di `commanded_raw` e `raw` in tick, ed errore di inseguimento. L'angolo in rad non è pubblicato, perché la calibrazione non è nello stato;
- errore di inseguimento di tutti i giunti, temperature, DALY.

## 3. Arresti

Le regole della suite restano autoritative:
- prima di ogni movimento: BMS, IMU, fault, 12 giunti freschi con status 0, temperatura < 65 °C, tensione ≥ 10 V;
- timeout di passo e stati inattesi fanno inviare `@STOP`; nessuna ripresa e nessun nuovo tentativo;
- `heap_guard`: MEMORY_STOP.

Dopo un arresto non si riprende automaticamente: si riporta lo stato e si porta il robot in sicurezza per la via qualificata.

## 4. Chiusura

PRONE_REST, coppie OFF, SAFE_OFF 8191, nessun fault; `@BMS STREAM OFF`; `session_proc.py check --expect-none`; sigillo `SHA256SUMS.txt`; commento su #70.
