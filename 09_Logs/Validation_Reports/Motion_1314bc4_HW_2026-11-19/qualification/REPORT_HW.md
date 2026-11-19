# MATDOG 19/11/2026: qualifica hardware di `1314bc411c58` (Motion FP32 integrato) — verdetto

**Sessione:** 06:00Z–06:52Z sulla fixture, zampe libere, operatore presente.
- Fasi: controlli non invasivi, flash APP-only autorizzato dall'operatore, catena A/B di 8 prove, dashboard D1 e D2 con STOP HTTP.
- **Dashboard:** usata in Chrome con la sessione amministratore già attiva. Non è servito alcun intervento dell'operatore; il token non è stato né letto né inserito.

**Procedura e criteri:** `../MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/hw_session/PROCEDURE.md`. Erano fissati prima della prova e non sono stati modificati.

## Verdetto

| Voce | Esito |
|---|---|
| Installazione e identità | **VERIFIED HARDWARE, PASS** |
| Boot (nessun panic né eccezione FPU), NVS gen 1, calibrazione 12/12 | **VERIFIED HARDWARE, PASS** |
| DALY | 692 letture `comm=OK` e 0 errori in ≈ 44 min dopo il reset del flash; 850 con il pre-flash |
| Catena sulla fixture (140 passi), sicurezza, memoria | **VERIFIED HARDWARE, PASS** |
| STOP USB | **PASS**, 14/14 |
| STOP HTTP dalla dashboard | **VERIFIED HARDWARE, PASS**, 2/2 (D1, D2), con ripresa STAND fino a STAND_REACHED |
| ACQUIRE dopo il reset del flash | **PASS**: B; S in 5,0 ms; W |
| Motion Timing P0 | **CHIUSO secondo il criterio pre-registrato**, in ogni prova eseguita: catena 8/8, D1, D2. Massimo 11 012 µs, 0 overrun, 0 release perse |
| Regressioni | Nessuna (§5) |
| **Decisione** | **KEEP `1314bc4`** (§6) |

## 1. Identità installata

**Prima del flash:**
- installato `30c894a`, APP `03c863cc…`, digest corrispondente;
- preflight 9/9 PASS;
- gate DALY 152/152;
- gate a vuoto del flash e del rollback PASS.

**Flash** (`flashlog/flash.log`):
- APP `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`, 1 777 952 B a `0x10000`;
- `Hash of data verified`, `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS`;
- tabella delle partizioni `8f756ecb…` invariata.

**Boot** (`usb_boot/serial.log`):
- `SOURCE_SIGNATURE build_id=1314bc411c58 version=1.0.2-dev.7 profile=ROBOT_POWERED`;
- `GIT_SHA=1314bc411c589c4216e95629f4665d9d01f05781 GIT_DIRTY=NO`;
- `ota_running_build_id=1314bc411c58`, ingest DISABLED.

Anche la dashboard mostra la build `1314bc411c58`.

## 2. Boot, NVS, DALY e sicurezza

**Boot:**
- un solo avvio, quello del flash; l'uptime è continuo fino a 2 606 s, a fine D2;
- nei 12 log seriali della sessione non compaiono panic, Guru, `Coprocessor exception` né reset;
- `MOTION_ACC_CEILING verified=220 target=220` al READY, senza comandi.

**NVS e calibrazione:**
- `CALIBRATION_PERSISTENCE_BOOT nvs=READY verdict=VALID_ACKNOWLEDGED`;
- `CALIBRATION_RESTORE_DECISION … generation=1 fingerprint=be8d5902bb4e37da restored=1 mapped=12/12 torque=UNCHANGED_OFF`;
- `LOAD_COUNT=4 LOAD_STATUS=OK … DEGRADED=0`.

Nessun comando di scrittura NVS, EEPROM, partizioni, limiti o calibrazione.

**Servo e IMU:** preflight 9/9 PASS dopo il flash: census 13/13 (`SERVO_POP … last_census=PASS`), IMU valida, SAFE_OFF 8191, coppie OFF.

**Memoria:**
- `heap_free` al preflight 76 944 B, contro 73 096 su `30c894a`: +3 848 B, come atteso dalla IRAM liberata.
- `heap_min_free` minimo 43 368 B in D2 con 2 client: `MEMORY_OK`, soglia 24 576 B, nessuna allocazione fallita.
- Stack minimo libero: loop 7 788 B (`30c894a`: 6 352), httpd 9 972 B (9 976).

**DALY:**
- gate: prima del flash 152/152; dopo il flash 423/423 e 214/214;
- poi in catena, D1 e D2 tutte le letture OK. Nei log seriali: 692 `comm=OK` dopo il flash (850 con le 158 del pre-flash su `30c894a`), 0 timeout, `fail_streak` 0, nessun allarme;
- pacco 11,1 V, SoC da 56,7 a 54,4 %, cella minima 3 708 mV;
- nessun silenzio in ≈ 44 min dopo il reset del flash. Il dato non chiude la RCA DALY, la cui causa resta UNKNOWN.

**Sicurezza:** 0 fault, 0 abort, 140/140 passi, 14/14 STOP USB e 2/2 STOP HTTP verificati.

## 3. P0 sul robot contro `30c894a`

Stessa catena ABBA e stessi profili HTTP:
- profilo A: poller 0,5 s e watcher 1 Hz;
- profilo B: poller 1 s.

Riferimento: catena `1148` del 18/11 su `30c894a`. Analisi in `analysis/AB_TABLES.md` e `analysis/CAND_VS_REF_1811b.md`.

| Misura | `1314bc4` (oggi, catena `0623`) | `30c894a` (18/11, catena `1148`) |
|---|---:|---:|
| Massimo per release | **11 012 µs** (efg_A, WALK_STANDARD) | 20 761 µs (ab_A, B3) |
| Overrun (> 20 ms) | **0** | 2 |
| Release ≥ 20 ms | **0** | 2 |
| Release ≥ 15 ms, profilo A / B | **0 / 0** | 188 / 29 |
| Release 10–15 ms, profilo A / B | 41 / 5 | 2 818 / 2 079 |
| Finestre con massimo ≥ 16 ms | **0** su 56 | 34 su 56 |
| Prove con massimo ≤ 16 ms | **8/8** | 1/8 |
| Release perse, durante un comando / a riposo | 0 / 0 | 0 / 0 |
| `missed_releases` dal boot | **0** su 130 230 release | 5, tutte IDLE al READY |
| Prima STAND dopo il boot (`settle_A`) | massimo 9 270 µs, 0 perse; ACQ S 5,0 ms | 18 141 µs, 0 perse; ACQ S 11 ms |

**Release ≥ 15 ms:** gli intervalli di Poisson al 95 % sono disgiunti in entrambi i profili. Riduzione dimostrata.

**Dashboard:**
- **D1, sola dashboard (1 client):** 10 054 release, 0 perse, 0 overrun.
  - Il massimo dal boot non è salito, quindi il massimo del blocco è ≤ 11 012 µs, limitato ma non risolto.
  - `D1_P0_COUNTERS = PASS` (`analysis/d1_counters.txt`).
- **D2, dashboard più watcher (2 client):** 5 982 release, 0 ≥ 15 ms, massimo 10 670 µs, 0 overrun, 0 perse. PASS.

**Sezioni della release più pesante di ogni finestra** (56 finestre, A+B; `analysis/SECTIONS_WINDOW_MAXIMA_vs_1148.txt`). Valori in µs, media / massimo:

| Sezione | `1314bc4` | `30c894a` |
|---|---:|---:|
| READ | 3 437 / 4 041 | 2 784 / 4 137 |
| NOMINAL | 1 960 / 5 433 | 2 985 / 13 376 |
| COMPENSATE | **1 781 / 3 345** | 8 741 / 13 641 |
| CONVERT | 444 / 617 | 400 / 546 |
| FRAME | 1 182 / 1 349 | 1 234 / 2 357 |
| REST | 280 / 2 122 | 255 / 406 |

- **Calo di NOMINAL e COMPENSATE:** il guadagno è tutto qui, cioè nel lavoro cinematico in FP32. In WALK, NOMINAL massimo scende da 13 376 a 3 883 µs; le PRONE più pesanti passano da 14–19 ms (COMPENSATE 6,7–13,6 ms) a 8,5–9,9 ms (COMPENSATE 1,1–2,5 ms).
- **Parti invariate:** READ, FRAME e CONVERT, cioè l'I/O del bus servo. Ora sono la quota principale del lavoro residuo.
- **Limiti della misura:** i valori riguardano la release più pesante di ogni finestra, non l'intera distribuzione. I contatori PMU per classe esistono solo nelle build diagnostiche.

**Margine:** il massimo misurato lascia ≈ 5 ms (31 %) sotto il limite di 16 ms. P0 è chiuso sui massimi misurati sul robot, non su QEMU né sui tempi medi.

## 4. STAND, WALK, STOP

**Catena, 140/140 passi PASS:**
- STAND da PRONE_REST e dopo STOP;
- HEIGHT 80, 110, 120 e 150 mm;
- WALK STANDARD e FAST con preset, e cambio di modalità durante il cammino (E4, E6);
- joystick in rotazione, oraria e antioraria (settle S3/S5, B3);
- RECOVERY e riprese (blocco H);
- PRONE fino a PRONE_REST con SAFE_OFF 8191.

**STOP USB, 14/14:**
- 2 a riposo (A1);
- 12 in movimento: B4 in rotazione, F4 in FAST, H5 e H12 in WALK, H7 e H14 in RECOVERY.
- Verificati in 2,2–3,2 s, mediana 2,4 s (`30c894a`: 2,0–3,2 s, mediana 2,4), sempre con ripresa ammessa e STAND fino a STAND_REACHED.

**STOP HTTP** (pulsante STOP della dashboard, `MOTION.STOP` con sessione autenticata):
- D1 e D2, entrambi durante il WALK STANDARD dopo almeno 5 s di cammino.
- Esito confermato dal Controller: `STOP · COMPLETED · STOPPED_SUPPORT_UNVERIFIED`, cioè arresto verificato con coppia ON.
- In D2 il tempo fra la richiesta e lo STOP verificato è fra 1,0 e 3,0 s (campioni a 1 Hz).
- In entrambi, ripresa STAND fino a STAND_REACHED, poi PRONE fino a PRONE_REST con coppie OFF verificate sui 13 servo.

**D1 dalla dashboard, tutti COMPLETED:** STAND 120 mm; HEIGHT 125 mm e ritorno a 120; WALK STANDARD per ≈ 10 s, poi WALK STOP.

## 5. Regressioni

**Nessuna.** Confronti con `30c894a` sulla catena `1148`, con lo stesso strumento `suite_summary.py`:

- **Compensazione IMU:** i motivi nelle prove A (ATTENUATED, SLEW, ENVELOPE, WORKSPACE) coincidono entro ±3 campioni e le ampiezze sono simili. `IMU_STALE` in `settle_A` c'è anche nel riferimento: 34 campioni contro 33.
- **Tempi di STOP:** equivalenti (§4).
- **Temperature:** picchi di un solo campione (LH_HIP 60 e 65 °C, RF_HIP 57 °C, con mediana della giuntura 28 °C).
  - Lo stesso schema c'è nel riferimento (RH_LOWER 69 °C).
  - Sono sotto 70 °C e nessuna protezione è scattata: lettura spuria già nota.
- **`heap_min_free`:** 44 592 B in catena contro 48 200 B su `30c894a`.
  - Il valore è già al minimo nel primo campione della catena, con i due client appena connessi; non scende più nelle 8 prove.
  - Il calo a 43 368 B avviene fra 06:49:25Z e 06:49:56Z, quando il watcher di D2 si connette con la dashboard già aperta.
  - Il 18/11 `85bad0e`, con lo stesso codice HTTP, era sceso a 34 344 B.
  - È variabilità fra sessioni, non attribuibile al candidato; l'heap statico è cresciuto di 3,8 KB.
- **REST 2 122 µs** in una release PRONE_HEIGHT.
  - È il piano dei segmenti, lasciato in double per mandato, nella prima release dopo una lettura di stato.
  - Questo tipo di release c'è anche in `30c894a` (QEMU: 250–270 k cicli in entrambe). Ora emerge perché COMPENSATE non domina più.

## 6. Decisione: **KEEP `1314bc4`**

**Motivi:**
- nessun criterio di arresto o di rollback scattato;
- identità, boot, NVS, calibrazione, memoria e DALY conformi;
- P0 chiuso con misure hardware, secondo il criterio pre-registrato, su catena 8/8, D1 e D2;
- STOP USB e HTTP verificati;
- nessuna regressione rispetto a `30c894a`.

Il rollback a `30c894a` resta pronto e non è necessario: script `812a2e83…`, backup `app_images/30c894a` verificato.

**Validità:** sulla fixture, con i carichi HTTP operativi (profili A e B, dashboard con 1 e 2 client). Ciò che resta da qualificare è nel §7.

## 7. Non eseguito e limiti

| Voce | Stato |
|---|---|
| **Joystick indietro e laterale** (blocco D: D2, D4, D6) | **NOT TESTED su HW.** Il blocco D non fa parte della catena A/B (`run_ab_chain.sh`: settle, blocchi A, B, E, F, G, H). **Errata:** la procedura del 19/11 lo indicava per errore come coperto. Qualificato solo offline (inviluppi joystick SLOW, STANDARD e FAST; matrice di perturbazioni) |
| S7, ciclo a freddo (KEY OFF/ON, ACQ B in `qon_cold`) | NOT EXECUTED: passaggio manuale non necessario a questa qualifica. Il codice ACQUIRE è invariato rispetto a `30c894a`, già qualificato su HW; l'avvio a freddo del BMS l'ha registrato Fable stamattina (358/358). La prima STAND dopo un avvio a freddo non è misurata: c'è solo quella dopo il reset del flash |
| Joystick dalla dashboard (D1, passo 4) | NOT EXECUTED: richiede di tenere premuto il comando, azione che gli strumenti del browser non hanno. Non sono stati usati eventi sintetici. Il joystick in rotazione via USB è nella catena |
| S9, SLOW sterzato (`kSteer`) | NOT EXECUTED (facoltativo): resta certificato solo offline |
| Cammino a terra | Fuori dalla procedura. STANDARD e FAST restano `hardware_qualified=false`, costante della tabella dal 09/11 e invariata; la dashboard la mostra come «NON QUALIFICATA SU HARDWARE» |

## 8. Stato finale

**Ultima lettura USB, 06:51:58Z:**
- READY/IDLE, PRONE_REST, nessun fault;
- coppie OFF, SAFE_OFF 8191;
- `missed_releases` 0 e overrun 0 su 130 230 release dal boot;
- modalità selezionata SLOW.

**Chiusura:**
- `@BMS STREAM OFF` inviato;
- registratori e watcher chiusi, `session_proc.py check --expect-none` = PASS;
- scheda della dashboard chiusa.

**SAFE_TO_POWER_OFF: YES.** Unica scrittura: il flash APP-only autorizzato. Nessuna modifica a NVS, EEPROM, partizioni, bootloader, limiti, calibrazioni o leggi di movimento. Nessun merge e nessun push.

## 9. File

- Prima del flash: `pre_usb/`, `pre/`.
- Flash: `flashlog/`.
- Boot: `usb_boot/`, `boot_reads.out`.
- Catena: `*_0623*`, `chain.log`.
- Dashboard: `dash1/`, `dash1_edge*_utc.txt`, `dash2/`.
- Analisi: `analysis/`:
  - `CAND_VS_REF_1811b.md` contro `30c894a`, `CAND_VS_REF_1811.md` contro `85bad0e`;
  - `AB_TABLES.md`, `ACQ_CHECK.md`, `d1_counters.txt`;
  - `HEAP_CHECK_0623.txt`;
  - `SUITE_SUMMARY_0623.txt` e `SUITE_SUMMARY_REF_1148_30c894a.txt`;
  - `SECTIONS_WINDOW_MAXIMA_vs_1148.txt`.
- Sigillo: `SHA256SUMS.txt`.
