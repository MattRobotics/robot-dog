# MATDOG 1.0.2-dev.6 — Addendum: tre velocità operative, frenata STOP, stabilizzazione, motion budget, QEMU ESP32-S3

**Data:** 9 novembre 2026 · **Branch:** `feat/gait-redesign-20261108` → `origin/feat/motion-perf-next-20261107` (PR #67, OPEN, DRAFT)
**Candidata finale:** commit `5159c7f` · build ID `5159c7f52e83` · `SOURCE_STATE=CLEAN` · `ROBOT_POWERED`, OTA ingest OFF
**APP:** 1 734 432 B · SHA256 `bce9a022f9ea825c487abe9c3d3d7165a0ea9c999da0c29f87b057e2e56d8c0a`
**Riferimento storico (non sovrascritto):** APP `3803960`, SHA256 `0593591938c686018141f5949713b67ac5b60bdf3ce942cd01e4134b0b89e88f`, mai flashata.
**Installato sul robot:** `1.0.2-dev.4` (`4358f13`, APP `3e7ee77d…`): baseline hardware, non toccata.

Nessun flash, reset, accesso hardware, comando servo, scrittura NVS/EEPROM, calibrazione, merge o modifica dei limiti meccanici. Il robot è rimasto spento.

Classi: **VH** verificato su hardware (dati di sessioni precedenti) · **VO** verificato offline · **INV** implementato, non validato su hardware · **STORICO** · **UNKNOWN**.

---

## 1. Le tre modalità WALK

| | **SLOW** (default al boot) | **STANDARD** | **FAST** |
|---|---|---|---|
| Falcata massima | 100 mm | 75 mm | 75 mm |
| Periodo | 5,0 s | 3,0 s | 2,5 s |
| **Velocità nominale (falcata massima)** | **20,0 mm/s** | **25,0 mm/s** (+25 % su SLOW) | **30,0 mm/s** (+20 % su STANDARD, +50 % su SLOW) |
| Altre falcate | 50 mm 10 · 75 mm 15 mm/s | 50 mm 16,7 mm/s | 50 mm 20 mm/s |
| Quota di camminata | 120 mm | 120 mm | 110 mm |
| Duty | 0,80 | 0,825 | 0,825 |
| Swing (rampa acc./dec.) | 1/3 simmetrica | 0,35 / 0,45 | 0,35 / 0,45 |
| Registro servo Speed/ACC | **{1000, 50}** | **{1500, 140}** | **{1900, 190}** |
| Frenata assunta da StopPlan | 7,67 rad/s² (ACC 50) | 12,89 rad/s² (84) | 12,89 rad/s² (84) |
| Qualifica hardware | VH (dev.4, legge e drive) | **INV** | **INV** |

- **SLOW** è la legge WALK STANDARD v2 della dev.4, bit per bit, con la correzione T2/STOP della dev.6. Test e digest sono invariati.
- **STANDARD e FAST** sono la stessa legge con un'altra scala dei tempi, più due modifiche geometriche:
  - duty 0,825: la finestra a quattro appoggi dura il 50 % in più, quindi le accelerazioni di sway sulle gambe caricate restano vicine a SLOW;
  - swing asimmetrico (`Gait.h swingRampEnd`): touchdown più lento proprio dove l'UPPER anteriore incontra il limite del box `kHeight`.
- **FAST** cammina inoltre 10 mm più in basso: il margine UPPER passa da 84 a 109 tick a 75 mm.
- Nessuna percentuale di «velocità massima» è stata presunta. I limiti sono ricavati dalla cinematica, dalla prova STOP e dai dati del banco ST3215 (§2–§3).

## 2. Studio velocità e accelerazione (VO + dati di banco VH)

### 2.1 Cosa dice il servo (dati grezzi del banco QC V6.1, 22 ST3215-C018, 500 Hz, 11,1 V, a vuoto)

Analisi: [`evidence/st3215_bench_accel.py`](evidence/st3215_bench_accel.py) → [`st3215_bench_accel.txt`](evidence/st3215_bench_accel.txt).

| Grandezza | Valore | Classe |
|---|---|---|
| Limite del registro Speed | 0 = massima; valori fino a 3400 nella documentazione | doc |
| Velocità a vuoto misurata | 2 829 step/s = **4,34 rad/s** (16 unità; due unità con limite coppia 500 a 1 495) | VH (banco) |
| ACC 50 comandato | accelerazione realizzata 4 918 step/s² mediana, p10 4 795: **il profilo è seguito** | VH (banco) |
| Frenata all'arrivo da velocità massima | 9 231 step/s² mediana, **p10 8 397** | VH (banco) |
| Frenata nei movimenti brevi con ACC 50 | 4 839 mediana: segue l'ACC | VH (banco) |
| ACC sopra 50 | **mai misurato** | UNKNOWN |
| Velocità sotto carico | non misurata; modellata 0,6–0,95 × a vuoto | UNKNOWN |

### 2.2 Cosa chiede la legge

Picchi dalle derivate esatte della legge. Strumenti: `walk_support_check --mode`, `walk_tilt_envelope_check --mode`; evidenza in [`support_final.txt`](evidence/support_final.txt) e `tilt_m*_s*.txt`.

| A falcata massima | SLOW 100 | STANDARD 75 | FAST 75 |
|---|---|---|---|
| Velocità giunto, picco (gamba in swing, scarica) | 1,275 rad/s (83 % del registro, 29 % della velocità a vuoto) | 1,826 (79 %, 42 %) | 2,312 (79 %, 53 %) |
| Accelerazione giunto, picco (swing) | 5,81 rad/s² (75 % del registro) | 16,67 (78 %) | 23,22 (80 %) |
| Velocità / accelerazione giunti in appoggio (caricati) | 0,61 / 5,81 | 0,68 / 6,90 | 0,85 / 10,53 |
| Decelerazione dei giunti caricati nell'arresto STOP | 7,67 | 12,87 | 12,88 rad/s² |
| Decelerazione del corpo nello STOP (modello) | 1,10 m/s² | 1,81 | 1,70 |
| Margine nominale al box (HIP/UPPER/LOWER, tick) | 9,3 / 31,7 / 93 | 9,3 / 84,3 / 142 | 8,7 / 108,8 / 224 |

**Regola dei registri:** picco della legge / 0,8, arrotondato a 50 step/s per la velocità e a 10 per l'ACC. Nessun valore è scelto per comodità. FAST {1900,190} usa il 67 % della velocità a vuoto misurata. ACC 190 è sopra la soglia misurata (50) e sotto il massimo del registro (254).

### 2.3 Variare il periodo a 100 mm (richiesto dal mandato)

A falcata 100 mm, velocità e accelerazione dei giunti scalano come 1/T e 1/T². Il vincolo che decide non è il servo ma la **prova STOP**: l'arresto dell'UPPER anteriore in swing percorre circa 0,75·v²/a. A 2 rad/s con la frenata misurata sono circa 0,25 rad, e vicino al limite `kHeight` non c'è quello spazio.

- **100 mm, 4,0 s (25 mm/s):** la prova STOP passa a inclinazione zero. La ricerca dell'inviluppo IMU dà però **pitch-down −2,5 mrad** e roll ±5 mrad: autorità praticamente nulla.
- **100 mm, 3,33 s (30 mm/s) a 110 mm:** stesso esito, pitch-down −3,0 mrad.
- **100 mm, 3,5 s o meno a 120 mm:** la prova STOP fallisce anche a inclinazione zero.

Da qui la scelta: STANDARD e FAST con falcata massima **75 mm** e periodo più corto. A pari velocità l'inviluppo IMU resta ampio (§5).

### 2.4 Limite realistico di STANDARD e FAST (VO)

- **FAST a 2,5 s e 75 mm è il periodo più corto per cui la prova STOP passa** con la frenata misurata. 2,25 s fallisce a ogni quota provata (100–110 mm) e con ogni sway.
- **Vincolo esatto:** decelerazione assunta nella prova = 8 400 step/s² (frenata misurata a vuoto); UPPER anteriore a fine swing vicino al limite di `kHeight`.
- **Se l'hardware qualificasse una frenata a ACC 150** (23 rad/s²), offline passerebbero:
  - 100 mm a 2,75 s (36 mm/s);
  - 75 mm a 2,0 s (37,5 mm/s).
- Esplorazione completa: [`evidence/mode_sweeps.txt`](evidence/mode_sweeps.txt), su falcata, periodo, duty, sway, rampe, quota e frenata.

## 3. STOP: accelerazione comandata e frenata reale

- **Distinzione introdotta (VO):** `StopPlan` usa `stopAccelerationRadS2(drive) = min(ACC del registro, kStopBrakeAcc = 84) · 100 step/s²`, sia in `beginStop` sia nella prova di `compensationSafe`.
  - Il registro ACC 140/190 serve all'inseguimento della legge, non viene preso come frenata garantita.
  - **SLOW (50) e STAND (13) restano identici.**
  - Un ACC più alto non concede autorità IMU in più: oltre 84 conta solo la frenata misurata.
- **Prova matematica e capacità fisica:**
  - la prova certifica che il *riferimento* di arresto resta nel box certificato;
  - fisicamente, a vuoto, il servo frena ad almeno 8 400 step/s² da velocità massima (banco). **La frenata sotto carico non è misurata.**
  - Nell'arresto, i giunti caricati di STANDARD/FAST decelerano fino a 12,9 rad/s², contro 7,67 di SLOW. Il corpo decelera fino a 1,8 m/s² (SLOW 1,1).
- **Ribaltamento (VO, modello):**
  - Il criterio che conta è il **capture point**: COM + v·√(z/g), con le zampe in appoggio congelate allo STOP. Margine minimo: SLOW 1,41 mm, STANDARD 1,45 mm, FAST 1,67 mm, cioè nessuna modalità peggiore di SLOW.
  - Lo ZMP transitorio dell'arresto esce dal poligono in tutte le modalità, SLOW compreso (−3,4 mm). Con un arresto di 40–60 ms indica un possibile sollevamento di bordo, non una caduta. SLOW ha frenato a terra sulla dev.4 senza ribaltarsi.
- **Arresto con servo reale simulato (VO, modello di risposta §8):**
  - 48/48 STOP con ripresa per modalità, a 11,1 e 9,6 V;
  - scostamento finale dall'arresto pianificato ≤ 2 tick;
  - verifica di quiete ≤ 2,12 s.
- **STOP, SAFE_OFF e perdita di coppia** restano distinti e invariati:
  - STOP = arresto pianificato con coppia mantenuta;
  - fault = SAFE_OFF, coppia OFF verificata 13/13, latch;
  - dopo la perdita di coppia nessuna ipotesi di frenata (QEMU scenario 7: fault latched, STAND rifiutato `FAULT_STOP_REQUIRED` prima e dopo lo STOP).
- **Limite di validazione:** la frenata fisica di STANDARD e FAST e l'inseguimento sopra ACC 50 si qualificano solo sul robot (§16).

## 4. Il significato degli angoli (chiarimento richiesto)

| Grandezza | Cos'è nel Controller | Misurata? |
|---|---|---|
| Orientamento reale del corpo | non osservato direttamente | no |
| Stima IMU | quaternione BNO085 → vettore «up» in base | sì (IMU) |
| Inclinazione del terreno | non stimata come tale: il fit d'appoggio (encoder FK delle zampe in appoggio etichettate e IMU) dà l'inclinazione della gravità nel frame d'appoggio | indiretta |
| Correzione richiesta (`imu_correction.wanted_mrad`) | roll/pitch del corpo nel frame d'appoggio per livellare | calcolata |
| Correzione applicata (`applied_mrad`) | richiesta limitata dall'inviluppo e dallo slew (40 mrad/s) | calcolata |
| Inclinazione dell'asse del piede | modello di contatto: striscia nominale o raccordo ≤ 12° | modello |
| Margine delle box | distanza giunto → box certificata (tick) | calcolata |
| Margine StopPlan | spazio nel box per l'arresto dal secante RAW | calcolata |

- **Ordini di grandezza (VH):**
  - in STAND la legge è proporzionale 0,5 con 5 mrad di zona morta, limitata a ±20 mrad;
  - a terra, in WALK100 sulla dev.4, la correzione richiesta era −15…−18 mrad, sul bordo dell'inviluppo in 102/103 campioni;
  - in HOLD statico sulla dev.17, −24…−29 mrad.
- Non ci sono correzioni STAND di decine di gradi.

## 5. Inviluppo IMU per modalità e strategia di stabilizzazione

| Modalità | Pitch-down | Pitch-up | Roll | Rispetto alla richiesta a terra (−15…−18 mrad) |
|---|---|---|---|---|
| SLOW 100 mm | −12,8 mrad | +36,7 | ±19 | tagliata di 3–5 mrad (come oggi) |
| SLOW 50/75 mm | −81,8 | +66,7 | ±19 | coperta |
| **STANDARD 50/75 mm** | **−36,8** | +63,5 | ±19 | **coperta, con margine** |
| **FAST 50/75 mm** | **−14,9** | +35,1 | ±11 | quasi coperta |

Classe: VO. Gli inviluppi derivano da una ricerca su 16 raggi con prova STOP T2 su 8 fasi e fattore 0,85. Verifica su vertici, punti medi e quarti, 0 fallimenti in tutte le modalità e falcate.

**Strategia adottata:** l'autorità di stabilizzazione cresce con il disegno delle modalità, senza allargare i poligoni e senza una seconda legge di controllo.

- **STANDARD**, la modalità normale prevista dopo la qualifica, è più veloce di SLOW100 e ha **2,9 volte la sua autorità pitch-down**.
- **SLOW** resta invariata, come chiesto.
- Ho valutato e scartato una riduzione automatica della falcata in SLOW (100→75 mm quando la richiesta esce dall'inviluppo), perché cambierebbe il comportamento validato su hardware. Non riuso la legge STAND in WALK: in STAND l'autorità è ±20 mrad, già dentro gli inviluppi.

**FAST è più conservativa, con un motivo fisico:**

- con il modello di risposta del servo (§8), FAST insegue fino a circa 140 tick in ritardo, perché la capacità di accelerazione sopra ACC 50 è ignota (campionata 6 000–15 000 step/s²);
- il fit d'appoggio sulle zampe in ritardo chiede fino a +50 mrad e la correzione satura sul bordo;
- con il poligono pieno la prova STOP rifiutava in una fase stretta (c ≈ 0,63) in 5–6 seed su 6, anche con l'IMU a −10 mrad;
- con il poligono × 0,6: 0 fault (WALK, 8 STOP + ripresa, IMU allo 0,9 del vertice, 11,1 e 9,6 V).

**Nessun accumulo:** la correzione è ristimata a ogni release dalla geometria, senza integrale. Una richiesta fuori inviluppo viene proiettata e non cresce: QEMU scenario 6 a −30 mrad in SLOW100 e rig con richiesta saturata, 0 fault.

**Partenza in pendenza (VO):** HOLD con roll/pitch fino a ±30 mrad, poi WALK nelle 3 modalità, WALK STOP e STAND: 75/75 casi senza fault.

Conservati: H1-RCA (unwind nel modello del raccordo), HOLD, WALK_SETTLE, HEIGHT, PRONE (suite Motion PASS).

## 6. Margini COM e supporto (VO, terreno rigido, COM URDF, inseguimento ideale)

| A falcata massima | SLOW | STANDARD | FAST |
|---|---|---|---|
| Supporto statico minimo | 2,12 mm | 2,13 | 2,24 |
| ZMP in camminata (cart-table) | 2,10 | 2,02 | 2,34 |
| Capture point allo STOP | 1,41 | 1,45 | 1,67 |

- Il margine statico della legge joystick (+0,39 mm, sterzata) riguarda solo SLOW: il joystick cammina sempre SLOW.
- La stabilità dinamica non è dichiarata sul solo poligono statico: lo ZMP in camminata e il capture point sono riportati sopra.
- **Sensibilità al COM reale:**
  - con il COM spostato in base_link (`--com-error-*`), il margine segue la geometria della legge;
  - **FAST cammina 10 mm più in basso:** a parità di sway, la posizione più bassa del COM migliora capture point e ZMP (+0,26 / +0,32 mm su SLOW). Non è una tolleranza più permissiva;
  - sway X 6 mm porterebbe il capture point a ~3 mm, ma le accelerazioni dei giunti caricati salirebbero a 16 rad/s² (2,8 × SLOW): scartato.

## 7. Motion budget e prima causa dei picchi

### 7.1 Cosa ho misurato (VO, codice Xtensa reale, QEMU `-icount`)

- **Lavoro deterministico per release** (sorgenti di produzione compilati per ESP32-S3, newlib, double software):

  | Scenario | Media | Massimo |
  |---|---|---|
  | WALK100 SLOW | ≈ 0,34 M istruzioni | 1,0 M |
  | STANDARD | 0,35 M | — |
  | FAST | 0,38 M | — |
  | joystick | 0,39 M | 1,13 M |

  Ricavato da CCOUNT × 1,5625 (CCOUNT a 80 MHz nel modello, 8 ns per istruzione con `shift=3`).
- **Operazioni dominanti** (profilo `--profile`, media per release WALK100):
  - 125 sin/cos, 43 trigonometriche inverse (atan2/asin/acos/atan), 62 sqrt, 9 hypot;
  - circa 14 000 chiamate double software, contando anche quelle interne alla libm.
- **Microbenchmark** (CCOUNT per chiamata): somma 16, prodotto 26, divisione 149, sqrt 283, sin/cos 550–593, atan2 1 032, asin/acos ~1 300.
- Pesate, **le trascendentali della libm valgono circa i 2/3 del lavoro deterministico.**
- Il codice FK calcola già una sola volta sin/cos per angolo: non resta una duplicazione eliminabile senza cambiare la matematica.

### 7.2 Confronto con l'hardware

- **VH (dev.4, H1RCA):** release più pesante 29,6 ms, di cui `compensate` 21,7 ms (2 solve, 8 IK).
- A 240 MHz con CPI 1, 1,0–1,13 M istruzioni sono **4–5 ms**. Il lavoro deterministico **non spiega da solo** i 29,6 ms (servirebbe un CPI effettivo di circa 6).
- **Prima causa più probabile dei picchi (ipotesi da misurare):**
  - attese della cache flash: la I-cache è condivisa con il core del Wi-Fi e il codice Motion, libm e libgcc gira dalla flash;
  - preemption sul core del loop: `arduino_events` è sul core 1.
- **Non dichiaro risolto il budget.** Il calcolo è dentro i 20 ms con ampio margine. I picchi reali vanno attribuiti con la build diagnostica (§7.3) e una misura sul dispositivo.
- **Interventi non fatti, deliberatamente:**
  - IRAM per Motion e libm: modifica globale del link senza necessità verificata;
  - float al posto di double: l'equivalenza RAW non è dimostrata;
  - ridurre i controlli di sicurezza.

### 7.3 CPU contro wall time

- Le statistiche runtime FreeRTOS sono attive (`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`, contatore U32 in µs da esp_timer). **Il contatore di una task cresce solo quando viene sospesa**, quindi non misura la CPU della task Motion dentro la release.
- **Build diagnostica `MATDOG_DIAG_PREEMPTION`** (spenta nella build operativa: nessun campo, nessuna chiamata):
  - misura il tempo delle *altre* task dello stesso core tra inizio e fine release, sommando i loro contatori;
  - la task Motion è in esecuzione in entrambi gli istanti, quindi ogni slice altrui cade dentro la release;
  - CPU Motion ≈ wall − altre task (ISR incluse);
  - output: `@MOTION TIMING` totals (`max_other_tasks_us`, valori della release più pesante e della prima persa).
- **Validata in QEMU sul target:** 0 µs senza disturbo; **0,303** del wall time con una task a priorità 5 sullo stesso core attiva 3 ms ogni 7 ms (atteso ≈ 0,30).
- **CCOUNT non è usato come CPU della task:** è il wall in cicli del core e include la preemption.

## 8. Banco offline: plant del servo e qualità rispetto al robot

- **Modello di risposta ST3215** (`servo_plant.inc`, `run_servo_plant.sh`):
  - profilo trapezoidale verso ogni goal, con Speed/ACC del frame;
  - capacità: velocità 2 829·V/11,1 × unità (0,9–1,05) × carico (0,6–0,95); accelerazione 6 000–15 000 × unità × carico;
  - latenza 0–3 ms, banda morta 1–3 tick, sag sotto carico 0,5–1,5 × il pattern della dev.15;
  - parametri estratti per seed.
- **Risultati** ([`evidence/servo_plant.log`](evidence/servo_plant.log), 6 seed, 11,1 e 9,6 V): **0 fault in tutte le modalità e scenari**, 48/48 STOP con ripresa per modalità.
  - Ritardo d'inseguimento massimo: SLOW 66 tick, STANDARD 101, FAST 137.
  - È la fisica di un servo in profilo di posizione con goal ogni 20 ms: ritardo ≈ v²/(2·a).
  - La parte di FAST dipende dall'accelerazione reale sopra ACC 50, che non è misurata.
- **Qualità rispetto al robot:**
  - il modello non simula il corpo né il contatto: IMU e giunti misurati non sono accoppiati, quindi sovrastima l'incoerenza del fit d'appoggio (motivo conservativo per FAST × 0,6);
  - i valori di `load` non sono trattati come forze al suolo;
  - il ritardo ≈ v²/(2·ACC) in SLOW (~66 tick a 1,275 rad/s, ACC 50) non è confrontato con tracce hardware di inseguimento in camminata. **Primo dato da raccogliere sul robot.**

## 9. QEMU ESP32-S3: fattibilità (addendum QEMU)

| Voce | Esito |
|---|---|
| Ambiente host | Ubuntu 24.04, kernel 7.0.0-42, x86_64 |
| Toolchain | Arduino ESP32 **3.3.11**, ESP-IDF **v5.5.5** (`b774170ff46`), GCC `esp-14.2.0_20260121` |
| Emulatore | Espressif QEMU **esp_develop_9.2.2_20260417** (`qemu-xtensa-softmmu…x86_64-linux-gnu.tar.xz`, SHA256 `0eecb2a3…8e35` = checksum ufficiale), in `~/MATDOG/tools/qemu-esp/`, fuori da worktree e artefatti |
| Test minimo | boot ROM → bootloader → app, UART0, FreeRTOS su 2 core, esp_timer periodico, CCOUNT; QIO non emulato (si usa DIO), PSRAM spenta |
| Firmware MATDOG completo | non avviato: USB-CDC, Wi-Fi, ServoBus e IMU non sono emulati. Non ho alterato la candidata `ROBOT_POWERED` |
| App eseguita | `qemu_motion` (scenari) con i **sorgenti Motion di produzione compilati senza modifiche** (SimpleMotion, Gait, IK/FK, TiltedContactIk, StopPlan, ViaPlan, …; un file wrapper per sorgente), porta servo simulata, clock simulato via `--wrap=esp_timer_get_time`, task da 16 KB sul core 1 |
| **Confronto con l'host** | **9/9 scenari con digest RAW, stati, motivi e fault identici bit per bit** (Xtensa + newlib + double software contro x86-64 + glibc) |
| Scenari | WALK100 SLOW 120 s (oltre il ciclo 19,5 del difetto T2), STANDARD/FAST 90 s, sequenza joystick con cambi di regime e inversione, 8 STOP in swing/appoggio + ripresa, IMU oltre l'inviluppo, fault latch, ritardo e sag |
| Stack e memoria | task Motion: minimo libero **7 288 B su 16 384**; heap invariato (−108 B, harness); oggetto SimpleMotion 14 088 B statico |
| FreeRTOS | statistiche runtime funzionanti; strumento di preemption validato (§7.3) |
| GDB | non necessario: nessuna divergenza da localizzare |
| Limiti | il tempo emulato **non** è tempo ESP32-S3: niente cache, flash, pipeline o contesa del bus; CCOUNT sotto `-icount` conta istruzioni, non cicli; niente periferiche reali |

**Decisione:** integro QEMU nel banco come livello opzionale, perché dà due vantaggi verificati che il rig host non dà:

1. la prova che il codice Xtensa compilato produce esattamente gli stessi riferimenti;
2. il conteggio deterministico delle istruzioni e il profilo delle operazioni sul target.

Una procedura: `scripts/offline_motion/run_qemu_motion.sh [--profile|--diag]`. **Non è un prerequisito** per compilare o flashare.

## 10. Diagnostica causale disponibile

- **Fault** — `@MOTION DIAG`:
  - `exit` + `exit_cause`: motivo, fase, controllo rifiutato (`refused_by`: POINT_SAFE … STOP_PLAN), ciclo, legge, correzione applicata e richiesta;
  - **nuovo:** `walk_mode`, periodo, drive, `stop_brake_acc`;
  - per ogni giunto: RAW comandato e letto, velocità, carico, corrente, tensione, temperatura, stato, coppia, goal, goal speed, ACC, età.
- **Status** — `walk_mode`: selected, active, `servo_read` (min/max di goal speed e ACC letti dai 12 servo, con l'età della lettura). Riusa la lettura di controllo esistente: **nessun traffico UART aggiuntivo**.
- **Capabilities:** la tabella delle tre modalità (periodo, falcata e velocità massima, quota, duty, drive, frenata STOP, qualifica).
- **Timing:** release più pesante e prima release persa con le parti; istogrammi; `MATDOG_DIAG_PREEMPTION` solo nella build diagnostica.
- **Elettrico:** solo ciò che i servo e il BMS già forniscono (tensione, corrente, carico, temperatura). Nulla di inventato.

## 11. Contratto dashboard e API

- **`GAIT.SELECT`** `{gait: SLOW|STANDARD|FAST}`:
  - dashboard: selettore «SLOW | STANDARD | FAST» con invio immediato;
  - USB: `@MOTION WALK MODE <nome>`, oppure `@MOTION JSON`.
- **Comportamento:**
  - selezione volatile (niente NVS né EEPROM), SLOW al boot;
  - non muove gli attuatori, non serve torque OFF, non diventa «comando attivo»;
  - rifiutata con STOP latched;
  - ACK `WALK_MODE_SELECTED`, oppure `WALK_MODE_NEXT_WALK` se è in corso un movimento;
  - nome sconosciuto: `WALK_MODE_UNKNOWN`.
- **Latch al WALK:**
  - la camminata, il suo WALK STOP e la ripresa dopo STOP mantengono la modalità con cui sono partite;
  - la modalità successiva si applica alla prossima partenza sicura: STAND alla quota della modalità, poi HOLD, poi WALK;
  - 100 mm in STANDARD/FAST è rifiutato (`STRIDE_NOT_IN_WALK_MODE`).
- **Compatibilità:**
  - i preset WALK 50/75/100 (vx = falcata / 5 s) restano l'identità del preset; la modalità decide il periodo;
  - **il joystick cammina sempre SLOW**: i suoi inviluppi sterzati sono certificati solo per quella legge (gate joystick: §12).
- **Dashboard:** mostra modalità selezionata e in uso, periodo, velocità, quota, Speed/ACC comandati e letti, frenata STOP, «NON QUALIFICATA SU HARDWARE». La tabella arriva dalle capabilities del Controller (unica fonte), letta una volta al caricamento.

## 12. Regressioni

Tutte VO, eseguite sulla sorgente finale.

| Suite | Esito |
|---|---|
| SimpleMotion (rig di produzione, ASan/UBSan) | PASS: nuovo `walkModes` (per modalità: latch, periodo, drive, quota, 8 STOP in swing/appoggio + ripresa, prossimo WALK, joystick SLOW); continuità C1/C2 del gait su tutte le modalità e falcate; tutte le regressioni precedenti (H1-RCA, PRONE, STOP, WALK100 lungo, joystick, sequenza, …) |
| Prova STOP/box/inviluppo (`walk_tilt_envelope_check --mode`, T2, 8 fasi) | 7/7 combinazioni modalità-falcata PASS, 0 fallimenti su 0,63–1,26 M controlli ciascuna |
| Supporto/ZMP/capture (`walk_support_check --mode`) | 7/7 PASS |
| Modello di risposta servo | 0 fault, 3 modalità × 3 scenari × 2 tensioni × 6 seed |
| Matrice di perturbazioni (dev.6) | 60 celle, 0 fault ([log](evidence/perturbation_matrix.log)) |
| Riproduzione dei difetti storici sulla build finale | R1–R4 assenti ([log](evidence/version_repro_final.log)) |
| Gate joystick (46 scenari + casuali) | PASS, supporto minimo 0,39 mm, invariato ([log](evidence/joystick_envelope.log)) |
| QEMU host contro ESP32-S3 | 9/9 identici |
| Dashboard: contratto (6 116 controlli), DOM (89 comandi), layout, ViewCube, STOP durante il caricamento del Twin (nel browser reale) | PASS |
| Test host (`run_host_tests.sh`) | PASS, incluso il contratto HTTP aggiornato (GAIT_SELECT, tabella delle modalità) |
| Audit statico | **PASS** (370 file) |
| Partenza in pendenza ±30 mrad | 75/75 |

**Contratti cambiati con test aggiornati:**

- capabilities: `GAIT_SELECT`, `walk_modes`;
- `GAIT.SELECT TROT`: `WALK_MODE_UNKNOWN`, prima `COMMAND_NOT_IN_SIMPLE_V1`;
- il test STOP durante il caricamento del Twin ammette la lettura delle capabilities;
- hash `Gait.cpp`/`Gait.h` aggiornati nel gate di convergenza (prassi dei commit precedenti).

## 13. Stack

- **Percorso statico peggiore da `loop()`: 11 264 B su 16 384** (era 11 248 B in 3803960; +16 B in `nextStand`).
  - Fonte: `-fcallgraph-info` sulla build finale ([`evidence/stack_worst_loop_5159c7f.txt`](evidence/stack_worst_loop_5159c7f.txt)).
- **QEMU:** 7 288 B liberi nella task degli scenari.
- **Soglia operativa** invariata: fermarsi se `loop_stack_min_free` < 1 500 B.

## 14. Controllo prima del READY

| # | Controllo | Esito |
|---|---|---|
| 1 | Contratto SLOW/STANDARD/FAST | OK (§11, test) |
| 2 | Profili Speed/ACC coerenti con gli inviluppi | OK (inviluppi ricavati con drive e frenata della modalità) |
| 3 | STOP | OK offline; fisico di STANDARD/FAST da qualificare (§16) |
| 4 | IMU | OK (§5) |
| 5 | Supporto | OK (§6) |
| 6 | Continuità | OK (C1/C2, digest) |
| 7 | Budget computazionale | deterministico OK; picchi hardware da attribuire (§7) |
| 8 | Stack | OK (§13) |
| 9 | Regressioni realistiche | OK (§12) |
| 10 | Identità esatta del binario | OK (sotto) |

## 15. APP finale e flash preparato (non eseguito)

- **Build:** `~/MATDOG/build-output/gait-redesign-20261108/build-5159c7f/`.
  - Manifest: `SOURCE_COMMIT=5159c7f52e83b9531d5fa6ba49e33644ed010a65`, `SOURCE_STATE=CLEAN`, `BUILD_ID=5159c7f52e83`, `FW_VERSION=1.0.2-dev.6`, `ROBOT_POWERED`, `OTA_INGEST_ENABLED=0`.
  - Layout `MATDOG_16M_2x5M_NVS_V1`, tabella partizioni `8f756ecb…46e7`.
  - APP 1 734 432 B (33,1 % dello slot), SHA256 `bce9a022…8c0a`.
- **Script APP-only:** `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6W_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.6-5159c7f.sh`.
  - **NON eseguito**; richiede `MATDOG_DEV6W_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`.
  - Rifiuta in caso di discrepanza di SHA, dimensione, manifest, CLEAN, commit, profilo, partizioni o backup.
- **Rollback:** `rollback-assets.sha256` verificato OK, compresi l'APP dev.4 `4358f13` e il backup full flash del 2026-09-10.
- Lo script di 3803960 resta intatto in `MATDOG_1_0_2_DEV6C_FLASH_PREP_20261109/`.

## 16. Limiti verificabili solo sul robot e test supervisionato minimo

**Non verificabili offline:**

- **Inseguimento reale a ACC 140/190 e Speed 1500/1900:** il banco ha misurato solo ACC ≤ 50.
- **Frenata reale:** dei giunti in swing con ACC > 84 e dei giunti caricati a 12,9 rad/s².
- **Stabilità dinamica reale:** contatto, attrito, compliance, COM reale.
- **Picchi di tempo:** attribuzione a cache, preemption o calcolo.
- **Corrente e temperatura** in STANDARD/FAST.
- **Il ritardo di inseguimento previsto in SLOW** (~66 tick in swing).

**Test minimo, nell'ordine, fermandosi al primo FAULT:**

1. Flash supervisionato, verifica dell'identità (SHA/build_id) e boot in SLOW. Verifica che `walk_mode.servo_read` legga {1000,50} in HOLD.
2. **SLOW**, regressione breve sulla fixture e a terra (WALK100 ≥ 3 min, STOP con ripresa): stesso comportamento della dev.4.
3. **Fixture, corpo sostenuto, zampe libere: STANDARD.**
   - WALK75 per 60 s. Registra `@MOTION DIAG` (goal speed/ACC letti = 1500/140), tracking RAW, corrente e temperatura.
   - STOP in swing: verifica dell'arresto e della ripresa.
4. Fixture: FAST, come al punto 3 (1900/190).
5. A terra: STANDARD WALK75 per 30 s, poi 2 min, con STOP in swing e in appoggio. Solo dopo: FAST.
6. Facoltativo, build diagnostica separata `MATDOG_DIAG_PREEMPTION` (da compilare su richiesta, non è la candidata) per misurare `other_tasks_us` nelle release più pesanti.

**Criteri di arresto:**

- FAULT;
- tracking > 48 tick sui giunti caricati in HOLD;
- temperatura ≥ 60 °C;
- `loop_stack_min_free` < 1 500 B;
- release oltre 40 ms ripetute.

## Verdetto

**READY FOR SUPERVISED FLASH (SLOW).** STANDARD e FAST sono **IMPLEMENTED, NOT HARDWARE QUALIFIED**: vanno qualificate prima sulla fixture e poi a terra, secondo il §16. Nessuna probabilità di successo hardware è stimata senza dati.

Commit di questo addendum:

| Commit | Contenuto |
|---|---|
| `1fb848a` | swing asimmetrico |
| `b987ed3` | modalità e frenata STOP |
| `f2bbc7a` | API e dashboard |
| `0b78ece` | contratto HTTP |
| `fc972c4` | strumenti `--mode`, capture point e ZMP |
| `746b5be` | modello di risposta del servo |
| `6922ecd` | banco QEMU e opzione diagnostica |
| `5159c7f` | identità della candidata |
| *(questo report)* | report |
