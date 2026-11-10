# MATDOG MuJoCo — M5: correlazione hardware/simulazione (rapporto definitivo)

**Natura.** Confronto tra log hardware, letti in sola lettura, e il gemello MuJoCo. Nessun comando al robot. Firmware, sicurezza STOP, calibrazione, NVS, URDF e geometria canonica non sono stati modificati. **Il modello fisico del gemello è invariato:** nessuna correzione di fisica, contatti, attrito o rigidezza.

**Codice.** Branch `sim/mujoco-twin-20261110`:
- baseline del modello `3345231`;
- strumenti M5 in `df9246f`, `70715be` e `fe66610`;
- questo rapporto nel commit di chiusura.

I risultati numerici sono in `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110/M5/`, con `SHA256SUMS` della cartella. Sono fuori da Git.

**Classificazione**
- **VERIFIED HARDWARE:** osservato sul robot, nei log HW o nel rapporto della sessione firmware.
- **VERIFIED OFFLINE:** calcolo o simulazione riproducibile.
- **HYPOTHESIS:** interpretazione dinamica non dimostrata.
- **UNKNOWN:** parametro non misurato.

## 0. Esito

1. **Sessione HW conclusa** (VERIFIED HARDWARE).
   - La candidata `450d2c793dcb` è installata (APP `82122e06…`) e qualificata sulla fixture (Q1–Q8) per i profili provati: SLOW, STANDARD e FAST, cambi di profilo, STOP, joystick in avanti.
   - A terra la verifica è positiva ma fuori protocollo: non è una qualifica G0–G5.
   - 14 STOP d'emergenza verificati su 14 (8 fixture, 6 terra), 0 fault Motion, registro 85 = 220 su 12 servo su 12.
2. **Campione M5** (§2). Riproduce in anello chiuso:
   - tutti gli 8 STOP della fixture;
   - 4 dei 6 STOP a terra: i 2 durante il PRONE non sono riproducibili.

   Mostra il gemello accanto all'HW in 11 episodi: 7 sulla fixture e 4 a terra.
3. **Fixture** (VERIFIED OFFLINE). Il gemello riproduce l'HW:
   - STOP verificati 8 su 8 in 1,90–2,14 s;
   - stati Motion quasi sempre entro la finestra di un campione;
   - errori articolari accoppiati con RMSE 3,8–10,4 tick;
   - duty pari a 1,2 volte l'HW in mediana.
4. **A terra: discrepanza residua, non risolta** (VERIFIED OFFLINE).
   - **Carichi:** il gemello porta 1,9 volte il duty HW in mediana (1,1–2,9).
   - **Anche:** margine da `kHeight` HW +5,3…+7,3 tick, gemello −2,7…−8,7.
   - **STOP:** l'HW ne verifica 4 su 4 negli episodi confrontati; il gemello va in fault nei 3 che raggiunge.
   - **Cause candidate:** HYPOTHESIS e UNKNOWN (§4). Nessuna è esclusa, inclusa la dinamica dei servo sotto carico.
5. **Offset IMU** (§5). La differenza di pitch IMU − encoder si ripete tra due sessioni: +22,9 e +23,6 mrad.
   - L'opzione `imu_mount` migliora le posture comandate e il pitch medio a terra; carichi e STOP non cambiano.
   - **Resta un'opzione e non il default:** l'angolo di montaggio è UNKNOWN finché non c'è una misura esterna.

## 1. Sessione hardware: stato e provenienza

Evidenze: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_HW_20261110/`. Versione riconciliata: rapporto di chiusura `REPORT_HW_SESSION.md` (SHA256 `6b55f7d5…`, 22:28 locali) e `SHA256SUMS` della cartella `35a98771…` (22:29). **Nessuna evidenza HW è stata modificata.**

| Voce | Stato | Classe |
|---|---|---|
| Sessione | Conclusa dall'operatore il 10/11; robot spento (KEY OFF) e sulla fixture | VERIFIED HARDWARE (rapporto HW) |
| Immagine | `450d2c793dcb`, commit `450d2c793dcb76d0…` sul branch locale `proposal/acc85-runtime`. Preflight H0: `1.0.2-dev.6`, build e OTA `450d2c793dcb`. 511 `/status` con la stessa identità e uptime monotono. `serial.log`: un solo boot, solo questo `build_id`, 0 panic | VERIFIED HARDWARE (log) |
| APP | `APPLICATION_SHA256 = 82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256`, 1 751 040 B. Verify riuscito, `APPLICATION_ONLY_FLASH = PASS`, solo `app0`. Gate offline post-sessione 20/20 PASS | VERIFIED HARDWARE (log) |
| Configurazione Motion | `MOTION_ACC_CEILING verified=220 target=220`; registro 85 = 220 su 12 servo su 12. Profili letti dai servo in camminata: SLOW 1000/50, STANDARD 1800/180, FAST 1950/220 (speed/ACC) | VERIFIED HARDWARE (log) |
| Qualifica | Fixture Q1–Q8 qualificata. A terra verifica positiva fuori protocollo, non G0–G5: batteria 10,6–10,2 V, anticaduta non confermata per iscritto, sequenza libera. Alle 19:57:21Z il criterio termico del runbook è scattato (servo 13 a 80 °C, lettura spuria filtrata, nessun fault) e non è stato applicato | VERIFIED HARDWARE (rapporto HW) |
| Esiti | 14 STOP verificati su 14; 0 fault. Nei log, nessuna istantanea in FAULT: 3 032 campioni distinti tra `http.jsonl` e tracce | VERIFIED HARDWARE (log) |
| Alimentazione | BMS da 10,9 a 10,2 V, SOC da 45 a 41,3 %; BATTERY_WARNING dopo PRONE_REST | VERIFIED HARDWARE (log) |
| Fixture e terra | Il rapporto HW pone il limite della fixture alle 19:43Z. Nei dati i piedi si caricano tra 19:45:51 e 19:45:57Z: l'IMU oscilla, la prua passa da −176° a −126°, il carico UPPER/LOWER da 19 a 52–64 ‰. Le analisi a terra di M5 partono da 19:45:57Z | VERIFIED OFFLINE |
| Integrità | I 6 log grezzi usati coincidono con il manifesto delle 21:02Z e con lo `SHA256SUMS` di chiusura. Sono `usb/http.jsonl`, `usb/serial.log`, `terra/dashboard_trace_47.json` e `_48.json`, `flash/flash.log`, `h0/preflight_h0.json`; SHA256 in `PROVENANCE.md`. Dopo le 21:02Z sono cambiati solo i riepiloghi derivati, rigenerati dalla sessione firmware, e i loro valori coincidono con quelli usati qui | VERIFIED OFFLINE |
| Escluso | `rehearsal/run/`: prova generale contro un dispositivo simulato, con la stessa identità di build. Nessun suo campione entra in M5 | VERIFIED OFFLINE |
| **`EVIDENCE.sha256`** | **Mancante.** Non generato da qui: lo chiude il responsabile firmware | – |
| Codice nel gemello | Motion compilato da `ae70e03`. `450d2c7` aggiunge solo il JS della dashboard e commenti: stessi sorgenti Motion | VERIFIED OFFLINE |

**Frequenza reale per giunto** (VERIFIED OFFLINE).
- **Controller:** 2 servo per release da 20 ms, quindi ogni giunto ogni 120 ms (8,33 Hz). Età delle letture 0–163 ms (p50 57).
- **Esportato:**
  - `http.jsonl`: 1,04 Hz per giunto, per tutta la sessione;
  - tracce 47 e 48: 2,86 Hz, ognuna su 84 s a terra (19:47:35–19:48:58Z e 19:57:23–19:58:47Z), con buffer degli ultimi 48 comandi.

**Misure a 50 Hz per giunto non esistono.** Ogni coppia HW/SIM è allineata all'istante della singola lettura (`now_ms − age_ms`, l'IMU al suo `sampled_at_ms`); in camminata, su `walk_cycles`.

## 2. Riconciliazione: totale HW e campione M5

Gli STOP HW sono tutti verificati: `stop.motion_verified`, stato `STOPPED_SUPPORT_UNVERIFIED`, coppia ON. Gli intervalli sono tra l'ultimo campione prima della verifica e il primo dopo.

| # | UTC | Fase | Contesto | HW (VERIFIED HARDWARE) | M5 | Gemello (VERIFIED OFFLINE) |
|---|---|---|---|---|---|---|
| 1 | 19:24:53 | fixture | Q3, STANDARD | 1,2–2,2 s | F2 | verificato a +1,92 s |
| 2 | 19:28:17 | fixture | Q3b, STANDARD | 1,7–2,7 s | F3 | +2,10 s |
| 3 | 19:29:09 | fixture | Q4, FAST | 2,1–3,1 s | F4 | +2,06 s |
| 4 | 19:29:40 | fixture | Q4, FAST | 1,1–2,1 s | F4 | +2,14 s |
| 5 | 19:35:52 | fixture | Q6, durante SLOW→FAST | 1,6–2,6 s | F6a | +1,98 s |
| 6 | 19:36:46 | fixture | Q6, FAST | 1,5–2,5 s | F6a | +1,92 s |
| 7 | 19:37:10 | fixture | Q6, FAST | 1,8–2,8 s | F6a | +2,02 s |
| 8 | 19:37:38 | fixture | Q6, durante FAST→SLOW | 1,0–2,0 s | F6b | +1,90 s |
| 9 | 19:56:32 | terra | SLOW, 150 mm | 1,9–2,9 s | G2 | **FAULT** `STOP_MEASURED_OUTSIDE_BOX` a +0,02 s |
| 10 | 19:57:09 | terra | STANDARD, 150 mm | 1,6–2,6 s | G3 | **FAULT** `STOP_NOT_SETTLED` a +3,06 s |
| 11 | 19:57:24 | terra | STANDARD dopo la ripresa | 1,9–2,2 s (traccia 48) | G3 | non raggiunto: gemello in SAFE_OFF dal #10 |
| 12 | 19:57:39 | terra | discesa PRONE | 2,0–2,4 s (traccia 48) | – | non riproducibile: il cosim non esegue PRONE |
| 13 | 19:58:06 | terra | discesa PRONE | 2,2–2,5 s (traccia 48; a 1 Hz non osservato) | – | non riproducibile |
| 14 | 19:58:22 | terra | FAST, 120 mm | 1,9–2,2 s (traccia 48) | G4 | **FAULT** `STOP_NOT_SETTLED` a +3,18 s |

**Sulla fixture gli STOP sono 8, non 10.** Il «10» comparso in chat è corretto dal §7 del rapporto HW, e il conteggio torna nei log. **M5 copre quindi tutti gli STOP della fixture e 4 dei 6 a terra.**

**Altre prove HW e campione M5:**
- **Confrontate:**
  - fixture: Q2 (F1), Q3 e Q3b (F2, F3), Q4 (F4), Q5 (F5), Q6 (F6a, F6b);
  - terra: WALK SLOW e WALK STOP (G1, traccia 47), SLOW con STOP (G2), STANDARD con due STOP (G3), FAST con STOP (G4);
  - le HOLD a terra a 105–150 mm, solo per l'IMU (§5).
- **Non confrontate, con il motivo:**
  - Q1 (STAND da PRONE) e Q8 (SAFE_OFF via PRONE): il cosim parte da STANDING;
  - una WALK SLOW di ripresa in Q6 (19:37:52Z): stesso scenario di F1;
  - joystick di Q7 e le tre camminate FAST a joystick a terra: a 1 Hz i rinnovi distano circa 1 s, come il timeout del joystick (`kJoystickTimeoutUs`);
  - SLOW sterzata: laterale e rotazione, mentre il DRIVE del cosim è solo frontale;
  - SLOW all'indietro: i primi comandi sono usciti dal buffer della traccia;
  - i tre PRONE.

## 3. Confronto HW/SIM (VERIFIED OFFLINE)

Ogni comando HW è riprodotto in anello chiuso, con SimpleMotion di produzione e il gemello, alla stessa quota:
- i comandi dati in camminata scattano allo stesso `walk_cycles`;
- gli altri con lo stesso ritardo.

### 3.1 Fixture: corpo saldato all'assetto IMU, zampe libere, 1 Hz

| Grandezza | Risultato |
|---|---|
| STOP | 8 su 8 verificati in entrambi. Gemello 1,90–2,14 s: 6 dentro l'intervallo HW, 2 a 10–20 ms dal bordo |
| WALK → WALKING | 10 su 12 dentro l'intervallo, 2 a 10 ms |
| WALK STOP → STANDING | 2 su 4 dentro, 1 a 40 ms, 1 in ritardo di 0,28 s (F4) |
| Commutazione della legge nei cambi di profilo (F5: attivo = destinazione) | HW 1,5–2,5 / 0,5–1,5 / 9,5–10,5 / 2,5–3,5 s, gemello 1,86 / 1,14 / 9,80 / 3,06 s. Un cambio che ne sostituisce un altro: HW ≤ 1,5 s, gemello 0,02 s (SELECT dedotto a ±0,5 s). Il rapporto HW misura anche la transizione completa fino alla velocità del nuovo profilo (4–13 s): qui non è confrontata |
| Errore articolare in camminata, RMSE accoppiato | anca 3,8–4,2, UPPER 4,3–10,4, LOWER 4,2–8,5 tick, su errori p95 HW di 55–145 tick |
| Duty p95 in camminata, gemello/HW | anca 0,84–1,33, UPPER 1,03–1,43, LOWER 0,81–1,82; mediana 1,2 |

Scarti minori a vuoto:
- **Swing ad alta velocità.** L'HW resta più indietro del gemello: errore UPPER p95 in STANDARD 104/124/145 tick contro 71/110/122; in FAST 89/124/110/67 contro 62/100/99/65. Il duty UPPER del gemello è più alto del 3–43 %. HYPOTHESIS: il termine di velocità del servo è identificato solo fino a 600 step/s.
- **Anche ferme in HOLD.**
  - Il gemello ha le 4 anche a ±5 tick con 43 ‰.
  - L'HW ha le anche sinistre a 0–2 tick con 0–20 ‰ e le destre a +1…+6 tick con 10–56 ‰.

  HYPOTHESIS: attrito statico del riduttore, dipendente dalla storia, non modellato; l'attrito di distacco sotto carico è UNKNOWN. In camminata il margine minimo delle anche è HW 10,3–12,3 tick, gemello 4,3–6,3.

### 3.2 A terra

1. **HOLD prima della camminata: postura comandata.**
   - L'HW applica −23,5…−24,4 mrad di correzione di pitch a 120 mm e −5,1 a 150 mm; il gemello circa 0.
   - Scarto dei frame UPPER/LOWER p95: 21/34 tick (G1), 8/15 (G2), 26/40 (G4).
   - Spiegato dall'offset IMU (§5).
2. **Camminata: carichi in appoggio.** Valori baseline, HW → gemello:

   | | G1 SLOW 120 | G2 SLOW 150 | G3 STANDARD 150 (n 5) | G4 FAST 120 |
   |---|---|---|---|---|
   | duty p95 anca/UPPER/LOWER (‰) | 72/124/200 → 125/355/400 | 64/149/204 → 123/330/368 | 73/198/209 → 120/412/342 | 94/335/238 → 103/515/516 |
   | RMSE accoppiato dell'errore anca/UPPER/LOWER (tick) | 8,1/18,6/19,7 | 8,3/16,7/20,7 | 7,3/17,7/19,1 | 6,5/17,1/20,2 |
   | duty p95 / errore p95 dell'anca (‰/tick) | 9,0 → 8,5 | 9,1 → 8,2 | 8,9 → 8,5 | 8,5 → 8,6 |
   | margine minimo dell'anca da `kHeight` (tick) | +5,3 → −8,7 | +5,3 → −4,7 | +7,3 → −5,7 | +7,3 → −2,7 |
   | campioni con anca fuori box | 0 → 36 % | 0 → 38 % | 0 → 60 % | 0 → 10 % |
   | roll/pitch picco-picco (mrad) | 38/31 → 36/27 | 25/19 → 24/28 | 21/13 → 12/10 | 20/23 → 23/16 |

   - **Duty:** il rapporto gemello/HW vale 1,1–2,9, mediana 1,9, contro 1,2 sulla fixture.
   - **Errori:** l'RMSE accoppiato è 2–5 volte quello della fixture.
   - **Dondolio del corpo:** comparabile.
3. **STOP:** righe 9–14 della tabella del §2.
4. **HOLD statiche:** non si confrontano giunto per giunto. Il duty LOWER del gemello vale 0,9–2,4 volte l'HW secondo la storia, cioè secondo le forze interne lasciate dalla salita.

## 4. Discrepanza residua a terra e cause candidate

**Discrepanza** (VERIFIED OFFLINE). È quantificata nel §3.2: carichi, anche fuori box, STOP non verificati nel gemello. Non si risolve con i dati attuali.

| Candidata | Evidenza | Classe |
|---|---|---|
| Forze interne fra piedi rigidi bloccati dall'attrito (attrito 0,8, IPOTESI del modello); il piede reale è in gomma eccentrica | Lo stesso meccanismo produce `STOP_NOT_SETTLED` nella RCA del gemello; a terra l'RMSE cresce di 2–5 volte rispetto alla fixture | HYPOTHESIS |
| Dinamica dei servo sotto carico: attrito dipendente dal carico, anello di velocità interno, gioco del riduttore | Il modello è verificato solo a vuoto (fixture) e a riposo sotto carico (guadagno P 8,4 ‰/tick misurato). Il rapporto duty/errore p95 dell'anca coincide, ma è solo un indicatore. **Non esclusa** | UNKNOWN |
| Attrito statico delle anche | Asimmetria delle anche HW ferme sulla fixture (§3.1) | HYPOTHESIS |
| Cedevolezza della struttura (staffe, link) | Non misurata | UNKNOWN |
| Massa e distribuzione | Massa totale 2,928 kg nel modello contro 2,807 kg pesati senza la Jetson (175 g): non spiega un fattore 2. COM e inerzie vengono dal CAD, non misurate | VERIFIED OFFLINE / UNKNOWN |
| Tensione | HW 10,2–10,9 V, modello 11,1 V: 2–9 %, e nel verso opposto (all'HW servirebbe più duty) | VERIFIED OFFLINE |

**Nessuna correzione introdotta.** Un parametro di contatto, attrito, cedevolezza o rigidezza tarato su questi episodi sarebbe arbitrario e servirebbe solo a trasformare i fault simulati in PASS.

## 5. Offset IMU: opzione `imu_mount`, non default

- **Stima** (VERIFIED OFFLINE). IMU meno piano dei quattro piedi ricavato dagli encoder, nelle HOLD a terra:

  | | dev.4, 08/11 | 450d2c7, 10/11 |
  |---|---|---|
  | Istantanee distinte | 184 | 693 |
  | Pitch, mediana (p5–p95) | +22,9 (21,8–37,0) mrad | **+23,6 (19,1–24,0)** mrad |
  | Roll, mediana (p5–p95) | −0,5 (−6,9…+1,2) | −6,9 (−7,3…−5,1) |
  | Prua (yaw dell'IMU) | +140…+178° | −142…−123° |

- **Interpretazione** (HYPOTHESIS). Con prue diverse di 54–89°, una pendenza del pavimento darebbe lo stesso pitch solo con rolli opposti di 12–23 mrad, che non si osservano. Il dato è compatibile con un offset fisso nel corpo, cioè montaggio o flessione solidale.
- **Cosa resta UNKNOWN.** La prua viene dall'IMU stessa. Senza una misura esterna di calibrazione l'angolo di montaggio resta UNKNOWN. Il roll non si ripete tra le due sessioni.
- **Variante.** `imu_mount` ruota il quaternione passato a SimpleMotion di roll −0,5 e pitch +22,9 mrad. I valori sono stimati solo sui dati dev.4 e non ritoccati. **Non è il default.**
- **Prima e dopo sui dati nuovi** (VERIFIED OFFLINE). Valori baseline → `imu_mount`, HW tra parentesi:

  | | G1 | G2 | G3 | G4 |
  |---|---|---|---|---|
  | frame UPPER/LOWER, \|HW−SIM\| p95 in HOLD (tick) | 21/34 → 4/6 | 8/15 → 3/5 | – | 26/40 → 14/23 |
  | … in WALKING | 17/25 → 8/10 | 16/26 → 7/11 | 21/29 → 4/7 | 30/36 → 8/13 |
  | pitch IMU medio in WALKING (mrad) | 1,7 → 11,7 (7,5) | 1,0 → 10,7 (7,9) | 6,0 → 12,7 (13,8) | 2,0 → 2,8 (4,6) |
  | WALK → WALKING (s) | 0,64 → 1,28 (1,11–1,46) | 2,54 → 2,68 (1,89–2,93) | 2,80 → 3,00 (2,39–3,39) | 1,84 → 2,38 (2,49–2,87) |
  | WALK STOP → STANDING (s) | 7,48 → 7,70 (8,18–8,53) | – | – | – |
  | duty p95 UPPER/LOWER (‰) | 355/400 → 362/422 | 330/368 → 327/373 | 412/342 → 403/352 | 515/516 → 562/526 |
  | margine minimo dell'anca (tick) | −8,7 → −8,7 | −4,7 → −4,7 | −5,7 → −6,7 | −2,7 → −3,7 |
  | STOP | – | FAULT → FAULT | FAULT → FAULT | FAULT → FAULT |

  In G3 la HOLD contiene istanti successivi al fault del gemello e non è confrontabile.

  - **Migliora:** scarto dei frame in camminata da 21/29 a 7/10 tick in media; errore sul pitch medio da 5,8 a 2,5 mrad; avvio della camminata dentro l'intervallo HW in 3 episodi su 4 contro 2.
  - **Non migliora:** carichi, margini delle anche, STOP.

## 6. Patch e riproducibilità

- **Modello fisico** (`servo.py`, `dynamics.py`, contatti, attrito, rigidezze): **invariato**.
- **Strumenti** (`fe66610`):
  - `m5_hw.py`: deduplica dei comandi; comandi usciti dal buffer delle tracce; etichette del joystick per parti; esclusione della prova simulata;
  - `m5_compare.py`: DRIVE solo frontale; SELECT solo al cambio; sorgenti HW di ogni episodio;
  - `m5_imu_tilt.py`: `--from` e prua;
  - `view.py`: 5 voci di catalogo.
- **Riproducibilità** (VERIFIED OFFLINE):
  - tutti i 30 CSV della sessione 450d2c7 rigenerati, identici byte per byte;
  - alla chiusura, G4 (baseline e `imu_mount`), F6b e t3 dei dati dev.4 rieseguiti con il codice di HEAD: CSV e JSON identici;
  - il viewer ricalcola ogni replay e lo confronta release per release con la corsa registrata (per esempio 905/905 release identiche in G4).

## 7. Limiti di validità

- Un robot e un pavimento per sessione. Pendenza non misurata; prua dall'IMU.
- A terra i dati densi coprono 2 finestre da 84 s; il resto è a 1 Hz. G3 ha solo 5 campioni di camminata.
- I SELECT dedotti a 1 Hz sono incerti di ±0,5 s; i comandi della dashboard hanno l'istante di accettazione del controller.
- Il gemello parte da uno STAND ideale, non dallo stato di forze interne del robot.
- A terra la batteria era sotto gli 11,0 V del protocollo.

## 8. Cosa manca per calibrare il carico a terra

1. Forze ai piedi: celle di carico o una pedana.
2. Letture dense delle anche a terra: il controller le ha a 8,33 Hz ma non le esporta. Serve una modifica firmware, qui solo documentata.
3. Una misura esterna d'assetto o di calibrazione dell'IMU.
4. L'attrito di distacco del servo e la sua dinamica sotto carico.
5. Un video sincronizzato degli STOP a terra.

## 9. File e uso

| Percorso | Contenuto |
|---|---|
| `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110/M5/sessione_450d2c7/` | G1–G4 (`baseline`, `imu_mount`), F1–F6b (`baseline`). Per ognuno: JSON, coppie HW/SIM (CSV), corsa del gemello (CSV), log. Inoltre `hw_session.txt`/`.json`, `imu_tilt_450d2c7.txt`, `imu_tilt_dev4_heading.txt` |
| `…/M5/storico_dev4/` | t1 e t3 dei dati dev.4, `imu_tilt_dev4.txt` |
| `…/M5/img/` | fotogrammi del viewer (HW in blu, gemello in arancio) |

```bash
cd 06_Software/Matdog_MuJoCo; PY=~/MATDOG/tools/mujoco-venv/bin/python; B=<build dir di fw_bridge/build.sh>
H=~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_HW_20261110
$PY m5_hw.py $H                                           # provenienza, frequenze, comandi, episodi
$PY m5_imu_tilt.py $H --from 1676                         # IMU − encoder nelle HOLD a terra
$PY m5_compare.py $B/cosim <out> g4_fast_walk_stop $H --from 2410.0 --to 2424.0 [--variant imu_mount]
$PY m5_compare.py $B/cosim <out> f4_q4_fast_stop_x2 $H --from 628.0 --to 730 --fixture
$PY view.py m5_terra_fast_stop m5_terra_slow m5_fixture_fast_stop
```

Finestre degli episodi (`--from`/`--to`, secondi del controller):

| Episodio | Finestra | Episodio | Finestra |
|---|---|---|---|
| F1 | 247.7–290 | F6b | 1161.2–1181.1 |
| F2 | 382.0–441 | G1 | 1792.0–1808.7 |
| F3 | 593.4–618.7 | G2 | 2299.9–2320.7 |
| F4 | 628.0–730 | G3 | 2332.3–2367.2 |
| F5 | 828.0–934 | G4 | 2410.0–2424.0 |
| F6a | 1042.0–1151.4 | | |

## Appendice: dati dev.4 dell'08/11

I dati dev.4 (a terra, tracce 1–3) avevano già mostrato le stesse due divergenze:
- **IMU − encoder:** pitch +22,9 mrad.
- **Camminata SLOW:** anche del gemello a 1,6–1,9 volte la coppia HW; margine HW +4,3/+5,3 tick, gemello −9,7/−6,7.
- **STOP di t3:** HW verificato in 2,13 s; gemello `STOP_MEASURED_OUTSIDE_BOX` a +20 ms.

Con `imu_mount` gli scarti dei frame scendevano da 28–42 a 4–15 tick (p95), mentre carichi e STOP restavano invariati.
