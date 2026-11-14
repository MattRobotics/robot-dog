# MATDOG MuJoCo — M6: correlazione con la dev.7 a terra e studio gait offline

**Natura.** Solo simulazione e log hardware letti in sola lettura. Nessun comando al robot, nessun flash, push o merge. Motion, ESP32, calibrazioni, box, NVS e sicurezza non sono stati modificati: le varianti gait sono patch applicate a **copie esportate** (`git archive`) di `b5610620d0ce`, mai a un checkout. **Nessun risultato di questo rapporto dimostra stabilità dinamica.**

**Codice.** Branch locale `sim/mujoco-dev7-correlation-20261112` (da `origin/main` 6838e4d, che contiene il gemello di `fc99e05`), worktree `~/MATDOG/worktrees/matdog-mujoco-dev7-corr-20261112`.
**Dati e risultati.** `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_DEV7_CORRELATION_20261112/` (README, `SHA256SUMS.txt`).

**Classi.** VERIFIED HARDWARE (nei log dev.7), VERIFIED OFFLINE (calcolo o simulazione riproducibile), HYPOTHESIS, UNKNOWN.

## 0. Esito

1. **Correlazione con la dev.7 a terra: migliorata, non completa** (VERIFIED OFFLINE).
   - Quattro correzioni con causa fisica: contatto senza scorrimento lento, massa senza Jetson, offset IMU misurato sulle HOLD della sessione (+38,9 mrad di pitch), attrito del riduttore proporzionale alla coppia (η 0,5, senza isteresi). Nessuna scelta dagli esiti degli STOP.
   - STOP a terra verificati: **baseline 1/7 → corretto 5/7**, tutti e 5 dentro l'intervallo HW di 1 Hz. S2 resta un **falso fault sistematico** (`STOP_MEASURED_OUTSIDE_BOX`), S4 è al limite.
   - Imbardata in rotazione (mai usata per tarare): HW −1,34 / +1,54 °/s, gemello −0,25 / +0,27 → **−1,35 / +1,37**.
   - HOLD statiche M1: carico LOWER da 2,2× a +8…+17 % dell'HW, senza isteresi come l'HW. Errori articolari in camminata 0,9–1,2× l'HW; duty UPPER/LOWER ancora **1,6×** (anche a vuoto sulla fixture).
   - Il criterio del handoff (7/7 senza fault) **non è raggiunto**. Il gemello serve per **confronti relativi** fra gait, non per i margini assoluti delle anche o per singoli STOP. Stabilità dinamica: **non verificata**.
2. **Gait** (VERIFIED OFFLINE, solo gemello). Fra 12 varianti, due modifiche hanno il miglior rapporto beneficio/rischio/complessità:
   - **A6 — profilo verticale dello swing STANDARD a sommità piatta** (stessa altezza 8 mm). Avanzamento utile in avanti 66 → 89 % della legge; diagonale 57 → 82 %; laterale 62 → 73 %; rotazione 65 → 75 %. Escursione laterale in avanti 7,6 → 3,6 mm, distacco reale del piede 1,2 → 3,0 mm, STOP su un ciclo 8/8 (dev.7 6/8). Stabile con massa ±10 % e COM ±5 mm. Test host Motion: PASS.
   - **B — sway sterzato proporzionale alla domanda.** In diagonale l'escursione laterale del corpo scende da 22,8 a 16,7 mm (−27 %) a pari avanzamento; nessun effetto a domanda piena. Richiede il gate joystick e la riscrittura del test `swayMap`.
   - Scartate: altezza 12/10 mm, profilo su FAST (workspace) e su SLOW (STOP), profili asimmetrici.
3. **Pronto per l'agente firmware:** A6 come proposta, da far passare per i gate offline (§9). B solo dopo il gate di supporto statico. Tutto il resto resta ipotesi. Nessuna modifica a Motion, ESP32, calibrazioni, box, NVS o sicurezza; nessun push, merge o flash.

## 1. Stato di partenza (VERIFIED OFFLINE)

| Voce | Stato |
|---|---|
| Archivio V2 `MATDOG_MUJOCO_V2_VNEXT_1160dce_20261111` | `sha256sum -c`: 1886/1886 OK |
| Dati HW dev.7 `MATDOG_1_0_2_DEV7_HW_20261111T1849Z` | `INVENTORY_SHA256.txt` 137/137 e `MANIFEST_SHA256.txt` 51/51 OK; letti senza modifiche |
| Worktree gemello `fc99e05`, dev.7 integrata `c3b67d0`, candidato vNext `a7f5c2c` | puliti (0 file); `c3b67d0` differisce da `b561062` solo in documenti (nessun file in `05_Firmware`) |
| Sorgente Motion compilato | worktree staccato `matdog-dev7-b561062-src` @ `b5610620d0ce`, 0 file sporchi (`build/SOURCE.txt`) |
| Repository principale | 11 file M0.4 pendenti di un'altra sessione: non toccati |

## 2. Ponte di co-simulazione sulla Motion installata

`fw_bridge/build.sh` compila ora l'albero indicato da `MATDOG_FW_TREE` (qui `b561062`, invariato) e ne scrive commit e file sporchi in `SOURCE.txt`; anche box `kHeight` e mappa gen-1 si leggono da quell'albero. `fw_bridge/cosim.cpp` accetta il joystick sui tre assi (`DRIVE vx vy yaw`), `HEIGHT h` (STAND da fermo, come le HOLD M1) e `LIMITS`. Nel ciclo chiuso il joystick viene rinnovato ogni 0,2 s fino al comando successivo (sull'HW i rinnovi visibili a 1 Hz sono ~1 al secondo, sotto il timeout di 1 s).

`m6_dev7.py` riproduce 16 finestre a terra della sessione (7 STOP e 9 camminate: avanti, indietro, laterale, diagonale, rotazione, cambi di modo) e 2 finestre sulla fixture, dalla HOLD alla quota della camminata, con il modo selezionato dall'HW; i comandi dati in camminata scattano allo stesso `walk_cycles`. Ogni istantanea HW è osservata sul gemello come la vedeva il controller (età di lettura per giunto, IMU al suo istante): `m5_compare.observe`.

**Limiti delle misure HW (VERIFIED OFFLINE).** Istantanee a 1 Hz per giunto (lettura vecchia 0–233 ms), IMU in seriale a 2 Hz. Non esistono misure di posizione del corpo, forze ai piedi o altezza reale del piede: avanzamento, appoggi e sollevamento HW **non sono misurabili** da questi log. Il valore di swing comandato non è stato usato come prova dell'altezza raggiunta.

## 3. Correzioni del modello (fase 1)

Ognuna ha una causa fisica e un dato su cui è identificata. Gli esiti degli STOP **non** sono stati usati per scegliere i parametri: servono da verifica.

| # | Correzione | Perché | Identificata su | Classe |
|---|---|---|---|---|
| 1 | Contatto senza scorrimento lento: `noslip_iterations` 5 | Il contatto morbido MuJoCo scorre sotto forza tangenziale costante: in HOLD le anche del gemello passano da ±3–4 a ±7–9 tick in 30 s (equilibrio "senza attrito"); l'HW resta a ±1–3 tick con 0–32 ‰ | HOLD statiche del gemello (4,5 s vs 30 s) contro M1 HW | VERIFIED OFFLINE (creep del gemello); HYPOTHESIS (gomma senza creep) |
| 2 | Massa del corpo senza Jetson: base 1,707 kg, totale 2,807 kg | Dev.7 senza Jetson (rapporto HW); pesata del 30/10 | Pesata | VERIFIED HARDWARE; posizione della Jetson UNKNOWN (COM invariato, vedi sensibilità) |
| 3 | IMU montata ruotata: roll +2,0, pitch +38,9 mrad | Nelle HOLD a terra IMU − piano dei piedi (encoder) = +38,9 mrad (p5–p95 38,4–42,6, n 576, prue −137…180°: non pendenza); l'HW applica −39,7 mrad di correzione in HOLD e la limita all'inviluppo in camminata (SLOW −12, STANDARD −17, FAST −31). Senza offset il gemello applica ~0 | `m5_imu_tilt.py` sulle HOLD dev.7 | VERIFIED OFFLINE da dati HW; valore diverso dai +23 mrad delle sessioni 08/11 e 10/11: causa UNKNOWN (rimontaggio, Jetson, flessione) |
| 4 | Attrito del riduttore proporzionale alla coppia, rendimento η 0,5, forma "smussata" (tanh, v0 0,02 rad/s) | A terra il gemello portava 2,2× il duty HW con lo stesso rapporto duty/errore (8,4 ‰/tick): a parità di guadagno, a riposo l'uscita tiene più coppia per unità di duty. La forma Coulomb (banda statica) è **falsificata**: dopo la risalita 105 → 120 mm dà 775 ‰ di somma LOWER contro 288 HW (isteresi), l'HW non ha isteresi (312/320/320/288) | Somma dei duty LOWER nelle 4 HOLD M1 | Forma HYPOTHESIS; η IDENTIFIED solo in statica (0,45–0,6 indistinguibili) |

**Non adottati.** Termine di velocità del servo (`ff_frac` 0,70, banco ≤ 600 step/s): sulla fixture dev.7 0,35–0,5 avvicina il ritardo in swing all'HW ma non il duty; resta 0,70. Attrito piede μ 0,8: con 0,6 gli STOP verificati diventano 6/7, ma tarare μ sugli STOP vorrebbe dire scegliere il parametro dal criterio: resta una sensibilità.

## 4. Correlazione: prima e dopo (VERIFIED OFFLINE)

Gemello di `fc99e05` (baseline) e gemello corretto (righe 1–4), stessa Motion `b561062`, stessi comandi HW.

### 4.1 I sette STOP a terra

Intervallo HW: tra l'ultima istantanea prima di `STOPPED` e la prima dopo (1 Hz). Gemello: release della verifica o del fault.

| # | Episodio (HW) | HW verificato | Baseline | Corretto |
|---|---|---|---|---|
| S1 | FAST indietro −20 mm/s, 105 mm | 1,79–2,80 s | FAULT `STOP_NOT_SETTLED` +3,08 | **verificato +1,98** |
| S2 | SLOW avanti 20 mm/s, 120 mm | 1,41–2,42 s | FAULT `STOP_MEASURED_OUTSIDE_BOX` +0,04 | FAULT `STOP_MEASURED_OUTSIDE_BOX` +0,04 |
| S3 | STANDARD avanti 30 mm/s, 110 mm | 2,00–3,01 s | FAULT `STOP_MEASURED_OUTSIDE_BOX` +1,16 | **verificato +2,70** |
| S4 | FAST avanti 32 mm/s, 105 mm | 1,92–2,93 s | FAULT `STOP_NOT_SETTLED` +3,12 | FAULT `STOP_NOT_SETTLED` +3,12 |
| S5 | STANDARD rotazione oraria, sterzato | 2,06–3,07 s | FAULT `STOP_NOT_SETTLED` +3,16 | **verificato +2,10** |
| S6 | SLOW dopo 5 cambi di modo | 1,91–2,92 s | verificato +3,02 (fuori intervallo) | **verificato +2,56** |
| S7 | STANDARD avanti 30 mm/s | 1,35–2,39 s | FAULT `STOP_MEASURED_OUTSIDE_BOX` +0,04 | **verificato +2,14** |
| | **Totale** | **7/7** | **1/7, 0 nell'intervallo** | **5/7, 5 nell'intervallo** |

Stabilità dell'esito (12 varianti provate, `runs/b1–b3`): S2 va in fault in **tutte**; S4 e S7 cambiano esito con η 0,5–0,6, μ 0,6–0,8 e noslip 1–5 (al limite); S1, S3, S5, S6 sono verificati in tutte le varianti con η ≤ 0,6.

### 4.2 Altre grandezze

| Grandezza (HW disponibile) | Baseline | Corretto | Nota |
|---|---|---|---|
| HOLD M1, somma duty LOWER 120/110/105/120 mm (HW 312/320/320/288 ‰) | 676–731 (2,2×) | 337/352/364/338 (+8…+17 %) | nessuna isteresi dopo la risalita, come l'HW; anteriori > posteriori come l'HW |
| HOLD M1, errore LOWER (HW −13…−3 tick) | −24…−16 | −12…−8 | |
| HOLD M1, anche (HW ±1–3 tick, 0–32 ‰) | ±7–9 tick, 57–78 ‰ (creep) | 0 tick, 0 ‰ | entro la banda morta |
| Camminata a terra, p95 \|errore\| gemello/HW anca/UPPER/LOWER | 1,29 / 1,09 / 1,52 | 0,91 / 0,93 / 1,20 | media geometrica su 16 episodi |
| … p95 \|duty\| | 1,55 / 2,16 / 2,35 | 0,99 / **1,66 / 1,62** | residuo, §5 |
| … p95 \|velocità\| (PresentSpeed) | 1,01 / 0,87 / 0,84 | 0,74 / 0,98 / 0,89 | |
| Margine minimo anche da `kHeight`, camminate avanti/indietro (HW +5,3 tick; media dei minimi +6,7) | −9,7 (media −2,5) | −1,7 (media +3,5) | il gemello resta pessimista di 4–7 tick |
| Roll/pitch picco-picco IMU, gemello/HW | 1,06 / 0,77 | 0,78 / 0,97 | |
| Velocità di imbardata in rotazione STANDARD (comando ±2,29 °/s; HW −1,34 / +1,54 °/s) | −0,25 / +0,27 | **−1,35 / +1,37** | non usata per tarare nulla |
| Scarto dei frame comandati HW − gemello, p95 massimo | 70 tick | 50 tick | la correzione IMU ora coincide |
| Fixture (F3b/F3c avanti): margine anche (HW 10,3) | 5,3 | 9,3 | |
| Fixture: p95 \|duty\| LOWER gemello/HW | 1,46 | 1,52 | anche a vuoto: §5 |

**Appoggi e sollevamento del piede.** Sull'HW non sono misurabili. Un indicatore (duty LOWER firmato in metà swing) è stato prima verificato sul gemello, dove la forza al piede è nota: accuratezza bilanciata 0,66, insufficiente per concludere. Le distribuzioni HW e gemello in metà swing sono simili (p10/50/90 HW −168/−40/112 ‰, gemello −218/−15/177 ‰). Nel gemello corretto il piede in swing si stacca in media di **2,2 mm** (massimo per swing, mediana) contro gli 8 mm comandati e tocca terra per metà dello swing: è un risultato del gemello, **non una misura HW**.

## 5. Differenze residue e rilevanza

| Differenza | Rilevanza | Causa candidata |
|---|---|---|
| S2: anca RH 11 tick fuori dal comando durante lo swing RF, margine −1,7 contro HW ≥ +6,3 | **Il gemello non può giudicare i margini delle anche allo STOP**, soprattutto in SLOW su tre appoggi: dà falsi `STOP_MEASURED_OUTSIDE_BOX` | forze laterali fra piedi bloccati (contatto rigido senza cedevolezza della gomma), attrito statico delle anche |
| S4 (e S7 in altre varianti): esito al limite della scadenza di assestamento | Esito STOP singolo non decisivo nel gemello; servono statistiche relative (variante vs dev.7 nello stesso gemello) | carichi dinamici in appoggio ancora 1,6× |
| Duty p95 in camminata 1,6× (anche a vuoto 1,5× sul LOWER) con errori e velocità corretti | I carichi assoluti del gemello non sono confrontabili con i limiti HW; i confronti relativi sì | PresentLoad filtrato nel servo o anello di velocità interno a 1000–1950 step/s: non identificabile a 1 Hz |
| Carichi UPPER statici (HW anteriori +40…+64 ‰, posteriori −24…−40; gemello 0…+20) | Forze interne avanti/indietro fra i piedi non riprodotte | rotolamento e cedevolezza del piede, storia del contatto |
| Velocità anca 0,74× | Minore | — |

**Parametri non identificabili con questi dati:** attrito e cedevolezza tangenziale piede-pavimento; rendimento del riduttore oltre la statica (0,45–0,6) e la sua forma; anello di velocità interno e filtraggio di PresentLoad oltre 600 step/s; causa dell'offset IMU (23 vs 39 mrad fra sessioni); posizione del COM e inerzie senza Jetson; altezza reale del piede in swing. Servirebbero letture dense (8,33 Hz già nel controller, non esportate), forze ai piedi o video sincronizzato, una misura esterna dell'assetto.

**Giudizio.** Il criterio del handoff (7 STOP su 7 senza fault) **non è raggiunto**: 5/7 nell'intervallo HW, 1 falso fault sistematico (S2), 1 al limite (S4). Il gemello corretto è adatto a **confronti relativi** fra varianti di gait (avanzamento, imbardata, trascinamento, scivolamento, tendenze dei carichi), non a giudicare margini assoluti delle anche, appoggio statico sub-millimetrico o esiti di singoli STOP. Stabilità dinamica: **non verificata**.

## 6. Fase 2 — studio gait: metodo

- **Build.** dev.7 = `b561062`; ogni variante = una patch in `gait_study/*.patch` applicata da `gait_study/build_variant.sh` a una copia esportata, compilata nel suo `fw_bridge` (provenienza in `build/SOURCE.txt`: commit + SHA-256 della patch). Nessun gait engine nuovo: le patch toccano costanti o una funzione della legge esistente.
- **Gemello.** Il modello corretto della §3 (μ 0,8), ciclo chiuso con la Motion della build.
- **Scenari** (`m6_gait.py`): avanti SLOW/STANDARD/FAST a velocità piena, indietro STANDARD, laterale SLOW/STANDARD/FAST, rotazione STANDARD, diagonale STANDARD, cambi di modo SLOW→STANDARD→FAST→SLOW, giro della leva (7 direzioni in sequenza); STOP a fine swing anteriore (p 0,85) per modo, STOP in FAST indietro e in rotazione; **8 STOP distribuiti su un ciclo** per ogni modo.
- **Metriche** (dalla fisica del gemello, mai dalla legge comandata): avanzamento utile per ciclo lungo la domanda e rapporto con la legge; escursione laterale del corpo (picco-picco attorno alla tendenza); distacco reale del piede (massimo per swing, mediana) e quota di swing con forza > 2 N; scivolamento dei piedi caricati; margine di appoggio dei piedi caricati (p05, minimo); margine anche da `kHeight`; errore, velocità e duty p95; esito e tempo degli STOP.
- **Sensibilità.** Massa del corpo ±10 %, COM x ±5 mm, y +3 mm.
- **Limiti.** Il margine di appoggio del gemello è dominato dai piedi in swing che toccano e dalla dinamica (negativo anche per la dev.7): si usa solo in confronto. Gli esiti dei singoli STOP sono al limite nel gemello (§5): conta il confronto su 8 STOP per modo. Nessun criterio è stato cambiato.

## 7. Varianti provate

| Variante | Modifica | Esito nel gemello |
|---|---|---|
| A (tutte le leggi) | Profilo verticale dello swing: salita e discesa smootherstep sul 35 % iniziale e finale, piatto a `liftM` (C2) al posto della campana 64u³(1−u)³ | Forte guadagno di avanzamento, ma **FAST: `WALK_IK_OR_WORKSPACE`** (105 mm, passo 80 mm) e STOP FAST non verificato |
| Altezza 12/10 mm (con e senza A) | `kWalkLiftM` 8 → 12 mm, FAST 6 → 10 mm | `WALK_IK_OR_WORKSPACE` in FAST, diagonale, laterale, rotazione: **scartata** |
| A2 / A4 (salita 30/25 %, discesa 60/75 %) | Piede su presto, atterraggio lungo | Nessun guadagno di avanzamento (il trascinamento che conta è a fine swing, col piede reale in ritardo); FAST e diagonale in `WALK_IK_OR_WORKSPACE`: **scartate** |
| A3 (simmetrica 45 %) | | Guadagno minore, FAST in fault: **scartata** |
| A5 (A sulle leggi da 8 mm: SLOW e STANDARD) | | FAST invariato; STANDARD migliore; **SLOW: STOP verificati 4/8 contro 6/8** (2 `STOP_NOT_SETTLED` in più) e fault con COM y +3 mm: SLOW escluso |
| **A6 (A solo sulla legge STANDARD)** | | **candidata 1** |
| **B** | Sway laterale sterzato proporzionale alla domanda sopra la soglia (da 5 mm a 3 % fino all'ampiezza piena a domanda piena) invece dell'ampiezza piena da 3,3 % | **candidata 2** |
| A6 + B | | somma dei due effetti, nessuna interferenza |

## 8. Confronto quantitativo con la dev.7 (VERIFIED OFFLINE, gemello corretto)

Avanzamento: mm per ciclo (rotazione: gradi per ciclo) e quota della legge. Distacco: massimo per swing, mediana. Errore/velocità/duty: p95.

| Scenario | Grandezza | dev.7 | A6 | B | A6 + B |
|---|---|---|---|---|---|
| Avanti STANDARD 30 mm/s (legge 90,2 mm/ciclo) | avanzamento | 59,2 (66 %) | **79,9 (89 %)** | = dev.7 | 79,9 (89 %) |
| | escursione laterale corpo | 7,6 mm | **3,6 mm** | = | 3,6 |
| | distacco piede / quota trascinata | 1,22 mm / 0,46 | **3,01 mm / 0,30** | = | = A6 |
| | scivolamento max | 4,4 mm | 3,0 mm | = | = A6 |
| | margine anche `kHeight` min | +0,3 t | +2,3 t | = | = A6 |
| | UPPER err / vel / duty | 48 t / 773 step/s / 444 ‰ | 35 / **894** / 336 | = | = A6 |
| | LOWER err / vel / duty | 48 / 578 / 424 | 42 / 558 / 386 | = | = A6 |
| Indietro STANDARD 16,7 mm/s (legge 49,8) | avanzamento | 45,2 (91 %) | 52,2 (105 %) | = | = A6 |
| Laterale STANDARD 6,7 mm/s (legge 20) | avanzamento | 12,5 (62 %) | 14,5 (73 %) | 12,4 | 14,5 |
| | escursione laterale corpo | 22,9 mm | 22,5 | 22,7 | 22,5 |
| Rotazione STANDARD 2,29 °/s (legge 6,9°/ciclo) | imbardata | 4,5° (65 %) | **5,2° (75 %)** | = | 5,2° |
| Diagonale STANDARD 13,7 / −2,7 mm/s (legge 41,9) | avanzamento | 23,7 (57 %) | **34,2 (82 %)** | 23,6 (56 %) | 33,5 (80 %) |
| | escursione laterale corpo | 22,8 mm | 22,1 | **16,7** | **16,1** |
| | margine appoggio p05 | −13,1 mm | −6,7 | −11,2 | −11,6 |
| Cambi di modo SLOW→STANDARD→FAST→SLOW | avanzamento medio | 52,8 mm/ciclo | 58,4 | – | – |
| | fault / rifiuti | 0 / 0 | 0 / 0 | – | – |
| SLOW e FAST (tutti gli scenari) | | | identici alla dev.7 | identici | identici |
| 8 STOP su un ciclo | STANDARD / SLOW / FAST verificati | 6/8 / 6/8 / 7/8 | **8/8** / 6/8 / 7/8 | – | – |
| STOP singoli (fine swing p 0,85, FAST indietro, rotazione) | | verificati | verificati (STANDARD +2,58 s vs +2,36) | verificati | verificati |

**Sensibilità a massa e COM** (STANDARD avanti / laterale / rotazione / STOP; dev.7 → A6): massa −10 % 69 → 91 %, 64 → 74 %, 66 → 76 %, verificato → verificato; massa +10 % 62 → 86 %, 60 → 73 %, 64 → 75 %, verificato → verificato; COM x −5 mm 67 → 88 %, 62 → 72 %, 64 → 73 %; x +5 mm 63 → 89 %, 63 → 74 %, 69 → 78 %; y +3 mm 65 → 88 %, 62 → 73 %, 68 → 75 %; STOP sempre verificato. Il vantaggio di A6 è stabile. La stessa modifica su SLOW (A5) dà fault con −10 %, +10 %, x +5 mm e y +3 mm (`WALK_IK_OR_WORKSPACE`): conferma l'esclusione di SLOW.

**Riferimento HW.** L'efficienza di imbardata misurata sulla dev.7 (1,34–1,54 °/s su 2,29: 58–67 %) coincide con quella del gemello dev.7 (65 %); avanzamento e escursione laterale non sono misurabili sull'HW (§2).

### Obiettivi della richiesta non coperti da una proposta

| Obiettivo | Esito |
|---|---|
| Maggiore altezza swing | Non con `liftM`: 12/10 mm esce dal workspace. Nel gemello il distacco reale (1–3 mm) è limitato dal ritardo dei servo in swing e dal cedimento del corpo su tre appoggi, non dall'altezza comandata. A6 lo raddoppia in STANDARD a parità di `liftM` |
| Traslazione laterale più lineare, meno sway preparatorio a domanda piena | La legge sposta il corpo di ±9–10 mm nelle finestre a quattro appoggi per restare nel triangolo; ridurlo a domanda piena richiede un'altra legge del COM (spostamento anche nelle fasi a tre appoggi): fuori perimetro, HYPOTHESIS. B riduce lo sway solo a domanda parziale |
| Rollio controllato, migliore impiego delle anche | Non provati: il gemello non risolve l'appoggio statico sub-millimetrico che questi cambi toccano (memoria V2). HYPOTHESIS |
| Rotazione più coordinata | A6: 65 → 75 % dell'imbardata comandata |
| Diagonali e cambi continui | A6: diagonale 57 → 82 %; giro della leva e cambi di modo senza fault in A6, B e A6+B. La dev.7 ferma la camminata all'inversione avanti/indietro (`JOYSTICK_REVERSING`, scelta di progetto): invariato |

## 9. Proposta per l'agente firmware

### Candidata 1 — profilo verticale dello swing STANDARD (A6) — miglior rapporto beneficio/rischio

- **Cosa.** In `Gait.cpp`, solo per la legge STANDARD (duty 0,825, lift 8 mm, comprese le sue leggi sterzate): `z = liftM · S(u/0,35) · S((1−u)/0,35)` con S = smootherstep troncata a 1 (piatta fra 35 % e 65 % dello swing), al posto di `64 liftM u³(1−u)³`. C2 a stacco e appoggio, stessa altezza massima. Patch di studio: `gait_study/A6_lift_profile_standard.patch` (la scelta "solo STANDARD" vi è codificata su duty e `liftM`; in firmware va un campo `GaitParameters` impostato per modo, da includere in `stepWalkMode` (forma della legge), nell'uguaglianza delle leggi e in `gaitRetarget`).
- **Beneficio** (gemello): +23…+25 punti di avanzamento utile avanti/indietro, +11 laterale, +10 imbardata, +25 diagonale; escursione laterale avanti −53 %; distacco reale del piede ×2,5; trascinamento −35 %; 8/8 STOP su un ciclo (6/8 dev.7); stabile su massa e COM.
- **Costo/rischio.** Velocità UPPER p95 in swing +16 % (773 → 894 step/s, drive 1800); picco della velocità verticale del piede ×1,56 (5,36 contro 3,43 `liftM` per unità di swing): rifare la derivazione del drive RAM ("picco della legge / 0,8"). Host test `run_simple_motion_tests.sh`: **PASS** sulla copia patchata (`host_tests/`).
- **Da rifare offline (agente firmware), prima di qualsiasi flash:** `walk_tilt_envelope_check --mode 1` (inviluppo IMU STANDARD), `walk_support_check` (supporto statico e capture point), `joystick_envelope` modo 1 (`kSteerStandard`), `run_servo_plant` (campagne STOP nelle finestre e a fine swing), verifica di `kHeight`/`kSteerStandard` sulle nuove traiettorie. Il gemello non sostituisce nessuno di questi controlli.
- **Esclusi.** FAST (workspace) e SLOW (assestamento STOP nel gemello, sensibilità COM).

### Candidata 2 — sway sterzato proporzionale alla domanda (B)

- **Cosa.** `joystickSwayYM` (`SimpleMotion.h`): oltre la soglia di attivazione l'ampiezza cresce da `kJoystickSwayLowM` (5 mm) a `swayYM` con la domanda laterale/imbardata invece di saltare subito a `swayYM`; isteresi e limiti di velocità invariati. Patch: `gait_study/B_steer_sway_proportional.patch`. Risponde alla richiesta 4 del rapporto HW dev.7.
- **Beneficio** (gemello): diagonale al 40 % di domanda laterale: escursione laterale del corpo 22,8 → 16,7 mm (−27 %), avanzamento invariato, margine di appoggio dinamico non peggiore (−13,1 → −11,2 mm). A domanda piena (laterale puro, rotazione) nessun cambiamento.
- **Rischio.** È l'appoggio **statico** a domanda parziale con meno sway: il gemello non può certificarlo (sub-mm). Va provato da `joystick_envelope` (sequenze casuali, supporto ≥ 0) per i tre modi. Il test host `swayMap` codifica la mappa attuale e **fallisce** con B: va riscritto con la nuova mappa.

### Combinazione A6 + B
Effetti additivi nel gemello (diagonale: 80 % di avanzamento con 16,1 mm di escursione), nessun fault in giro della leva e cambi di modo.

## 10. Cosa è pronto e cosa resta ipotesi

| Elemento | Stato |
|---|---|
| Modello del gemello corretto (§3) e strumenti M6 | Pronti nel branch locale; default invariato (regressione byte per byte contro `fc99e05`) |
| A6 come patch firmware | **Pronta come proposta** da far passare per i gate offline del §9; beneficio solo simulato; nessuna evidenza HW |
| B come patch firmware | Proposta condizionata al gate joystick e alla riscrittura di `swayMap` |
| Altezza swing maggiore, profilo su FAST/SLOW, profili asimmetrici | Scartati nel gemello |
| Sollevamento reale del piede sull'HW, appoggi HW | **Non misurati**: ipotesi |
| Margini delle anche e STOP singoli giudicati dal gemello | **Non affidabili** (S2 falso fault, S4 al limite) |
| Stabilità dinamica | **Non verificata** |

**Per la prossima sessione HW (se autorizzata):** esportare le letture a 8,33 Hz del controller in una finestra breve, oppure filmare lateralmente i piedi a STANDARD; basterebbe a misurare il distacco reale prima e dopo A6.

## 11. Riproducibilità

```bash
cd 06_Software/Matdog_MuJoCo; PY=~/MATDOG/tools/mujoco-venv/bin/python
A=~/MATDOG/verification-artifacts/MATDOG_MUJOCO_DEV7_CORRELATION_20261112
H=~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z
export MATDOG_FW_TREE=<checkout di b5610620d0ce>/05_Firmware/MATDOG_Controller
bash fw_bridge/build.sh $A/build
M="--variant dev7_ground"      # = noslip 5, base 1,707 kg, IMU 2,0/38,9 mrad, eta 0,5 smussato
$PY m6_dev7.py $A/build/cosim <out> $H all $M          # 16 episodi a terra e 2 sulla fixture
$PY m6_dev7.py $A/build/cosim <out> $H hold $M         # HOLD statiche M1
$PY m6_summary.py <out>                                # STOP, rapporti, anche, IMU, imbardata
$PY m6_legs.py <out> <variante>                        # indicatore di sollevamento (validato sul gemello)
bash gait_study/build_variant.sh b5610620d0ce gait_study/A6_lift_profile_standard.patch <dir>
$PY m6_gait.py <dir>/build/cosim <out> A6 [scenari] $M  # fase 2
$PY m6_gait_summary.py <out>
```
