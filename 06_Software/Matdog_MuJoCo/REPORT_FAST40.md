# MATDOG — FAST 40 mm/s: RCA del limite e proposta (studio MuJoCo, 12/11/2026)

Solo controlli offline e simulazione. Nessun hardware, flash, push o merge. La Motion dev.7
`b5610620d0ce`, i box, le protezioni, le soglie e le calibrazioni restano invariati.

I gate sono quelli del firmware, compilati da sorgenti non modificate o dalla patch del candidato:
- `walk_support_check`;
- `walk_tilt_envelope_check`;
- `joystick_envelope`;
- `run_servo_plant`;
- `run_simple_motion_tests`.

I criteri sono quelli già scritti in `SimpleMotion.h` per la scelta dei modi:
- supporto statico ≥ 1,70 mm e punto di cattura ≥ 1,41 mm;
- margine dell'anca ≥ 8,7 tick;
- picco della legge ≤ 0,8 × drive {1950, 220};
- prova StopPlan a ACC 84: 0 violazioni.

Obiettivo: FAST 40 mm/s, con SLOW 20 e STANDARD 30 invariati. Le velocità intermedie (34–38 mm/s)
servono solo per la diagnosi.

## 0. Esito

- **Il limite a 40 mm/s è identificato e superato offline.**
  - La violazione STOP dell'UPPER anteriore, 0,4–3,9 tick, è un overshoot del punto finale dell'arresto StopPlan
    sul box kHeight (F1).
  - Compare solo con la correzione IMU vicino al vertice pitch-down e sulla corda T2.
  - Ha accanto una seconda famiglia sull'anca posteriore (F2). Non è un limite geometrico né IK (§2).
- **Proposta F40.** Legge FAST a 40,0 mm/s che passa, con criteri invariati, tutti i gate offline del firmware:
  - `walk_support_check` e `walk_tilt_envelope_check` su 3 preset e in retromarcia a 50 e 58 mm per ciclo;
  - `joystick_envelope` FAST, 46 scenari;
  - test host della Motion (exit 0);
  - `run_servo_plant` FAST: 0 fault, tranne i cambi di modo dentro la camminata (§4, §7).
- Rispetto alla FAST dev.7 cambiano:
  - la riga FAST (falcata, periodo, duty, lift, rampe, sway);
  - le rampe dell'arto avanzante;
  - una modulazione della velocità del corpo di 1,2 mm;
  - lo sway sterzato della FAST.
- Restano invariati box, protezioni, soglie, ACC 84, drive {1950, 220}, kFastTilt, SLOW e STANDARD.
- **Gemello** (correlazione dev.7 5/7, quindi solo per confronti):
  - avanti 32,6 mm/s reali contro 21,9 della dev.7 (+49 %);
  - STOP verificati 28/28 in 16 + 8 fasi del ciclo e nelle 4 direzioni, contro 25/28 della dev.7;
  - con massa ±10 % e COM ±5 mm: avanti 25,8–34,7 mm/s, STOP 31/32.
- **Non ancora pronto per l'hardware.**
  - Cambi di modo dentro la camminata: la transizione di sway x, modulazione e rampe va implementata; oggi fault nel
    servo plant.
  - Laterale e imbardata più lenti della dev.7 nel gemello.
  - Stacco reale del piede minore.
  - Margine del box con correzione IMU di 6,2 tick (dev.7: 10,3).
  - Offset IMU di 39 mrad da misurare.
  - Stabilità dinamica non verificata.

## 1. Perché la dev.7 si ferma a 32 mm/s

La FAST dev.7 a 32 mm/s è già sul bordo di due gate. Una legge a 40 mm/s con la sola riga FAST li supera
entrambi.

| Gate | dev.7 FAST 32 | Meccanismo |
|---|---|---|
| Punto di cattura STOP ≥ 1,41 mm | **1,41** | Offset v·√(z/g): ≈ +0,9 mm da 32 a 40 mm/s. Con sway e COM condivisi dai modi, nessuna delle 11 550 combinazioni di falcata, duty, rampe, altezza e lift passa a 40 mm/s, e nessuna nemmeno a 34. |
| Secante RAW ≤ 31 tick/release | **30,5 / 31** | Residuo StopPlan 0,35 rad a ACC 84. Con 32 tick l'arresto dura 0,30 s e il residuo vale 0,368 rad. |
| Accelerazione della legge ≤ 27 rad/s² | 26,4 | Drive ACC 220, tetto del registro 85. |
| Box kHeight durante l'arresto | margine 11,1 tick (anca) | Durata comune dell'arresto fissata dal giunto più veloce. |

Nota per l'agente firmware: quando fallisce `walk_tilt_envelope_check` stampa "X ends N ticks from the box" con il giunto
peggiore sul punto finale, anche quando la causa è il residuo. Esempio: le leggi a 40 mm/s con la sola riga FAST
risultano "LF_HIP ends 4.8 ticks", ma falliscono sul residuo dell'UPPER anteriore.

## 2. RCA della violazione STOP a 40 mm/s

Strumento: `gait_study/fast40/stop_rca.cpp`. È `walk_tilt_envelope_check.cpp` dev.7 con verdetti identici, output
confrontato byte per byte su dev.7 e N40, più una riga diagnostica per ogni fallimento StopPlan.

Il caso analizzato è N40, la migliore legge a 40 mm/s senza modulazione di velocità:
- falcata 110 mm, periodo 2,75 s, duty 0,80, altezza 105 mm, lift 6 mm;
- sway x 4,5 mm, COM 11 mm, sway y 0,75 mm;
- rampe dell'arto avanzante 0,15/0,65.

N40 ha 87 violazioni, tutte di tipo box, tra 0,4 e 3,9 tick.

| | F1, 47 violazioni | F2, 40 violazioni |
|---|---|---|
| Giunto | UPPER anteriore (LF/RF), lato inferiore del box | HIP posteriore (RH/LH) |
| Fase | swing anteriore a u = 0,63–0,73 | primo release della finestra a 4 appoggi dopo il touchdown anteriore (c = 1,03, 1,53, …) |
| Secante | T2 "exact after midpoint", la corda della look-ahead | exact |
| Arresto | 0,18–0,22 s, fissato dallo stesso UPPER a 1,38–1,76 rad/s | 0,20 s, fissato dall'UPPER anteriore in appoggio a 1,61 rad/s (trasferimento dello sway x) |
| Margine comandato | 102–151 tick nominali; 77–126 tick con la correzione IMU | 14,9 tick; 14,3 con la correzione |
| Raggi IMU che falliscono | pitch da −30,6 a −34,2 mrad (vertice pitch-down di kFastTilt) | pitch da −15 a −27 mrad con roll di 4–9 mrad |

Classificazione:
- **Geometria/FK-IK: non è la causa.** La traiettoria comandata resta dentro kHeight in ogni release, con margine
  minimo 12,3 tick, 0 fallimenti IK e 0 fallimenti di box.
- **Box di STOP: è la condizione violata.** Il punto finale dell'arresto quartico (q + t·v/2) esce dal box. Residuo,
  vmax e amax non sono mai il motivo.
- **Correzione IMU: determinante.** Senza correzione non c'è nessuna violazione. In pitch puro N40 passa fino a
  −25 mrad e fallisce solo da −30.
  - Sulla dev.7 il robot lavora proprio lì. Per l'offset IMU di 38,9 mrad (M6), la correzione resta tagliata
    sull'inviluppo (`ENVELOPE`) per circa il 75 % dei campioni di camminata, nel gemello come sull'hardware.
- **Quantizzazione: dimensione del sovraccarico.** Tutte le 87 violazioni passerebbero con la secante non
  quantizzata, o arrotondata al tick più vicino invece che per eccesso.
  - È solo una classificazione: il caso peggiore è voluto e non si tocca.
- **Frenata assunta: margine dell'1–7 %.** Tutte le 87 passerebbero con ACC 85–90 invece di 84.
  - Nemmeno questo si tocca: 84 è la frenata misurata a vuoto e quella sotto carico non è misurata (§6).
- **Tracking del servo, gemello contro hardware:** la violazione esiste già sulla traiettoria ideale, perché la
  prova è sui riferimenti. Il comportamento dinamico è in §5.

## 3. Varianti mirate a 40 mm/s

Si parte da N40 e si cambia una cosa per volta. Ogni riga è stata provata con supporto, cattura, amax e la prova
STOP completa (tutti i raggi di kFastTilt, 8 sottofasi, corde T2, uno STOP per ogni quarto di ciclo).

| Variante | Supporto / cattura (mm) | amax (rad/s²) | Violazioni STOP | Effetto |
|---|---|---|---|---|
| N40 | 2,06 / **1,24** | 26,8 | 87 (F1 47, F2 40) | — |
| COM 10,5 | 1,80 / 1,51 | 26,8 | 197 (F1 fino a 6,9 tick) | La cattura sale, ma il piede anteriore atterra più avanti |
| sway y 0,6 | 1,95 / 1,13 | 26,6 | 34 (solo F1) | F2 sparisce, la cattura scende |
| Rampa avanzante 0,15/0,70 | = | **28,1** | 44 → 0 con la modulazione | F1 si risolve, ma l'amax supera la regola |
| Altezza 107,5 / 102,5 mm | — | — | 625 / 22 836 | Peggio: cambia il lato del box che lega |
| **Modulazione di velocità 1 mm, fase 0,25** | 1,75 / **1,56** | 26,8 | 33 (**F2 = 0**) | Cattura e F2 risolti |
| + lift 5, rampa avanzante 0,18/0,70 | 1,75 / 1,56 | 26,8 | **0** | Ma i vicini (lift 4,5, fase 0,24) falliscono di 0,9–2,9 tick |
| **F40**: modulazione 1,2 mm, fase 0,265, rampa avanzante 0,19/0,70 | 1,79 / 1,49 | 26,78 (in entrambi gli strumenti) | **0** | Tutti i vicini passano: modulazione 1,1–1,3 mm, fase 0,265–0,27, lift 4,5–5,5 mm, rampe ±0,01 |

**Modulazione di velocità.** Si somma all'offset longitudinale del corpo un termine
A·sin(4π(c − c₀)), a media zero, con due periodi per ciclo (uno per swing anteriore).
- Con c₀ = 0,265 il corpo rallenta nell'ultima parte di ogni swing anteriore, senza essere arretrato quando il piede
  tocca: il punto di cattura migliora senza peggiorare la portata dell'UPPER.
- Nella finestra a 4 appoggi dopo il touchdown la modulazione frena il corpo proprio mentre lo sway x trasferisce
  il COM. Gli UPPER in appoggio rallentano, l'arresto comune si accorcia e l'anca posteriore non esce più dal box: è
  il meccanismo che elimina F2.
- L'ampiezza scala con il passo in avanti della legge: piena a 110 mm per ciclo, 0,49 mm al limite sterzato di
  45 mm, nulla in laterale, in rotazione e in retromarcia.
  - Il primo prototipo non scalava, e il gemello e i test host andavano in fault proprio in regime sterzato:
    `superseded/`.
- La modulazione entra con la rampa di partenza ed esce nel ciclo di decelerazione di un WALK STOP, con raccordi
  C2, così al terminale non resta nulla da congelare.
  - Il secondo prototipo la congelava al terminale e il test di continuità C1 della Motion ha rilevato un salto di
    velocità del corpo di circa 5,5 mm/s: `superseded/F40_surge_no_stop_fade`.

**Rampe dell'arto avanzante.** Le gambe che a fine swing si allungano in avanti (le anteriori; le posteriori in
retromarcia) hanno un trapezio proprio. La velocità del loro UPPER cresce lungo lo swing per la geometria della
gamba, mentre il LOWER posteriore ha il picco al distacco: un'unica forma non serve entrambe.
- Con 0,19/0,70 il piede anteriore rallenta prima del touchdown, in u = 0,63–0,73, dove nasceva F1.

**Regime sterzato.** Con lo spec sterzato della dev.7 (sway 5/7,5 mm) il gate `joystick_envelope` della FAST di F40
dà 9 fault su 46 scenari (`WALK_IK_OR_WORKSPACE`). La finestra a 4 appoggi è più corta: 0,1375 s contro 0,1875.
- Con sway sterzato 4,5/5,5 mm e gli stessi 12 mm laterali e 0,08 rad per ciclo: **PASS su 46 scenari**, supporto
  minimo 0,72 mm, contro 0,51 della dev.7 FAST.

## 4. F40 — la legge proposta e i gate offline

| | dev.7 FAST | F40 |
|---|---|---|
| Falcate dei preset, periodo | 60/70/80 mm, 2,5 s | 80/95/110 mm, 2,75 s |
| Velocità dei preset | 24 / 28 / **32** mm/s | 29,1 / 34,6 / **40,0** mm/s |
| Duty, altezza, lift | 0,825, 105 mm, 6 mm | 0,80, 105 mm, **5 mm** |
| Rampe dello swing | 0,35/0,55 su tutte le gambe | arto avanzante **0,19/0,70**, altre **0,35/0,30** |
| Sway x / COM / sway y | 3 (condiviso) / 11 / 1,0 mm | **4,5** (proprio della FAST) / 11 / **0,75** mm |
| Modulazione di velocità | — | **1,2 mm, fase 0,265**, scalata sul passo in avanti |
| Sterzata: laterale, imbardata, sway | 12 mm, 0,08 rad, 5/7,5 mm per ciclo | 12 mm, 0,08 rad, **4,5/5,5** mm per ciclo |
| Drive, kFastTilt, box, ACC 84 | — | **invariati** |

I gate offline sono tutti PASS e in molti casi peggiorano poco o migliorano rispetto alla dev.7:

| Controllo | dev.7 FAST 32 | F40 p0 / p1 / p2 | F40 retromarcia 50 / 58 mm |
|---|---|---|---|
| Supporto statico | 1,71 | 2,10 / 2,02 / **1,95** | 2,30 / 2,25 |
| Punto di cattura STOP | 1,41 | 1,92 / 1,71 / **1,49** | 1,97 / 1,92 |
| Anca, margine da kHeight | 11,1 | 12,4 | 13,7 |
| Picco velocità / accelerazione | 2,350 / 26,4 | 2,137 / 26,78 | 1,37 / 24,96 |
| Secante RAW per release (limite 31) | 30,5 / 31 | 27,9 / **29** | 17,7 / 18 |
| Prova STOP | 0 | **0 / 0 / 0** | 0 / 0 |
| Margine box con correzione IMU | 10,3 tick | 12,3 / 12,3 / **6,2** | 13,3 |
| Prova STOP con ACC assunto 82 (controfattuale) | 7 231 violazioni | 216 violazioni | — |
| Errore di COM +2 / −2 mm in x (supporto / cattura) | 1,12 / 0,24; 0,56 / 0,73 | 1,64 / 0,28; 0,79 / 1,23 | — |
| Errore di COM +5 mm in x | −0,58 / −1,52 | −0,14 / −1,54 | — |

Le colonne F40 vengono dalla patch finale (`builds/F40`). La tabella del §3 viene invece dall'albero di studio,
dove la modulazione non si spegne nel ciclo di decelerazione dello STOP. Lì il supporto minimo è 1,79 mm, perché
cadeva proprio in quel ciclo; con la dissolvenza sale a 1,95 mm. Cattura e prova STOP sono identiche.

## 5. Dinamica: modello servo, gemello, test host

**Test host della Motion** (`run_simple_motion_tests.sh` sulla patch F40): **exit 0**. Coprono:
- continuità C1/C2 della legge a ogni giunzione;
- joystick multidirezionale con STOP in 6 istanti per direzione;
- 36 STOP durante i cambi di modo;
- preset FAST a 110 mm e 2,75 s.

**Servo plant** (`run_servo_plant.sh`, 6 seed × 2 tensioni, servo che ritarda e cede; modello del firmware):

| Scenario FAST | dev.7 | F40 |
|---|---|---|
| WALK 90 s, 8 STOP con ripresa, IMU al vertice pitch | 0 fault | **0 fault**, STOP 96/96, misura fuori box 0 tick |
| Sterzata: STOP×8 e IMU | 0 fault | **0 fault**, 96 + 96 STOP |
| Retromarcia: STOP×8 e cambi di modo | 0 fault | **0 fault** |
| **Cambi di modo dentro la camminata** (MODE_CHANGES, anche SLOW) | 0 fault | **5–6 fault su 6 seed** (`WALK_IK_OR_WORKSPACE`) a ciclo fisso: è la transizione di legge che manca (§8) |

**Gemello** (dev7_ground: correzione IMU sull'inviluppo come sull'hardware; ogni legge alla propria velocità
massima, metriche sulla fisica del gemello):

| Scenario | dev.7 FAST | F40 | Nota |
|---|---|---|---|
| Avanti, velocità reale | 21,9 mm/s (68 % della legge) | **32,6 mm/s (81 %)** | +49 % |
| Avanti con comando 32 mm/s | 21,9 | **25,5** | +16 % a pari comando |
| STOP su un ciclo (8 fasi) | 7/8, `STOP_MEASURED_OUTSIDE_BOX` | **8/8** | |
| STOP densi (16 fasi, sfasati di mezzo passo) | 14/16, un `OUTSIDE_BOX` e un `NOT_SETTLED` | **16/16** | verifica entro 2,94 s |
| STOP in avanti, indietro, imbardata, diagonale | 4/4 | **4/4** | |
| Diagonale: velocità / escursione laterale | 7,7 mm/s / 20,9 mm | 7,3 mm/s / **14,9 mm** | −5 % / −29 % |
| Laterale, sinistra / destra | 1,38 / 2,83 mm/s | **0,80 / 1,85** | −37 %: regressione |
| Imbardata | 0,92 °/s | **0,53 °/s** | −42 %: regressione |
| Retromarcia | 18,1 mm/s | 13,9 | −23 %; con 58 mm per ciclo (gate PASS) la legge torna a 21 mm/s |
| Stacco reale del piede, p50 in avanti | 0,79 mm | **0,10 mm** | Il gemello non sa giudicarlo (M6), ma il confronto relativo peggiora |
| Slittamento massimo in avanti | 5,9 mm | 9,3 mm | |
| Margine minimo anca da kHeight in avanti | 3,3 tick | 8,3 tick | |

**Sensibilità** (gemello, perturbazioni verificate nel campo `definition`; avanti e 8 STOP su un ciclo):

| | massa −10 % | massa +10 % | COM x +5 mm | COM x −5 mm |
|---|---|---|---|---|
| dev.7 avanti / STOP | 22,7 / 7 su 8 | 21,2 / 7 su 8 | 21,2 / 7 su 8 | 22,6 / 7 su 8 |
| F40 avanti / STOP | **33,9 / 8 su 8** | 26,9 / 7 su 8, `NOT_SETTLED` | 25,8 / 8 su 8 | **34,7 / 8 su 8** |

La velocità reale di F40 dipende da massa e COM in avanti (26–35 mm/s); quella della dev.7 no (21–23).

**STOP dentro lo swing anteriore** (`twin_stop_rca.py`: STOP a u = 0,55–0,80 di LF, correzione IMU da −29 a −32 mrad
in `ENVELOPE`):

| | F40 (6 STOP) | N40 (4 STOP; offline non passa) |
|---|---|---|
| Esito | 6/6 verificati | 4/4 verificati |
| LF_UPPER: margine comandato → misurato alla richiesta | 57–174 → 110–261 tick | 76–148 → 135–235 tick |
| LF_UPPER: punto finale del piano runtime | dentro di 12–42 tick | dentro di 9–38 tick |
| Anca posteriore: margine misurato minimo durante lo STOP | 6,3–11,3 tick | 6,3–10,3 tick |

Lettura:
- **UPPER anteriore:** nel modello servo resta indietro rispetto al riferimento di 50–110 tick. La posa misurata è
  quindi lontana dal limite inferiore, e la violazione F1 della prova (sui riferimenti, nel caso peggiore di
  quantizzazione) non si traduce in un'uscita misurata dal box. La prova resta comunque il gate: F40 la passa,
  N40 no.
- **Anca posteriore:** è il vincolo misurato più stretto durante lo STOP, a 6–11 tick. Il gemello è pessimista
  sulle anche di 4–7 tick (M6).

## 6. Periodo di 20 ms

- **Secante per release:** RAW di picco 27,9 nominale e 29 con correzione, contro 30,5/31 della dev.7. Restano 2 tick
  di margine sul limite StopPlan che oggi blocca la FAST, dove la dev.7 ne ha 0.
- **Corde T2:** la look-ahead a 40 ms è dentro la prova STOP (F1 era proprio una corda T2) e passa su tutte le 8
  sottofasi del release.
- **Calcolo:** la legge aggiunge per ogni valutazione un seno (la modulazione) e un confronto (la scelta delle
  rampe). Nessun nuovo solve IK.
  - Il costo sull'ESP32 non è misurato qui. È compito dell'agente P0 con `motion_cost_probe`, da ripetere sulla patch:
    non duplico la diagnosi Timing P0.

## 7. Cosa è verificato, cosa no, misure necessarie

**Geometria verificata** (gate offline del firmware, legge ideale, suolo rigido): F40 a 40 mm/s rispetta tutti i
criteri di selezione dei modi.
- Box kHeight con correzione IMU su tutto kFastTilt.
- Prova StopPlan a ACC 84 con le corde T2 e 8 sottofasi.
- Supporto e punto di cattura.
- Regola del drive.
- Inviluppo sterzato.
- Retromarcia.

Sono margini di modello statico, non qualifiche hardware. Il margine del box con correzione (6,2 tick) è più
stretto di quello della dev.7 (10,3).

**Comportamento nel modello servo:**
- servo plant: 0 fault in avanti, in sterzata e in retromarcia; fault solo nei cambi di modo;
- gemello: 28/28 STOP e la posa misurata dentro il box.

Il gemello riproduce 5/7 STOP hardware ed è pessimista sulle anche. I suoi esiti contano come confronto relativo,
non come certificazione.

**Incertezze ancora presenti:**
1. **Offset IMU di 38,9 mrad**, causa ignota (M6). Mette la correzione sul vertice pitch-down dell'inviluppo, la
   zona dove nascono le violazioni F1. Se è un errore di montaggio, il corpo viene davvero inclinato di 30 mrad
   in avanti mentre corre.
2. **Frenata sotto carico non misurata.** La prova usa ACC 84, la frenata a vuoto.
   - F40 e dev.7 dipendono allo stesso modo da questo valore. Con 82, F40 ha 216 violazioni e la dev.7 7 231.
3. **Stacco reale del piede.** F40 comanda 5 mm invece di 6. Nel gemello lo stacco reale è 0,10 contro
   0,79 mm, ma il gemello sovrastima l'affondamento (M6).
4. **Laterale e imbardata** più lenti nel gemello (−37 % e −42 %).
   - Con lift 5,5 recuperano in parte (+45 % e +23 %), ma lift 5,5 non passa la prova STOP: 5 violazioni.
5. **Sensibilità della velocità reale a massa e COM** in avanti (26–35 mm/s).
6. **Stabilità dinamica:** non verificata.

**Misure hardware necessarie:**
1. **Pitch reale del corpo** in HOLD e in WALK FAST, con un inclinometro sulla piastra, contro la lettura IMU: dice
   se l'offset di 39 mrad è montaggio o assetto. Prima di qualunque prova a 40 mm/s.
2. **Frenata sotto carico** dell'UPPER anteriore e dell'anca posteriore: arresto da 1,5–2 rad/s con il robot appoggiato.
3. **Stacco reale del piede** a fine swing anteriore, in avanti a 40 e a 32 mm/s: video laterale o contatto.
4. **Margine misurato delle anche durante gli STOP** in avanti: il vincolo reale è lì (6–11 tick nel gemello).
5. **Massa e COM** del robot senza Jetson, come configurato.

## 8. Proposta per l'agente firmware

La patch è `gait_study/fast40/` → `patches/F40.patch` negli artefatti: 46 righe aggiunte su `Gait.h/.cpp` e
`SimpleMotion.h/.cpp`, generata da `make_fast40_patch.py`. La Motion operativa non va toccata finché P0 non è chiuso.

**1. Riga FAST**
- Periodo 2,75 s, duty 0,80, altezza 105 mm (invariata), lift 5 mm.
- Preset 80/95/110 mm.
- Rampe 0,35/0,30 e rampe dell'arto avanzante 0,19/0,70.
- sway x 4,5 mm, sway y 0,75 mm.
- Modulazione 1,2 mm in fase 0,265.
- Drive {1950, 220} e kFastTilt invariati.

**2. Legge**, tutto opzionale e identico bit per bit per SLOW e STANDARD:
- `leadSwingRamp/End`: rampe proprie dell'arto avanzante, cioè le anteriori in avanti e le posteriori in
  retromarcia;
- `surgeXM/surgePhase/surgeStrideM`: modulazione della velocità del corpo in `sway()`.
  - Scala con il passo in avanti e si annulla senza parte in avanti.
  - Entra con la rampa di partenza ed esce nel ciclo di decelerazione dello STOP.
  - È esclusa in retromarcia.
- I nuovi campi sono inclusi in `validGaitParameters`, `gaitRetarget`, `sameLawShape` e nel passo di forma.

**3. sway x proprio della FAST:** il `static_assert` che lo vincola uguale tra i modi diventa SLOW == STANDARD.

**4. Spec sterzato FAST:** sway 4,5/5,5 mm; laterale 12 mm e imbardata 0,08 rad per ciclo invariati.

**Da implementare prima di qualsiasi hardware** (non nel prototipo):
- **a. Transizione nei cambi di modo dentro la camminata.**
  - sway x e ampiezza della modulazione vanno portati a rampa limitata, solo a quattro appoggi, come già
    `swayYM` in `stepWalkMode` e `gaitRetarget`.
  - Le rampe dell'arto avanzante vanno cambiate al passo di forma.
  - Senza questo, il servo plant MODE_CHANGES va in fault e una FAST raggiunta per cambio modo cammina con
    sway x 3 mm, sotto il criterio di cattura.
- **b. Ricertificazione di kSteerFast** (`certify_steer_box.py`) sulla nuova legge sterzata.
- **c. Retromarcia a 58 mm per ciclo** (opzionale, per non scendere sotto i 20 mm/s): gate `--reverse` già PASS.
- **d. Laterale e imbardata:** il rapporto lift/sway sterzato va rivisto con il gate sterzato e il gemello. Lift
  5,5 aiuta ma va riconciliato con la prova STOP, ad esempio rampa dell'arto avanzante 0,18/0,70 con modulazione
  1,3 mm.

**Verifiche da ripetere sulla patch** (tutte già PASS qui, salvo dove indicato):
- `walk_support_check`, `walk_tilt_envelope_check` (3 preset, `--reverse` 50/58), `run_joystick_envelope.sh`
  JOY_MODE=2;
- `run_simple_motion_tests.sh`, `run_servo_plant.sh` (MODE_CHANGES dopo il punto a);
- `stop_matrix`/`perturbation_matrix`: non eseguiti qui;
- `motion_cost_probe` per il costo della legge, a cura dell'agente P0.

**Benefici rispetto alla dev.7:**
- FAST della legge da 32 a **40 mm/s**; nel gemello da 21,9 a 32,6 mm/s reali.
- STOP verificati nel gemello 28/28 contro 25/28.
- Margine RAW per release 2 tick contro 0.
- Supporto e cattura a 40 mm/s ≥ quelli della dev.7 a 32 mm/s; margine anca 12,4 contro 11,1 tick.
- Sensibilità statica all'errore di COM migliore.

## 9. Riproducibilità

Artefatti: `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_FAST40_20261112/`, con `CHECKPOINT.md` e `SHA256SUMS.txt`.
Strumenti: `06_Software/Matdog_MuJoCo/gait_study/fast40/` sul branch locale `sim/mujoco-dev7-correlation-20261112`.

| Strumento | Uso |
|---|---|
| `grid.py`, `tilt_rows.py` | Griglie per velocità e leve (`grid/`) |
| `stop_rca.cpp` (`build_rca.sh`) | Prova StopPlan del firmware più una riga diagnostica per fallimento (`rca/*.csv`) |
| `variants40.sh` | Varianti mirate a 40 mm/s (`rca/v1`…`v7`) |
| `make_fast40_patch.py` | Patch del candidato (`patches/`); `build_variant.sh` costruisce export, ponte e `SOURCE.txt` |
| `run_twin.sh` | Gemello sugli scenari `fast_*` di `m6_gait.py` (`twin/final`, `twin/sens_*`); verifica che la perturbazione arrivi al modello |
| `twin_stop_rca.py` | STOP dentro lo swing anteriore (`rca/twin_stop_*.json`) |

`superseded/` contiene i prototipi scartati, ciascuno con il motivo:
- modulazione non scalata: fault in sterzata;
- spec sterzato della dev.7: 9 fault nel gate;
- modulazione congelata al terminale: salto C1.

`twin/_invalid_sens_no_perturbation/` è una sensibilità non valida, conservata solo come traccia del difetto.
