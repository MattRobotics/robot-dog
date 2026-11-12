# MATDOG — FAST 40 mm/s, fase 2: transizioni, stacco del piede, laterale e imbardata (13/11/2026)

Solo controlli offline e simulazione: nessun hardware, flash, push o merge. Motion dev.7 `b5610620d0ce`,
box, protezioni, ACC, soglie, criteri STOP e calibrazioni sono invariati. Il candidato è una patch su una
copia esportata della dev.7, come in fase 1 (`REPORT_FAST40.md`).

Il mandato ha cinque punti:
1. risolvere le regressioni di F40;
2. capire lo stacco del piede;
3. migliorare laterale e imbardata;
4. confrontare il candidato con la dev.7 nelle stesse condizioni;
5. valutare in breve il trotto.

Obiettivo invariato: FAST 40, STANDARD 30, SLOW 20 mm/s.

## 0. Esito

- **Candidato F40T_JCB66** (`patches/F40T_JCB66.patch`) = F40 più tre modifiche:
  - **T**, la transizione fra le leggi nei cambi di modo (§2);
  - **JC**, i passi sterzati FAST da 16 mm e 0,10 rad per ciclo invece di 12 mm e 0,08 rad (§4.1);
  - **B66**, il passo all'indietro FAST da 66 mm per ciclo invece di 50 (§4.2).

  La riga FAST, cioè la legge in avanti, è quella di F40.
- **Gate firmware su F40T_JCB66: tutti PASS.**
  - `walk_support_check` e `walk_tilt_envelope_check`: 3 preset e retromarcia a 50, 58 e 66 mm per ciclo.
  - Gate sterzato: 45 scenari, supporto minimo 0,38 mm (dev.7: 0,34 mm).
  - Test host della Motion: exit 0.
  - Servo plant: 48/48 righe senza fault, cambi di modo inclusi.

| Regressione di F40 | Stato | Nel gemello |
|---|---|---|
| Fault nei cambi di modo | **risolta** | — |
| Indietro | **risolta** | 19,9 mm/s contro 18,1 della dev.7 |
| Laterale | **risolta** | 1,64 / 2,71 mm/s contro 1,38 / 2,83 (sx +19 %, dx −4 %) |
| Imbardata | **ridotta da −42 % a −12 %** | 0,81 °/s contro 0,92 |
| Stabilità di rotta in avanti | **aperta** | dipende dall'attrito (§3.4) |
| Stacco del piede | **non risolvibile sul gemello** | serve una misura fisica |

Più in dettaglio:
- **Valori comandati:** laterale, imbardata e indietro sono ora sopra la dev.7.
- **Stacco del piede (§3):**
  - il ritardo dei giunti in swing, che nel gemello tiene il piede a terra, coincide con quello misurato
    sulla dev.7;
  - l'avanzamento simulato però dipende dall'attrito del piede, un parametro non validato.
- **Proposta all'agente firmware (§7):**
  - la **transizione** è pronta: inerte sulla tabella dev.7, verificato byte per byte;
  - la **riga FAST e gli spec joystick** restano subordinati alla misura dello stacco reale e dell'attrito
    del pavimento.
- **Trotto:** valutazione di fattibilità in §6; nessuna implementazione.

## 1. Le quattro categorie richieste

| Categoria | Contenuto |
|---|---|
| **1. Verificato offline** | Gate firmware di F40T_JCB66 (§2, §4). Scansione densa degli STOP nei cambi di modo e a regime, 10 188 per candidato (§2.3). Cinematica della legge: picchi articolari, passo RAW, margini (§5). Inerzia della transizione sulla tabella dev.7 (§7.1) |
| **2. Regressioni risolte** | Fault dei cambi di modo (§2). Indietro e laterale, comandati e nel gemello (§4, §5). Imbardata ridotta da −42 % a −12 % |
| **3. Limiti del modello non identificabili** | Stacco reale del piede e trascinamento: il ritardo dei servo è confermato dall'HW, il cedimento del corpo e il contatto no (§3). Attrito del piede: governa avanzamento e deriva simulati (§3.4, §5.2). Cedimento statico delle anche negli STOP del servo plant (§2.3). Gemello a 5/7 STOP HW |
| **4. Verifiche hardware necessarie** | §7.3 |

## 2. Priorità 1 — transizione fra le leggi

### 2.1 Causa dei fault di F40

F40 copiava nel passo di forma della legge solo duty, lift e rampe. Lo sway in avanti `swayXM`, prima uguale
per i tre modi, restava quello di partenza, e la modulazione cambiava di colpo.

Traccia del servo plant (`p2/plant_F40_trace`, FAST→SLOW, seme 1):
- la camminata aveva già la forma SLOW (duty 0,80, lift 8 mm, rampa 1/3) ma ancora lo sway in avanti FAST di
  4,5 mm, mentre la SLOW è certificata a 3,0;
- a c 4,467, con l'RF a u = 0,83 di fine swing anteriore, il passo RAW di RF_UPPER cresce fino a 17 tick per
  release;
- il controllo `STOP_PLAN` rifiuta l'arresto: `WALK_IK_OR_WORKSPACE[STOP_PLAN]`, 5–6 fault su 6 semi.

Nel verso SLOW→FAST la modulazione si accendeva al passo di forma. Lì sin(4π(s − 0,265)) vale ±0,19, cioè un
salto di posizione di 0,22 mm e di velocità di circa 5 mm/s. La FAST girava inoltre con lo sway in avanti
della SLOW.

### 2.2 La modifica (`transition=1` di `make_fast40_patch.py`)

Segue la regola già scritta in `stepWalkMode`: altezza e sway fra due leggi si percorrono con la forma della
legge più veloce al clock della più lenta.

- `swayXM` si avvicina al valore di destinazione a `kWalkModeSwayRate`, 4 mm per ciclo, solo a quattro
  appoggi, come lo sway laterale.
- `surgeXM` si avvicina alla stessa velocità.
- Entrambi si muovono:
  - **prima** del passo di forma se la destinazione è più lenta;
  - **dopo** il passo di forma se è più veloce.
- Il clock prende il ritmo della destinazione solo quando anche questi due valori sono arrivati.
- `gaitRetarget` accetta la differenza dei due valori. Come per lo sway laterale, sono offset nel riferimento
  di direzione: nessun piede appoggiato e nessuna traiettoria di swing cambia.
- `sameLawShape` li ignora, quindi nessuna release ha due IK complete in più (memoria D1).
- Uno `static_assert` garantisce che solo il modo più veloce abbia la modulazione. Fase e scala della
  modulazione cambiano quindi solo con ampiezza zero.

Dimensioni: 26 righe in più di F40.

Cosa resta invariato rispetto a F40:
- le leggi a regime;
- l'output di `walk_support_check` e `walk_tilt_envelope_check` (10 file confrontati, identici byte per byte);
- il riepilogo del gate sterzato.

Il tutto compila con `-Wall -Wextra -Werror`.

### 2.3 Risultati

| Prova | F40 | F40T e F40T_JCB66 |
|---|---|---|
| `run_servo_plant` MODE_CHANGES (6 semi, 11,1 e 9,6 V) | SLOW 4+4 fault, FAST 5+6 | **0 fault**, STOP 12/12 per ogni modo e tensione |
| `run_servo_plant`, matrice completa | 44/48 righe senza fault | **48/48** |
| Test host: cambio di modo con preset | PASS | PASS; salto di velocità della legge ≤ 0,352 mm/s per release, passo RAW ≤ 29 tick (bound a regime 29) |
| Durata dei cambi (test host) | STANDARD→FAST 3,24 s, FAST→SLOW 9,84 s | 6,08 s, 12,62 s: in più il tempo di sway e modulazione |
| STOP in 6 istanti di ogni cambio (test host, 36 STOP) | verificati | verificati |
| Ciclo dei modi nel gemello (SLOW→STANDARD→FAST→SLOW) | nessun fault | nessun fault |

**Scansione densa degli STOP** (`mode_stop_sweep.inc`, `p2/mode_sweep/`).
- Usa lo stesso modello servo del firmware.
- Copre 6 coppie di modi × 6 semi × 2 tensioni, con uno STOP ogni 0,3 s da 0 a 24 s dopo la selezione:
  5 832 STOP.
- A modo fisso, uno STOP ogni 0,1 s per 12 s: 4 356 STOP.
- I cambi di modo con preset non usano lo spec joystick: F40T e F40T_JCB66 coincidono.

| STOP con fault | dev.7 | F40 | F40T |
|---|---|---|---|
| Durante il cambio | 3 | 12 | 8 |
| A cambio concluso | 176 (166 dopo FAST→SLOW) | 395 (`STOP_PLAN` dopo FAST→SLOW) | 22 |
| A modo fisso SLOW / STANDARD / FAST | 4 / 8 / 25 | 4 / 8 / 2 | 4 / 8 / 2 |

Tutti i fault residui, di **F40T e della dev.7**, sono `STOP_MEASURED_OUTSIDE_BOX` sull'anca LH, misurata
0,7–4,7 tick fuori dal box. Due situazioni:
- lo STOP chiesto nella finestra a quattro appoggi subito dopo il touchdown anteriore (c mod 1 = 0,02–0,07),
  dove gira il trasferimento di sway;
- semi con cedimento statico modellato alto (`sagScale` ≈ 1,5).

Esempio dalla dev.7, seme 3, dopo FAST→SLOW:
- l'anca LH è comandata e arriva a −7 tick;
- il cedimento modellato vale 10,4 tick;
- la lettura risulta −17 contro un box di ±16,3.

È lo stesso vincolo, l'anca posteriore, che il gemello indicava come il più stretto in fase 1. **È un limite
della dev.7, non introdotto da F40T.**

Il gate `MODE_CHANGES` del firmware prova 2 STOP per seme e non lo vede (§7.2).

## 3. Priorità 2 — perché il piede si stacca appena

### 3.1 Scomposizione (`foot_lift_rca.py`, `p2/foot_lift/`)

Camminata in avanti a velocità piena, gemello `dev7_ground`, finestra a regime. Per ogni swing e ogni campione
da 1 ms, la quota del punto di contatto, cioè il punto più basso del cilindro del piede, la stessa
`referenceM` del firmware. Cinque versioni:
- **legge:** la quota comandata dalla legge;
- **nom:** la cinematica diretta MuJoCo dei giunti comandati, sul corpo nominale (quota di legge, livellato);
- **cmd:** gli stessi giunti sul corpo simulato;
- **reale:** il piede del gemello.

Valori in mm, picco per swing, mediana:

| | Legge | Nom | Cmd | Reale | Corpo (dz, pitch) | Ritardo a metà swing | Quota di swing caricata |
|---|---|---|---|---|---|---|---|
| dev.7 anteriori | 5,9 | 1,9 | 2,4–2,6 | **0,05–0,07** | −2,6 mm, −22 mrad | UPPER 104–108 tick | 0,55 |
| dev.7 posteriori | 5,9 | 9,8–10,8 | 3,0–3,2 | **0,9–1,3** | −1,4 mm, −36…−39 mrad | LOWER 65 tick | 0,53 |
| F40T anteriori | 5,0 | 0,5–1,0 | 3,7–3,8 | **0,06–0,07** | −1,3 mm, −28…−30 mrad | UPPER 153–160 tick | 0,58 |
| F40T posteriori | 5,0 | 9,1–10,4 | 2,0–2,3 | **0,14–0,17** | −1,6 mm, −35…−39 mrad | LOWER 64–67 tick | 0,39 |

- **Coerenza cinematica.** Senza offset IMU, i piedi in appoggio ricostruiti sul corpo nominale stanno a
  0,01–0,02 mm dal piano. La cinematica diretta del gemello e la legge coincidono: la traiettoria comandata è
  quella voluta.
- **Correzione IMU.**
  - L'offset di 38,9 mrad fa correggere un pitch inesistente di −30 mrad.
  - Sul corpo nominale il comando abbassa di circa 4 mm il piede anteriore e alza il posteriore.
  - Il corpo poi si inclina davvero, quindi sul corpo simulato l'effetto si compensa.
  - Senza offset: F40T 5,0 → 4,7 → 2,4 → 0,10 mm; dev.7 6,0 → 5,4 → 2,6 → 0,38 mm.
- **Corpo:** 1,3–2,6 mm più basso della legge e inclinato verso l'angolo in swing. Costa 2–3 mm.
- **Servo:** il giunto in swing resta indietro, l'UPPER anteriore di 100–165 tick e il LOWER posteriore di
  55–70. Il piede resta caricato per il 40–58 % dello swing: trascina.
- **Perché F40T stacca meno della dev.7:** sulle posteriori, con 1 mm di lift in meno e il comando reale a
  2,0–2,3 mm invece di 3,0–3,2. Sulle anteriori nessuna delle due leggi stacca.

### 3.2 Il ritardo dei servo è reale (`swing_tracking_hw.py`, `p2/swing_tracking_hw/`)

Rifaccio nel gemello i 16 episodi a terra della sessione HW dev.7, osservati come M6: ogni giunto alla sua età
di lettura. Poi confronto l'errore misurato − comandato per fase della gamba al momento della lettura, su
3 700 campioni.

| Giunto, fase | n | Mediana HW / gemello | \|err\| p90 HW / gemello |
|---|---|---|---|
| FAST, UPPER anteriore, metà swing | 12 | 60 / 61 tick | 198 / 187 |
| Tutti i modi, UPPER anteriore, metà swing | 56 | 22 / 22 | 146 / 136 |
| Tutti i modi, LOWER posteriore, metà swing | 50 | −5 / −9 | 74 / 47 |
| FAST, LOWER posteriore, inizio swing | 5 | 55 / −2 | 76 / 27 |
| Tutti i modi, LOWER posteriore, appoggio | 498 | −8 / −12 | 29 / 41 |

- **Ritardo in swing:** il gemello riproduce quello del robot. Il motivo principale per cui il piede resta
  basso nel gemello vale anche sull'HW.
- **LOWER posteriore al distacco:** sull'HW ritarda più che nel gemello, ma su soli 5 campioni.
- **Cedimento in appoggio:** nel gemello è leggermente maggiore, quindi il gemello è pessimista sulla quota
  posteriore.
- **Cosa resta non identificabile:** l'altezza reale del piede, perché i log HW non hanno né quota del piede
  né forza.

### 3.3 Profili di swing valutati

Profilo verticale (`liftRise/liftFall`, `lift_profile=rise,fall`), sviluppato da A6 di M6: salita e discesa
smootherstep, piatto in mezzo, stessa altezza massima.

| Variante | Gate firmware | Effetto nel gemello |
|---|---|---|
| Su tutte le zampe, salita 0,20–0,35 (24 varianti, `p2/lift/superseded_alllegs`) | **FAIL** a 40 mm/s: l'UPPER anteriore arriva a 31–32 tick per release, arresto 0,30 s, 2 067–6 716 violazioni StopPlan; amax 31–41 rad/s² contro 27 | non provato |
| Solo zampe non avanzanti (`lift_legs=trail`), salita 0,25 | FAIL: LOWER posteriore 28,4 rad/s² a p1 | non provato |
| Solo zampe non avanzanti, salita ≥ 0,30 (6 varianti) | **PASS**, margini identici a F40T | picco posteriore 0,15–0,31 mm invece di 0,14–0,17; quota caricata 0,29–0,35 invece di 0,39; laterale e imbardata leggermente peggiori |

- **Zampe anteriori:** a 40 mm/s il profilo non si può accelerare. L'UPPER anteriore è il collo di bottiglia
  sia del passo sia dello StopPlan.
- **Zampe posteriori:** il profilo passa i gate, ma il guadagno è sotto il decimo di millimetro.

**Non lo adotto.** Il limite non è la forma comandata ma il ritardo dei servo e il cedimento del corpo,
come già indicava M6 per l'altezza dello swing.

### 3.4 Il contatto governa l'avanzamento simulato (`p2/twin_drift/`, `p2/twin_final/mu*`)

| Avanti, gemello | μ 0,6 | μ 0,8 (default) | μ 1,0 |
|---|---|---|---|
| F40T: velocità, deriva | 39,9 mm/s, −0,11 °/ciclo | 32,6 mm/s, −1,98 °/ciclo | **−6,6 mm/s**, −0,85 °/ciclo; duty UPPER p95 1 000 ‰ |
| dev.7 | 28,8 mm/s, −0,12 | 21,9, −0,39 | 12,6, −0,14 |

**Effetto dell'attrito.** Con piedi in swing che trascinano, l'attrito decide quanto il robot avanza. F40T è
più sensibile della dev.7: a μ 1,0 arretra con l'UPPER in saturazione.

**Deriva in avanti.** La deriva di −2 °/ciclo è instabile:
- massa ±10 %: −0,2 / −0,6 °/ciclo;
- COM ±5 mm: −0,6 / −3,0;
- lift 5,5 mm: −0,6;
- offset IMU in roll a zero: +1,4;
- offset IMU in pitch a zero: −0,7.

È un comportamento da soglia di trascinamento, non certificabile sul gemello. **Serve la misura fisica**
(§7.3).

## 4. Priorità 3 — laterale, imbardata e retromarcia

### 4.1 Sway sterzato e passi

Varianti sterzate di F40T (`p2/twin_steer/`, `p2/joy/`; gemello, stick pieno):

| Variante (per ciclo) | Gate sterzato | Laterale sx / dx | Imbardata |
|---|---|---|---|
| dev.7 FAST: 12 mm, 0,08 rad, sway 5/7,5 | (dev.7) PASS | 1,43 / 2,87 mm/s | 0,92 °/s |
| F40T: 12, 0,08, sway 4,5/5,5 | PASS (0,72 mm) | 0,80 / 1,85 | 0,53 |
| JA: sway 5/6,5 | PASS (J1 di fase 1) | 0,64 / 1,96 | 0,51 |
| JB: sway 5/7,5 | FAIL (9 fault in fase 1); fault anche nel gemello | fault / 2,05 | fault |
| **JC: 16 mm, 0,10 rad, sway 4,5/5,5** | **PASS, 0,38 mm** | **1,64 / 2,71** | **0,81** |
| JD: 18 mm, 0,11 rad | FAIL: supporto −0,43 mm | — | — |
| JE: 20 mm, 0,12 rad | FAIL: supporto −0,95 mm | — | — |

- **Sway sterzato:** più sway non aiuta. Il trascinamento del piede in swing non dipende da lì.
- **Passi più ampi:** aiutano, e sono anche più efficienti (laterale dal 21–37 % al 31–42 % della legge;
  imbardata dal 32 % al 39 %).
- **Scelta:** JC è il massimo che passa il gate.

Rispetto alla dev.7, i valori comandati di JC sono:
- **laterale:** 5,82 mm/s contro 4,80;
- **imbardata:** 2,08 °/s contro 1,83;
- **margine dalla box sterzata certificata `kSteerFast`:** almeno 19,1 mrad su tutti i 45 scenari, IMU
  inclusa (dev.7: 17,0). Non serve ricertificare la box.

### 4.2 Retromarcia

FAST in retromarcia, controlli offline del firmware:

| Passo per ciclo | Supporto | Cattura | Accelerazione |
|---|---|---|---|
| 50 mm | 2,30 | 1,97 | 24,7 |
| 58 mm | 2,25 | 1,92 | 25,0 |
| 62 mm | 2,22 | 1,90 | 25,1 |
| 66 mm | 2,18 | 1,87 | 25,3 |
| 70 mm | 2,15 | 1,85 | 25,4 |

Supporto e cattura in mm, accelerazione in rad/s². Tutti con 0 violazioni.

| Variante | Gate sterzato | Gemello indietro | STOP all'indietro |
|---|---|---|---|
| F40T_JC, 50 mm | PASS | 13,9 mm/s (78 %) | verificato |
| **F40T_JCB66, 66 mm** | **PASS** | **19,9 mm/s (84 %)** | verificato in 2,44 s |
| F40T_JCB70, 70 mm | PASS | 21,5 mm/s (86 %) | verificato in 2,20 s |
| dev.7, 50 mm a 2,5 s | PASS | 18,1 mm/s (90 %) | verificato |

**Scelta: 66 mm.** Recupera la dev.7 con margine (+10 %) e tiene le velocità articolari più basse di 70 mm:
UPPER p95 731 contro 773 step/s nel gemello.

Verifiche su F40T_JCB66:
- servo plant: `JOY_BACK_STOP8` 96/96 STOP verificati e `JOY_BACK_MODES` senza fault;
- test host: indietro 24,0 mm/s comandati, 9 direzioni × 6 STOP verificati.

### 4.3 Rollio controllato del corpo

Il punto di aggancio naturale è la correzione IMU: `compensateLevel` applica già roll e pitch al corpo. La
prova d'inviluppo certifica qualunque correzione dentro il poligono `kFastTilt`, quindi un rollio aggiunto
dentro il poligono sarebbe già coperto dalle prove StopPlan.

Il poligono però è già consumato:
- a pitch −30 mrad, dove la FAST lavora a causa dell'offset IMU, ammette circa **±2,4 mrad** di roll;
  il vertice è a −34,2 mrad con roll 0 e il punto (±8,2; −19,8);
- con il pivot al suolo del firmware, 2,4 mrad spostano il baricentro di ≤ 0,25 mm e alzano l'anca in swing
  di ≤ 0,15 mm;
- lo sway laterale attuale vale 4,5–5,5 mm;
- anche a pitch zero, con ±11,8 mrad, il baricentro si sposterebbe di circa 1,2 mm e l'anca salirebbe di
  0,7 mm.

**Non conviene, e non lo implemento.** Ha senso riconsiderarlo solo dopo aver misurato e corretto l'offset
IMU, che oggi consuma il budget dell'inviluppo.

## 5. Priorità 4 — F40T_JCB66 contro dev.7 nelle stesse condizioni

Stesso gemello corretto (`dev7_ground`), stessi scenari `fast_*` a stick pieno sugli intervalli pubblicati di
ciascuna legge:
- dev.7 ed F40 vengono da `twin/final` di fase 1;
- F40T_JCB66 da `p2/twin_final/`.

F40T_JCB66 differisce da F40T_JC solo nel passo all'indietro. Quindi:
- gli scenari senza componente all'indietro sono quelli di F40T_JC;
- `fast_back`, `fast_back_stop` e `fast_sweep` sono rifatti su F40T_JCB66.

Il gemello resta a 5/7 STOP HW: i valori sotto servono al confronto, **non certificano nulla**.

### 5.1 Nominale (μ 0,8)

| Grandezza | dev.7 | F40 | **F40T_JCB66** |
|---|---|---|---|
| Avanti, velocità utile (quota della legge) | 21,9 mm/s (68 %) | 32,6 (81 %) | **32,6 (81 %)** |
| Deriva di rotta in avanti | −0,39 °/ciclo | −1,98 | −1,98 (§3.4) |
| Indietro | 18,1 mm/s (90 %) | 13,9 (78 %) | **19,9 (84 %)** |
| Laterale sx / dx | 1,38 / 2,83 mm/s | 0,80 / 1,85 | **1,64 / 2,71** |
| Laterale utile per ciclo sx / dx | 3,6 / 7,2 mm | 2,2 / 5,1 | 4,5 / 7,5 |
| Imbardata | 0,92 °/s (50 %) | 0,53 (32 %) | 0,81 (39 %) |
| Diagonale / diagonale con imbardata | 7,7 / 7,8 mm/s | 7,3 / 7,3 | 7,2 / 7,4 |
| Ciclo dei modi, media | 13,6 mm/s | 17,5 | 17,3 |
| Oscillazione roll/pitch p2p in sterzata | 29–31 / 20–24 mrad | 20–22 / 15–18 | **20–21 / 14–19** |
| Escursione laterale p2p in sterzata | 17,1–17,5 mm | 12,5–12,9 | **13,0–13,8** |
| Escursione laterale p2p in avanti | 10,8 mm | 38,9 (deriva) | 38,9 (deriva) |
| Stacco p50 avanti / laterale | 0,79 / 0,63 mm | 0,10 / 0,33 | 0,10 / 0,40 |
| Quota di swing trascinata, avanti / laterale | 0,55 / 0,62 | 0,47 / 0,66 | 0,47 / 0,70 |
| Scivolamento massimo, avanti / laterale | 5,9 / 1,2 mm | 9,3 / 2,4 | 9,3 / 2,3 |
| Giunti p95 avanti, velocità UPPER / LOWER | 962 / 478 step/s | 985 / 617 | 985 / 617 |
| Giunti p95 avanti, duty UPPER / LOWER | 489 / 497 ‰ | 614 / 523 | 614 / 523 |
| Legge (offline): vmax, amax | 2,350 rad/s, 26,39 rad/s² | 2,138, 26,78 | 2,138, 26,78 |
| Legge: passo RAW, margine box kHeight compensato | 31 tick, 10,3 tick | 29, 6,18 | 29, 6,18 |
| Legge: supporto / cattura al preset massimo | 1,71 / 1,41 mm | 1,95 / 1,49 | 1,95 / 1,49 |
| Durata dello swing | 0,44 s | 0,55 s | 0,55 s |
| STOP in 28 fasi | 25/28 | 28/28 | **28/28** |
| Cambi di modo, servo plant del firmware | 0 fault | 19 fault (SLOW 8, FAST 11) | **0 fault** |
| Cambi di modo, scansione densa (5 832 STOP) | 179 fault | 407 | **30** |

Le 28 fasi degli STOP sono: 8 nel ciclo, 16 densi, e poi avanti, indietro, imbardata e diagonale.

### 5.2 Perturbazioni (`p2/twin_final/sens_*`, `mu*`)

| Perturbazione | Avanti dev.7 / candidato | Laterale sx/dx dev.7 / candidato | Imbardata dev.7 / candidato | STOP in 8 fasi dev.7 / candidato |
|---|---|---|---|---|
| Massa +10 % | 21,2 / 26,9 mm/s | 1,97/2,05 / 1,30/2,52 | 0,85 / 0,75 | 7/8 / 7/8 |
| Massa −10 % | 22,7 / 33,9 | 2,26/2,14 / 1,87/2,99 | 0,96 / 0,97 | 7/8 / 8/8 |
| COM x +5 mm | 21,2 / 25,8 (deriva −3,0 °/ciclo) | 1,47/2,96 / 1,59/2,66 | 0,95 / 0,79 | 7/8 / 8/8 |
| COM x −5 mm | 22,6 / 34,7 | 1,24/2,73 / 1,73/2,69 | 0,91 / 0,84 | 7/8 / 8/8 |
| μ 0,6 | 28,8 / 39,9; indietro 18,9 / 21,8 | 1,67/3,12 / 2,03/3,09 | 1,01 / 0,93 | 4/4 / 4/4 |
| μ 1,0 | 12,6 / **−6,7**; indietro 16,8 / 14,9 | 1,11/2,56 / 1,41/2,42 | 0,83 / 0,70 | 2/4 / 2/4 |

Totale STOP con massa e COM: dev.7 28/32, candidato 31/32.

**Lettura.**
- In laterale il candidato è comparabile alla dev.7 in ogni condizione.
- In avanti e all'indietro è più veloce, tranne con l'attrito alto: a μ 1,0 avanti −6,7 contro 12,6 mm/s,
  indietro 14,9 contro 16,8.
- L'imbardata resta fino al 17 % sotto la dev.7, in media circa il 10 %.
- Il corpo oscilla meno in sterzata.
- Gli STOP sono più robusti.
- I punti deboli sono gli stessi in tutte le prove: piedi che non staccano e sensibilità all'attrito.

### 5.3 Spot e Go2 come riferimento qualitativo

- **Gestione del corpo.** Quei robot tengono il corpo quasi fermo in assetto. Il candidato si avvicina a quel
  comportamento: in sterzata l'oscillazione di roll/pitch scende da 30/24 a 20/18 mrad e l'escursione laterale
  da 17 a 13 mm. Una camminata statica però deve comunque spostare il baricentro sopra il triangolo di
  appoggio.
- **Piazzamento del piede.** La legge integra il comando fino a metà appoggio del passo successivo
  (`GaitTrack`): è un piazzamento in anticipo come il loro, ma senza retroazione.
- **Fluidità.** La differenza netta è lo swing: loro sollevano il piede di centimetri con traiettoria chiara.
  Qui il gemello e il ritardo misurato sulla dev.7 indicano un piede che sfiora o trascina. È la prima cosa da
  misurare.

## 6. Trotto a due appoggi diagonali: fattibilità

Valutazione distinta dalla qualifica della FAST WALK. Nessuna implementazione.

- **Cosa esiste già.**
  - `Gait.cpp` ha `GaitType::TROT`: duty ≥ 0,5, sfasamenti {0; 0,5; 0; 0,5}, sway vietato.
  - È una legge cinematica, senza equilibrio. `SimpleMotion` non la usa e nessun gate la copre.
- **Equilibrio.** Su due piedi diagonali il poligono di appoggio è un segmento: margine statico ≤ 0.
  - I gate della camminata (supporto ≥ 1,70 mm, cattura ≥ 1,41 mm) non sono applicabili per costruzione.
  - Con COM a circa 0,105 m, τ = √(h/g) ≈ 0,10 s. Uno scarto laterale cresce di cosh(T/τ) per ogni fase a
    due appoggi: 2,3× in 0,15 s, 5,6× in 0,25 s, 23× in 0,4 s.
  - Servono fasi brevi, a 2,5 Hz o più, e la correzione del piazzamento del piede a ogni passo (tipo Raibert),
    cioè un controllore di equilibrio dinamico che oggi non c'è.
- **Attuatori.**
  - Sono servo di posizione con profilo, a 50 Hz.
  - Ritardo in swing misurato sulla dev.7 (FAST a 32 mm/s): 60 tick di mediana e 198 di p90 sull'UPPER
    anteriore, cioè 0,09–0,30 rad, quanto un passo intero di trotto.
  - Un trotto a 100 mm/s con swing di 0,2 s porta l'UPPER oltre 2 rad/s in circa 0,05 s, cioè circa
    40 rad/s², sopra il limite di legge di 27.
  - Un trotto lento (≤ 40 mm/s) rientra nei limiti articolari ma non risolve l'equilibrio.
- **Sensori.**
  - L'IMU (BNO085) è accettata fino a 200 ms di età e la correzione è limitata a 40 mrad/s: è un livellamento
    lento.
  - Un trotto richiede assetto a ≥ 100 Hz con latenza ≤ 10 ms e correzioni di decine di mrad in circa 0,1 s.
  - Non c'è rilevamento del contatto; il proxy dal carico dei servo ha accuratezza 0,66 (M6).
- **STOP.**
  - Lo StopPlan arresta i giunti dove sono, assumendo stabilità statica in ogni istante.
  - Su due appoggi un arresto fa cadere il robot. Uno STOP del trotto deve completare il passo fino a quattro
    appoggi: nuova semantica, nuove prove.
- **Gemello.** Riproduce 5/7 STOP HW, non giudica margini sotto il millimetro, e l'avanzamento dipende
  dall'attrito (§3.4). Non può certificare l'equilibrio dinamico.
- **Riferimenti.** Spot e Go2, solo qualitativi: attuatori ad alta banda con controllo di coppia, stima dello
  stato a centinaia di Hz, controllo predittivo con pianificazione delle forze di contatto. Il trotto è il
  loro passo di base perché l'architettura è dinamica; MATDOG oggi è una piattaforma cinematica a camminata
  statica.

**Esito: non fattibile come estensione della FAST WALK.** Prerequisiti, nell'ordine:
1. identificazione HW dei servo a 1 000–2 000 step/s, sotto carico, con banda e ritardo;
2. latenza e frequenza IMU misurate;
3. stima del contatto;
4. gemello validato su eventi a due appoggi, con attrito misurato;
5. progetto di uno STOP dinamico e di gate dinamici propri (ZMP o punto di cattura), mai i gate statici della
   camminata a tre appoggi.

## 7. Proposta circoscritta per l'agente firmware

### 7.1 Pronto: la transizione dei cambi di modo (parte `transition` della patch)

- **File:** `Gait.h/.cpp` (`gaitRetarget`) e `SimpleMotion.cpp` (`stepWalkMode`, `sameLawShape`,
  `static_assert`).
- **Indipendenza:** non dipende dalla riga FAST di F40. È verificato con `patches/D7T.patch`, cioè la tabella
  dev.7 più la transizione e i campi nuovi a zero. Rispetto alla dev.7 non modificata sono identici:
  - i 10 output di `walk_support_check` e `walk_tilt_envelope_check`;
  - la matrice `run_servo_plant`;
  - i test host, a parte la riga del tempo host per frame, che non è deterministica.
- **Beneficio:** con la riga F40 elimina i fault MODE_CHANGES del servo plant.
- **Costo:** 3–4 s in più per entrare in o uscire da FAST.
- **Verifiche:** quelle di §2.3.

### 7.2 Osservazione sulla dev.7 operativa (nessuna modifica proposta)

Nel modello servo del firmware, una scansione densa degli STOP trova `STOP_MEASURED_OUTSIDE_BOX` sull'anca LH
anche nella dev.7:
- 25 su 1 452 STOP in FAST a regime;
- 166 dopo FAST→SLOW.

La causa è l'anca in appoggio a circa −7 tick più il cedimento modellato fino a 10,5 tick, contro un box di
±16,3. Il gate `MODE_CHANGES` usa solo 2 STOP per seme.

**Proposta:** misurare il cedimento reale delle anche posteriori sotto carico prima di qualificare qualunque
riga FAST, e valutare una scansione densa degli STOP nel gate.

### 7.3 Subordinati a misure fisiche: riga FAST di F40 e spec joystick JC/B66

Il pacchetto è `patches/F40T_JCB66.patch`. Tutti i gate offline sono PASS. Il limite che resta non è
identificabile sul gemello. Misure necessarie, in ordine di priorità:

1. **Stacco reale del piede e trascinamento**, dev.7 FAST a 32 mm/s.
   - Ripresa laterale dei piedi, oppure export delle letture a 8,33 Hz in una finestra breve (M6).
   - Decide se il trascinamento del gemello (40–58 % dello swing) è reale.
2. **Attrito del piede sul pavimento d'uso.** Prova di scivolamento, o forza laterale a carico noto.
   - Con μ 1,0 il gemello fa arretrare la legge FAST di F40.
3. **Offset di montaggio IMU** (38,9 mrad). Oggi consuma tutto l'inviluppo pitch (F1 di fase 1) e il budget
   di roll.
4. **Frenata dei giunti sotto carico** e **cedimento delle anche** negli STOP (§2.3, §7.2).
5. **Rettilineità in avanti a FAST 40** su almeno 3 m, se si procede: il gemello prevede una deriva a soglia.

Se la misura 1 conferma il trascinamento anche sull'HW, la leva successiva non è la forma dello swing (§3.3):
sono la banda dei servo e il cedimento in appoggio, entrambi fuori dal perimetro di questo studio.

## 8. Riproducibilità

Artefatti: `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_FAST40_20261112/p2/`, patch in `patches/` con gli
argomenti esatti in `patches/ARGS.txt`. Strumenti in `gait_study/fast40/`:

| Strumento | Uso |
|---|---|
| `make_fast40_patch.py` (`transition`, `lift_profile`, `lift_legs`, `joy=…[,back]`) | Genera F40, F40T, F40T_JC, F40T_JCB66 e varianti; F40, F40T e F40T_JC si rigenerano identici |
| `build_plant.sh`, `mode_stop_sweep.inc` | Servo plant del firmware ricompilabile; scansione densa degli STOP nei cambi di modo e a regime |
| `offline_variant.sh` | Una variante attraverso i gate offline del firmware, con la regola di scelta di `SimpleMotion.h` |
| `foot_lift_rca.py` | Scomposizione della quota del piede (§3.1) |
| `swing_tracking_hw.py` | Ritardo in swing, HW dev.7 contro gemello (§3.2) |
| `run_twin.sh`, `battery_p2.sh`, `compare_p2.py` | Gemello e confronto (§5); `compare_p2.md` contiene le tabelle complete |
| `m6_gait.py --fast-joy` | Intervalli joystick FAST in prova |

Varianti scartate, ciascuna conservata con la sua cartella:
- profilo verticale su tutte le zampe e solo sulle posteriori (`p2/lift`);
- sway sterzato JA e JB, passi JD e JE (`p2/twin_steer`, `p2/joy`);
- retromarcia a 70 mm (`builds/F40T_JCB70`, `p2/twin_back`).

Nella fase 2 non c'è stato alcun risultato invalidato. Due problemi di strumentazione, tutti e due corretti:
- **Gate sterzato di JD/JE:** si era fermato al secondo stadio perché l'export non conteneva l'URDF. Lo stadio
  è stato rieseguito sugli stessi frame (`p2/joy/gate_J?_contact_rerun.txt`).
- **Prima griglia dei profili:** era incompleta, perché `set -e` interrompeva lo script al primo FAIL. È stata
  rilanciata.
