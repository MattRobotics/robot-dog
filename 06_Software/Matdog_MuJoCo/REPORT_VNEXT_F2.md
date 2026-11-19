# MATDOG — Locomozione vNext, prosecuzione mirata (13/11/2026)

**Ambito.** Gemello MuJoCo e strumenti offline del firmware.
- Nessun hardware, flash, push, merge, reset o clean.
- Firmware installato, NVS, box, STOP, calibrazioni e soglie dei gate invariati.
- Ogni legge candidata è una patch su dev.7 `b561062` + F40T_JCB66, generata da `gait_study/vnext/make_vnext_patch.py`.
- Le opzioni nuove del generatore, ai valori di default, rigenerano V6f20, C2 e T3 **byte per byte identici**.

Base di partenza: `REPORT_VNEXT_LOCO.md` (V6f20, VERIFIED OFFLINE, non qualificato per l'hardware).

Classi usate: **VERIFIED OFFLINE**, **IMPLEMENTED NOT VALIDATED**, **UNKNOWN**. Le prove "diagnostiche" usano copie strumentate
e non sono mai verdetti.

## 0. Esito

**In una riga.** Il candidato WALK diventa **V7**: V6f20 con **una** modifica, la compensazione resta attiva in sterzata e
si dissolve solo la sollevata.
- Avanti / indietro: invariato rispetto a V6f20.
- Sterzata nel gemello: da 21–65 % a **57–80 %** della legge.
- Gate non rigidi e servo plant: invariati e PASS.
- Restano due FAIL, che sono quelli dei gate rigidi, ora scomposti (§3).

La clearance posteriore FAST è limitata dalla **prova di arresto**:
- **V7L60** (lift posteriore 6 mm) la raddoppia, ma consuma 1 dei 3 tick/rilascio di margine rimasti;
- 6,5 mm supera il limite e va in fault.

Nessun handoff a Claude Motion: C sul robot resta UNKNOWN e i gate rigidi non sono risolti.

| Requisito | V6f20 | **V7** (candidato) | Opzione V7L60 | Prima causa verificabile di ciò che manca |
|---|---|---|---|---|
| WALK avanti 20 / 30 / 40 (μ 0,8) | 20,2 / 29,7 / 40,7 mm/s | identico (corse ripetute: bit per bit) | FAST 40,6 | — (a μ 1,0 FAST 39,0) |
| Clearance posteriore FAST al picco di legge (μ 0,6 / 0,8 / 1,0) | 0,7 / 0,5 / 0,4 mm | identica | **1,4 / 1,2 / 0,8** | corpo che si inclina verso l'angolo in swing (−10 mrad, −2,8 mm) + ritardo del LOWER posteriore (≈ 60 tick, −1,7 mm); il lift è limitato dal rateo ammesso dalla prova STOP (§1) |
| Laterale | 65 / 62 / 21–37 % (S / ST / F) | **80 / 77 / 60–67 %** | FAST come V7 | piede posteriore ancora caricato nel 46–62 % dello swing; la sollevata non è ammessa in sterzata dal box sterzato certificato (§2) |
| Diagonale | ST 56 %, F 49–52 % | **ST 77 %, F 68 %** | — | idem |
| Rotazione | ST 65 %, F 32 % | **ST 79 %, F 57 %** | — | idem |
| STOP (gemello) | 46/47 avanti + sterzati ok | identico; sterzati 3/3 ok | 8 a ottavi di ciclo come V6f20 | `fast_stopcyc_0` `STOP_MEASURED_OUTSIDE_BOX` (HIP LF 0,1 tick), invariato |
| Gate tilt+support / servo plant | PASS / 0 fault, 1296/1296 STOP | **PASS / 0 fault, 1296/1296** | PASS / 0 fault, 1296/1296 | — |
| Gate joystick | sequenza PASS, geometria FAIL | **sequenza PASS** (0 fault, 0 tilt_fail; margini di box 14,2 / 15,3 / 19,1 mrad e di tilt 3,2 / 14,3 / 9,5 mrad, uguali o migliori della dev.7); geometria FAIL | FAST uguale a V7 | modello rigido (§3.2) |
| Test host | FAIL solo `WalkBoardPlant` | **FAIL solo `WalkBoardPlant`** (stessi valori); tutte le altre asserzioni passano nella copia diagnostica | idem | modello rigido; con un banco cedevole fallisce anche la dev.7 (§3.1) |
| CRAWL C2 | prototipo | **conservato, invariato** | — | — |
| TROT T3 | non fattibile | **invariato, separato** | — | condizioni minime in §4 |

## 1. Clearance posteriore FAST

### 1.1 Prima causa verificabile

Scomposizione RCA 01 (`foot_lift_rca.py`, invariato) su V6f20 a μ 0,8, mediana degli swing posteriori all'istante di picco
della legge:
- **legge** 4,98 mm;
- **giunti comandati sul corpo simulato** 2,21 mm: **−2,8 mm per assetto e cedimento**. Durante lo swing posteriore il corpo
  sta a −37…−41 mrad di pitch, mentre la correzione IMU vuole −26…−31 mrad: circa **10 mrad verso l'angolo in swing**;
- **piede reale** 0,52 mm: **−1,7 mm di inseguimento**, con il LOWER posteriore in ritardo di ≈ 60 tick.

Picco reale 1,7 mm contro 2,35 mm dell'anteriore.

Senza l'offset di montaggio IMU (diagnostico, `imu_pitch_mrad=0`) il posteriore resta a 0,65 mm: **l'offset IMU non è la
causa**.

### 1.2 Modifiche provate (solo FAST, tutto il resto invariato)

| Variante | Gate tilt+support | Passo max compensato (gate) | Gemello | Esito |
|---|---|---|---|---|
| V6f20 (lift 5 mm) | PASS | 29 tick/rilascio | 0 rifiuti `STOP_PLAN` (cosim diagnostica) | riferimento |
| **V7L60**: lift posteriore 6,0 mm | PASS | 30 | velocità 41,2 / 40,6 / 39,0; clearance posteriore al picco di legge 1,4 / 1,2 / 0,8 mm; picco reale 2,6 mm; LOWER in ritardo 62 tick (invariato); nessun fault; STOP come V6f20 | **migliora, a margine ridotto** |
| V7L65: lift posteriore 6,5 mm | PASS | 31 | **fault `WALK_IK_OR_WORKSPACE` a 14,42 s in ogni corsa FAST a μ 0,8 che cammina oltre quell'istante** (21 STOP FAST interrotti dal fault prima dell'istante di STOP): la prova STOP rifiuta il riferimento livellato (LH_UPPER **2,454 rad/s**) | **respinta** |
| V7L75: lift posteriore 7,5 mm | **FAIL** (33 fallimenti della prova STOP) | 32 | — | respinta |
| Sway avanti FAST 6,0 / 7,5 mm (distribuzione del carico) | **FAIL**: amax 34–44 rad/s² (limite 27), 1656–13 237 fallimenti STOP | — | — | respinta |

**Vincolo.** `StopPlan::build` richiede un arresto con residuo ≤ 0,35 rad all'accelerazione di prova 12,885 rad/s² (drive
1950/220). Il rateo massimo ammesso è quindi √(0,35 · 12,885 / 0,75) = **2,452 rad/s = 31,97 tick/rilascio**.
- La legge FAST compensata usa già 29 tick (V6f20).
- In runtime il livellamento aggiunge circa 1–2 tick: V7L65 sta a 31 tick nel gate e a 2,454 rad/s nel gemello, quindi fault.
- Il lift posteriore massimo con margine positivo è quindi **≈ 6 mm**.
- La sollevata FAST a 3 mm era già esclusa (fault IMU_PITCH_VERTEX nel servo plant).
- Lo sway avanti è escluso dalla finestra a 4 appoggi da 0,1375 s.

**Raccomandazione.** V7L60 non peggiora velocità, esiti STOP, gate né servo plant, ma porta il margine della prova di arresto
da 3 a 2 tick/rilascio. Lo lascio come **opzione documentata, non inclusa nel candidato**: la scelta spetta all'operatore, e
va presa dopo aver verificato sull'HW il rateo reale dei riferimenti (rumore IMU).

## 2. Sterzata: il primo vincolo reale e la modifica minima

**Diagnosi.** Gate joystick strumentato (`build_joy_stopplan.sh`, `run_joy_stopplan.sh`, solo diagnostico), varianti senza
dissolvenza:

| Variante (senza dissolvenza in sterzata) | Fault `STOP_PLAN` m0 / m1 / m2 (45 scenari ciascuno) |
|---|---|
| compensazione + sollevata (V5) | 0 / 16 / 11 |
| **solo compensazione (V5h0)** | **0 / 0 / 0**; margini di box (14,2 / 15,3 / 19,1 mrad) e di tilt identici alla base dev.7 |
| solo sollevata 3 mm | 0 / 2 / 5 |

**Primo vincolo reale: la sollevata, non la compensazione.**
- Con la sollevata, i LOWER in appoggio (estesi di 3 mm) escono dal **box sterzato certificato** (`kSteer`) lungo la
  traiettoria di arresto: sconfinamenti fino a 36 tick.
- Il box è derivato dall'inviluppo della legge dev.7 e non si tocca.

**Modifica minima (V7, `compliance_steer_fade=0`, una riga).**
- La compensazione non si dissolve più in sterzata; la sollevata sì.
- La continuità resta quella di V6f20: fattore di ingresso e uscita e pesi di carico nelle finestre a 4 appoggi.

| Gemello, μ 0,8 | V6f20 | **V7** |
|---|---|---|
| Laterale SLOW / STANDARD | 65 / 62 % | **80 / 77 %** |
| Laterale FAST (sinistra / destra) | 21 / 37 % | **60 / 67 %** |
| Diagonale STANDARD / FAST | 56 / 49 % | **77 / 68 %** |
| Rotazione STANDARD / FAST | 65 / 32 % | **79 / 57 %** |
| Peso sulle zampe in swing (sterzata) | 0,08–0,11 | **0,03–0,04** |
| Rotazione indesiderata in laterale | 1,1–1,5 °/ciclo | 0,9–1,2 °/ciclo |
| STOP sterzati (`yaw_standard_stop`, `fast_yaw_stop`, `fast_diag_stop`) | ok | **ok** |

SLOW e STANDARD in sterzata vengono dalla batteria di V7L65, che ha la stessa legge di V7 fuori dal FAST.

**Ciò che manca al 100 %.** Il piede posteriore in sterzata resta caricato nel 46–62 % dello swing (clearance posteriore
0–2,4 mm), perché la sollevata non è ammessa. Un aiuto in più richiederebbe di ricertificare il box sterzato, cosa fuori
mandato.

## 3. I FAIL dei gate rigidi, scomposti

### 3.1 `WalkBoardPlant` (test host)

Il banco del test è **rigido**: il giunto reale insegue il comandato con un primo ordine, senza cedimento sotto carico. Copia
diagnostica (`run_board_compliant.sh`): il banco cede di C_plant · Jᵀ F, con lo stesso modello statico della compensazione.
Errore d'assetto a regime, massimo sui 33 hold (limite 5 mrad):

| C del banco (rad/N·m) | 0 (= test) | 0,0112 | 0,0224 | 0,0336 |
|---|---|---|---|---|
| Legge dev.7 (V0c), cedono UPPER+LOWER | **2,0** | 5,8 | 10,8 | 15,9 ¹ |
| Legge dev.7, cede anche l'HIP | 2,0 | 8,3 | 14,7 | 21,7 ¹ |
| V6f20 / V7, UPPER+LOWER | 13,5 | 9,7 | 7,3 | 7,5 ¹ |
| V6f20 / V7, anche l'HIP | 13,5 | 7,9 | 7,6 | 14,3 ¹ |

¹ Il test si interrompe più avanti con `NATIVE_TARGET_NOT_REACHED`: il cedimento statico del banco supera la tolleranza del
target nativo. Valori sugli hold eseguiti.

**Classificazione:**
- **Asserzione basata sul modello rigido: sì.** La dev.7 passa solo perché il banco è rigido. Con un terzo della cedevolezza
  del gemello fallisce anche lei (31/33 hold oltre soglia).
- **Condizione reale possibile: sì, per ogni legge.** Su un robot cedevole l'oscillazione d'assetto periodica supera i 5 mrad.
  A C corrispondente, V7 la dimezza (7,5 contro 15,9 mrad), ma resta sopra soglia; il cedimento HIP, non compensato per il box
  kHeight, pesa di nuovo.
- **Limitazione da C non identificato:** l'errore dipende da C in entrambe le direzioni; senza il C dell'HW nessuna delle due
  leggi è giudicabile con questo test.

### 3.2 Gate joystick (verifiche geometriche)

Il gate posa un robot **rigido** su un pavimento piatto all'altezza di legge. Diagnostica `joy_realized.py`: lo stesso URDF e
lo stesso campionamento, ma con il corpo **appoggiato sui propri piedi in appoggio** (pavimento = piano dei contatti). Robot
ancora rigido, cioè il caso peggiore: C sovrastimato del 100 %.

| | Supporto min m0 / m1 / m2 | Clearance non‑piede min | Penetrazione piede max |
|---|---|---|---|
| dev.7 (V0b), gate | 0,12 / 0,38 / 0,38 | 2,28 mm | 0,23 mm |
| dev.7, realizzata (controllo del metodo) | 0,12 / 0,38 / 0,29 | 2,23 mm | — |
| V7, gate | −0,59 / −0,22 / −0,08 | −3,35 mm | 6,1 mm |
| **V7, realizzata (robot rigido)** | **1,16 / 1,63 / 1,81** | **1,00 mm** (tibia RH in appoggio, FAST) | — (corpo inclinato ≤ 0,94°) |

**Classificazione:**
- **Sollevata: puro artefatto rigido.** Nel gate sposta la clearance di esattamente −heave e la penetrazione di +heave; nella
  geometria realizzata sparisce, perché il corpo si alza.
- **Compensazione con C corretto: artefatto rigido.** La geometria realizzata coincide con la legge dev.7, che passa.
- **Condizioni realmente possibili: solo se C è sbagliato.** Con il robot più rigido di C, la tibia posteriore in appoggio
  scende fino a 1,0 mm dal suolo (soglia 2 mm). Il supporto realizzato resta invece positivo (≥ 1,16 mm), più ampio della base.
  - In lineare, la soglia dei 2 mm regge fino a circa **19 %** di sovrastima di C in FAST e **37 %** in SLOW / STANDARD.

### 3.3 Proposta tecnica (non applicata; nessun gate né soglia cambiati)

1. **Misurare C sul robot**, per UPPER, LOWER e HIP, con un HOLD supervisionato all'altezza di camminata. Senza questo
   numero i due gate non possono distinguere un artefatto da un rischio.
2. **`WalkBoardPlant`:** aggiungere una variante del banco con la cedevolezza misurata (banda min–max, HIP incluso), con le
   soglie 5 / 2,5 mrad invariate.
   - Dalla tabella 3.1, a C ≥ 0,011 nemmeno la dev.7 rispetta le soglie.
   - L'operatore deve quindi decidere prima a quale robot si riferisce il criterio. Il dato utile è l'oscillazione d'assetto
     registrata dall'IMU in WALK sull'HW.
3. **Gate joystick:** valutare supporto e clearance sulla posa realizzata, con il corpo sui propri contatti e i giunti deflessi
   della banda di C misurata, con soglie invariate (supporto > 0, clearance ≥ 2 mm).
   - Se la banda misurata includesse un robot rigido, V7 continuerebbe a fallire, giustamente (1,0 mm).

## 4. CRAWL e TROT

- **CRAWL C2:** conservato, nessun 4° modo, patch identica (rigenerata byte per byte). Stato invariato: VERIFIED OFFLINE come
  prototipo.
- **TROT T3:** separato, nessun nuovo controllore. Due misure nuove:
  - **Appoggio diagonale.** Velocità con servo in appoggio più rigidi (controfattuale): ×1 24 %, **×2 53 %**, ×4 96 %.
    - **Condizione minima:** il cedimento residuo dell'appoggio diagonale deve scendere a circa ¼ di quello del gemello, cioè
      una compensazione di ≥ 75 % sotto W/2 per gamba.
    - La forma meno invasiva sarebbe la stessa compensazione con ripartizione statica W/2 sulla coppia diagonale. Non è
      implementata: richiede di ammettere la compensazione per il TROT in `validGaitParameters`.
    - Anche a ×4 il piede posteriore resta caricato nel 53 % dello swing e il roll cresce (55 → 68 mrad p2p).
  - **Fault alla WALKSTOP**, cosim diagnostica. Rifiuto `STOP_PLAN` del riferimento livellato: LF_LOWER a **2,99 rad/s**
    contro 2,45 ammessi (prova STOP).
    - La legge con i raggi dell'inviluppo, nel gate, non supera 21 tick (1,61 rad/s): l'eccesso nasce in runtime.
    - Ipotesi (non verificata): slew della correzione IMU sotto i 55 mrad di roll del trotto.
    - **Condizione minima:** ogni riferimento del TROT ≤ 2,45 rad/s (31,97 tick/rilascio) anche durante l'arresto, il che
      richiede di contenere l'oscillazione d'assetto o lo slew della correzione.
  - Sterzo e retromarcia: assenti nel generatore, quindi NOT YET FEASIBLE.

## 5. Limiti che richiedono una misura hardware

1. **C** di UPPER, LOWER e HIP sotto carico statico. Decide se la compensazione è giusta, l'assetto reale in WALK e la
   tolleranza del gate joystick (≈ 19 % FAST).
2. **Rateo reale dei riferimenti FAST in runtime**, con rumore e slew dell'IMU: margine di 3 tick (V7) o 2 tick (V7L60) dal
   limite di arresto.
3. **Ritardo di inseguimento del LOWER posteriore in swing** (≈ 60 tick nel gemello, modello servo).
4. **Assetto in WALK FAST**: l'inclinazione di ≈ 10 mrad verso lo swing posteriore, da log IMU (l'offset di montaggio di
   38,9 mrad è già misurato).
5. **Velocità effettive in sterzata** (dipendono dallo stesso C).

## 6. Stato finale

- **WALK, candidato V7: VERIFIED OFFLINE** nel gemello, gate non rigidi e servo plant.
  - Avanti 20 / 30 / 40 raggiunti a μ 0,8; sterzata 57–80 %.
  - Clearance posteriore FAST 0,5 mm (opzione V7L60: 1,2 mm).
  - **IMPLEMENTED NOT VALIDATED** come legge firmware: due gate rigidi in FAIL, C UNKNOWN.
- **CRAWL C2:** prototipo conservato.
- **TROT T3:** separato, NOT YET FEASIBLE, condizioni minime in §4.
- **Nessun handoff a Claude Motion.** Restano aperte la misura di C e la decisione dell'operatore sui gate rigidi.

## 7. Riproducibilità

Artefatti: `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/`.

- **Patch:**
  - `patches/V7.patch` = V6f20 + `compliance_steer_fade=0`;
  - `V7L60` / `V7L65` / `V7L75` con `lift_hind=.0060` / `.0065` / `.0075`;
  - `V7S60` / `V7S75` con `fast_sway_x`;
  - `V0c_zero` (tutti i termini a 0, con i campi correnti);
  - `f2/stopplan_diag/patches/H3only.patch`.
- **Gate:** `gates/<nome>.txt`, `gates/joy_V7*`.
- **Servo plant:** `plant/V7.txt`, `plant/V7L60.txt`.
- **Test host:** `host_tests/V7*.log` (verdetto e copia diagnostica).
- **Gemello:** `v7/` (V7L65 e sterzata), `v7/V7/`, `v7/L60/`, `f2/footlift/`, `t3/cf/sg2/`.
- **Diagnostiche:**
  - `f2/stopplan_diag/` (gate joystick strumentato);
  - `f2/cosim_diag/` (cosim strumentate: V6f20, V7L65, T3);
  - `f2/board/` (banco cedevole);
  - `f2/joy_realized/`.
- **Script:** `scripts/run_v7.sh`, `run_f2_batch.sh`, `run_f2_footlift.sh`, `build_joy_stopplan.sh`, `run_joy_stopplan.sh`,
  `build_cosim_diag.sh`, `run_board_compliant.sh`, `run_host_diag.sh`.
- **Strumento nuovo:** `gait_study/vnext/joy_realized.py`.

Worktree non committato. Il firmware e i suoi gate non sono stati modificati: tutte le strumentazioni sono su copie degli
export.
