# MATDOG — Locomozione vNext nel gemello: WALK 20/30/40, CRAWL, TROT (13/11/2026)

**Ambito.** Solo gemello MuJoCo e strumenti offline del firmware.
- Nessun hardware, flash, push, merge, reset o clean.
- Firmware installato (dev.7 `b561062`), calibrazioni NVS, limiti meccanici, box certificati, STOP e gate invariati.
- Ogni legge candidata è **una patch** sulla dev.7, applicata a un export `git archive` (`gait_study/vnext/make_vnext_patch.py`). Nessun file del repository firmware è modificato.
- Claude Motion resta congelato. Per Codex l'oggetto di revisione è questo rapporto.

Classi: **VERIFIED OFFLINE** (gemello o gate firmware), **IMPLEMENTED NOT VALIDATED**, **UNKNOWN**. Nessun risultato qui è
hardware.

## 0. Esito

> **Prosecuzione (13/11/2026):** `REPORT_VNEXT_F2.md`. Il candidato WALK diventa V7 (V6f20 con la compensazione attiva anche
> in sterzata). In quel rapporto: la clearance posteriore FAST, la scomposizione dei gate rigidi e le condizioni minime del TROT.
> I risultati V6f20 qui sotto restano validi.

**In una riga.** Nel gemello il candidato WALK **V6f20** (compensazione della cedevolezza in appoggio e sollevata del corpo,
§4) raggiunge **20,2 / 29,7 / 40,7 mm/s** a μ 0,8 con il piede che si stacca davvero. Però non è un candidato per il robot: due
gate firmware che modellano un robot **rigido** (test host `WalkBoardPlant`, gate joystick geometrico) falliscono **per
costruzione** della compensazione, e la cedevolezza reale del robot è **UNKNOWN**. CRAWL è verificato offline come
prototipo. TROT resta separato: si conosce la prima causa verificabile, ma non è fattibile nell'attuale generatore.

| Funzione | Requisito | Gemello (μ 0,8; sweep μ 0,6 / 1,0) | Gate firmware offline | Stato |
|---|---|---|---|---|
| WALK SLOW avanti | 20 mm/s effettivi | **20,2** (101 %); 20,7 / 18,8 | tilt+support PASS; servo plant PASS; **host FAIL**; **joystick FAIL** | VERIFIED OFFLINE nel gemello; non qualificabile con i gate rigidi |
| WALK STANDARD avanti | 30 | **29,7** (99 %); 30,6 / 29,3 | idem | idem |
| WALK FAST avanti | 40 | **40,7** (101 %); 41,3 / **39,0** | idem | idem; a μ 1,0 −2,5 % |
| WALK indietro | come la legge | STANDARD 111 %, FAST 103 % | gate rev50 PASS | VERIFIED OFFLINE (gemello) |
| WALK laterale, diagonale, rotazione | velocità di legge | **invariato rispetto alla base**: laterale 21–65 %, diagonale 49–56 %, rotazione 32–65 % | come base | **non raggiunto**; causa nota (§5.5) |
| Transizioni SLOW↔STANDARD↔FAST, STAND, STOP | continue, STOP intatto | cicli dei modi 98–100 %; STOP 46/47, 1 fault di protezione (§5.4) | servo plant 1296/1296 STOP | VERIFIED OFFLINE con 1 eccezione |
| Clearance effettiva | il piede si stacca | picco ant./post. SLOW 5,5/4,6 mm, STANDARD 4,2/2,5, FAST 1,8/**0,5** (base 0–1,9) | — | VERIFIED OFFLINE; FAST posteriore marginale |
| **CRAWL** (prototipo C2, slot SLOW) | lento, tre appoggi | **10,9 mm/s** su 10 di legge (108–109 % a ogni μ), clearance 9,0/9,5 mm, STOP 9/9, laterale 77 %, rotazione 80 % | tilt+support PASS (supporto statico **4,61 mm**, la dev.7 ha 2,1); joystick FAIL (stessa causa); host vedi §6 | VERIFIED OFFLINE come prototipo |
| **TROT** (prototipo T3, slot FAST) | diagonale, dinamico | **9,4 mm/s** su 40 (24 %; 34 % a μ 0,6, 14 % a μ 1,0); fault alla WALKSTOP | sterzo, indietro e laterale: fault o rifiuto | **NOT YET FEASIBLE**; prima causa verificabile in §7 |
| LOW BODY | funzione posturale distinta | non toccata | — | invariata |

Le velocità sono quelle **misurate** del COM nella finestra a regime, sul modello di contatto documentato (`dev7_ground`, μ
0,8), senza modificare il modello. 0,6 e 1,0 sono sensibilità.

## 1. Basi e correzioni della RCA 01

| Voce | Valore |
|---|---|
| Worktree | `~/MATDOG/worktrees/matdog-mujoco-dev7-corr-20261112`, HEAD `dddaeed`; tutto il lavoro della sessione è **non committato**. Lo stato RCA 01 precedente è salvato in `preserve/` degli artefatti |
| Base delle leggi | dev.7 `b5610620d0ce` + `F40T_JCB66.patch` (FAST 40, transizione dei modi, joystick FAST) |
| Gemello | `m6_dev7 --variant dev7_ground`, μ 0,8 di default, sweep 0,6 / 1,0 |
| Artefatti | `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/` |

**RCA 01, revisione 2 e 2.1** (`REPORT_RCA01_MU.md` §12):
- tutti i 7 rilievi Codex recepiti;
- il margine di supporto di `m6_gait` **non** è un artefatto;
- D5 vale come sensibilità al limite del modello attuatore;
- D9 mostra solo l'avvicinamento a zero;
- clearance ricalcolata su swing completi;
- in più, trovato qui: il fantasma D4 lasciava toccare il suolo alla tibia nel 23,7 % dei campioni. Il nuovo controfattuale
  `swing_leg_contact=0` corregge D4 (D4′).

## 2. Unitree Go2 e Spot: principi trasferibili a MATDOG

Fonti:
- `unitree_sdk2` `sport_client.hpp`;
- Spot SDK `robot_command.proto` (`MobilityParams`);
- *Choreography move reference*.

Gli SDK espongono interfacce, non gli algoritmi.

| Principio | Go2 / Spot | MATDOG oggi | Adottato qui |
|---|---|---|---|
| Velocità omnidirezionale separata dal gait | `Move(vx, vy, vyaw)`; `vel_limit` + `locomotion_hint` | `DRIVE vx vy yaw` + `selectWalkMode` | già presente; nessuna modifica |
| Gait come scelta separata | Go2 `StaticWalk`, `TrotRun`, `ClassicWalk`; Spot `HINT_CRAWL` ("una zampa alla volta"), `HINT_TROT` ("coppie diagonali"), `HINT_AMBLE` | WALK a 4 tempi in 3 modi | CRAWL e TROT come leggi dello stesso generatore (prototipi, §6–7) |
| Altezza di swing indipendente e verificabile | `SwingHeight` LOW / MEDIUM / HIGH; `liftoff_velocity` / `touchdown_velocity` | `liftM` comandato, mai verificato | clearance **effettiva** misurata e usata come criterio (`vnext_eval.py`); il lift di legge non basta (§3) |
| Postura coordinata | `BodyControlParams` (offset del corpo sul footprint), `Euler()`, `BalanceStand` | sway avanti/laterale nelle finestre a 4 appoggi; correzione IMU lenta | sollevata del corpo durante lo swing (heave) e compensazione del cedimento dell'appoggio |
| Trasferimento del carico | crawl con passi su `swing_slices`, `stand_time` prima di fermarsi | sway nelle finestre; nessun modello del carico | distribuzione statica del carico sui piedi in appoggio, rampe nelle finestre (§4) |
| Base fisica | attuatori a coppia, stima dello stato ad alta frequenza, MPC | servo di posizione solo‑P a 50 Hz, nessuna misura di forza | **non** copiata: niente MPC, RL o controllori paralleli. Tutto è feedforward nella stessa catena legge → IK → prove |

## 3. Diagnosi nel gemello: perché le velocità non si raggiungono

**Baseline W0.** Gemello μ 0,8, velocità raggiunta su velocità di legge, finestra a regime:

| Legge | Velocità di legge | Raggiunta | Clearance effettiva di picco (ant. / post.) | Swing a terra |
|---|---|---|---|---|
| dev.7 SLOW | 20 mm/s | 9,7 mm/s (49 %) | 1,2 / 1,9 mm | 64–76 % |
| dev.7 STANDARD | 30 | 19,7 (66 %) | 0,5 / 1,6 | 64–77 % |
| dev.7 FAST | 32 | 21,9 (68 %) | 0 / 0,1 | 79–100 % |
| F40T_JCB66 FAST 40 | 40 | 32,6 (81 %) | 0 / 0,07 | 100 % |

**Controfattuali**, solo diagnostici e mai configurazioni:

| Controfattuale | Effetto | Lettura |
|---|---|---|
| Tutta la parte distale della zampa in swing non collidente (`swing_leg_contact=0`) | SLOW **98 %**, STANDARD **96 %**, FAST 40 **97 %** anche a μ 1,0 | la perdita di velocità viene **dal contatto della zampa in swing col suolo**. Senza contatto la legge viene eseguita |
| Fantasma del solo piede (`swing_contact=0`, il D4 originale) | 60–80 % | la tibia tocca e frena (12,5 N, −9 N): controfattuale difettoso |
| Guadagno P ×4 sui servo in appoggio (`stance_gain=4`) | SLOW 49 → **94 %**, clearance ~5 mm | la causa del contatto è la **cedevolezza dell'appoggio**: il treppiede cede e il piede in swing non si stacca |
| Guadagno P ×4 sui servo in swing | 83 % | il ritardo in swing conta, ma meno |
| Attrito μ 2 | 66 % | l'attrito non è la leva |

**Bilancio per fase** (fantasma SLOW, piede solo): gli swing posteriori seguono la legge. Ogni swing anteriore perde circa 13 mm
perché, nella seconda metà, la tibia tocca e il corpo arretra.

Nota sul supporto: la legge dev.7 ha circa **2 mm di margine statico** (gate `walk_support_check`). Lo sway laterale è limitato
dalla stanza HIP del box kHeight (±25 mrad), che ammette 1–1,5 mm. È il motivo per cui, appena il piede in swing tocca, porta
carico (26–48 % del peso a μ 0,8–1,0, RCA 01 rev. 2).

**Causa primaria verificabile, nel gemello:** sotto carico i servo solo‑P dell'appoggio cedono di 10–20 tick (LOWER, UPPER,
HIP). Il treppiede scende verso l'angolo in swing, la gamba in swing tocca, trascina e frena. Il lift di legge (5–8 mm) non
copre il cedimento. **VERIFIED OFFLINE** (gemello). Sull'HW la cedevolezza statica del LOWER è nella stessa banda (HOLD M6:
HW −13…−3 tick, gemello −12…−8).

## 4. Modifiche implementate (minime, nella stessa catena di generazione)

Tutte in `make_vnext_patch.py`, sopra F40T_JCB66. Ogni termine a 0 riproduce F40T_JCB66 bit per bit: regressione V0b, gate e
`fast_fwd` identici.

### 4.1 Compensazione della cedevolezza in appoggio (`GaitParameters::complianceRadPerNm`)

- **Formula:** riferimento di ogni zampa in appoggio = IK − C · Jᵀ F.
  - F: reazione verticale statica del piede. Distribuzione a norma minima pesata del peso URDF (28,71 N) sui piedi in appoggio,
    con il COM al corpo di legge −9,4 mm (dato già in `SimpleMotion.h`).
  - J: jacobiano del punto di contatto, la stessa espressione di `differential()`.
- **Pesi di carico:** 0→1 nella finestra a 4 appoggi dopo il touchdown, 1→0 in quella prima del lift‑off. Le reazioni negative
  sono tagliate a zero e rinormalizzate: il riferimento è continuo per costruzione.
  - La prima versione escludeva la gamba e perdeva l'offset: salti di 15–20 tick, prova STOP fallita nel gate.
- **Ingresso e uscita graduali:**
  - in entrata nel ciclo di partenza;
  - in uscita dalla decelerazione dello STOP di WALK alla fine della finestra di assestamento (`CartesianSample::complianceScale`).
  - Senza questo, il primo riferimento WALK saltava rispetto all'HOLD e la camminata andava in fault.
- **Dove si applica:** in `solveGait`, nella soluzione livellata di `compensationTarget` (WALK / WALK_HOLD), nell'interpolazione
  T2 (`lerpFrame`) e nel `compensated()` del gate `walk_tilt_envelope_check`.
  - Ogni riferimento WALK lo porta **prima** di box e prove STOP: una sola autorità.
  - `lerpFrame` non propagava il fattore: la compensazione spariva in runtime. Corretto.
- **HIP escluso** (`complianceHip = 0`): l'offset HIP (±9–10 tick) eguaglia il margine del box kHeight e la camminata andava in
  `WALK_IK_OR_WORKSPACE`. Si compensano UPPER e LOWER, che fanno il cedimento verticale.
- **Valore:** C = **0,0336 rad/N·m**, la cedevolezza statica del modello servo (k 8,4 ‰/tick MEASURED, τ_stall VENDOR, η 0,5
  HYPOTHESIS). Sul robot è **UNKNOWN**. Con 2/3 della compensazione (C = 0,0224) FAST 40 resta al 100 % e STANDARD al 96 %,
  SLOW scende al 73 % (§5.3).

### 4.2 Sollevata del corpo durante lo swing (`heaveM`)

- **Cosa fa:** i piedi in appoggio vanno `heaveM` sotto le ancore durante lo swing di un'altra zampa, con un plateau a rampe
  smootherstep (25 %), zero nelle finestre a 4 appoggi.
- **Effetto:** il corpo di legge resta all'altezza nominale, quindi la traiettoria articolare della zampa in swing non cambia.
  Le zampe in appoggio si estendono e alzano corpo e piede in swing.
- **Valori:** 3 mm in SLOW e STANDARD, **2 mm in FAST**. Con 3 mm, la FAST con l'IMU al vertice pitch dell'inviluppo andava in
  fault nel servo plant del firmware (6/6 seed).

### 4.3 Regime sterzato: dissolvenza legata allo sway (`vnextSwayBaseM`, `vnextSwayFadeM`)

- **Regola:** compensazione e heave calano linearmente mentre lo sway laterale sale dalla base del modo a `swayLowM` (5 mm)
  della `JoystickSpec`. Lo sway cambia solo a 4 appoggi e con rampa limitata, quindi la dissolvenza ne eredita la continuità.
- **Motivo:** con i termini pieni il regime sterzato andava in fault `STOP_PLAN` nel gate joystick (lo sway sterzato era già al
  limite della prova STOP).
- **Effetto:** in sterzata la legge torna quella di F40T_JCB66.

Patch del candidato: `patches/V6f20.patch` (+178 righe su dev.7, F40T_JCB66 incluso), SHA256 in `SHA256SUMS.txt`.

## 5. WALK 20/30/40: risultati del candidato V6f20

V6f20 differisce da V5 solo per la sollevata FAST (`kWalkHeaveM[2]` da 3 a 2 mm; diff delle patch = 1 riga). Quindi le
corse SLOW e STANDARD di V5 valgono per V6f20. Le corse FAST sono state rifatte su V6f20. Base di confronto: V0b =
F40T_JCB66 con tutti i termini vNext a 0.

### 5.1 Velocità avanti raggiunte (gemello)

| Modo | Legge | V0b μ 0,6 / 0,8 / 1,0 | **V6f20 μ 0,6 / 0,8 / 1,0** |
|---|---|---|---|
| SLOW | 20,0 mm/s | 16,5 / 9,7 / 5,0 | 20,7 / **20,2** / 18,8 |
| STANDARD | 30,1 | 27,1 / 19,7 / 14,3 | 30,6 / **29,7** / 29,3 |
| FAST | 40,4 | 39,9 / 32,6 / **−6,6** (inversione) | 41,3 / **40,7** / 39,0 |

- **Inversione con μ** (RCA 01): risolta per STANDARD e FAST. In SLOW resta una pendenza residua (101 % a μ 0,8, 94 % a 1,0).
- **Sensibilità a C** (μ 0,8, SLOW / STANDARD / FAST): C = 0,0224 con heave 3 mm dà 73 / 96 / 100 %; C = 0,0336 senza heave dà 82 / 93 / 98 %.
  Il risultato dipende dal valore di C, che sul robot è **UNKNOWN**.

### 5.2 Clearance e carico trasmesso (μ 0,8, finestra a regime, swing completi)

| Modo | Picco ant. / post. | Swing con il piede caricato (ant. / post.) | Peso sulle zampe in swing | Contatti non‑piede |
|---|---|---|---|---|
| SLOW | 5,5 / 4,6 mm (base 1,2 / 1,9) | 0,29 / 0,27 (base 0,66 / 0,50) | 0,04 (base 0,17) | 0 |
| STANDARD | 4,2 / 2,5 (base 0,5 / 1,6) | 0,27 / 0,28 (base 0,57 / 0,47) | 0,03 (base 0,13) | 0 |
| FAST | 1,8 / **0,5** (base 0 / 0,1) | 0,27 / 0,24 (base 0,58 / 0,39) | 0,04 (base 0,17) | 0 |

In FAST il piede posteriore si stacca appena: la velocità c'è, il margine no. A μ 1,0 il picco posteriore scende a 0,4 mm.

### 5.3 Indietro, transizioni

- Indietro: STANDARD −18,4 mm/s (111 % della legge), FAST −18,3 mm/s (103 %).
- `modes_cycle` (SLOW→STANDARD→FAST a 32 mm/s→SLOW, cambi di modo in moto): 98 %. `fast_modes_cycle` (lo stesso con FAST a
  40 mm/s): 100 %. Nessun fault, nessun rifiuto.
- La compensazione entra ed esce in modo continuo (`complianceScale`). Il cambio di modo cambia la legge solo a fine ciclo,
  come in dev.7: le grandezze nuove sono strutturali in `gaitRetarget`.

### 5.4 STOP e protezioni

- **46/47 STOP verificati** (1,88–2,84 s). Coperti: SLOW 9, STANDARD 10, FAST 28 (8 a ottavi di ciclo, 16 densi, avanti,
  indietro, rotazione, diagonale).
- **1 fault:** `fast_stopcyc_0` → `STOP_MEASURED_OUTSIDE_BOX` 0,58 s dopo lo STOP, a 4 appoggi. È la protezione che scatta,
  non un moto fuori box. Nella diagnosi V5 (`diag3/`) l'HIP LF misurato è 0,1 tick fuori dal box kHeight.
  - Base V0b sullo stesso istante: STOP ok.
  - Nel gemello il margine HIP misurato in FAST passa da 8,3 a 3,3 tick: con il piede che si stacca davvero, l'appoggio è
    un vero treppiede e il corpo rolla di più (roll p2p FAST 19 → 32 mrad).
  - M6 documenta che il gemello è pessimista sui margini HIP allo STOP. Il fault resta comunque un **FAIL nel gemello**, non
    riclassificato.
- I rifiuti nelle corse con STOP sono `RENEW DRIVE` durante `STOP_IN_PROGRESS` / `STOP_LATCHED`: è la semantica attesa.

### 5.5 Regime sterzato (laterale, diagonale, rotazione): non raggiunto

| Comando | SLOW | STANDARD | FAST |
|---|---|---|---|
| Laterale | 65 % | 62 % | 21–37 % |
| Diagonale | — | 56 % | 49 % |
| Rotazione | — | 65 % | 32 % |

- Identico alla base bit per bit: la dissolvenza §4.3 riporta la legge a F40T_JCB66.
- **Prima causa verificabile:** con i termini pieni, il gate joystick va in `STOP_PLAN` (lo sway sterzato è già al limite
  della prova STOP). Senza termini, le zampe in swing restano a terra: clearance 0–3 mm, piede caricato nel 44–85 % dello
  swing.
- È la stessa causa del moto avanti (appoggio cedevole), con in più un margine STOP che non lascia spazio. Non risolto.

### 5.6 Gate firmware offline sul candidato

| Gate | V0b (= dev.7 + F40T_JCB66) | V6f20 | Lettura |
|---|---|---|---|
| `walk_tilt_envelope_check` + `walk_support_check`, 3 modi × 3 preset + indietro 50 mm | PASS | **PASS**: supporto 1,94–2,50 mm, cattura STOP 1,42–2,19 mm, HIP 9,3–12,4 tick, vmax ≤ 2,207, amax ≤ 26,78; box compensato FAST p2 12,3 tick (base 6,18) | non peggiora la dev.7 |
| Servo plant (48 righe: 6 seed × 2 V, tutti gli scenari) | 0 fault, STOP 1296/1296 | **0 fault, STOP 1296/1296**, 0 mancati, out‑of‑box misurato 0 | PASS |
| Test host Motion (`run_simple_motion_tests.sh`) | PASS | **FAIL** (`WalkBoardPlant::hold`: errore d'assetto a regime 0,0116 rad, roll 0,0091; limiti 0,005 / 0,0025) | vedi sotto |
| Gate joystick (46 scenari × 3 modi) | PASS: supporto ≥ 0,12 mm, non‑piede ≥ 2,28 mm | **FAIL**: supporto −0,80 / −0,12 / −0,05 mm, non‑piede −2,88 / −3,35 / −2,43 mm | vedi sotto |

**Perché falliscono test host e gate joystick.** Entrambi modellano un robot **rigido**: posizione articolare reale =
comandata. La compensazione comanda apposta una geometria che è corretta solo se l'appoggio cede come nel gemello.
- **Test host.** Le zampe più cariche risultano "estese" e il corpo fittato sui tre appoggi si inclina di 0,67°.
  - V5h0 (solo compensazione, heave 0) fallisce con lo stesso valore (0,0117 / 0,0093) → la causa è la compensazione.
  - In una **copia diagnostica** del test, l'unica asserzione di assetto stampa invece di interrompere: 33 casi la
    violerebbero (0,0116–0,0135 rad) e tutte le altre asserzioni passano. La copia non è un verdetto: il verdetto è il test
    invariato, FAIL.
- **Gate joystick.**
  - Con la sollevata, gli appoggi comandati scendono sotto il piano nominale: le tibie in appoggio "penetrano" di 2,4–3,4 mm.
  - Con la sola compensazione: −0,15…−0,58 mm, e un supporto COM fino a −0,8 mm.
  - Nel gemello i contatti non‑piede sono zero in tutte le corse.
- **Nessun criterio è stato rilassato.** Cambiare il modello di questi gate (aggiungere la cedevolezza misurata) è una
  decisione dell'operatore, da prendere solo dopo aver misurato C sul robot.

## 6. CRAWL (prototipo C2)

**Legge.** Stesso generatore, gait WALK a 4 tempi nello slot SLOW, nessun controllore nuovo:
- periodo 8 s, duty 0,85 (finestre a 4 appoggi dello 0,10 del ciclo);
- altezza 120 mm, sway avanti 8 mm, sway laterale 1,5 mm;
- rampa di swing 1/3, passi 40 / 60 / 80 mm (top 10 mm/s), lift 10 mm;
- heave 4 mm e la stessa compensazione di §4.

Patch: `patches/C2_crawl.patch`.

| Verifica | Risultato |
|---|---|
| Gate tilt+support, SLOW 3 preset + indietro | **PASS**: supporto statico **4,61 mm** (la dev.7 SLOW ha 2,13), cattura STOP 4,54–4,61, HIP 9,1 tick, box 7,3–8,3, vmax 0,955 rad/s, amax 3,99 |
| Velocità avanti (gemello), μ 0,6 / 0,8 / 1,0 | **10,9 / 10,9 / 10,8 mm/s** su 10,0 di legge (108–109 %, indipendente da μ). Il 9 % in più non è spiegato: **UNKNOWN** |
| Clearance effettiva | picco 9,0 / 9,5 mm (lift 10), minimo centrale 1,1 / 0,8 mm, libero nel 73–76 % dello swing, caricato nel 17–20 % |
| Carico | peso sulle zampe in swing 0,01; margine geometrico di appoggio di legge 5,5–6,1 mm (WALK 1,2–1,5) |
| Stabilità | roll/pitch p2p 31 / 15 mrad; contatti non‑piede 0; nessun fault |
| Laterale, rotazione | 77 % (1,9 mm/s, con 1,5°/ciclo di rotazione indesiderata), 80 % |
| STOP | 9/9 verificati (`crawl_stop` + 8 a ottavi di ciclo, 1,88–2,40 s) |
| Gate joystick (mode 0) | **FAIL**: supporto +1,33 mm (meglio della base, +0,12), ma non‑piede −4,14 mm (heave 4 mm nella geometria rigida, §5.6) |
| Test host | **FAIL** per costruzione: `walkModes` verifica che SLOW sia la legge dev.4 bit per bit e il prototipo occupa lo slot SLOW. Questa asserzione interrompe il test anche nella copia diagnostica, quindi gli altri test non sono stati valutati su C2. Un 4° modo CRAWL è lavoro di integrazione, non fatto |

**Stato: VERIFIED OFFLINE come prototipo.** È lento per legge (10 mm/s) ma robusto all'attrito, con il doppio del margine
statico della WALK. Ha le stesse dipendenze di V6f20 (C sul robot, gate rigidi).

## 7. TROT (prototipo T3): non ancora fattibile

**Legge.** `GaitType::TROT` dello stesso generatore nello slot FAST:
- coppie LF+RH / RF+LH, nessuno sway;
- periodo 0,8 s, duty 0,6, altezza 105 mm;
- passi 16 / 24 / 32 mm (top 40 mm/s), lift 8 mm;
- nessuna heave e nessuna compensazione.

T2 era invalido: `walkGait` assegnava C anche al TROT e `validGaitParameters` lo rifiuta; nel gemello andava in
`WALK_IK_OR_WORKSPACE` all'avvio. Il generatore ora assegna C = 0 al TROT (`make_vnext_patch.py`; V6f20 e C2 rigenerati
byte per byte identici).

| Verifica | Risultato |
|---|---|
| Gate tilt+support, FAST 3 preset | nominale / IK / box / STOP 0 fail. Il "supporto statico 93,7 mm" conta solo i campioni con ≥ 3 appoggi, cioè le finestre a 4 appoggi: per un trotto la prova statica **non si applica** |
| Indietro (gate rev50 e gemello) | non disponibile: `reverse` è solo WALK (rifiuto `JOYSTICK_VELOCITY_RANGE`) |
| Sterzo (gate joystick) | laterale e rotazione → `WALK_IK_OR_WORKSPACE` all'avvio; `tilt_fail` 4 in una sequenza → **FAIL** |
| Velocità avanti (gemello), μ 0,6 / 0,8 / 1,0 | **13,4 / 9,4 / 5,8 mm/s** su 40 (34 / 24 / 14 %): la stessa inversione con μ della RCA 01 |
| Clearance, carico | picco ant. / post. 3,8 / **0,9 mm**; piede posteriore caricato nel 56 % dello swing; peso sulle zampe in swing 0,15; roll p2p **55 mrad** (3,2°); slittamento ≤ 1,3 mm; contatti non‑piede 0 |
| WALKSTOP dal TROT | `WALK_IK_OR_WORKSPACE` 0,9 s dopo la richiesta, a tutti e tre i μ e anche con i servo irrigiditi. Il firmware rifiuta un riferimento durante la decelerazione; il ramo esatto (livellamento IMU o box nominale) è **UNKNOWN** |
| STOP | 9/9 verificati (8 a ottavi di ciclo + `fast_fwd_stop`) |
| Transizione SLOW→STANDARD→TROT→SLOW | nessun fault (`fast_modes_cycle`) |
| Test host | **FAIL**: `walkModes`, fault durante la ripresa WALK dopo uno STOP nel modo TROT |

**Prima causa verificabile della velocità.** Il controfattuale `stance_gain=4` (servo in appoggio 4 volte più rigidi, solo
diagnostico) porta il trotto a **38,3 mm/s (96 %)**. La causa è la cedevolezza dell'appoggio, come per la WALK, ma più
forte:
- in appoggio diagonale ogni zampa porta circa metà del peso, non un terzo;
- la compensazione di §4.1 non è applicabile: con due appoggi su una diagonale il bilancio statico dei momenti è
  indeterminato, e il trotto è dinamico per natura.

Anche con i servo rigidi restano il fault alla WALKSTOP e il piede posteriore caricato. Servirebbero quindi, insieme, un
modello di carico dinamico, una legge di sterzo e di inversione per il TROT e un percorso di arresto propri. È oltre il
perimetro KISS di questo mandato (niente MPC, niente controllori paralleli).

Il controfattuale `swing_leg_contact=0` sul trotto si è interrotto in `m6_gait.show` (margine di supporto `None`): non
usato.

## 8. Velocità raggiunte e cause dei requisiti mancati

| Requisito | Raggiunto (gemello, μ 0,8) | Mancato? | Prima causa verificabile |
|---|---|---|---|
| WALK SLOW 20 | 20,2 mm/s | no (a μ 1,0: 18,8) | — |
| WALK STANDARD 30 | 29,7 | no (−1 %) | — |
| WALK FAST 40 | 40,7 | no (a μ 1,0: 39,0) | clearance posteriore 0,4–0,5 mm: margine quasi nullo |
| Laterale, diagonale, rotazione | 21–65 % | **sì** | swing a terra per cedevolezza; compensazione esclusa dal margine della prova STOP sterzata (§5.5) |
| CRAWL | 10,9 (legge 10) | no | — |
| TROT 40 | 9,4 | **sì** | cedevolezza in appoggio diagonale (controfattuale 96 %), più fault alla WALKSTOP e assenza di sterzo e retromarcia |
| Qualificabilità firmware di V6f20 | — | **sì** | test host e gate joystick rigidi (§5.6); C sul robot UNKNOWN |

## 9. Stato delle funzioni

- **VERIFIED OFFLINE** (gemello e gate firmware non rigidi):
  - WALK avanti 20 / 30 / 40 a μ 0,6 e 0,8;
  - WALK indietro;
  - transizioni tra i modi;
  - STOP 46/47;
  - clearance effettiva SLOW / STANDARD;
  - CRAWL C2 avanti, laterale, rotazione e STOP;
  - diagnosi della causa (cedevolezza dell'appoggio) per WALK e TROT.
- **IMPLEMENTED NOT VALIDATED:**
  - V6f20 come legge firmware: test host e gate joystick FAIL, nessuna prova hardware;
  - CRAWL come modo firmware: occupa lo slot SLOW e rompe il test che lo vincola.
- **UNKNOWN:**
  - C sul robot (le HOLD M6 HW −13…−3 tick sul LOWER sono nella banda del gemello, ma non bastano a fissare C);
  - effetto sull'HW del roll maggiore;
  - sovra‑avanzamento della CRAWL del 9 %;
  - ramo del fault TROT alla WALKSTOP.
- **NOT YET FEASIBLE** nell'attuale generatore senza nuova architettura:
  - velocità di legge in sterzo;
  - TROT (velocità, sterzo, retromarcia, arresto).
- **Non dichiarato:** nessuna stabilità hardware. Le forze e i margini qui sono simulati.

## 10. Handoff a Claude Motion

**Nessun handoff di flash.** Non esiste un candidato credibile per il robot: V6f20 fallisce due gate firmware e il suo
parametro chiave è UNKNOWN sull'HW. Claude Motion resta congelato.

Se l'operatore vuole proseguire, l'ordine minimo è:
1. **Misura di C sul robot.** Una sessione HOLD supervisionata all'altezza di camminata: errore articolare UPPER / LOWER per
   zampa con il carico noto della postura. I log HOLD esistenti danno la banda, non il valore per giunto.
2. **Decisione dell'operatore** sui gate rigidi (`WalkBoardPlant`, gate joystick): se e come modellare la cedevolezza misurata.
   È un cambio dei criteri di accettazione, quindi non lo propongo come correzione.
3. Solo dopo: `patches/V6f20.patch` con C misurato, rifatto sul gemello e sui gate. È l'unico oggetto da passare a Claude
   Motion: +178 righe in `Gait.h`, `Gait.cpp`, `SimpleMotion.h`, `SimpleMotion.cpp` e nel gate tilt.

## 11. Riproducibilità

Artefatti in `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/`:
- **Patch:** `patches/`, da `gait_study/vnext/make_vnext_patch.py` su dev.7 `b561062` e F40T_JCB66.
  - V6f20: `compliance=0.0336 compliance_hip=0 heave=.003,.003,.0020`.
  - C2: in più `heave=.004,…` `crawl=8.0,.85,.120,.008,.3333,.040/.060/.080,.010,.0015`.
  - T3: `heave=.003,.003,0 trot=0.8,.60,.105,.35,.30,.016/.024/.032,.008`.
- **Build:** `scripts/build.sh` → `builds/<nome>/build/cosim`.
- **Gate:** `scripts/gates.sh` → `gates/<nome>.txt`. Gate joystick → `gates/joy_<nome>/m*.txt`.
- **Servo plant:** `plant/<nome>.txt`.
- **Test host:** `scripts/run_host_tests.sh`, `scripts/run_host_diag.sh` → `host_tests/`.
- **Gemello:**
  - `scripts/run_v0b_battery.sh` (base);
  - `run_v5_battery.sh` (SLOW / STANDARD, valide per V6f20);
  - `run_v6_fast.sh`;
  - `run_c2_t2.sh` (C2; T2 invalido, conservato);
  - `run_t3.sh`, `run_t3_cf.sh`.
  - Output in `v0b/ v5/ v6/ c2/ t2/ t3/`, una JSON per scenario (`vnext_eval.py`).
- **Tabelle:** `gait_study/vnext/vnext_table.py <dir A> <A> <dir B> <B>`.
- **Integrità:** `SHA256SUMS.txt`, `PROVENANCE.txt`.

Il lavoro nel worktree resta **non committato**. Lo stato RCA 01 precedente è in `preserve/`.
