> Copia della valutazione offline. Gli strumenti e le uscite (`workspace_use.cpp`, `workspace_use_e1b.txt`) sono in `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/loco/`.

# Fase E — valutazione quantitativa della locomozione avanzata (16/11/2026)

Obiettivo di progetto: avvicinare MATDOG alle capacità di Spot e Unitree Go2 nei limiti fisici misurati dei componenti installati.

Fonti di questa pagina:
- workspace delle traiettorie del rig host (`workspace_use.cpp`, `workspace_use_e1b.txt`), sul `SimpleMotion` di produzione con la conversione gen-1 reale;
- costanti firmware di `46b3175`;
- gemello MuJoCo del 15/11 (`MATDOG_1_0_2_OFFLINE_DEV_20261115/REPORT.md` §4);
- identificazione di C del 13/11 (`MATDOG_MUJOCO_VNEXT_LOCO_20261113/report/REPORT_C_IDENT_HOLD.md`);
- hardware del 16/11.

Ogni numero porta il suo stato.

## 1. Workspace: CAD certificato contro quello usato dalla gait

Il vincolo operativo sono i box di giunto certificati CAD/FCL, cioè intervalli indipendenti per giunto (`CollisionData.h`, `Steer*CollisionData.h`):
- cammino frontale: `kWalkBox = kHeight` (HIP fisso entro ±25 mrad);
- cammino sterzato: `kSteer`, `kSteerStandard`, `kSteerFast`.

Margine minimo delle traiettorie, rig host, 64 scenari, rilasci WALK (VERIFIED OFFLINE):

| Scenario | Box | HIP usato / box (mrad), margine | UPPER usato / box, margine | LOWER usato / box, margine | Margine minimo ai limiti URDF / RAW servo |
|---|---|---|---|---|---|
| WALK SLOW 120 s | kHeight | 21 / 50, +14 | 577 / 1 176, +34 | 420 / 1 465, +159 | ≥ +696 / ≥ +588 |
| WALK STANDARD 90 s | kHeight | 20 / 50, +14 | 477 / 1 176, +172 | 341 / 1 465, +362 | ≥ +622 / ≥ +604 |
| WALK FAST 90 s | kHeight | 17 / 50, +16 | 523 / 1 176, +115 | 342 / 1 465, +409 | ≥ +564 / ≥ +546 |
| JOY steer SLOW | kSteer | 261 / 440, **+88** | 238 / 620, +61 | 213 / 621, +98 | ≥ +653 / ≥ +563 |
| JOY steer STANDARD | kSteerStandard | 253 / 408, **+74** | 250 / 684, +75 | 215 / 580, +55 | ≥ +614 / ≥ +614 |
| JOY steer FAST | kSteerFast | 192 / 422, **+104** | 245 / 671, +104 | 183 / 562, +75 | ≥ +601 / ≥ +583 |
| JOY handover SLOW / STANDARD / FAST | steer | 250–253 / 408–440, +74…+104 | 336–563, **+31…+55** | 258–391, +57…+80 | ≥ +554 |

Le spanne frontali UPPER/LOWER includono la salita alla quota del cammino.

**Lettura:**
- I limiti meccanici URDF e i limiti RAW dei servo sono lontani: 550–900 mrad di margine.
- I vincoli attivi sono i box CAD: 31–104 mrad nei cammini sterzati, 14–16 mrad di HIP nel frontale.
- Gli intervalli indipendenti per giunto sono per costruzione un'approssimazione interna dell'insieme senza collisioni: possono essere restrittivi.
- **Che lo siano inutilmente non è dimostrato.** Servirebbe una certificazione CAD/FCL della traiettoria effettiva, per esempio un volume spazzato per gamba lungo la gait, con lo stesso strumento `certify_steer_box.py`. Oggi non lo propongo come modifica: è il prossimo lavoro offline se si vuole allargare sidestep e rotazione.

## 2. Attuatori ST3215 (fatti e lacune)

| Grandezza | Valore | Stato |
|---|---|---|
| Velocità del drive WALK | 1 000 tick/s = 1,53 rad/s (`kWalkDrive`) | firmware |
| Accelerazione WALK / frenata STOP | ACC 50 = 7,7 rad/s²; frenata ACC 84 = 12,9 rad/s² (`kStopBrakeAcc`) | firmware; il tetto del registro 85 a 220 è verificato sull'HW il 10/11 ma è **volatile** |
| STAND | 0,55 rad/s, 1,10 rad/s² | firmware |
| Regolatore | solo P: PresentLoad = 8 ‰·(\|e\|+1) per \|e\| > 1 tick | VERIFIED OFFLINE dai log HW (13/11) |
| Cedimento statico sotto carico | 10–20 tick per appoggio | HW + gemello |
| Cedevolezza C | condivisa UPPER+LOWER 0,036 rad/N·m, in [0,031; 0,039] solo se l'attrito ≤ 0,27 N·m | parzialmente identificata |
| C per giunto, C dinamica, attrito statico per giunto | — | **UNKNOWN** |
| Velocità a vuoto e coppia di stallo a 11,4 V sotto carico | — | **UNKNOWN** sul robot (solo i dati di catalogo, non usati) |
| Ritardo di inseguimento per giunto alla velocità WALK | — | **UNKNOWN**: lo stato HTTP ha `commanded_raw` vecchio di ~0,7 s; manca un log sincrono ≥ 50 Hz |

## 3. Appoggi, clearance, supporto, sway (gemello, μ 0,8, legge attuale)

| Grandezza | SLOW | STANDARD | FAST |
|---|---|---|---|
| Velocità comandata → realizzata | 20 → 9,7 mm/s (49 %) | 30 → 19,7 (66 %) | 32 → 21,9 (68 %) |
| Clearance mediana effettiva (`liftM` di legge 6–8 mm) | 1,6 mm | 1,2 mm | 0,8 mm |
| Swing trascinato | 54 % | 46 % | 55 % |
| Supporto statico minimo / cattura STOP (modello rigido) | 2,12 / 1,41 mm | 1,99 / 1,51 | 1,71 / 1,41 |
| Sway comandato / escursione laterale del corpo | 1,5 / 8,2 mm p2p | 1,2 / 7,6 | 1,0 / 10,8 |

- **Prima causa** (controfattuale `swing_leg_contact=0`, VERIFIED OFFLINE nel gemello): contatto della zampa in swing, perché il treppiede cede verso la zampa sollevata. Senza contatto si arriva al 93–98 %, e l'escursione laterale scende a 4,1–6,7 mm.
- **Sway:** quello comandato è già al minimo compatibile con la cattura allo STOP: ogni 0,5 mm in meno costa ~0,37 mm di margine. L'escursione in più non è comandata; viene da cedevolezza e trascinamento. **Ridurre lo sway non è proposto.**

## 4. Sidestep, diagonale, rotazione (gemello)

| Movimento | Realizzato | Moto indesiderato |
|---|---|---|
| Laterale STANDARD / FAST | 62 % / 36 % | rotazione +0,45 / +0,36 °/s |
| Diagonale STANDARD / FAST | 56 % / 48 % | — |
| Rotazione STANDARD / FAST | 65 % / 50 % | traslazione 1,1–1,4 mm/s |

Nei cammini sterzati la zampa «in swing» porta l'8–9 % del peso: senza il suo contatto il gemello si ribalta. Il margine di supporto sterzato è insufficiente sotto cedevolezza. Aumentare i passi del joystick non è proposto: lo sway sterzato è già al limite della prova STOP, e le zampe si avvicinerebbero ai box del §1, che oggi hanno 31–104 mrad di margine.

## 5. FAST 40, CRAWL, TROT

- **FAST 40: non certificabile con i box e la frenata qualificata di oggi**, non un'impossibilità fisica dimostrata (VERIFIED OFFLINE, 14–15/11; i limiti distinti sono nel `GROUND_MEASUREMENT_PLAN.md` §0).
  - Alla frenata qualificata (ACC 84) LF_HIP, LF_UPPER o LH_HIP escono dal **box certificato** in tutte le 396 configurazioni a 40 mm/s che rispettano supporto, ZMP e cattura. È un vincolo di certificazione (B) e di frenata qualificata (C), non del limite meccanico (A): i giunti restano a 550–900 mrad dai limiti URDF e servo.
  - Lo swing medio cresce come v/(1−d): 229 mm/s a 40 mm/s con d 0,825 (legge, D).
  - Capacità credibile con la legge FAST attuale: **~30 mm/s utili**, risolto il contatto in swing.
  - FAST 40 diventa rivalutabile con la frenata reale sotto carico, C per giunto, la clearance reale e una certificazione CAD della traiettoria.
- **CRAWL (modo distinto):**
  - prototipo C2 (13/11, gemello dev.7, con compensazione della cedevolezza): supporto 4,61 mm contro 2,1, clearance 9 mm, laterale 77 %, rotazione 80 %, STOP 9/9;
  - richiede C per giunto, box sterzati nuovi e prove STOP nuove.
- **TROT: non supportato dalle evidenze attuali**, non escluso:
  - nel gemello T3 (13/11) ha dato 9,4 mm/s su 40 e un fault alla WALK STOP, ma il modello attuatore non è identificato dinamicamente;
  - il dato che decide è il ritardo di inseguimento sotto carico degli ST3215 a 50 Hz (M5 del piano a terra), oggi non misurabile con la telemetria esistente;
  - nessuna promessa di dinamica prima di quella misura.

## 6. Confronto di scala con Spot e Go2

Numero di Froude Fr = v² / (g·L), con L = altezza dell'anca:

| Robot | v | L | Fr |
|---|---|---|---|
| MATDOG | 0,03 m/s | ~0,12 m | ~8·10⁻⁴ |
| Spot, cammino | 1,6 m/s | ~0,55 m | ~0,5 |

- A parità dinamica di un cammino lento (Fr 0,1), MATDOG dovrebbe andare a ~0,34 m/s, cioè ~11 volte la velocità attuale.
- Le cause sono misurate o verificate offline:
  - attuatori a sola posizione, con cedimento di 10–20 tick e clearance effettiva ~1 mm;
  - controllo a 50 Hz con ~11 ms di calcolo per release e picchi di 22–27 ms (oggi);
  - frenata STOP dimostrata dentro box CAD con 31–104 mrad di margine.
- **Avvicinarsi a Spot o Go2 in senso dinamico non è possibile con questi attuatori.** Il margine reale sta nel rendere efficace il cammino statico: più velocità utile, laterale e rotazione vicini al 100 %, clearance reale.

## 7. Miglioramenti di gait sviluppabili offline senza installarli

1. **Compensazione della cedevolezza in appoggio** (V7: `ref = IK − C·JᵀF`) + sollevata del corpo nello swing.
   - Nel gemello porta il frontale al 99–101 % e la clearance a 5,5/4,6 mm (SLOW).
   - Bloccata da C per giunto (UNKNOWN) e dai gate rigidi. Proposta di criterio: i gate valutano sia il riferimento sia la posa prevista con C misurata; resta da decidere dopo la misura di C.
2. **Certificazione della traiettoria al posto dei box per giunto** (§1). È l'unica via per allargare laterale e rotazione senza spostare i limiti a mano.
3. **CRAWL C2 come modo distinto**, dopo i punti 1 e 2.

Nessuno di questi è stato portato sul firmware oggi.

## 8. Dati mancanti (in ordine di necessità)

1. C per giunto (UPPER, LOWER, HIP) statica e dinamica sotto carico, e attrito statico per giunto.
2. Velocità reale al suolo per SLOW, STANDARD e FAST, con riferimento esterno al pavimento (il metodo video del 13/11 è riusabile).
3. Clearance reale del piede in swing.
4. Ritardo di inseguimento per giunto alla velocità WALK: log sincrono riferimento e posizione ≥ 50 Hz.
5. Curva velocità/coppia degli ST3215 a 11,4 V sul robot.
6. Attrito piede–pavimento.

## 9. Prossima prova fisica strettamente necessaria per la locomozione

Piano dettagliato: `GROUND_MEASUREMENT_PLAN.md` (16/11, seconda parte).

La prima misura è **M1**, una sessione a terra di sola misura, senza modifiche hardware e con la build installata:
- STAND 80, 120 e 150 mm, ciascuna raggiunta salendo e scendendo;
- HOLD di 30 s per quota;
- osservazione passiva `GROUND_CONTACT.OBSERVE` su ogni zampa: angoli misurati contro riferimento, PresentLoad, Jacobiano.

Chiude C per giunto e l'attrito, il blocco comune a V7, CRAWL e gate. Seguono M2–M4 (velocità al suolo, clearance, frenata reale) con i soli modi qualificati. Va decisa con un mandato dedicato: non è una prova di fixture.
