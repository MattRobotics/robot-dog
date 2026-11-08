# MATDOG — WALK STANDARD v2: riprogettazione della locomozione — candidata `1.0.2-dev.2` — 2026-11-08

**Natura:** sessione autonoma **offline** (ricerca, analisi, patch, test host, build). Nessun flash, riavvio, movimento, scrittura NVS/EEPROM o calibrazione. Installato resta `1.0.1-dev.17` (Stable `v1.0.1`, APP `14e055a0…`). Classi: **VH** verificato hardware, **VO** verificato offline (riproducibile), **INV** implementato non validato, **HYP** ipotesi, **UNK** ignoto.

## 0. Esito

| | v1.0.1 (WALK60) | **WALK STANDARD v2 (100 mm)** | Classe |
|---|---:|---:|---|
| Velocità comandata | 9,23 mm/s | **20,0 mm/s (×2,17)**; preset 10 / 15 / 20 mm/s | VO |
| Supporto statico minimo (COM URDF, tutte le fasi: avvio, regime, ogni WALK STOP) | +0,88 mm | **+2,12 mm** | VO |
| Margine ZMP carrello-tavolo | +0,91 mm | +2,10 mm | VO |
| Picco giunto v / a vs drive `{1000,50}` (1,534 rad/s, 7,67 rad/s²) | 0,82 / 4,47 | 1,275 (83 %) / 5,81 (76 %) | VO |
| Margine box `kHeight` UPPER anteriore / anca | 15 / 6,9 tick | 32 / 9,3 tick | VO |
| WALK STOP → STANDING (rig, tracking ideale) | 7,12 s (hw dev.17 8,9 s) | **6,62 s** | VO |
| Drive servo, box, geometria, calibrazione, STOP | — | **invariati** | VO |
| Joystick (laterale 3,08 mm/s, yaw 1,76 °/s) | — | invariato (legge e certificato dev.15) | VO |

Obiettivo 20–25 mm/s: raggiunto il **limite inferiore con il profilo servo già qualificato** e margini migliori della Stable. Il limite superiore e WALK FAST richiedono un profilo drive più alto da qualificare su hardware (§9).

## 1. Provenienza

PR #67 Draft, branch `feat/motion-perf-next-20261107`, HEAD locale = remota = `e933da5` (sorgente `4c897ed`, candidata `1.0.2-dev.1` APP `03c84606…`, non flashata). Lavoro nel worktree isolato `~/MATDOG/worktrees/matdog-gait-redesign-20261108` (branch locale `feat/gait-redesign-20261108`), poi pubblicato sul branch della PR #67. Il patrimonio di `4c897ed` (A1/A2, T1b/T2, STOP con storia ≥1 periodo, joystick proporzionale, inviluppo steered nel frame heading, istogrammi D11) è conservato integralmente.

## 2. Ricerca comparativa (fonti lette)

| | MATDOG v1.0.1 | **Proposta position-based (v2)** | Alternativa WALK FAST / TROT |
|---|---|---|---|
| Sequenza appoggi | crawl laterale RH→RF→LH→LF, duty 0,825 | stesso crawl, duty 0,80 | trot a coppie diagonali, duty 0,5–0,7 con fasi a 4 appoggi |
| Impronte | Raibert: canonico a metà stance (`v·T_st/2`) | idem, **pattern centrato sul COM** (offset corpo 11 mm) | Raibert `v·T_st/2` (CHAMP `raibertHeuristic`, Pupper `alpha=0.5`) |
| Corpo | sway avanti/indietro ±10 mm + laterale ±2 mm a finestre (4 appoggi) | ±3 / ±1,5 mm a finestre + offset costante | nessuno sway; equilibrio dinamico sulla diagonale |
| Swing | smootherstep (picco v 1,875×, a 5,77×) | **velocità trapezoidale a terzi** (1,5×, 4,5×) | CHAMP Bézier 12 punti 0,25 s; Pupper velocità costante + altezza triangolare 0,15 s; XGO lift/move/fall lineari |
| Stabilità | statica, margine +0,9 mm | statica, margine +2,1 mm | dinamica: COM sulla linea di supporto (contatto a striscia ±4 mm) |
| Prova | VH (dev.17) | VO, collaudo da fare | nessuna per MATDOG (serve modello dinamico + drive ≥ {2000,200}) |

Fonti e cosa si è preso: **XGO-Lite** (`xgolite-low-level-reconstruction`, milestone G/G.2, firmware v4.3.7, statico): modo 0 trot duty 0,625 (lift/move/fall 1:2:1), modo 3 trot 0,5, modo 1 crawl duty 0,75 senza sway, modo 2 crawl intermittente (corpo fermo durante gli swing); movimento orizzontale dello swing a velocità costante. **DOGZILLA/Yahboom** (PDF del corso, hash verificati): solo interfacce (`speed = step frequency × width`, passo ±25, pace normal/slow/high), nessun motore di gait. **CHAMP** (`libchamp`: `phase_generator.h`, `trajectory_planner.h`, `leg_controller.h`, `gait.yaml`): solo trot, `stance_duration` 0,25, swing Bézier, `com_x_translation`. **Stanford Pupper** (`Gaits.py`, `StanceController.py`, `SwingLegController.py`, `Config.py`): trot con overlap 0,10 s / swing 0,15 s, `x_shift`, swing a velocità costante. Riusati: regola d'impronta Raibert (già in MATDOG), **traslazione del COM** (`com_x_translation`/`x_shift`, qui derivata dal COM URDF), **swing a velocità costante** (qui C1 a rampe lineari). Scartato per ora il trot: richiede stabilità dinamica e accelerazioni di swing (~13–20 rad/s²) oltre il drive qualificato.

## 3. RCA del vincolo v1.0.1

- **Osservazione:** 9,23 mm/s; ogni cadenza > ~1,1× falliva la prova STOP (`compensationSafe`/StopPlan) sull'UPPER anteriore al bordo inferiore di `kHeight` (VO, report precedente).
- **Prima divergenza (VO):** con l'IK di produzione a quota 120 mm il box `kHeight` consente al piede anteriore dx ∈ [−80, +38] mm e al posteriore [−70, +47] mm dai contatti canonici, ma la legge **centrava la corsa su dx = 0** e ci sommava ±10 mm di sway: la corsa toccava il bordo anteriore (15 tick) usando solo il 60 % dell'intervallo utile.
- **Causa:** il COM URDF sta **9,4 mm dietro** il centro dei contatti canonici; lo sway ±10 mm compensava questo sbilanciamento e consumava corsa e accelerazione (trasferimenti di 20 mm in finestre di 0,075 cicli).
- **Concause:** profilo di swing smootherstep (picco di velocità 1,875× la media): a velocità costante la v_max del giunto non dipende dalla falcata (VO: 1,25–1,29 rad/s per 60/80/100 mm a 15,4 mm/s), quindi il limite è il profilo, non la corsa.
- **Non cause:** IK, CPU, box troppo stretto in assoluto.

## 4. Legge scelta (Gait.cpp, tre primitive, default = legge v1.0.1 bit a bit)

1. `comShiftXM` — offset longitudinale costante del corpo, sommato allo sway con le stesse finestre d'ingresso/ricentraggio (si parte e si arriva nella stand canonica). 11 mm (ottimo fra 9,4 geometrico e 13; scansione).
2. `swingRamp` — velocità orizzontale del piede trapezoidale, rampe lineari su 1/3 dello swing (C1). La frenata finale (1/3 dello swing) dura più dell'arresto StopPlan dal picco (1,5 v/a): con rampe 0,25 l'inviluppo pitch-down a 100 mm collassava proprio su quella prova (attribuito per giunto).
3. `settleCycles` — finestra di ricentraggio dopo l'ultimo appoggio (0,1 ciclo = 0,5 s a corpo fermo).

Più, in `SimpleMotion`: duty 0,80, periodo 5,0 s, sway 3 / 1,5 mm, falcate 50/75/100 mm; **clock d'avvio con rampa C2 di 1 s** (`walkClockS`, la stessa del recupero dopo STOP) per WALK STANDARD (il joystick mantiene il clock costante certificato); `walkGait()`/`steerGait()` esposte (unica definizione per firmware e strumenti); il joystick ha costanti proprie (periodo 6,5 s, duty 0,825, sway 10/2 mm) e resta identico. Scartati con misura: offset differenziale anteriore/posteriore (+5 %), profilo di sollevamento rapido (a_max +25 %), rampa d'avvio 0,5 ciclo (supporto −0,98 mm), arresto < 1 ciclo (−258 tick fuori box), sway 5 mm (a_max 8,3–9,2 rad/s²).

## 5. A/B offline (stesso banco, stessa semantica)

`scripts/offline_motion/run_walk_support_check.sh` (supporto/ZMP/v/a/box) e `run_walk_tilt_envelope_check.sh` (IMU, RAW, StopPlan):

| Legge | v mm/s | supporto min / regime mm | ZMP mm | v_max rad/s | a_max rad/s² | margine UPPER / LOWER / HIP tick |
|---|---:|---:|---:|---:|---:|---:|
| v1.0.1 WALK45 | 6,92 | 0,93 / 2,08 | 0,95 | 0,731 | 4,37 | 52 / 213 / 6,9 |
| v1.0.1 WALK60 | 9,23 | 0,88 / 2,44 | 0,91 | 0,822 | 4,47 | 15 / 187 / 6,9 |
| v2 50 mm | 10,0 | 2,13 / 2,69 | 2,10 | 0,711 | 5,81 | 144 / 192 / 9,3 |
| v2 75 mm | 15,0 | 2,13 / 2,95 | 2,10 | 0,986 | 5,81 | 89 / 146 / 9,3 |
| **v2 100 mm** | **20,0** | **2,12 / 3,19** | **2,10** | **1,275** | **5,81** | **32 / 93 / 9,3** |

WALK STOP → STANDING (rig `run_motion_frame_digest.sh`, ideale): attesa al prossimo quarto 0–1,25 s + decelerazione 5,0 s + ricentraggio 0,5 s + arrivo 0,14 s = 6,62 s al caso di 1 s d'attesa (v1.0.1: 7,12 s rig, 8,9 s hardware). La decelerazione resta di 1 ciclo per necessità geometrica (ogni zampa deve fare un ultimo passo verso la stand canonica); si accorcia col periodo.

## 6. STOP, sicurezza e inviluppi IMU

- **STOP operatore** invariato (`beginStop`, StopPlan, storia ≥ 1 periodo). Durata del piano alla velocità di picco: 0,25 s (v1.0.1 0,16 s). Stress 36 semi (comando a ogni release, jitter 0–19 ms, release perse): **0 fault, 18/18 STOP verificati**, 0 frame fuori box, passo RAW max 20 tick (VO).
- **WALK STOP/SETTLE** distinto e invariato nella logica (decelerazione geometrica, ultimo appoggio canonico).
- **Prova STOP con l'arrotondamento RAW peggiore (nuovo criterio dello strumento):** la secante letta dal firmware può mostrare ⌈|Δq|/tick⌉ tick per periodo. Con questo criterio la v1.0.1 passa a tutti i vertici dei suoi inviluppi, e la v2 con sway laterale 2 mm **falliva** a tilt zero (anca a −1,7 tick nella finestra LF→RH): da qui lo sway 1,5 mm (anca +9,3 tick).
- **Inviluppi IMU ricalcolati** (`--search 16`, poligoni convessi interni, ×0,85 per slew/T2/ritardi; verifica su vertici, punti medi e quarti, 187 330 controlli per falcata, 0 fallimenti): 50/75 mm roll ±1,09°, pitch +3,8°/−4,7°; 100 mm roll ±1,09°, pitch +2,2°/−1,07° (v1.0.1 60 mm: ±0,82°, +5,8°/−0,48°). A 100 mm il pitch-up è limitato dal bordo superiore `kHeight` del LOWER posteriore a fine corsa (attribuzione per giunto); oltre l'inviluppo la correzione viene saturata (`ENVELOPE`), non è un fault.

## 7. Difetti trovati e corretti in integrazione (RCA)

| Osservazione | Prima divergenza | Causa | Patch | Prova |
|---|---|---|---|---|
| FAULT in recupero dopo STOP, roll 0,57° | StopPlan: anca LH −0,1 mrad dal box, secante 3 tick | trasferimento laterale 4 mm in 0,25 s + correzione roll al bordo + durata comune fissata dal LOWER | sway laterale 1,5 mm; inviluppi col criterio d'arrotondamento peggiore | suite STOP-support PASS |
| Passo RAW 24 tick in 20 ms (TX 7 ms, release in ritardo) | ri-sincronizzazione T2 in ritardo inviava la soluzione al punto di griglia | frame 15 ms indietro → la release dopo recupera 35 ms di legge, secante STOP ×1,75 | ri-sincronizzazione in ritardo risolve il proprio `now` (sempre una soluzione per release) | `walkTxLatency` PASS (difetto latente anche in v1.0.1, 18,7 tick) |
| Gate joystick FAIL (random_4, 10 raggi a −3 mrad) | sfasamento di 0,5 s fra comandi casuali e fasi | rampa d'avvio applicata anche al joystick | rampa solo per WALK STANDARD | gate PASS, numeri identici a `4c897ed` |

## 8. CPU, memoria, precisione

- **Costo per release (proxy libm host, NON tempo ESP32-S3):** WALK 100 mm p50/p90/p99/max 242/314/472/537 (`4c897ed` WALK60 225/261/471/497, v1.0.1 438/510/545/555); joystick invariato. Tempo Xtensa: **UNK** fino alla misura (istogrammi D11 nel comando).
- **APP / RAM / stack:** §10.
- **float/double (non respinto):** errore misurato del float (sessione precedente, 78 200 casi) ≤ 6,1·10⁻⁷ rad = 0,0004 tick, contatto < 0,06 µm: tre ordini di grandezza sotto il quanto RAW (1,53 mrad) e sotto il gioco servo. I fallimenti venivano dai predicati di validità tarati sul double (FK 1e-9 m, ortonormalità 1e-10), non dalla fisica. Proposta (non implementata qui per non mescolare il cambio della legge con un cambio della catena numerica di sicurezza): IK/FK/contatto in float con predicati derivati dalla precisione (residuo 1 µm, ortonormalità 1e-5), prove di sicurezza finali (box, RAW, StopPlan) sui 12 valori decodificati come oggi. Beneficio atteso **HYP** 3–10× sulla parte numerica (FPU single vs soft-double), da misurare sul chip.
- **−O2/IRAM:** nessun dato nuovo oltre il report precedente (−O2 selettivo +16,7 kB APP, +2,1 kB stack STOP, tempo N.D.): non adottati.

## 9. WALK FAST e TROT

- **WALK FAST = stessa legge v2, periodo più corto + drive più alto** (nessuna nuova state machine): margine ZMP quasi costante (+2,05 mm a 3,35 s, +1,96 mm a 2,45 s) perché gli spostamenti del corpo avvengono a quattro appoggi. **1,5× (29,9 mm/s)** a 3,35 s richiede drive `{1500,100}` (v 1,90 / a 12,8, 83 %); **2× (40,8 mm/s)** a 2,45 s richiede `{2000,200}` (2,60 / 24,0); StopPlan nominale fattibile in entrambi (VO). Serve qualifica hardware del profilo (tracking, corrente, termica ID21/31) e ricalcolo degli inviluppi: **NOT VALIDATED, non implementato**.
- **TROT:** altra sequenza (coppie diagonali, duty 0,5–0,65 con overlap a 4 appoggi, impronte Raibert come CHAMP/Pupper/XGO modo 0). Con contatto a striscia (±4 mm attorno alla diagonale) e COM che attraversa la diagonale di ±0,57×(2/3 della corsa) serve un modello dinamico e swing da 0,3–0,4 s (≥ 13 rad/s²): fuori dal drive qualificato. Non implementato.

## 10. Build candidata (VO)

| Campo | Valore |
|---|---|
| Sorgente | **`c68842edb0de47d9609450ea48d985d8cb2f888f`**, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0` |
| Profilo / layout | `ROBOT_POWERED`, `OTA_INGEST_ENABLED=0`, FQBN pinnato, `MATDOG_16M_2x5M_NVS_V1` (tabella `8f756ecb…`, identica a dev.17 e `4c897ed`) |
| Versione | `1.0.2-dev.2` (proposta, da concordare prima del flash) |
| APP | `MATDOG_Controller.ino.bin` **1.721.040 B** (`4c897ed` 1.718.736: +2.304 B; dev.17 1.707.808: +13.232 B), slot 32,8 % |
| SHA256 APP | **`0d0a7cbe2f485fca145fa074b1621003c151a1f712437402625b6c777531ceb9`** (verificato con `sha256sum`) |
| RAM statica | 137.980 B (`4c897ed` 137.908: +72 B) |
| Frame di stack (ENTRY decodificato) | `update` 1.296, `generateWalk` 1.200 (+16), `nominalRelease` 688, `lerpFrame` 352, `solveGaitInto` 1.232, campionatore legge 544 (+16), `compensateLevel` 592, `compensationSafe` 800, `beginStop` 688 B: catena nominale +16 B; high-water reale N.D. |
| Percorso (solo ASUS) | `~/MATDOG/build-output/gait-redesign-20261108/build-c68842e/artifacts/ROBOT_POWERED-ota0/` + `README_CANDIDATE.txt` |
| Static audit | **PASS** (`STATIC_AUDIT = PASS`, `HOST_TESTS = PASS` incluso `MOTION_CONVERGENCE_SOURCE`) |

**Non flashata.** Flash APP-only, se autorizzato: `scripts/flash_app_only.sh` con verifica del manifest; NVS gen-1, partizioni e bootloader non vengono scritti. La candidata `4c897ed` (APP `03c84606…`) è superata.

## 11. Limiti e UNKNOWN

Nessuna prova su servo, contatto, attrito, stabilità dinamica reale, tempo di calcolo sul chip, lag di tracking a 1,27 rad/s, scuff del piede (lo swing trapezoidale avanza ~5 mm mentre il piede sale al primo millimetro, v1.0.1 ~3 mm a parità di falcata), termica. Il modello COM è CAD (REV01-FIX con Jetson, 2,928 kg). Laterale e yaw invariati: limitati dal box `kSteer`, dal raccordo del piede (banda 5–9 mm) e dalle finestre di sway del joystick; migliorarli richiede una legge steered nuova e la ri-certificazione di `kSteer`.
