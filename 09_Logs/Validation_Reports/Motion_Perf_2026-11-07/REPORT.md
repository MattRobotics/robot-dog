# MATDOG Motion Performance — candidata 1.0.2-dev.1 (motion-perf-next) — 2026-11-07/08

**Natura:** sessione autonoma offline (analisi, patch, test host, build). **Nessun flash, riavvio, movimento, scrittura NVS/EEPROM o calibrazione.** Il robot resta spento su fixture con la Stable `v1.0.1` (firmware `1.0.1-dev.17`, APP SHA256 `14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83`) installata; dev.16 (`73952eec…`) è il rollback. Tutto quanto segue è **IMPLEMENTED NOT VALIDATED HARDWARE** salvo dove marcato VERIFIED HARDWARE (letture delle tracce del robot) o VERIFIED OFFLINE (risultati riproducibili su host).

Classi usate: **VERIFIED HARDWARE** (dati delle tracce dev.16/dev.17), **VERIFIED OFFLINE** (host, riproducibile), **IMPLEMENTED NOT VALIDATED HARDWARE**, **HYPOTHESIS**, **UNKNOWN**.

## 0. Risposte brevi

| Domanda | Risposta | Classe |
|---|---|---|
| Dove vanno i 20 ms su dev.17? | WALK STANDARD: release piena tipica ~17–18 ms (NOMINAL ~6 + COMPENSATE ~8 + I/O ~3,5), leggera T1 ~11–12 ms; gli overrun (9,4 %) sono release dello **stesso lavoro** inflazionate ×1,7–2 (max 33 ms). JOYSTICK: piena tipica ~29,5 ms (NOMINAL ~10 + COMPENSATE ~13–16, firma 8 IK), leggera ~17,5 ms: strutturalmente oltre budget, 20 % overrun, 1,3 perse/s. HOLD: 0 overrun. | VERIFIED HARDWARE (§2) |
| Prima divergenza riducibile offline | L'IK nominale analitica del gait (4 contact IK + FK di verifica + condizionamento) è la voce numerica più pesante della release WALK: 104 trig + 48 inverse + 44 sqrt/hypot + 31 remainder su ~380 chiamate libm; la compensazione IMU la segue (50/5/25). | VERIFIED OFFLINE (§3) |
| Cosa fa la candidata | A2 (IK esatta più economica), A1 (campionamento gait senza jet inutilizzati), T2 (nominale esatto a release alterne con interpolazione a metà), T1b (riuso T1 anche durante i retarget joystick). Legge del gait, box, geometria, calibrazione, STOP e profilo servo invariati. | IMPLEMENTED (§4) |
| Quanto costa in meno (proxy host) | libm/release WALK45: media 383 → 236 (−38 %), p50 440 → 223, p90 512 → 269; JOYSTICK: media 484 → 306 (−37 %), p50 523 → 261, p90 621 → 465; HOLD invariato. p99/max invariati dove T1 non è leggera. Frame di stack della catena nominale +112 B, compensazione −592 B (§9). | VERIFIED OFFLINE (§5) |
| Quanto vale sul robot | **Non misurabile offline.** Se il tempo resta proporzionale al calcolo (RCA dev.15, HYPOTHESIS), la release piena joystick tipica scende sotto i 20 ms e la WALK piena da ~17,5 a ~12 ms; l'inflazione ×2 resta UNKNOWN e continuerà a produrre overrun, in numero minore. | HYPOTHESIS |
| Revisione T2 (retarget/jitter) | Difetto **confermato e corretto** (§12): il campionamento sulla griglia gonfiava la secante RAW della prova di arresto sotto jitter → fault dove la Stable completava; ora riferimento a `now` e rate STOP su ≥ 1 periodo; 36 semi: 0 fault (Stable 3). | VERIFIED OFFLINE |
| Completamento integrato (A–D, §13) | Rate STOP su storia ≥ 1 periodo al posto del floor; una sola risoluzione per release; **sway joystick proporzionale** (gate PASS, +0,44 mm min); cadenza: nessun preset ammissibile senza qualifica del drive (sway X 5 mm → supporto −1,49 mm); WALK_SETTLE decomposto; istogrammi D11. | VO / INV |
| Velocità del gait | Con la legge attuale e il profilo servo `{1000,50}` il modello ammette solo ~1,1–1,25× e in modo fragile; `kWalkStopCycles` 0,5 è geometricamente invalido; ≥1,5× richiede un profilo drive ≥ `{1500,100}` (2× ≥ `{2000,200}`) da qualificare su hardware. **Nessun cambio di cadenza nella candidata.** | VERIFIED OFFLINE (modello, §6) |

## 1. Identità e perimetro

| Oggetto | Valore |
|---|---|
| Base | `main` `40728b7ed2370f1cbfc5a73238fb0054af811ed7` (HEAD verificato, solo documentazione oltre il tag `v1.0.1` = `35f5ca4`) |
| Branch / worktree | `feat/motion-perf-next-20261107`, `~/MATDOG/worktrees/matdog-motion-perf-next-20261107` |
| Commit firmware | 7029e17 (A2 + digest tool) → 99329c5 (probe) → 52e2865 (A1) → d7ed9da (T2) → affa60c (T1b + test) → d890da7 (checker) → 67a4bc6 (identità 1.0.2-dev.1) → 27e60dc (checkpoint) → 2569401, e01e95a, **2d95b99** (T2 frame di stack: `solveGaitInto`, frame scritti in place, rimozione `reuseGait_`) → 27c409f (pin del gate sorgenti, solo dati di test) → 6d44393 (dossier) → 131ca53 (revisione §12) → e281202 (docs) → 172726d (rate STOP su storia ≥ 1 periodo, una sola risoluzione per release) → 05e8423 (sway joystick proporzionale) → **128c480** (istogrammi D11, scenari tool) |
| Versione proposta | `1.0.2-dev.1` (`kFirmwareVersion`), da concordare prima di qualunque flash |
| Build candidata | §7: `128c480`, APP 1.718.304 B, SHA256 `e3e69368…` |
| Kit di ingresso | `~/MATDOG/verification-artifacts/MATDOG_MOTION_PERF_KIT_20261107/` — ZIP `6c183db5…`, tracce dev.16 `3e21a255…`, dev.17 `b1edd6cf…` (SHA256 verificati) |

Invariati: `SimpleMotion.h` costanti del gait (`kWalkPeriodS` 6,5, `kWalkDuty` 0,825, lift 8 mm, quota 120 mm, stride 30/45/60, sway 2/14 mm, `kJoystickRampCycles` 2), `kWalkDrive{1000,50}`, box `kHeight`/`kSteer`, inviluppi `kWalkTilt45/60`/`kSteerTilt`, calibrazione gen-1, URDF, STOP/SAFE_OFF/deadman, HTTP, dashboard, `MotionService`.

## 2. RCA causale sul budget dei 20 ms — tracce dev.17 (VERIFIED HARDWARE)

Metodo: osservazione → percorso esecutivo → prima divergenza misurabile → causa primaria / concausa / correlazione / ipotesi → patch minima → regressione → confronto con v1.0.1. Strumento: [`tools/trace_phase_analysis.py`](tools/trace_phase_analysis.py) (sola lettura sui JSON originali, campioni deduplicati per `now_ms`, finestre non sommate). Le grandezze sono quelle già esportate da `MotionTiming` (nessuna strumentazione nuova nel firmware): `workUs` = tempo di esecuzione della release (parete, include preemption); `latenessUs` = latenza di scheduling all'inizio della release; `sinceReleaseUs` = intervallo fra fine della release precedente e inizio di questa (lavoro del loop fuori Motion); `externalUs` = tempo dei sottosistemi nel loop (router, IMU/BMS, HTTP, telemetria); `missed` = periodi interi saltati; `overruns` = release con `workUs` > 20 ms.

**1. Osservazione.** Finestra (35), WALK STANDARD 45/60: 3287 release, 1 persa, 310 overrun (9,4 %), max 33,4 ms. Finestra (36), joystick: 2353 release, 62 perse, 474 overrun (20,1 %), max 47,2 ms, 1,3 perse/s. HOLD (34)/(35)/(36): 0 overrun su 4285 release. Le due finestre dedupicate insieme (480 campioni): WALK_STANDARD 5640 release, 63 perse, 784 overrun.

**2. Percorso esecutivo.** `Controller::update` → `SimpleMotion::update` (griglia 20 ms, `nextUs_`) → READ (2 letture servo) → `generateWalk`: `gaitTrackCartesian` + `solveGait` [NOMINAL] → `compensate`/`compensateLevel`: fit supporto + IK inclinata o riuso T1 + `compensationSafe` (box, RAW, StopPlan) [COMPENSATE] → `send` (RAW, SYNC_WRITE) [CONVERT/FRAME]. Il loop serve una richiesta HTTP per giro fra una release e l'altra, sullo stesso core.

**3. Prima divergenza misurabile.** Sul robot, nelle release **tipiche** (le «previous» dei record di massimo):

| Caso | READ | NOMINAL | COMPENSATE | CONVERT+FRAME+REST | Totale |
|---|---:|---:|---:|---:|---:|
| WALK piena tipica | 1,6–1,8 | 6,0 | 7,0–8,3 | 2,1 | 16,7–18,4 ms |
| WALK leggera T1 tipica | 1,6–2,1 | 5,8–6,5 | 2,1–3,5 | 2,1 | 11–12 ms |
| Joystick piena tipica | 1,6 | 10,1 | 13,3–15,8 | 2,0 | **27–30 ms** |
| Joystick leggera T1 tipica | 1,6 | 10,1 | 2,7 | 2,0 | 17,5 ms |

e nei record di **massimo**: WALK 28–33 ms con la **stessa firma di lavoro** (1/4/8/15) delle tipiche, NOMINAL 10,4–11,6 e COMPENSATE 12,6–18,0 ms; joystick 38–47 ms (firma 1/8/12/23), NOMINAL 16,5–18,5, COMPENSATE 17,4–25,7 ms. `sinceReleaseUs` 1 ms e `externalUs` ≈ 1 ms nelle finestre in overrun: il loop fuori Motion non è la divergenza. `latenessUs` fino a 22 ms nel joystick = accumulo di overrun consecutivi (regime saturo), non latenza di scheduling intrinseca.

**4. Separazione.**
- *Causa primaria (joystick)*: il contenuto numerico della release piena (IK nominale a contatto sul raccordo ~10 ms + compensazione IMU 8 IK ~13–16 ms) supera il periodo da solo. Dimostrato dalle release tipiche, non dai soli massimi.
- *Causa primaria (WALK STANDARD)*: nessuna strutturale: le release tipiche stanno sotto i 20 ms. Gli overrun sono release a **lavoro identico e tempo ×1,7–2** — *concausa C1*, già osservata in dev.15, **UNKNOWN** nella sua origine (interferenza I-cache/flash dal core 0, interrupt). Il JSON non la può discriminare; servirebbe il dump USB `@MOTION TIMING` già esistente (nessuna strumentazione nuova).
- *Correlazioni non causali*: HTTP/polling (`externalUs` ≈ 1 ms), lettura servo (READ 1,6–2,1 ms stabile), WALK_SETTLE (poche release).
- *Ipotesi*: il tempo Xtensa resta proporzionale al conteggio di operazioni soft-double (rapporto quasi costante host/robot in dev.15): è l'ipotesi su cui poggia la stima del beneficio.

**5. Patch minima sulla causa dimostrata** (§4): ridurre il lavoro numerico per release senza toccare legge, geometria, prove di sicurezza: A2/A1 (stesso risultato bit a bit), T2 (metà delle release senza IK nominale), T1b (più release leggere quando il joystick è in uso). Non si è toccata la concausa C1 (nessuna prova della causa → nessun intervento su cache/IRAM/`-O2`, già scartato in dev.17).

**6–8. Regressione, confronto, conservazione**: §4–§5 e §8; ogni passo conservato solo con digest identico (A1/A2) o con errore misurato < 1 tick e suite PASS (T2/T1b).

## 3. Costo per sezione sul rig host — VERIFIED OFFLINE

`scripts/offline_motion/run_motion_cost_probe.sh` (SimpleMotion di produzione, IMU inclinata costante, conteggio chiamate libm per release; ora conta anche hypot/remainder/floor). v1.0.1, chiamate per release WALK45 (trig / inverse / sqrt+hypot):

| Sezione | trig | inverse | sqrt+hypot | Note |
|---|---:|---:|---:|---|
| gait sample (legge, jet) | 20 | 0 | 0 | jet di velocità/accelerazione inutilizzati dalla release |
| solveGait (IK nominale analitica ×4, FK verifica, condizionamento) | 104 | 48 | 44 | + 31 `remainder` (boundedAngle) |
| supportLevelTarget (fit supporto, 3 FK) | 10 | 3 | 8 | |
| compensationTarget (IK inclinata Newton ×4) | 50 | 5 | 25 | joystick: 88/8/28 (8 IK) |
| compensationSafe (StopPlan) | 0 | 0 | 18 | |
| altro (attitude, headed, …) | 10 | 3 | 4 | |

Sull'host la seconda risoluzione «edge» del joystick scatta nel 10–12 % delle gambe (la firma 8 IK del robot è il **massimo**, non il tipico): non era la leva principale.

## 4. Patch della candidata

| ID | File | Cosa | Prova |
|---|---|---|---|
| A2 | `motion/LegInverseKinematics.cpp` | niente `remainder` per angoli già in (−π, π]; un solo `atan2` per i due rami del ginocchio (atan2 dispari in y); costanti di catena (b, φ, outer) memorizzate per input esatti | digest frame RAW **identici** su 8 scenari (`run_motion_frame_digest.sh`) |
| A1 | `motion/Gait.cpp`, `generateWalk` | legge del gait templata sul tipo di clock: le release campionano senza jet (A1); il percorso con jet resta per gli sweep | digest identici; dump 5301 campioni con jet **bit-identico** (posizioni, velocità, accelerazioni, condizionamento) |
| T2 | `SimpleMotion::nominalRelease`, `lerpFrame` | soluzioni esatte della legge sui punti di griglia a due periodi (una risoluzione ogni due release, track campionato in ordine); **ogni release invia l'interpolazione lineare in spazio giunti fra le due soluzioni che racchiudono il proprio istante** (legge a `now` come v1.0.1, §12), con campione cartesiano ricostruito a quell'istante per la compensazione; ricostruzione di entrambe le soluzioni su cambio strutturale della legge (terminale WALK STOP, recupero da STOP, fine clock) o punto di griglia fuori bracket; retarget steerWalk tollerati; **tutte** le prove (pointSafe, box WALK, RAW, loaded, StopPlan) a ogni release; T1 leggera solo sulle release che pagano una soluzione | interpolato vs esatto a legge ferma 0,57–0,75 tick (< 1 tick RAW); alternanza 663/662 e 1109/1110; 2–3 ricostruzioni per camminata; stress retarget+jitter 36 semi: 0 fault (Stable 3), §12; test `lookaheadWalk` |
| T1b | `compensateLevel` | il riuso leggero della correzione IMU non è più annullato da un retarget joystick | joystick casuale: release leggere 8 % → 40 %, errore shadow T1 invariato (1,15–1,25 tick; meccanismo dev.16) |

Non adottati (misurati e scartati): `kWalkStopCycles` 0,5 (§6), `kJoystickRampCycles` 1 (gate inviluppo joystick FAIL: `random_1` → `FAULT WALK_IK_OR_WORKSPACE`, 110 raggi di tilt fuori; test `joystickWalk` fallito), `-O2` selettivo (già scartato in dev.17, nessuna evidenza nuova), `double→float` (non affrontato: richiede prova differenziale dedicata).

## 5. A/B offline — VERIFIED OFFLINE (proxy soft-double, NON tempo ESP32-S3)

Chiamate libm per release (`run_motion_cost_probe.sh`, 975 release per scenario):

| Scenario | v1.0.1 media | candidata media | Δ | v1.0.1 p50 / p90 / p99 / max | candidata p50 / p90 / p99 / max |
|---|---:|---:|---:|---|---|
| HOLD100 inclinato | 97 | 97 | 0 | 177 / 177 / 213 / 227 | invariato |
| WALK45 inclinato | 383 | 236 | −38 % | 440 / 512 / 542 / 555 | 223 / 269 / 487 / 507 |
| WALK60 inclinato | 381 | 231 | −39 % | 438 / 510 / 545 / 555 | 225 / 261 / 471 / 497 |
| JOYSTICK inclinato | 484 | 306 | −37 % | 523 / 621 / 970 / 1008 | 261 / 465 / 845 / 895 |

Firme di lavoro (`solves/ik/newton/fk`) invariate ai massimi (1/4/8/15, 1/8/12/23): sul robot la T2 si vede nella sezione NOMINAL (metà delle release ≈ 0 solve), non nei contatori di compensazione. Il p99/max resta alto nelle release in cui T1 non è leggera (correzione IMU in slew) e sui lift-off del joystick (integrazione della rampa). **Tempo host** medio WALK45 17,7 → 12,4 µs: solo indicativo.

## 6. Fase B — prestazioni del gait (modello, nessun hardware)

Strumento nuovo: [`scripts/offline_motion/run_walk_tilt_envelope_check.sh`](../../../05_Firmware/MATDOG_Controller/scripts/offline_motion/run_walk_tilt_envelope_check.sh): percorre la legge sulla griglia dei 20 ms (rampa di avvio, cicli stabili, ogni terminale WALK STOP a quarti e finestra di settle), risolve l'IK nominale e, su vertici e punti medi dell'inviluppo IMU `kWalkTilt45/60`, l'IK livellata come `compensationTarget`; applica le prove di `compensationSafe` (limiti, `kHeight` prima/dopo arrotondamento RAW, StopPlan sulla secante RAW col drive WALK); riporta picchi di velocità/accelerazione articolare dai jet, passo RAW per release, accelerazioni del corpo. Validato sulla legge v1.0.1: PASS, 0 fallimenti su 30.240/55.440 verifiche.

| Variante | Esito | Perché |
|---|---|---|
| v1.0.1 (45/60 mm, 6,5 s, stop 1) | PASS | margine box compensato 6,1 tick; v 0,73–0,82 rad/s (drive 1,53); a 4,4–4,5 rad/s² (drive 7,67); a_y corpo 0,10 m/s² |
| `kWalkStopCycles` 0,5 | **FAIL** | la regola di ancoraggio (`td>terminal-1`) manda i touchdown dell'ultimo ciclo al punto terminale mentre il corpo è a piena velocità: nominale −59/−139 tick fuori `kHeight`. WALK_SETTLE si accorcia solo via cadenza |
| cadenza 1,25× (5,2 s) | 45 mm PASS; 60 mm FAIL | StopPlan infattibile in 6/44.396 prove di decelerazione (in firmware → `WALK_IK_OR_WORKSPACE` → SAFE_OFF in cammino) |
| scansione fine 60 mm | 6,0 e 5,8 e 5,4 s PASS, **5,6 s FAIL** | margine fragile: dipende dall'allineamento dei terminali a quarti con la griglia |
| 1,5× (4,33 s) | FAIL | a_max 9,8–10,1 rad/s² > 7,67; StopPlan 69/492 fallimenti |
| 2× (3,25 s) | FAIL | a_max 17,5–17,9; v_max 1,46–1,64 ≈ cap 1,53; StopPlan 804/2131; a_x corpo 1,95 m/s² (0,2 g) nelle finestre di sway |
| 3× (2,17 s) | FAIL | a_max 39–40 rad/s²; v 2,2–2,5 rad/s |
| what-if drive `{1500,100}` | 1,5× PASS (45 e 60); 2× FAIL | StopPlan 32/307 a 2× |
| what-if drive `{1500,150}` | 2× PASS a 45, FAIL a 60 (1 prova) | |
| what-if drive `{2000,200}` | 2× PASS (45 e 60) | v_max 1,64 vs 3,07; a_max 17,9 vs 30,7 |

Conclusioni Fase B: la velocità utile è limitata dal **profilo servo** (`kWalkDrive{1000,50}`, cioè 1,53 rad/s e 7,67 rad/s²) attraverso la prova di arresto e dai picchi di accelerazione della legge (lift 8 mm in 0,175 cicli, sway fore/aft ±10 mm in 0,075 cicli), non dall'IK. Il percorso verso 2× passa da una qualifica hardware del profilo drive (≥ `{1500,100}` per 1,5×, ≥ `{2000,200}` per 2×: corrente, termica ID21/ID31, tracking) e, a 2×, da una revisione dello sway fore/aft (a_x 1,95 m/s²); il modello **non** copre tracking, attrito, stabilità dinamica. Nessuna cadenza è stata inserita nella candidata.

**Sway proporzionale continuo del joystick (non implementato).** Oggi `steerGait` impone 14 mm per qualunque joystick (WALK 2 mm). La legge cambia l'offset laterale solo nelle finestre a quattro appoggi e `gaitRetarget` rifiuta variazioni di `swayYM` in cammino: un'ampiezza funzione continua di |laterale|/|yaw| richiede (a) un'estensione di `lateralRegular` che porti ogni transizione dal valore tenuto al nuovo ±a (oggi salterebbe da −a_old a −a_new all'inizio della finestra), (b) la ri-certificazione del box `kSteer` e del gate di inviluppo (`run_joystick_envelope.sh` + `certify_steer_box.py`, ~10–20 min) sulle configurazioni intermedie 2–14 mm, (c) il margine statico di supporto per ampiezze < 14 mm con componente laterale non nulla (a 12 mm il margine casuale era −0,90 mm in dev.15). Lasciato fuori perimetro per mancanza di prova nel modello.

## 7. Build candidata — VERIFIED OFFLINE

| Campo | Valore |
|---|---|
| Sorgente | `128c480` (candidata integrata, §13), `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0` |
| Profilo | `ROBOT_POWERED`, `OTA_INGEST_ENABLED=0`, FQBN pinnato di `build.sh`, layout `MATDOG_16M_2x5M_NVS_V1` (tabella `8f756ecb…`, identica a dev.17) |
| Versione | `1.0.2-dev.1` (nome proposto, da concordare prima del flash) |
| APP | `MATDOG_Controller.ino.bin` **1.718.304 B** (dev.17: 1.707.808, **+10.496 B, +0,61 %**), slot 32,8 % |
| SHA256 APP | **`e3e693689538373bd2e2452d518cdbe50a3af0aa97de72387dd85d05c1fa86ba`** |
| RAM statica | 137.908 B (dev.17 137.684: +224 B: storia riferimenti, frame T2, istogrammi, buffer stato +128) |
| Percorso (solo ASUS) | `~/MATDOG/build-output/motion-perf-next-20261107/build-128c480/artifacts/ROBOT_POWERED-ota0/` (APP, ELF, map, manifest, sdkconfig) + `README_CANDIDATE.txt` |
| Build precedenti | `build-131ca53` (`e32c062c…`), `build-2d95b99` (`492f5ac1…`), `build-27e60dc`, `build-2569401`, `build-e01e95a`, `compilecheck-dirty`: **superate, non flashare** |

**Non flashata.** Flash APP-only, se autorizzato: `scripts/flash_app_only.sh` con verifica del manifest; NVS gen-1, partizioni e bootloader non vengono scritti.

## 8. Regressioni eseguite (offline)

| Suite | Esito |
|---|---|
| `run_simple_motion_tests.sh` (+ `lookaheadWalk`, `-DMATDOG_T2_SHADOW`) | PASS (board plant WALK45/60 errore stabile max 0,14°, STOP in ogni fase WALK, joystick, PRONE, STAND) |
| `run_motion_frame_digest.sh` | A1/A2 digest identici a `main`; T2/T1b: tutti gli scenari a STANDING, STOP verificato, interpolazione < 1 tick a legge ferma; stress retarget+jitter 36 semi 0 fault / 18 STOP verificati (§12) |
| `run_stop_support_tests.sh`, `run_prone_tests.sh` (dopo la modifica di `beginStop`) | PASS |
| `run_simple_dashboard_tests.sh` (fixture 71, DOM 67 comandi; node 22 + jsdom), `test_http_fence_contention.py` 723, `test_slack_exempt_session.py`, `test_motion_http_transport.py` 97 | PASS (feed Twin/vitest non rieseguiti: viewer non toccato) |
| `run_joystick_envelope.sh` | PASS: supporto statico min 0,55 mm (casuale) / 1,85 mm (stabile), clearance ≥ 2,18 mm, inviluppo tilt 0 fail — identico a dev.15 |
| `run_walk_tilt_envelope_check.sh` (legge v1.0.1) | PASS 45 e 60 mm |
| `run_motion_cost_probe.sh` | §5 |
| `static_audit.py` completo (deps offline pinnate su `PYTHONPATH`) | **PASS** (`STATIC_AUDIT = PASS`, `HOST_TESTS = PASS` incluso `MOTION_CONVERGENCE_SOURCE` dopo il re-pin di Gait.cpp/Gait.h in aaf6d40; log `~/MATDOG/build-output/motion-perf-next-20261107/build-128c480/static_audit_final.log`) |
| Compilazione Xtensa | OK, solo warning preesistenti della libreria SCServo |

Limiti: nessuna prova su ESP32-S3, radio, servo o browser reali; i test DOM/HTTP non sono stati rieseguiti (nessun file di rete/dashboard toccato).

## 9. Stack

Frame di stack statici decodificati dalle istruzioni `entry a1, N` dell'ELF (`objdump -d`; per le funzioni che iniziano con pool di literal il word ENTRY grezzo è decodificato: `byte0=0x36`, `imm12 = byte2<<4 | byte1>>4`, frame = imm12·8). Il task loop ha 16 KB; dev.17 ha misurato un minimo libero di 2.060 B nella traccia (36), su un percorso non identificabile dal JSON (il servizio HTTP sul loop, `HttpTransport::serviceRequest` 7.264 B, è il candidato più profondo).

| Funzione / catena | dev.17 (B) | candidata 2d95b99 (B) |
|---|---:|---:|
| `SimpleMotion::update` | 1.312 | 1.296 |
| `generateWalk` | 1.760 | 1.184 |
| `nominalRelease` / `lerpFrame` (nuove) | — | 688 / 352 |
| `solveGait` / `solveGaitInto` | 1.232 | 32 / 1.232 |
| `compensateLevel` / `compensationSafe` / `beginStop` / `rateFromHistory` | 592 / 816 / 704 / — | 592 / 800 / 688 / 112 |
| **Catena nominale** update→generateWalk→(nominalRelease)→solveGait(Into)→IK→FK | **6.224** | **6.320** (+96); release senza soluzione (lerpFrame+FK) 4.240 |
| **Catena compensazione** update→generateWalk→compensate→…→IK inclinata→FK | **7.040** | **6.448** (−592) |
| Catena compensationSafe/StopPlan | 5.440 | 4.944 |
| Catena beginStop/StopPlan | 2.528 | 2.608 (+80, `rateFromHistory`) |
| Frame massimo del firmware (handler HTTPD, task proprio) | 9.648 | 9.792 (buffer testo di stato +128 B, `HttpTransport::handleMotion`/`handleCalibrationRestore`) |

Frame misurati sulla build `128c480`. Storia della correzione: la prima forma di T2 (build `27e60dc`) aveva `nominalRelease` a **3.184 B** (catena nominale ~8,8 KB, +2,6 KB); la forma «per valore» 2.240/2.608 B; la forma finale scrive i frame in place tramite `solveGaitInto` (un solo corpo, `solveGait` è il wrapper); la revisione §12 aggiunge `lerpFrame` (352 B) sulle release senza soluzione. Misura reale del minimo libero: solo su hardware (piano H0/H2).

## 10. UNKNOWN e rischi residui

- **Inflazione ×2 dello stesso lavoro (C1)**: non attribuita; la candidata riduce il lavoro, non la causa. Strumento minimo: dump USB `@MOTION TIMING` (lateness, `outsideVisitUs`, `externalUs[]`) in WALK/joystick; A/B con radio inattiva.
- **Tempo reale su Xtensa**: il proxy libm non è una misura; i −37/−38 % sono attesi proporzionalmente (HYPOTHESIS), da verificare con `timing.max_work`/`overruns` per fase.
- **T2**: il riferimento sulla release intermedia differisce dalla soluzione esatta fino a 0,75 tick; la T1 già accettata su hardware vale 1,25 tick. Il punto medio non è una soluzione IK: pointSafe/box/StopPlan lo verificano a ogni release; il contatto reale non è comunque misurato.
- **Firma hardware di T2**: sul JSON si vede in `timing.max_work[1]` (NOMINAL) alternato ≈ 0/6 ms; `max_work_solves` non conta il nominale.
- Nessun cambiamento di STOP: la corsia STOP e `beginStop` sono invariati; il tempo di arresto non peggiora per costruzione (meno calcolo per release).

## 11. Decisioni che restano all'operatore

1. Nome della versione (`1.0.2-dev.1` proposto) e autorizzazione al flash APP-only del candidato per il collaudo di [HW_TEST_PLAN.md](HW_TEST_PLAN.md).
2. Se qualificare un profilo servo più alto (`kWalkDrive` ≥ `{1500,100}`) come premessa a qualunque cadenza > 1×.
3. Se finanziare la legge di sway continuo (estensione della legge + ri-certificazione `kSteer`).
4. Se misurare l'inflazione C1 con il dump USB prima di ulteriori ottimizzazioni (float, IRAM).

## 12. Revisione conclusiva T2 sotto retarget e jitter (08/11/2026) — difetto CONFERMATO e corretto

**Domanda.** `sameLawShape()` ignora `advanceXM/advanceYM/yawRad`: il riuso del look-ahead dopo un retarget joystick produce riferimenti coerenti con la soluzione esatta? Il campionamento sulla griglia `nextUs_` invece che a `now` introduce un errore rispetto alla Stable?

**Metodo (rig host, nessuna modifica preventiva).** Due scenari nuovi in `run_motion_frame_digest.sh` (`JOY_retarget_jitter_*`, semi configurabili con `MATDOG_STRESS_SEEDS`): comando joystick cambiato **a ogni release** (passo casuale ≤ 10 % per asse, dentro l'ellissoide, solo avanti), release in ritardo casuale 0–19 ms rispetto al punto di griglia (mai un periodo intero), una release saltata ogni 97, poi WALK STOP (semi dispari) o STOP (pari). I retarget cadono quindi su ogni transizione stance/swing. Per ogni frame trasmesso: box del WALK e limiti gen-1, passo RAW massimo fra frame consecutivi, stato finale; con `MATDOG_T2_SHADOW` il confronto con la soluzione esatta. Il runner compila il tool **dall'albero selezionato** (`MATDOG_SKETCH`): un vizio precedente (header del worktree con sorgenti pristine) rendeva inattendibili i confronti pristine dello stress, ora corretto; i confronti A1/A2 di ieri erano validi (header identici allora).

**Prima divergenza (candidata, VERIFIED OFFLINE).** Fault `WALK_IK_OR_WORKSPACE` dopo 16–41 release, mentre la Stable completava lo scenario. Strumentando `compensationSafe`: `StopPlan FAIL dt=4,2 ms, |Δq|=4 tick, |v|=1,455 rad/s (cap 1,534)`. Il riferimento campionato sulla griglia avanzava di un passo di legge per release, mentre l'intervallo di parete fra due riferimenti emessi era il solo jitter: la secante RAW letta dalla prova di arresto (e da `beginStop`) era gonfiata fino al cap. **Sì, il passaggio da `now` alla griglia introduceva un errore temporale significativo**: fino a un passo di legge (~10–13 tick) per release in ritardo, con la secante incoerente col tempo di parete.

**Patch minima 1 (T2).** Le soluzioni esatte restano sui punti di griglia a due periodi (una risoluzione ogni due release); ogni release invia l'interpolazione lineare in spazio giunti fra le due soluzioni che racchiudono il **proprio istante** (`u=(now−t_e)/2T`), cioè la legge a `now` come nella Stable; il campione cartesiano per la compensazione è ricostruito a quell'istante (appoggi dal lato piantato del bracket, piede in volo = riferimento di contatto dei giunti interpolati con il lift del raccordo rimosso, posa del corpo interpolata, fasi della legge). Release persa o cambio strutturale → ricostruzione di entrambe le soluzioni.

**Seconda divergenza (condivisa con la Stable).** Dopo la patch 1, fault residui con `dt=1,2–2,5 ms, |Δq|=1–2 tick, v=0,94–1,53 rad/s` vicino a un bordo del box: due riferimenti emessi a 1–2 ms di distanza (release molto in ritardo seguita da una puntuale) differiscono per quantizzazione RAW, e 1 tick su 1,6 ms vale ~1 rad/s. **La Stable v1.0.1 fallisce allo stesso modo: 3 fault su 36 semi** (settle del WALK STOP e cammino); la candidata ne mostrava 7/36 perché l'interpolazione si muove alla pendenza media del bracket anche dove la legge istantanea è quasi ferma (più flip di 1 tick a `dt` minuscoli). Sul rig le release sono istantanee: sull'hardware l'intervallo fra due invii è almeno la durata della release (~6–12 ms), quindi lo scenario è più severo del reale (nessun fault di questo tipo nelle tracce dev.16/17).

**Patch minima 2 (prova e STOP, stesso punto in due sedi).** La rate del riferimento per `StopPlan` è letta su almeno un periodo di release (`dt = max(dt, 20 ms)`) sia in `compensationSafe` sia in `beginStop`, che devono restare identiche perché la prova sia garanzia dello STOP reale; `StopPlan.h` dichiara già che al passaggio si assume solo C0 e nessuna derivata del segnale precedente. Una compensazione piena dopo una leggera limita inoltre il proprio passo di slew all'intervallo di parete dall'ultimo riferimento emesso (rate ≤ 0,04 rad/s fra riferimenti consecutivi). Nessuna prova è stata tolta o allentata nei box, limiti, RAW, loaded.

**Risultati (36 semi, VERIFIED OFFLINE).**

| | Stable v1.0.1 | candidata prima della revisione | candidata corretta |
|---|---:|---:|---:|
| Fault `WALK_IK_OR_WORKSPACE` | 3 | 7 (grid) / 7 (lerp) / 6 (slew) | **0** |
| STOP verificati (18 scenari) | 18/18 | — | **18/18** |
| Frame fuori box/limiti | 0 | 0 | 0 |
| Passo RAW max per release | 19–22 tick | 18–23 | 20–23 |

Interpolazione vs soluzione esatta a `now` sullo stesso track (legge ferma): 0,57–0,75 tick (WALK45/60, joystick stabile). Sotto retarget continui il confronto con un track campionato ogni release **non è definito**: i due track pianificano gli appoggi in istanti diversi con goal diversi (fino a ~27 tick ≈ 5 mm di appoggio), lo stesso effetto che una release persa ha su v1.0.1; la prova resta comportamentale (tabella). Regressioni: suite SimpleMotion PASS, gate inviluppo joystick PASS (supporto 0,57 mm), STOP support PASS, PRONE PASS, libm/release +4 (FK del piede in volo).

**Costo e identità.** build CLEAN `131ca53`: APP 1.717.024 B (+1.936 B rispetto alla build pre-revisione, +9.216 vs dev.17), SHA256 `e32c062cee1209ba1511e4637dc313089f46e6804f5cb73a6313766b5acc1819`; frame di stack: catena nominale 6.320 B (dev.17 6.224), compensazione 6.448 (7.040), `beginStop` invariato 704; libm/release +4 (FK del piede in volo nelle release interpolate). Static audit: **PASS** (`STATIC_AUDIT = PASS`, `HOST_TESTS = PASS` incluso `MOTION_CONVERGENCE_SOURCE`; log `~/MATDOG/build-output/motion-perf-next-20261107/build-131ca53/static_audit.log`).

**Idoneità al collaudo supervisionato.** La candidata corretta è idonea al piano `HW_TEST_PLAN.md` (H0–H7); la Stable resta installata. Il piano H2/H5 ora legge `timing.max_work` per quello che è: il record della release di massimo del comando (sezioni correnti e della release precedente), non una misura per release.

## 13. Completamento integrato (mandato 08/11/2026) — aree A–D

Convenzioni: **VH** = VERIFIED HARDWARE (tracce dev.17), **VO** = VERIFIED OFFLINE (host, riproducibile), **INV** = IMPLEMENTED NOT VALIDATED HARDWARE, **HYP** = HYPOTHESIS, **UNK** = UNKNOWN, **N.D.** = non misurato.

### A. Budget 20 ms

**A1 — contributo marginale (VO, `run_motion_cost_probe.sh`, chiamate libm per release, p50/p90/p99/max).**

| Scenario | Stable v1.0.1 | A1+A2 soli (52e2865) | candidata integrata |
|---|---|---|---|
| WALK45 | 440 / 512 / 542 / 555 | 378 / 450 / 480 / 492 | **228 / 271 / 468 / 500** |
| WALK60 | 438 / 510 / 545 / 555 | 376 / 448 / 482 / 492 | **230 / 267 / 456 / 482** |
| JOYSTICK | 523 / 621 / 970 / 1008 | 454 / 544 / 855 / 892 | **270 / 443 / 812 / 880** |
| HOLD100 | 177 / 177 / 213 / 227 | invariato | invariato |

A1+A2 valgono −14 % sulla mediana a zero rischio (bit-identici). T2 porta la mediana a −48 % (WALK) / −48 % (joystick) e il p90 a −47 % / −29 %; il p99/max scendono poco (10–15 %) perché restano le release che risolvono la legge **e** fanno la compensazione piena. Decisione: T2 e T1b conservati (beneficio p50/p90 non ottenibile con alternative più semplici); costo: 3 LocomotionFrame + 1 GaitTrack di stato (~3 KB RAM statica), +96 B di frame sulla catena nominale. Tempo Xtensa reale: **UNK** fino alla misura (piano H2/H5 e istogrammi D11).

**A2 — picchi ×1,7–2 a lavoro identico (VH, 45 record di massimo dev.16/17 con release precedente piena, stessa firma).** Inflazione mediana del calcolo (NOMINAL+COMPENSATE) ×1,44 (≥1,5 in 19/45), della sezione READ (attesa UART) ×1,18 (≥1,5 in 10/45); correlazione di Spearman READ↔calcolo **−0,31**: non è un rallentamento uniforme della release. Letture: (i) 6 record con calcolo ≥×1,5 e READ non inflazionato → rallentamento lato CPU (contesa I-cache/flash dal core 0, Wi-Fi/HTTPD) **HYP**; (ii) 3 record con READ ≥×1,5 e calcolo non inflazionato → tempo sottratto al task loop (preemption/interrupt/ritentativi bus) **HYP**; `sinceReleaseUs` ≈ 1 ms e `externalUs` ≈ 1 ms nei regimi saturi: il lavoro del loop fuori Motion non è la causa; la `lateness` fino a 22 ms è l'accumulo di overrun, non latenza di scheduling intrinseca. Causa dominante: **UNK** (i dati del JSON non discriminano oltre). Strumento aggiunto per deciderlo sull'hardware: istogrammi D11 (work, READ, lateness) nel comando. Nessuna patch su cause non dimostrate (niente IRAM/-O2/cache).

**A3 — percentili e stack.** Vedi tabella A1; stack statico catena nominale 6.224 → 6.320 B (+96), compensazione 7.040 → 6.448 B; high-water fisico **N.D.** (misura: dump USB `loop_stack_min_free`, piano H0).

### B. STOP e sicurezza

**B4 — stima della rate (sostituito il floor di 131ca53).** `rateFromHistory()`: secante fra il riferimento campionato e il riferimento accettato più recente **più vecchio di almeno un periodo** (storia: corrente, precedente, precedente-ma-uno; +32 B RAM), il più vecchio disponibile se nessuno copre un periodo, limite 100 ms. È una media reale su una finestra ≥ T (il floor divideva Δq di un intervallo breve per 20 ms: stima bassa senza significato fisico). Rumore di quantizzazione: ≤ 0,077 rad/s (1 tick/20 ms) invece di ~1 rad/s (1 tick/1,5 ms). Usata identicamente da `beginStop` e da `compensationSafe`, quindi la prova resta garanzia dello STOP reale. Non riduce protezioni: box, limiti, RAW, loaded, deadline invariati; una rate più bassa del vero allunga di poco la rampa di arresto del riferimento (C0 al passaggio, come dichiarato in `StopPlan.h`), mai fuori box. Distinzione: clock di riferimento = `issuedUs_` (campionamento), istante TX = fine `frameTimed`, secante RAW ≠ velocità articolare reale (servo in ritardo 16–18 mrad osservato su joint 10 in dev.14), capacità di arresto fisica **N.D.** (solo hardware).

**B5 — STOP immediato e completo (VO, `run_motion_frame_digest.sh`, Stable vs candidata, in WALK45 e in 3 scenari joystick con jitter).** Piano costruito alla **prima release** dopo `stop()`, primo frame di arresto inviato in quella release, durata del piano 0,08–0,14 s, `motion_verified` dopo 101–103 release (2,02–2,06 s = piano + popolazione letture di controllo 1,44 s + dwell 0,5 s): **identico** su Stable e candidata. STOP→WALK diretto, PRONE, SAFE_OFF: suite `stopThenWalk`, `run_stop_support_tests.sh`, `run_prone_tests.sh` PASS. Stress 36 semi (comando a ogni release, jitter 0–19 ms, release perse): candidata **0 fault**, 18/18 STOP verificati; Stable 3 fault. Tempo comando→prima azione sull'hardware (ACK 20–83 ms in dev.15): **N.D.** per la candidata.

### C. Gait

**C6 — cadenza (modello, `run_walk_tilt_envelope_check.sh` + supporto URDF).** Esplorate variazioni mirate della legge con il drive certificato `{1000,50}`:

| Variante | 1,25× (5,2 s) | 1,5× (4,33 s) | Supporto statico WALK | Esito |
|---|---|---|---|---|
| legge attuale (lift 8, swayX 10, duty .825) | 45 PASS; 60 FAIL (StopPlan 6) | FAIL (a_max 9,8–10,1 > 7,67; StopPlan 69/492) | +1,07/+1,08 mm | cadenza ≤ ~1,1–1,25×, fragile |
| swayX 5 mm | PASS | **PASS** (a_max 5,8/6,7) | **−1,49 mm** | non ammissibile |
| lift 6 mm | come 8 | FAIL | n.d. | nessun effetto utile |
| duty .85 / .875 / .90 | FAIL | FAIL (a_max 8–21) | n.d. | peggiora |
| what-if drive `{1500,100}` | — | PASS 45 e 60 | +1,07 | richiede qualifica servo |
| what-if drive `{2000,200}` | — | 2× PASS | +1,07 | richiede qualifica servo |

Lo sway fore/aft da 10 mm è ciò che tiene il margine statico positivo (+1,07 mm) **e** ciò che impone l'accelerazione articolare al cambio di appoggio; la cadenza con il profilo servo attuale è bloccata dalla prova di arresto (ACC 50 → 7,67 rad/s²). Il miglior miglioramento *robusto* nel modello è +8 % (60 mm a 6,0 s), con fallimenti a 5,6 s: per KISS **nessun preset di cadenza nel candidato**. La legge è già parametrica: un preset 1,5× (period 4,33 s) diventa ammissibile nel modello solo dopo la qualifica hardware di un profilo drive ≥ `{1500,100}` (tracking, corrente, termica ID21/31). Velocità comandate: WALK45 6,92 mm/s, WALK60 9,23 mm/s, joystick max 6,92/3,08 mm/s e 1,76°/s — **invariate**; 2×/3×/5×: non raggiungibili senza qualifica (vedi what-if).

**C7 — WALK_SETTLE (VO sul rig + VH dev.17).** Decomposizione di WALK STOP → STANDING: attesa al prossimo touchdown 0–1,63 s (media 0,81), decelerazione 1 ciclo 6,50 s (regola di ancoraggio: `kWalkStopCycles` < 1 produce nominale fuori box, §6), ricentraggio 0,49 s, arrivo nativo 0,14 s con tracking ideale (hardware ~0,3–1 s): rig 7,12 s a un touchdown, dev.17 8,9 s. Il 73 % è la legge di decelerazione; si riduce solo con la cadenza (→ C6). Nessun accorciamento implementato: ogni alternativa interrompe appoggi o finisce fuori box.

**C8 — joystick proporzionale anche nello sway (INV).** Vedi commit 05e8423: sway 2 mm nel puro longitudinale, continuo fino a 14 mm al 10 % della domanda laterale/yaw, rampa mezzo ciclo (≤ 0,08 mm/release), retarget anche in stance/swing, `kSteer`, limiti, STOP, deadman invariati. Gate di inviluppo (26 scenari, COM URDF, contatti reali sul raccordo) **PASS**: supporto minimo +0,44 mm (random_1; 14 mm fissi: +0,55), puro longitudinale +1,07/+1,29 mm (geometria WALK STANDARD), clearance ≥ 2,18 mm, box margin ≥ 40 mrad, 0 raggi di tilt fuori. Mappe più morbide (piena al 50 %/25 %, rampa 2 cicli) bocciate (−1,71/−0,93 mm con joystick quasi fermo e componente laterale piccola). Nessun nuovo `WALK_IK_OR_WORKSPACE` nei 36 semi. Stabilità dinamica e sensazione fisica: **N.D.** (collaudo H5/H6).

**C9.** Nessun TROT/WALK FAST.

### D. Diagnostica causale

**D10 — inventario di ciò che esiste (nessun duplicato aggiunto).** JSON `/api/motion/status`: `releases`, `missed_releases`, `max_gap_us`, `overrun_releases`, `max_release_work_us`; `command_timing` (missed, prima perdita fase/lateness, max work/fase/compensate/solves, overrun) e `timing.max_work`/`first_miss` (sezioni correnti e della release precedente, contatori di lavoro) con `*_meta` (lateness, intervallo dalla release precedente, lavoro esterno); `stop.*` (`active`, `in_progress`, `motion_verified`, `stand_resume_*`); giunti con `age_ms`/`control_age_ms`, `goal`, `torque`, load/temperatura/tensione; `thermal` (spike, finestre calde, conferme dirette ID); `imu_correction`; `reference_guard`; identità e tempi del comando attivo (`accepted_ms`, `finished_ms`); `input_ms` dello STOP nella traccia dashboard. USB: `@MOTION TIMING` (lateness, visit gap, outside, esterni per categoria e massimi), `@MOTION DIAG` (ExitSnapshot LOADDIAG: stato al primo exit, RAW comandati, feedback, deadline). Dashboard: RTT per campione, `poll_errors`, `late_dropped`.

**D11 — aggiunta minima (INV).** Istogrammi a memoria fissa per comando: work (8 bin), READ (4), lateness (6); 72 B RAM, 3 confronti nell'epilogo della release, nessuna allocazione o testo; `command_timing.work_hist` nel JSON (+16 B snapshot saturato, budget testo HTTP 5248 → 5376), tutti e tre nel dump USB. Costo diagnostico: < 1 µs/release (stima), APP +~0,6 kB, RAM +~0,2 kB. Non nuove autorità o gate.

**D12 — distribuzioni ricavabili oggi (VH).** Overrun/perse per fase e tipo comando: §2 (es. WALK 9,4 %, joystick 20,1 %, HOLD 0 su 4.285 release, n e finestre esplicitati); coda di `workUs` sull'hardware: solo i massimi per comando (p95/p99 **N.D.** fino ai D11); lag TX/feedback: `age_ms`/`control_age_ms` (100–200 ms per popolazione) **non per release**; spike termici ID21/31: `thermal.spikes` conteggia campioni isolati respinti; nessuna attribuzione al servo senza conferma diretta (invariato).

**D13 — discriminazione.** Blocco prima dell'attuazione → `active_command.reason`/`stop.*`/`stand_resume_reason`; UART/servo → `SERVO_*` fault, `control_age_ms`, bus outcome nelle conferme termiche, READ histogram; timing Motion → `command_timing`, `work_hist`, lateness histogram, `@MOTION TIMING`; contatto/supporto → **non misurato** (solo encoder: `pose_region`, `height_estimate`); elettrico → `power_reason`, `bms_age_ms`, voltaggi per giunto; browser/HTTP → RTT, `poll_errors`, `late_dropped`, `transport.*` (#62).

### Tabella requisiti → stato

| Requisito | Baseline | Candidata | Metrica | Prova | Classe | Rischio residuo |
|---|---|---|---|---|---|---|
| Costo release WALK45 | p50 440 / p90 512 | 228 / 271 | libm/release | probe host | VO | tempo Xtensa non proporzionale (UNK) |
| Costo release joystick | 523 / 621 | 270 / 443 | libm/release | probe host | VO | idem |
| Overrun WALK / joystick | 9,4 % / 20,1 % (dev.17) | N.D. | % release | tracce | VH (baseline) | da misurare (H2/H5) |
| Riferimento T2 vs esatto | — | ≤ 0,75 tick | rad | shadow | VO | — |
| STOP: piano / prima azione / verifica | 1 release / sì / 2,0 s | identico | release, s | digest | VO | tempo fisico N.D. |
| Rate STOP sotto jitter | 3 fault/36 | 0 fault/36 | fault | stress | VO | — |
| Velocità WALK45/60 | 6,92 / 9,23 mm/s | invariata | mm/s comandati | legge | VO | 1,5× solo con drive qualificato |
| WALK STOP → STANDING | 8,9 s (dev.17) | invariato (legge 6,99 s + arrivo) | s | rig + traccia | VH/VO | — |
| Sway joystick | 14 mm fissi | 2–14 mm proporzionale | mm | gate inviluppo | INV | stabilità dinamica N.D. |
| Supporto statico joystick (min) | +0,55 mm | +0,44 mm | mm | gate | VO | margine sottile, come Stable |
| Stack catena nominale/compensazione | 6.224 / 7.040 B | 6.320 / 6.448 B | B statici | ELF | VO | high-water fisico N.D. |
| Diagnostica | max per comando | + istogrammi work/READ/lateness | — | codice | INV | — |
| APP / RAM | 1.707.808 B / 137.684 B | 1.718.304 B / 137.908 B | B | manifest, build log | VO | — |

## 14. Revisione finale P1–P5 (mandato 08/11/2026) — candidata `4c897ed`

Classi: VH = VERIFIED HARDWARE, VO = VERIFIED OFFLINE, INV = IMPLEMENTED NOT VALIDATED, HYP = HYPOTHESIS, UNK = UNKNOWN. Nessuna azione hardware; la Stable installata resta `1.0.1-dev.17`.

### P1 — Rate STOP (`rateFromHistory`): invariante verificato, nessuna patch

- **Griglia delle release (VO).** Con lateness < 1 periodo (una lateness ≥ periodo è conteggiata come release saltata e sposta la griglia) il riferimento due posizioni indietro è sempre ≥ 1 periodo più vecchio: `issued ≥ grid_k`, `previous2 ≤ grid_{k−2} + T = grid_{k−1} ≤ grid_k − T`. Test diretto (`stopRateHistory`, caso A): 2.644 release con lateness casuale 0–19 ms, una release saltata ogni 97, retarget joystick a ogni release, STOP a un istante casuale in 6 semi: finestra minima scelta **20,0 ms**, rate pianificata massima **0,378 rad/s** (legge < 0,82 + 1 tick/periodo), STOP verificato in tutti.
- **Fallback «più vecchio disponibile» (VO).** È raggiungibile solo quando esiste un riferimento fuori griglia: le tre `rememberReference(now)` dell'ammissione STAND dopo STOP, seguite da una release a < 20 ms dalla hold-release precedente in ritardo. In quel caso (span 1 ms misurato) il primo frame di ripresa **è il RAW tenuto** (bridge dal RAW fermo nella ripresa WALK; HEIGHT dal RAW tenuto): passo RAW 0 in **55/55** istanti di STOP in WALK (ogni 6 tick di un ciclo) e nella ripresa HOLD→HEIGHT. Secante 0 qualunque sia la finestra: nessuna prima divergenza → nessun floor aggiunto; il test pinna `span < T && step == 0` (un passo ≠ 0 qui sarebbe la prima divergenza che giustifica un floor).
- **Casi diretti.** RAW identici (HOLD): v = 0, piano 0,04 s. Storia stantia (150 ms senza release): `MOTION_DEADLINE_LOST`/`STOP_REFERENCE_UNAVAILABLE` → SAFE_OFF (fail-closed). STOP nelle prime 6 release dopo ENABLE (fase DISENGAGE, mossa nativa senza riferimento sincrono): `STOP_REFERENCE_UNAVAILABLE` → SAFE_OFF, **identico sull'albero Stable 40728b7** (6/6). STOP prima/dopo il TX: `send()` serve lo STOP prima di `frameTimed` (storia vecchia) o dopo `rememberReference` (storia nuova): la storia riflette sempre ciò che è stato trasmesso (test `walkTxLatency` PASS).
- **`beginStop` vs `compensationSafe` (VO, per ispezione).** Stessi estremi temporali: `compensationSafe` campiona a `nowUs_` con skip 0 sulla storia [issued, previous, previous2]; `send(q,drive,now)` registra lo stesso `now` della release, quindi `beginStop` (skip 1 su [now, issued, previous]) legge gli stessi punti. Stesso drive per fase (WALK → `kWalkDrive`, HOLD/WALK_HOLD → `kStandDrive`, uguale a `commandedDrive_`), box comparabili (`arrestBox`/`stopBox_` con lo stesso ripiego `kLift`). Secante del riferimento, non velocità misurata dei servo: dichiarato così in entrambe.
- Comando→prima azione, fallimenti StopPlan, arresto verificato, holding, SAFE_OFF: digest invariato (piano alla release successiva, primo frame subito, verifica 2,04–2,06 s, 0 fault nei semi di stress).

### P2 — Sway joystick: mappa misurata, causa fisica, nuova mappa

**Misura della mappa trasmessa** (sweep stazionaria, `joy_sway` scratch + `foot_edge_contact` per frame, avanti 80 % + laterale/yaw k):

| domanda laterale/yaw | sway trasmesso (mappa 128c480) | supporto statico min | dove |
|---|---|---|---|
| 0 | 2,0 mm | +1,07 mm | c 0,08–0,11, LF RF LH a terra (RH in swing) |
| 2 % | 4,4 mm | +2,88 mm | idem |
| 3,3 % | 6,0 mm | **−0,08 mm** | idem (13 frame < 0; quarti successivi +0,78) |
| 5 % | 8,0 mm | +1,44 mm | idem |
| 6,7 % | 10,0 mm | +2,96 mm | idem |
| 8,3 % | 12,0 mm | +4,39 mm | idem |
| ≥ 10 % | 14,0 mm | +5,9 mm | idem |

**Causa (VO, dati del frame c = 0,089).** A 4,4 mm il tilt dell'asse piede delle zampe in appoggio è 1,5–1,7° (striscia nominale, contatto = riferimento). A 6 mm è 2,04–2,35° > 2°: il contatto reale migra sul raccordo (RF −98,40 → −94,94 mm, LH 89,60 → 93,07 mm: **+3,5…+5 mm nella direzione dello sway**) mentre il COM si è spostato di 1,6 mm → la diagonale RF–LH supera il COM (margine 2,89 → −0,08). Il poligono «insegue» il COM finché il raccordo lo consente; oltre 9–10 mm il margine riprende a crescere. Le mappe graduali già bocciate (piena al 50 %/25 %) fallivano per questo: sosta stazionaria in 5–9 mm. La mappa lineare di 128c480 ha lo stesso buco a una domanda **stazionaria** del 3,3 % (joystick quasi dritto), che il gate a 26 scenari non conteneva.

**Nuova mappa (commit 7c8ea01, INV):** 2 → 5 mm proporzionale fino al 3 % della domanda normalizzata; 14 mm dal 3 % con rilascio sotto l'1,5 % (isteresi sull'ampiezza corrente, nessuno stato nuovo); l'ampiezza cambia **solo con quattro piedi a terra** (campione nominale precedente), rampa 0,5 ciclo (≤ 24 mm/ciclo = 0,074 mm/release; 0,25 ciclo non migliora: random_1 0,46 vs 0,42). Confronto sulle stesse sequenze:

| scenario | lineare 128c480 | **nuova** | 14 mm fissi (dev.17) |
|---|---|---|---|
| sweep stazionaria 0,5–30 % (min) | −0,08 (3,3 %) | **+1,45** (0,5 %), monotona | +5,9 |
| random_1 / 2 / 3 / 5 | 0,44 / 0,54 / 1,53 / 1,44 | **0,57 / 2,36 / 1,45 / 3,04** | 0,57 / 2,36 / 3,30 / 3,04 |
| step avanti→laterale 20 %→avanti, yaw 30 %, laterale 4 % | — | 1,07 (fase puro avanti) | 5,9 |

Il minimo residuo (random_1, 0,57 mm, c 2,08, 14 mm, yaw 27 %) è lo stesso dei 14 mm fissi: transitorio dello sterzo, non della mappa. Continuità: passo RAW massimo 12–13 tick/release (12 in 128c480), 22 con release saltate (identico); nessun frame fuori box; T2 resync 2–3 per scenario (invariati: `sameLawShape` ignora `swayYM`). Gate di inviluppo esteso a 32 scenari (+ `lat/yaw_small` 2,9 %, `lat/yaw_mid` 3,3 %, giri completi): **PASS** — supporto minimo +0,57 mm (random_1), stazionari piccoli +3,26 mm (2,9 %) e +5,91 mm (3,3 %), clearance non-piede ≥ 2,18 mm, 0 raggi di tilt fuori, tilt compensato massimo 11,63°. **Limiti:** modello statico su contatto ideale; stabilità dinamica e sensazione: N.D. (collaudo H5/H6/H8/H9).

### P2-bis — Difetto preesistente trovato: inviluppo tilt steered nel frame di supporto (fix in `4c897ed`)

Prolungando le rotazioni joystick oltre i 26 s del gate (4 cicli a 6,5 s) i raggi del poligono `kSteerTilt` (vertice pitch-up 0,07 rad) portano il tilt del piede d'appoggio al limite del raccordo (12°) da heading ≈ 0,9 rad in poi: **181–283 raggi al bordo su 7 cicli**, presenti in 128c480 **e nella legge v1.0.1** (stesso codice). In firmware un bersaglio di compensazione non valido in WALK è `WALK_IK_OR_WORKSPACE` → SAFE_OFF: con l'IMU al bordo dell'inviluppo (pendenza 4° lungo l'asse del supporto) e heading 50–75° il robot si sarebbe spento in rotazione. Causa: il poligono vincola il carico sull'anca/raccordo, grandezza del **frame corpo**, ma era applicato al tilt nel **frame di supporto** (il frame del fit e di `headed()`), e l'heading accumulato ruota un pitch di supporto in roll corpo. Fix minimo: `limitWalkTilt` porta il vettore di tilt nel frame heading (`R_z(θ)ᵀ`), proietta sul poligono e riporta indietro; heading identità bit-identico (WALK STANDARD invariato). Il gate proietta i raggi allo stesso modo e aggiunge due giri completi (cw/ccw, 206 s): **0 raggi al bordo in 2,1 giri**, tilt compensato ≤ 11,42° (era 11,95 al bordo). Test unitario sulla proiezione (identità, heading 90°). Classe INV (hardware: H8).

### P3 — ESP32-S3: configurazione reale e prove offline

**Configurazione della candidata (VO su ELF/map/sdkconfig/compile_commands):** `-Os` su tutte le unità (`CONFIG_COMPILER_OPTIMIZATION_SIZE`), `F_CPU 240000000`/`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240` (frequenza effettiva da leggere al boot), flash QIO 80 MHz, PSRAM 80 MHz (FQBN `PSRAM=opi`; lo `sdkconfig` esportato riporta `SPIRAM_MODE_QUAD`: discrepanza da chiarire sul log di boot), I-cache 16 kB 8 vie/32 B, D-cache 32 kB. Codice `motion::`+`simple::` **95.982 B, tutto in flash mappata** (IROM 0x4200…); `libm` double (`sin/cos/atan2/acos/asin/sqrt/hypot/remainder/floor`, ~5,5 kB) in flash, **nulla in IRAM né ROM**; `.iram0.text` 82 kB (IDF/Wi-Fi), segmento IRAM 358 kB. Hot set di una release WALK (legge + IK + FK + libm + driver) ≫ 16 kB di I-cache: miss strutturali plausibili (HYP).

**Scheduling (VO su sorgenti/sdkconfig):** core 1: `loopTask` prio 1 (Motion + Controller, `delay(1)`), `arduino_events` prio 19 (`ESP_TASKD_EVENT_PRIO−1`, eventi Wi-Fi/IP), `ipc1` prio 24, IDLE1; core 0: `md_imu` prio 2, `md_network` prio 1, `md_http_life` prio 1, HTTPD ×2 prio 2 (24/28 kB), Wi-Fi, lwIP, `esp_timer`, NimBLE. Nessuna scrittura flash/NVS durante Motion (solo salvataggi di calibrazione e configurazione di rete). Le cache I/D sono condivise fra i due core: il traffico di core 0 può evincere le linee di Motion (rallentamento diffuso), mentre i dati dev.17 (calcolo ×1,44 vs READ ×1,18, ρ −0,31) indicano eventi discreti: candidati `arduino_events`, ISR UART/USB-CDC su core 1, miss di I-cache dopo preemption. Causa **UNK**; nessun task o core cambiato; gli istogrammi forniscono le distribuzioni ma non discriminano da soli (serve il dump USB con `externalUs[]` e `sinceReleaseUs`).

**`-Os` vs `-O2` selettivo (scratch `/tmp/mp-o2`, 9 unità Motion con `#pragma GCC optimize("O2")`, stesso FQBN):** APP **1.718.304 → 1.735.024 B** (+16.720), codice Motion **+16,2 %** (94.961 → 110.336 B; `sampleLaw` +1.708, `update` +1.198, `compensationSafe` +1.158); frame di stack: catena nominale **−176 B**, compensazione **+1.104 B**, prova STOP **+2.160 B** (inlining di `StopPlan::build`), STOP +304 B; tempo Xtensa **N.D.**; la parte soft-double (`libm`, `__muldf3`…) è identica. **Respinto**: costo misurabile (cache/stack), beneficio non misurabile offline.

**`double` → `float` selettivo (scratch `/tmp/mp-float`: LegKinematics, LegInverseKinematics, FootContact, BodyPose, TiltedContactIk in `float`; 78.200 casi dalla legge: WALK45/60, sterzo laterale/yaw con raccordo, 9 raggi di tilt):**

| percorso | esito con tolleranze di produzione | con tolleranza piana 1e-6 | dove entrambi OK: |Δq| p50/p95/p99/max | contatto (mm) | rami | tick RAW cambiati |
|---|---|---|---|---|---|---|---|
| IK piana analitica (`solveGait`) | float fallisce **19.217/19.520** (`NUMERICAL_FAILURE`, FK 1e-9 irraggiungibile) | ancora **2.032** fallimenti (`validBodyPose` 1e-10) | 1,1e-7 / 2,6e-7 / 3,4e-7 / 4,8e-7 rad (≤ 0,0003 tick) | 1,5e-5 / 2,7e-5 / 3,2e-5 / 5,1e-5 | 0 | 5 / 52.464 (0,01 %) |
| Newton inclinata (`compensationTarget`, 1e-6, 8 passi) | float fallisce **49.940/58.680** (`INVALID_INPUT`: ortonormalità 1e-10) | idem | 1,4e-7 / 3,2e-7 / 4,2e-7 / 6,1e-7 rad | 1,6e-5 / 3,0e-5 / 3,7e-5 / 5,4e-5 | 0 | 5 / 26.220 (0,02 %) |

La sola tolleranza (double 1e-9 vs 1e-6) non cambia nulla (0 differenze). Conclusione: numericamente il `float` sarebbe entro 0,0004 tick/0,05 µm, ma la catena di validità (1e-9, 1e-10, 1e-12) delle protezioni andrebbe ri-derivata per intero: **respinto** (nessun allentamento). Tempo on-target N.D. **IRAM:** non adottato (wrapper in IRAM non sposta `libm`; nessun beneficio verificabile offline).

### P4 — Cadenza e WALK_SETTLE: vincolo attivo, margini, decisione

**Attribuzione (checker con attribuzione, scratch):** a 1,25× (5,2 s, 60 mm) i 6 fallimenti StopPlan sono sui giunti **UPPER anteriori** (j1/j4) a q ≈ 0,416 rad contro il bordo inferiore `kHeight` 0,385, v −0,54 rad/s, zampa anteriore in swing sotto i raggi di tilt 15/17; a 1,5× (4,33 s, 45 mm) si aggiungono a_max 9,85 rad/s² (UPPER, finestre a quattro appoggi = transizione dello sway fore/aft) e StopPlan su j1/4/8/11. **Scan del periodo** (griglia ideale, con offset di campionamento 1/4 e 1/7 di release — un primo tentativo con secante su 5 ms era un artefatto di quantizzazione): 45 mm PASS fino a 5,3 s; 60 mm PASS da 6,5 a 5,65 s, FAIL a 5,6 (6–9 prove), poi alternante; 5,8 s PASS a qualunque offset. **Rig con latenza TX 7 ms e release in ritardo di 15 ms (test `walkTxLatency`, board plant con lag servo):** 6,2 s PASS, **6,0 e 5,8 s FAIL** (`WALK_IK_OR_WORKSPACE` da `compensationSafe`: j4 v −0,92 rad/s verso lo 0,385 con margine 0,074 rad; la durata comune dello StopPlan imposta dal giunto più veloce dà una distanza d'arresto 0,083 > margine; a 6,5 s v 0,82 → 0,066, **margine ≈ 8 mrad ≈ 5 tick**).

**Decisione:** `kWalkPeriodS` **invariato** (6,5 s). Il guadagno robusto a tutte le prove offline è +5 % (6,2 s): non giustifica la ri-qualifica hardware della cadenza validata; velocità 6,92/9,23 mm/s e WALK STOP→STANDING (legge 6,99 s + arrivo) **INVARIATI**. Il vincolo è la prova di arresto al bordo `kHeight` dei giunti UPPER anteriori con il profilo `{1000,50}`: la via per 1,5× resta un profilo drive ≥ `{1500,100}` qualificato (piano minimo: (1) WALK45 a 6,5 s con registri `{1500,100}` 2 cicli, tracking/corrente/termica ID21-31 vs `{1000,50}`; (2) STOP a metà swing: tempo fisico d'arresto e `STOP_HOLD_TRACKING`; (3) solo se (1)-(2) puliti, periodo 4,33 s per 2 cicli con gli stessi osservabili); oppure la ri-certificazione del bordo UPPER del box (non ammessa). Finestre dello sway fore/aft: la transizione in 0,075 cicli è l'altro vincolo (a_max); allargarla sposta il COM in tre appoggi dove il margine è +1,07 mm → limite fisico, non riparametrizzato. I test che codificavano 6,5 s come letterale (SE(2) su due cicli, JSON di stato/capabilities) ora derivano dal periodo.

### P5 — Diagnostica: costo misurato, utilità, nessun layer

Compilazione Xtensa della sola `SimpleMotion.cpp` con la riga di `compile_commands.json`, con e senza istogrammi: `timing::Release::~Release` **244 → 400 B** (+156 B), tabella `kWorkBinsUs` 28 B, `timing::state` **824 → 896 B** (+72 B `.data`), `update()` 0x131c invariata, frame di stack invariati. Dinamica per release: tre scansioni di soglia (≤ 7+4+6 confronti) + 3 incrementi ≈ < 100 istruzioni → < 0,5 µs a 240 MHz **(HYP, non misurato)**. Riarmo a ogni comando (`timing::arm`), quindi un istogramma per **comando**, non per fase: il comando WALK include HEIGHT→WALK e WALK_SETTLE; il p99 del solo WALK va letto per differenza (HOLD = 0 overrun). Saturazione `uint16` solo nel JSON (65.535 release = 21,8 min dello stesso comando; USB `uint32`). Mantenuti tutti e tre perché ciascuno decide una cosa del collaudo: work → accettazione A/B di p99/overrun; READ → UART/preemption vs calcolo; lateness → latenza di schedulazione vs esecuzione. Nessun logging per release, nessun ring buffer.

### Build e tabella finale

`ROBOT_POWERED` CLEAN da **`4c897ed0bb7fb6976c3a76ef6e97dee8cc9666b1`**: APP **1.718.736 B** (+432 B vs 128c480, +10.928 B vs dev.17), RAM 137.908 B (= 128c480, +224 B vs dev.17), SHA256 **`03c84606e3d45616cbcd6985fdad6253d9af13691edeb3664503b6f5bdcbeb9e`**, `FW_VERSION=1.0.2-dev.1`, `~/MATDOG/build-output/motion-perf-next-20261107/build-4c897ed/`. Frame di stack Xtensa (somme di catena con la stessa decodifica ENTRY, dev.17 → candidata): nominale 6.624 → **7.200 B** (+576: `sampleLaw` separata e `solveGaitInto`), compensazione 5.200 → **4.608 B** (−592), prova STOP 3.248 → **2.656 B** (−592), STOP 2.048 → 2.128 B; identici a 128c480 (`limitWalkTilt` 96 → 112 B); i 6.320/6.448 B del §9 usano catene più corte. Il candidato 128c480 (APP `e3e69368…`) è **superato**.

| Requisito | Stable v1.0.1 | A1+A2 | integrata `4c897ed` | Prova | Classe | Rischio |
|---|---|---|---|---|---|---|
| libm/release WALK45 p50/p90/p99/max | 440/512/542/555 | 378/450/480/492 | 230/277/468/510 | probe host | VO | tempo Xtensa N.D. |
| libm/release JOYSTICK | 523/621/970/1008 | 454/544/855/892 | 278/462/815/898 | probe host | VO | idem |
| -Os → -O2 selettivo | — | — | APP +16.720 B, codice Motion +16 %, stack prova STOP +2.160 B, tempo N.D. → respinto | ELF scratch | VO | — |
| double → float selettivo | — | — | Δq ≤ 0,0004 tick, 0,01–0,02 % tick RAW, ma 19.217/19.520 e 49.940/58.680 fallimenti di validità → respinto | 78.200 casi | VO | — |
| Rate STOP: finestra | floor/2 campioni | — | ≥ 20,0 ms su griglia (2.644 release), fallback solo con passo 0 | test diretti | VO | — |
| STOP comando→piano/frame/verifica | 1 release / sì / 2,0 s | — | identico | digest | VO | tempo fisico N.D. |
| Sway joystick: supporto stazionario min | +5,9 (14 mm fissi) | — | **+1,45** (0,5 %), monotona; 128c480 lineare: **−0,08** | sweep | VO | dinamica N.D. |
| Sway joystick: random min | +0,57 | — | **+0,57** (128c480: +0,44) | gate | VO | — |
| Inviluppo steered in rotazione > 4 cicli | 181–283 raggi al bordo | — | **0** (2,1 giri), tilt ≤ 11,42° | gate giri | VO/INV | H8 |
| Velocità WALK45/60 | 6,92 / 9,23 mm/s | — | **INVARIATA** (vincolo StopPlan bordo UPPER; +5 % non adottato) | checker + rig | VO | — |
| WALK STOP → STANDING | 8,9 s (dev.17) | — | **INVARIATO** | rig/traccia | VH/VO | — |
| Stack catene nominale / compensazione / prova STOP / STOP (stessa decodifica) | 6.624 / 5.200 / 3.248 / 2.048 B (dev.17) | — | 7.200 / 4.608 / 2.656 / 2.128 B; frame massimo 9.648 → 9.792 B (HTTPD, task proprio) | ELF | VO | high-water N.D. |
| Diagnostica | max per comando | — | +156 B codice, +72 B RAM, < 0,5 µs/release (HYP) | ELF | VO/HYP | — |
| APP / RAM | 1.707.808 / 137.684 B | — | 1.718.736 / 137.908 B | manifest | VO | — |
