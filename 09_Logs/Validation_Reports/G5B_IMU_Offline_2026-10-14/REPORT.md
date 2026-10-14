# MATDOG G5-B — chiusura offline, 14 ottobre 2026

## Stato della consegna

| Stato | Risultato |
|---|---|
| IMPLEMENTED | Pipeline C++17 a memoria fissa che riusa G5-A, applica correzioni cartesiane e valida atomicamente 12 joint; modello contatti/piano esplicito, timing, envelope operativo, lifecycle e fault. |
| OFFLINE VALIDATED | 811 verifiche C++; 72 casi URDF/contact/SciPy; replay byte-identico; 46 test Python pertinenti; G1–G5-A e sanitizer PASS. |
| SIMULATED | 12 scenari, 7.200 campioni, modello sintetico primo ordine + ritardo/disturbo. Piano inclinato e damping sono esperimenti con gate. |
| HARDWARE UNVERIFIED | Tutta la catena sensore→servo, carichi/aderenza, guadagni, montaggio/calibrazione IMU e performance ESP32. Nessuna prova fisica. |
| BLOCKED | Integrazione driver/runtime protetti; qualificazione hardware; calcolo CoM/dinamico in attesa di REV01. Sono contratti documentati, non modifiche applicate. |

Non è una release firmware e non è un controller approvato per l'auto-bilanciamento fisico. Main resta `fdd04345c39b50137cc7cdc50fb08cd592b8c2af`. Branch dedicata `feat/matdog-g5b-imu-offline`, clone Git indipendente. Nessun merge, rebase, cherry-pick, reset, clean o checkout nei working tree di Claude.

## Implementazione e perimetro

Sono aggiunti solo `06_Software/Matdog_Core/stabilization_g5b/` e questo dossier. Nessun file esistente modificato. Audit iniziale in [AUDIT.md](../../../06_Software/Matdog_Core/stabilization_g5b/AUDIT.md); architettura, lifecycle, frame, contratti e riproduzione in [README.md](../../../06_Software/Matdog_Core/stabilization_g5b/README.md).

File software nuovi: `StabilizationPipeline.h/.cpp`, `InertialInput.h`, `tests/Fixtures.h`, `tests/test_pipeline.cpp`, `tests/bridge.cpp`, `tests/offline_oracle.py`, `run_validation.py`, README e AUDIT. Il codice non entra nel build firmware. La pipeline non ha I/O, clock, heap, exceptions, RTTI, bus ID, raw tick, q0, lease o backend attuatore. Il solver/contatto/limiti/legge G5-A restano invariati. La correzione candidata viene confermata solo se tutte le gambe e i controlli passano: nessun windup da target IK rifiutati.

Durante fault o disable non vengono pubblicati nuovi target, neppure quelli di rampa G5-A: la rampa è solo stato diagnostico interno. Il riarmo richiede disable, drenaggio, reset e anchor confermato. Anche durante polling duplicato vengono controllati timing e supporto; un duplicato coerente non reintegra e non pubblica. L'owner futuro deve implementare heartbeat, stop e conferma dell'anchor. Non c'è un ACK di attuazione in G5-B.

## Test PASS / FAIL / BLOCKED

Il runner finale registra **39 gate PASS, 1 gate FAIL preesistente, 0 gate BLOCKED**. `required_gates_passed=true`, `all_passed=false`. Il FAIL non viene trasformato in PASS né corretto fuori perimetro.

| Suite | Esito | Evidenza |
|---|---|---|
| G5-B C++ strict GCC C++17 | PASS | 811 check, 0 failure; [log](g5b_cpp.log) |
| G5-B e G5-A Clang ASan/UBSan | PASS | Build/run distinti, leak e undefined behavior fail-closed |
| G5-B oracle URDF/SciPy | PASS | 72/72, residuo massimo 9.56834552382e-14 m; differenza joint 2.09721129352e-12 rad |
| Oracle legge G5-A + polling/jitter | PASS | 100 nuovi campioni + 100 duplicati, errore correzione 5.20417042793e-18 rad |
| Riproduzione deterministica | PASS | `offline_results.json` e `simulation_trace.csv` identici byte per byte in due run |
| G1 motion kinematics | PASS | 1.228 check, 0 failure; oracle Python storico PASS |
| G2 contact/stand | PASS | 1.925 check, 0 failure; C4 oracle 52/52 pose PASS |
| G3 startup/timed stand | PASS | 12.225 check, 0 failure; 40 state/event pair; oracle Python PASS |
| G3.5 pose support | PASS | 448 combinazioni di policy; startup gate invariato |
| G4 gait | PASS | 148.291 check, 0 failure; oracle indipendente 3.872 campioni + 46 lifecycle frame |
| G4.1 contact mode | PASS | 68 check, 0 failure |
| G5-A originale | PASS | 10.111 check, 0 failure; 4.000 quaternion, tilted IK e loop simulations |
| Test Python cinematici pertinenti su main | PASS | 46 test, nessun errore; [log](pertinent_kinematics_python.log) |
| Intera vecchia suite Python cinematica | FAIL preesistente | 50 test; 4 test visual-zero producono 16 errori di status calibrazione; [log](existing_kinematics_python.log) |
| Source/purity/provenance main | PASS | Manifest PR2 e dati protetti invariati; nuovi sorgenti fuori firmware |

Il FAIL viene dal parser `matdog_leg_fk_live.py`: attende `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`, mentre `MATDOG_JOINT_CALIBRATION.yaml` contiene `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED`. Test, parser e YAML sono verificati byte-identici a main e i loro hash sono in [validation_results.json](validation_results.json). Non è una regressione introdotta da G5-B. La correzione del parser/profile resta fuori perimetro; nessun file di calibrazione è stato modificato.

Le suite storiche non presenti in main sono estratte dal commit `e1704719979789cd9c9f18741e4546725558797e`, indicato dal manifest PR2, in una directory temporanea. Il runner verifica gli hash dei file deferred e compila contro i sorgenti main attuali. Nessun archivio storico, materiale terzo o test viene aggiunto ai path firmware originali. Gli oracle legacy che inizializzano il Model REV00 sono riusati per prove cinematiche; le sue inerzie non diventano input o evidenza dinamica G5-B.

## Report numerico delle simulazioni

Ogni riga è **SIMULATED**: non è una misura fisica. Gain `ki=1/s`, nominal step 20ms, correction bound ±0,03rad, rate bound 0,04rad/s e ritardi sono fixture esplicite. Il plant sintetico è primo ordine con tau=80ms, ritardo di comando 40ms (60ms nel confronto damping), più disturbo additivo. Il jitter varia il campionamento 14–26ms e l'età del campione 3–9ms; l'applicazione prevista aggiunge 5ms e viene inclusa nella history del plant. Il controllo non assume questo modello sulla scheda reale.

600 campioni per scenario, circa 12s. Errore finale: massimo assoluto negli ultimi 100 campioni, sui due assi. Assestamento: primo tempo dopo il quale entrambi gli errori restano sotto 1mrad; overshoot: massimo errore di segno opposto rispetto al disturbo step, normalizzato. Per la sinusoide l'overshoot step non è definito. Saturazione/rate: conteggio range-limit G5-A e passi con correzione alla velocità massima. Tutti i 7.200 frame dei 12 scenari sono accettati; gli angoli dei 12 joint e le ragioni sono nel [CSV](simulation_trace.csv).

| Scenario | Errore tail rad | Assestamento s | Overshoot step | Saturati / rate limit | Max delta joint rad |
|---|---:|---:|---:|---:|---:|
| level | 0 | 0.000 | 0.000% | 0 / 0 | 0 |
| roll_positive | 1.95087e-07 | 2.540 | 0.000% | 0 / 0 | 0.000400234 |
| roll_negative | 1.95087e-07 | 2.540 | 0.000% | 0 / 0 | 0.000400234 |
| pitch_positive | 3.25145e-07 | 3.000 | 0.000% | 0 / 0 | 0.00133752 |
| pitch_negative | 3.25145e-07 | 3.000 | 0.000% | 0 / 0 | 0.00133252 |
| combined | 1.95087e-07 | 2.540 | 0.000% | 0 / 0 | 0.0011194 |
| periodic | 0.0021306 | N/A | N/A | 0 / 0 | 6.13019e-05 |
| delay_jitter | 1.68007e-07 | 2.506 | 0.000% | 0 / 0 | 0.00165204 |
| saturation | 0.27 | N/A | 0.000% | 569 / 37 | 0.00231132 |
| gradual_plane | 1.61878e-06 | N/A | 0.000% | 0 / 0 | 0.000604258 |
| damping_baseline | 1.90218e-07 | 2.760 | 0.000% | 0 / 0 | 0.00139123 |
| damping_experiment | 1.3097e-06 | 3.220 | 0.000% | 0 / 1 | 0.00320476 |

Il livello iniziale resta invariato; disturbi roll ±0,015rad e pitch ±0,025rad convergono nel modello. La saturazione con disturbo pitch 0,3rad resta limitata e lascia 0,27rad di errore: non converge a zero. Si osservano 569 campioni range-saturated e 37 rate-limited, con rate massimo 0,04rad/s.

La perturbazione sinusoidale roll 0,002rad a 0,6Hz rimane bounded ma il rapporto RMS errore/disturbo è **1,0835**, una lieve amplificazione. Non viene dichiarata reiezione efficace di quella frequenza. La variante damping Td=0,15s porta l'assestamento da **2,76s a 3,22s**, senza vantaggio in questa fixture; il gate resta disabilitato nella baseline. Non si estrapolano questi risultati a robot, servomeccanica, noise gyro o ritardi variabili generali.

## Piano inclinato e fault

La fixture C++ inclina gradualmente il piano fino a roll=0,006rad e pitch=0,015rad mantenendo una body pose nominale orizzontale: massimo residuo contatti **2,6151e-16m**. È una verifica cinematica con contatti/normal forniti dalla fixture, non una prova che i quattro piedi siano portanti. Il caso dinamico `gradual_plane` nel CSV usa inoltre un disturbo sintetico additivo, distinto dalla prova geometrica. Il gate non autorizza hardware e non sostituisce nuove prove di collisione, carico, strip reale o attrito.

| Input/condizione | Uscita / comportamento verificato | Ragione |
|---|---|---|
| IMU stale/disconnessa | frame invalido, joint zero, latch; nessun target di rientro | IMU_REJECTED(STALE) / DISCONNECTED |
| Quaternion non unitario/NaN, quality bassa | nessun target; latch | IMU_REJECTED con AttitudeStatus specifico |
| Accuracy negativa/status >3 | nessun target; latch | INCOHERENT_SAMPLE |
| Duplicato esatto e fresco | nessun target; correzione non reintegrata, no latch | DUPLICATE_SAMPLE |
| Payload/stamp cambiati a stessa sequence | nessun target; latch | INCOHERENT_SAMPLE |
| Fuori ordine, nuovo stamp non crescente | nessun target; latch | IMU_REJECTED / INCOHERENT_SAMPLE |
| Cambio epoch/reset/counter wrap | riarmo esplicito, nessun replay accettato | EPOCH_CHANGED / INCOHERENT_SAMPLE |
| Gap, tempo regressivo, comando tardivo | nessun target; latch | SAMPLE_GAP / INVALID_TIME / COMMAND_TOO_LATE |
| UNKNOWN/confidence bassa o invalidità su duplicato | non si assume carico dalla geometria; frame invalido | UNCONFIRMED_CONTACT |
| Normale invalida, supporto stale, ordine incoerente | frame invalido; latch | INVALID_SUPPORT / STALE_SUPPORT / SUPPORT_ORDER |
| Piano mobile senza gate o movimento eccessivo | frame invalido; latch | PLANE_GATE / PLANE_MOTION |
| IK irrisolvibile/strip incompatibile | nessuna soluzione parziale, rollback integrale candidato | IK_FAILURE + stato solver |
| Margine operativo insufficiente | nessun clipping; frame invalido | OPERATIONAL_LIMIT |
| Seed iniziale sul limite | enable rifiutato | anchor invalido |
| Condizionamento o salto/rate joint eccessivi | frame invalido; latch | CONDITION_LIMIT / TARGET_DISCONTINUITY |
| Rate sperimentale incoerente | frame invalido; latch | INVALID_RATE |
| Disable e riabilitazione dopo fault | nessun frame durante drain; reset/enable anticipati rifiutati; riarmo esplicito PASS | DISABLED / FAULT_LATCHED |

[Log C++](g5b_cpp.log) registra ragione, validità, joint zero, correzione diagnostica, latch e stati IMU/IK per i rifiuti. Convergenza/overshoot non sono applicabili ai rifiuti: sono marcati NA. [offline_results.json](offline_results.json) contiene altri sei casi fault numerici con i 12 target zero; [legacy_perturbation_results.json](legacy_perturbation_results.json) conserva i test/simulazioni G5-A. Saturazione e continuità dei frame validi sono registrate nel CSV.

## Timing, costo e memoria

L'età misura→generazione della fixture jitter è 3–9ms; misura→comando previsto 8–14ms. Sono timestamp simulati. Nessuna latenza IMU→servo è stata misurata. Freshness, gap, sequence, epoch, command lead/deadline sono controllati; nominale 50Hz non significa tempo reale qualificato. Il consumer dovrà verificare l'età effettiva al bus e richiedere ACK/feedback.

Sul host: stato pipeline **1112 byte**, uscita **448 byte**; 10.000/10.000 update di benchmark accettati, media **33.882us**, massimo osservato **71.992us**. GCC riporta frame stack update **1.824 byte bounded** e configure **1.200 byte** nel build host; [dettaglio](host_stack_usage.txt). Sono misure/ABI host, non WCET, dimensioni stack complessive o performance ESP32.

Il lavoro è bounded: 4 solver, massimo 40 iterazioni, 6 valutazioni Jacobian e fino a 10 backtracking per iterazione, più valutazioni iniziali/finali. Un limite strutturale conservativo è 2.588 contact-FK per update, senza heap o ricorsione. Le frame stack delle chiamate G5-A, task ESP32 e scheduler devono essere misurate sulla toolchain target. Non sono aggiunte nuove dipendenze embedded; le dipendenze Python storiche sono riusate dalla directory di artifact esistente.

## Dipendenze ancora da qualificare e integrazione futura

- Snapshot BNO085 completo e atomico: quaternion, quality, count, timestamp originale, reset epoch; timestamp SH2 vs ricezione ESP32, wrap e dropout. Il driver è protetto: sola proposta getter read-only nel README.
- Livellamento/mounting reference misurato, DCD/accuracy con vibrazioni, interferenze magnetiche, sincronizzazione rate/gyro e conversione gyro→derivate tilt.
- Osservazione esterna di contatto/load/confidence, attrito e slip; stima/misura del piano indipendente dall'IMU; quattro supporti non inferiti da posizione.
- Latenza acquisizione/processamento/scheduling/scrittura sequenziale/servo-response, jitter reale, ACK e tracking; guadagni, rate, saturazioni e oscillazioni sotto carico.
- Envelopes operativi, collisioni e sweep tra frame, velocità/accelerazione, power/BMS/termica e policy stop/safe-off del proprietario runtime.
- REV01 con mass budget aggiornato per elettronica/supporti/antenna/fusibile/pad e altri elementi; per-link massa/CoM/tensori, aggregati e provenance. Nessuna massa inventata o REV00 promossa ad autorevole per l'assemblaggio attuale.

Il [contratto futuro](../../../06_Software/Matdog_Core/stabilization_g5b/README.md) descrive la consegna a MotionService/coordinator/validator/Executor/Safe Actuator Layer. Solo `JointTargetFrame` e metadati cinematici possono essere forniti: `target.valid` non può impostare collision/support/path/authority/hardware-approved, né bypassare q0, JointLimits, owner bus, permit o tracking. Se un frame accettato non è applicato, serve anchor confermato e ricostruzione/riarmo; il modulo offline non inventa un ACK. Nessuna integrazione è stata eseguita.

## Compatibilità e chiusura Git

PR #47 HEAD `13404187b1a2abf1ac1aafb4bf379b58cac36916`; PR #49 HEAD `34e686ff42f240a44c7906c908abebd0e9bd8521`, verificati nuovamente. Zero overlap dei file modificati e moduli G5-A letti identici a main. Driver #49 e header MotionService/Executor sono stati solo letti via oggetti Git del clone indipendente; le modifiche PRONE/Entry/AdaptiveStand/tracking/autorizzazione di Claude non sono state studiate per correggerle o incorporate.

[compatibility_and_isolation.json](compatibility_and_isolation.json) distingue i file letti dalle sole due directory aggiunte. [worktrees_before.json](worktrees_before.json) e [worktrees_after.json](worktrees_after.json) mostrano 16 checkout preesistenti con gli stessi HEAD, status e hash dei file dirty rilevati. Nessun accesso seriale, comando servo, flash, OTA, prova hardware, modifica di calibrazione/configurazione o build firmware eseguito. Tutti i nuovi file saranno tracciati nella sola branch dedicata; nessuna modifica al comportamento Motion operativo.

Riproduzione: `python3 06_Software/Matdog_Core/stabilization_g5b/run_validation.py`; vedere README per PYTHONPATH delle dipendenze storiche e oggetto Git richiesto. I JSON/CSV deterministici sono riproducibili; benchmark/tempi host e path temporanei dei log non sono byte-deterministici. Questo dossier registra una validazione offline e lascia esplicite le dipendenze BLOCKED e HARDWARE UNVERIFIED.
