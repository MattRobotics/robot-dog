# MATDOG Motion — handoff notte 12 ottobre 2026

Questa consegna supersede lo stato operativo dei report precedenti, che restano conservati come evidenza storica. Worktree esclusivo `/home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog`, baseline/main `fdd04345c39b50137cc7cdc50fb08cd592b8c2af`, branch `matdog/motion-night-20261012`, origin `MattRobotics/robot-dog`. Primo commit runtime/UI/test `ce3073d`; Review HEAD `225a9d0d9e0e19c93212be20eb926545c1ab9944`; [Draft PR #47](https://github.com/MattRobotics/robot-dog/pull/47) aperta verso main. Il commit di questo handoff aggiunge solo documentazione; HEAD finale è nel checkpoint esterno e nella PR. Nessun lavoro sul vecchio ramo ASUS, reset/clean, merge, force push, tag/release o operazione hardware. I 1.336 file iniziali sono conservati: nessuna rimozione; soltanto tre file del viewer sono cambiati dopo il checkpoint, per correggere il rendering XYZ e verificarlo.

**Risultato: candidato geometrico offline completo e sorgenti reviewable. Non è pronto per una prima prova attuativa finché i gate software/evidenza indicati sotto non sono chiusi. MOTION_AUTHORIZED=0, OTA_INGEST=0.**

## Prona → STAND 150 mm

[Compiler originale](../../../06_Software/Matdog_Core/motion/paired_prone/README.md), [waypoint](acquisition_keypoints.json), [prova continua](continuous_pair_path.json), [bound rafforzato](strengthened_nonadjacent_v2.json), [12 angoli/CSV](paired_joint_trajectory.csv), [17 trasformazioni/contact witnesses](keypoint_world_transforms.json), [provenance](geometry_proof_receipt.json).

Il candidato iniziale è **sintetico**, non una lettura del robot. L'operatore conferma che non esiste un log prono dei 12 encoder: non è stata ripetuta la ricerca. Belly edge e punti geometrici dei piedi sono sul piano Z=0. Il COM nominale CAD è 2,48 kg complessivi; inerziali/accessori/cablaggio e distribuzione reale del peso restano incerti. Il contatto ventrale può contribuire al sostegno; non si dimostra che porti tutto il peso.

| coppia | HIP destra / sinistra | UPPER | LOWER |
|---|---:|---:|---:|
| RH/LH | +1° / −1° | 90° | −92° |
| RF/LF | +1° / −1° | circa 83,737° | −92° |

Gli angoli esatti LF/RF/RH/LH sono in `acquisition_keypoints.json`. Base_link iniziale Z = **11,389821 mm**, pitch **+5,545851°**, roll 0. Z deriva dal minimo della superficie underside ruotata, non da IMU o Z imposto a zero. LOWER iniziali sono sul limite canonico URDF: il margine operativo non è approvato. PRE-ALIGN è speculare, coordinato con i soli aggiustamenti geometrici necessari; HIP diventano neutre URDF 0° e rimangono ferme. La neutralità installata e i differenti mapping dei lati devono essere verificati separatamente.

Dispiegamento posteriore prioritario, anteriore coordinato, passaggio verso supporto quadrupede e risalita: **242 campioni**, **251 segmenti**, **315.545 valutazioni intervallari** (il precedente progress log mostrava 315.544 prima dell'ultimo segmento). L'attivazione anteriore qui segue la prima estensione posteriore; il pitch deriva dai vincoli di contatto e raggiunge circa **+10,236°**. È un limite dichiarato rispetto all'obiettivo di corpo il più possibile orizzontale. A 50 mm pitch/roll sono zero; a 100 mm si raccorda alla geometria C4 e termina a **150 mm** senza discontinuità delle articolazioni. La correzione di riferimento del contatto STL è micrometrica: il raccordo riusa G1/G2 e footprint C4, non pretende identità byte-per-byte con il vecchio C4-C.

Tutte le 17 mesh sono controllate sul piano; nessuna penetrazione oltre la soglia invariata **1 nm**. Il continuous profile applica una correzione geometrica verticale vincolata alla prossimità di contatto già definita **10 µm**. Non è un sensore di altezza né una prova di piede portante. Le velocità dei giunti sono ≤0,192486 rad/s e accelerazioni ≤2 rad/s² nel clock offline di circa 60,032 s; questi rate non sono hardware-approved.

Le 120 coppie non adiacenti sono swept collision-free: il bound iniziale molto debole era 0,176 µm; una verifica triangolare conservativa dello **stesso percorso** ora prova un minimo di **118,303 µm**. Il precedente fallimento del certificatore a segmento 56 HIP–LOWER RH/LH era una limitazione AABB; i link non si intersecano e il bound superficiale locale è almeno 1 mm. Nessuna posa o soglia è stata cambiata. Il riferimento operativo **3 mm resta non qualificato**: bound positivo non equivale a margine operativo approvato.

Le 12 coppie adiacenti revolute passano nei campioni. Swept aggiuntivo: LF base↔HIP certificato, LF HIP↔UPPER esaurisce il budget conservativo, altre 10 coppie non certificate dal run limitato. [Risultati](check_adjacent_swept.json). Questo è **BLOCKED**, non PASS completo né collisione dimostrata. Non estendere la traiettoria o allentare il checker per nasconderlo.

HIP inward 3° è rifiutato per LOWER anteriore che penetra 1,412153 mm. La vecchia posa RH HIP −45° / pitch −8,59° con floor penetration LOWER 3,539 mm e FOOT 3,396 mm resta soltanto una regressione. La nuova coppia posteriore elimina quell'apertura laterale e il pitch prescritto errato. I vecchi percorsi fixture con 212 intersezioni restano etichettati **non geometry-approved**; il vecchio C5 bloccato non è una prova fisica riuscita. Nessuna nuova ricerca XGO: solo pattern ammessi ADR-004, nessun firmware/code/parametro trasferito.

## Altre condizioni iniziali

| classe | risultato e ammissione |
|---|---|
| PRONE | Percorso offline limitato al candidato identificato e ai suoi campioni; niente promessa su tutte le pose torque-off. Non collegato al runtime attuativo. |
| PARTIAL_SUPPORT | Classifier/route matching implementati; richiedono contact mask, supporto, body pose e route geometricamente verificati. Senza route/witness coerenti: rifiuto esplicito. Nessuna acquisizione generica certificata. |
| FOUR_FEET_SUPPORTED | Risalita C4 riutilizzabile da quote compatibili; regressioni runtime sintetiche 117→150 mm e 153→150 mm. Riavvio offline da 50 mm usa il suffisso della medesima prova geometrica. |
| ALREADY_STANDING | Conferma nominale 150 mm senza nuovi target attuativi; perdita di supporto/tracking revoca la conferma. |
| UNKNOWN / UNRECOVERABLE | Missing/stale height, supporto non osservabile, pose/route/limits errati: nessun target eseguibile. IMU non inventa quota o contatto. |

## WALK / STOP / rearm

[G4 evidence](walk_after_stand.json): **214 campioni**, prima falcata 5 mm in 90 s di clock di studio, stance/swing LF/RF/RH/LH canoniche, STAND→GAIT_START→WALK→STOPPING→STAND→riammissione simulata. Raccordo STAND entro **5,6552×10⁻¹² rad**; adapter ricompilato riproduce esattamente i 12 giunti dei 214 campioni (derivate entro 3,4×10⁻¹⁶). Nessuna intersezione campionata o penetrazione; COM dentro il poligono stance con margine minimo stimato **16,014 mm** sotto ipotesi di contatti. Swept WALK, friction/loads/dynamics e tracking reale restano aperti.

Questo studio G4 usa sway 40 mm e un adapter cartesiano del riferimento cerchio G2→superficie STL. **Non è il planner runtime FeedbackWalk qualificato**: quest'ultimo non adotta ancora quei parametri/adapter. Il solo passaggio dei test dello scheduler non dimostra la stessa clearance sotto carico. Derivate G4 sono calcolate dal core; quelle della correzione STL devono essere qualificate prima del runtime.

Timeout/stale, sequence, watchdog, fault, inhibit, revoca permit/authority, failed tracking/feedback e rearm sono testati con backend doppi. La variazione nonzero vx/vy/yaw durante WALK provoca STOP coordinato e rifiuta il nuovo comando: serve una nuova richiesta da STAND riammesso. È un gap software dichiarato; non viene mascherato come teleoperazione continua.

## Servizi, Dashboard e twin

Dashboard→MotionService→Coordinator→Executor→Safe Actuator Layer→ServoBus è preservato e testato nei contratti di produzione con fatti sintetici e driver doppi. Un solo bus owner ESP32-S3; nessun endpoint concede authority o accede direttamente ai registri. RESTORE e safety non sono stati indeboliti. Il Controller manca ancora dei producer fisici, della configurazione/prime/arm e dei witness locali; mantiene `hardware_approved=false`, `support_observable=false`. RUN_STA richiede un trasporto operativo autenticato distinto dalla sola admission AP.

[Browser](browser/browser_checks.json), [DOM/HTTP](regression_results.json): 48 azioni visibili, 7 implementate in software e 41 planned/backend-disabled; nello stato fisico consegnato 46 disabilitate e STOP/status disponibili. LIE_DOWN, TROT, preset, BODY_POSE e altre funzioni non validate restano non eseguibili. Joystick vx/vy/yaw, release/focus/deadman, errori e gating sono verificati; sei schede manutenzione conservate. Browser desktop/mobile senza overflow, zero richieste attuative durante display. Il test HTTP production-handler verifica enforcement, distinta dal server demo display-only.

Il twin carica URDF e 17 STL, quaternion, actual/target distinti, piedi e timestamps/stale. Il difetto che ignorava base XYZ è corretto con test: entrambe le sagome ricevono posizione completa, dati conflittuali sono rifiutati. Le actual di questa demo sono giunti **ideali simulati**, non encoder reali; orientamento è sintetico, non BNO085 letto dal robot. Rendering e feed sono separati. Asset completi restano sul viewer ASUS per non saturare la partizione embedded.

[Animazione](canonical_motion_3d.gif), [3D](canonical_motion_3d.png), [XZ/YZ](projections_xz_yz.png), [angoli/quota/assetto](paired_angles_height_attitude.png), [piedi/contatti](feet_contacts.png), [trace](paired_motion_trace.jsonl): 558 frame con STAND, WALK, STOP e ripartenza 50 mm. Grafici/animazione derivano da STL; solo il display decima le facce, gli audit usano tutte le mesh.

## Validazione e conservazione

[Campagna](regression_results.json): **18.267 assertion esplicitamente conteggiate**, 0 failure di produzione; inoltre receipt 5 test, viewer 68 test/6 file, indipendente XML/STL/FK 7 test, DOM Motion/browser e completa suite static/host/mutation PASS. Il precedente jsdom ENVIRONMENT BLOCKED è conservato; l'ultima campagna risolve NODE_PATH e conclude PASS. Le failure dei due mutant introdotti deliberatamente significano difetti rilevati, non failure del codice consegnato. I 286 artifact storici DEFERRED non sono esecuzioni PASS.

Build ROBOT_POWERED, manifest/layout/SHA256 sono nel report finale di build e nel pacchetto esterno. La build precedentemente verificata resta intatta in `MOTION_RC_20261012`, SHA256 app `6737d7e9a7f4042e5c9a4da9a225a0e271f667db9cbf2f0363768266f4d4a733`. La nuova build da commit congelato non cancella quel pacchetto. Nessuna QUALIFIED HARDWARE.

Pubblicazione selettiva: originali MATDOG, test, report/manifest/proof JSON/CSV e immagini derivate; niente .so/ELF/firmware binaries, NVS backup, credenziali o materiale XGO escluso. Intero pacchetto locale conservato in `/home/matteo-manicardi/MATDOG/verification-artifacts/PAIRED_PRONE_TRAJECTORY_20261012` e `MOTION_NIGHT_RUN_20261012`. Il diff è selezionato per file, non `git add -A`; nessuna cancellazione accidentale. [Dossier hardware](HARDWARE_DOSSIER.md), [12 joint](joint_readiness.json), [template read-only](OBSERVATION_TEMPLATE.json).

## Ripresa esatta, in ordine

1. Conservare questa baseline e i suoi pin. Concludere swept adiacente con metodo che gestisca il fit del core articolare; non rifare pose/search. Applicare separatamente la policy di clearance operativa, senza promuovere un lower bound inadeguato.
2. Qualificare il modello di contatto/sliding, margini sui 12 giunti, mapping neutro e condizioni iniziali. Formalizzare la route inclinata nel runtime con observation/assessment indipendenti e complete; il vecchio interpolatore cartesiano flat non può eseguirla.
3. Portare nel planner WALK l'adapter di contatto verificato e i parametri di supporto, poi swept WALK e transizioni di velocità qualificate. Implementare i producer locali e il trasporto RUN_STA autenticato senza authority da Web.
4. Review della Draft PR e del manifest; nessun merge automatico. Solo dopo i gate software/evidenza: osservazioni read-only supervisionate e programma micro-test del dossier, con nuova autorizzazione hardware esplicita.
