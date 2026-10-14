# MATDOG G5-B — pipeline posturale offline

Modulo C++17 portabile, deterministico, a memoria fissa. Produce proposte cinematiche dei 12 joint semantici; nessun backend, periferica, clock interno o autorità sugli attuatori. Non è una release firmware. Il codice è fuori da `05_Firmware`, quindi il normale build del controller non lo incorpora.

## Architettura e riuso

```text
Bno085Sample (contratto dati, acquisizione futura)
  -> controlli sessione/coerenza/timing G5-B
  -> G5-A AttitudeMonitor
  -> G5-A BodyStabilizer (copia candidata, baseline integrale)
  -> BodyPose nominale + correzione roll/pitch
  -> contatti world e normale esplicita -> base_link
  -> G5-A contactInverseKinematicsGroundNormal, quattro gambe
  -> limiti operativi, conditioning, delta e rate dei 12 joint
  -> commit atomico del regolatore e JointTargetFrame valido
  -> futuro controller / validatore geometrico / Safe Actuator Layer
```

Si compilano direttamente i sorgenti G5-A già presenti, senza copiarli o modificarli. `TiltCompensation::compensateStand` resta la baseline per stand canonico, yaw zero e piano orizzontale. Il wrapper generalizza soltanto la trasformazione degli input: usa `bodyPoseFromRollPitch`, `pointToBase`, `pointToWorld` e il medesimo solver G5-A con normale esplicita. I test storici vengono recuperati fuori dal repository dal commit del manifest, con verifica SHA-256.

Frame: metri e radianti URDF; X avanti, Y sinistra, Z alto; ordine LF, RF, RH, LH e hip, upper, lower. Il quaternion è `R_world_from_base`; il suo heading non determina lo yaw comandato. Con convenzione ZYX, `roll_nom = atan2(R21,R22)`, `pitch_nom = asin(-R20)`, `yaw_nom = atan2(R10,R00)`:

```text
R_corretto = Rz(yaw_nom) Ry(pitch_nom + c_pitch) Rx(roll_nom + c_roll)
t_corretto = t_nom
b_i = R_corretto^T (contatto_world_i - t_nom)
n_base = R_corretto^T n_world
```

Il regolatore corregge verso il riferimento roll/pitch calibrato fornito nella configurazione. Il riferimento ha revisione e provenienza obbligatorie. Una futura richiesta di inclinazione intenzionale richiederà un riferimento coerente, un riarmo e una verifica separata: G5-B non sceglie automaticamente il riferimento del motion planner. Non si sommano correzioni agli angoli dei servo; non si clippano soluzioni IK.

## Contratto di utilizzo

Il chiamante fornisce `PipelineConfig` completo; gli zeri di default non approvano alcuna configurazione. `OperationalBounds` deve essere un sottoinsieme dei limiti URDF e avere provenienza/revisione: in fixture si usano limiti URDF con un margine esplicito, senza sostituire JointLimits di produzione.

1. `configure(config)` con riferimento, policy temporali, envelope cinematico e gate sperimentali espliciti.
2. `enable(nominal, nowS)` con un frame semantico iniziale riconosciuto e confermato dal chiamante. L'anchor non è un comando da eseguire: serve come seed IK e riferimento di continuità. Il modulo non acquisisce una posa iniziale.
3. `update(sample, nominal, support, nowS, intendedCommandS, optionalRate)`; controllare `reason == OK && target.valid` e consumare il frame una sola volta. La revisione dell'anchor viene letta solo durante `enable`; negli update viene usata la precedente soluzione accettata.
4. `disable()` interrompe immediatamente la pubblicazione. Continuare gli update con tempo monotono per drenare internamente la correzione G5-A.
5. Dopo fault: disable, drenaggio, `resetFault()`, nuovo anchor confermato ed enable esplicito. Reset e riconfigurazione sono rifiutati finché la correzione non è zero. Una configurazione non può cancellare un fault.

Fault di input, tempo, contatto, IK o envelope latched immediatamente dal wrapper: `target.valid=false`, tutti i joint zero, nessuna soluzione parziale o target memorizzato nell'uscita. I numeri della correzione sono solo diagnostici: durante fault/disabilitazione G5-A drena internamente, ma G5-B non pubblica neppure target di rientro. Il comportamento fisico di stop/hold/safe-off appartiene al controller e alla safety layer, con dati e permessi propri. Non eseguire il frame zero di un rifiuto.

Un duplicato esatto e ancora fresco restituisce `DUPLICATE_SAMPLE`, senza latched fault e senza target. Non avanza il clock del regolatore, così il polling non riduce l'integrazione. Cambiare payload o stamp mantenendo la sequence è incoerente. Nuove sequence richiedono stamp strettamente crescente. Regressioni, counter wrap e cambio di acquisitionEpoch richiedono riarmo; non sono accettati come campioni nuovi. Gap accettabili sono contati in `skippedSamples`; gap oltre la policy sono fault. Il modulo deve ricevere un update/watchdog anche quando il sensore non produce campioni: senza invocazioni non esiste un rilevamento autonomo del dropout.

`nowS` è il tempo di valutazione; `intendedCommandS` è la prevista applicazione, non un timestamp misurato del bus. Entrambi usano la stessa base temporale monotona dello stamp di acquisizione. Si verificano interarrival, età all'uscita, anticipo del comando e età prevista al comando. `validUntilS` è un limite temporale per la futura safety layer. I timestamp non si devono aggiornare quando si rilegge lo stesso quaternion. Il consumer deve ricontrollare scadenza e feedback al momento dell'uso; il modulo non può conoscere il ritardo reale successivo all'uscita.

Il criterio sul guadagno G5-A viene verificato anche al massimo intervallo configurato. La formula deriva dal modello a ritardo costante: questo controllo aggiuntivo e i replay con jitter non costituiscono una prova generale di stabilità per ritardi variabili, servo, attrito o compliance.

## Contatti e superficie mobile

`SupportSnapshot` è un modello completo e atomico: normale unitaria world, origine del piano, quattro riferimenti cilindrici G2 world, stato e confidence di ogni contatto, stamp, sequence, revisione, provenienza e modalità. Tutti i contatti devono essere esplicitamente `CONFIRMED_SUPPORT`. Vicinanza geometrica al terreno, quattro contatti IK, quaternion e strip nominale non provano carico, aderenza o supporto fisico. UNKNOWN, SWING o confidence insufficiente rifiutano l'intera uscita; non si stabilizzano tre gambe né fasi swing di walk in G5-B.

Nominale, contatti e normale devono condividere lo stesso frame world del planner, con gravità lungo -Z. L'allineamento di una sorgente esterna del piano è responsabilità del chiamante; lo yaw magnetico arbitrario del BNO085 non viene usato per allinearla.

`WORLD_LOCKED_HORIZONTAL` mantiene i contatti fissi e richiede normale +Z. `MOVING_PLANE_EXPERIMENTAL` richiede `allowMovingPlaneExperiment=true`: può usare posizioni/normal fornite dalla fixture o da una futura sorgente indipendente. La normale va trasformata con `R^T n_world`, non con il wrapper IK G5-A che assume +Z world. Vengono controllati coplanarità dei riferimenti, degenerazione, freshness, ordine, velocità dei contatti e della normale. Non si extrapola il movimento del piano e non si inferiscono pendenza, quota o carico dall'IMU. L'origine identifica il piano di riferimento dei contatti G2; non viene dichiarata superficie reale del battistrada deformabile.

Il solver conserva la strip nominale G2 (circa 2 gradi), seed e limiti canonici. Un piano inclinato può pertanto produrre CONTACT_MODE/IK_FAILURE: il gate non elimina il limite geometrico. Anche i casi risolti necessitano di prove collisioni, contatto reale, carico, attrito e geometria prima di qualunque futura autorizzazione hardware.

## Smorzamento sperimentale

Baseline invariata: G5-A integra `deadband(tilt-reference)` con saturazione e rate limit. Solo con gate e `dampingTimeS>0`, il wrapper passa al medesimo regolatore `tilt + dampingTimeS * tiltRate`. Il dato di velocità deve essere finito, bounded e appartenere alla stessa sequence/stamp del quaternion. Sono derivate del roll/pitch, non automaticamente gyro X/Y. Non si stima la velocità dai soli campioni di orientamento nella pipeline.

La formula corrisponde, nel modello continuo, a `dc/dt = -ki*(e + Td*de/dt)`; non è un nuovo regolatore G5-A. La configurazione e la stabilità della variante sono sperimentali. Il confronto sintetico disponibile non mostra vantaggi per la fixture testata: mantenere il gate disabilitato. Nessun guadagno o limite in `tests/Fixtures.h` è autorizzato per l'hardware.

## Masse, CoM e REV01

`InertialInput.h` è un contratto futuro bounded per 17 link: massa per-link, CoM in ciascun frame, tensori al CoM, massa/CoM/inerzia totali, assembly revision, provenance, hash e readiness. Non contiene masse compilate, non carica le inerzie REV00 e non influenza G5-B.

È definito anche il contratto per la futura proiezione del CoM: trasformazione world, proiezione lungo la gravità sul piano fornito, poligono convesso dei contatti confermati e margine signed. Il valutatore non è implementato; la dipendenza è la futura REV01 verificata. Occorre validare completezza, unità, frame, somma delle masse, tensori simmetrici/semidefiniti e provenienza prima del calcolo. Mancanza di REV01 non impedisce la pipeline cinematica, ma impedisce dichiarazioni di stabilità dinamica o margine fisico del CoM.

## Integrazione futura, non eseguita

Il driver Bno085Imu di main non offre i campi necessari. In PR #49 `orientationSnapshot()` espone valid, stamp millisecondi e quaternion, ma manca uno snapshot atomico di sequence, accuracyRad/status, reset epoch e velocità. Proposta di interfaccia al suo owner: getter read-only che copi insieme SH2_ROTATION_VECTOR, timestamp di ricezione/acquisizione distinguibili, count, accuracy e reset generation. Serve conversione monotona ms->s con gestione esplicita wrap/sessione; nessun restamping alla lettura. Non implementato né modificato il driver. La telemetria ~500ms non è un input di controllo 50Hz.

Il futuro MotionService/MotionCoordinator sceglierà quando la stabilizzazione è eleggibile, costruirà nominale/anchor e accetterà solo contatti osservabili. Un candidato G5-B non sostituisce posa classificata, entry plan, tracking o permit. Il consumer dovrà legare le revisioni di riferimento/bounds/nominale/supporto alla sessione, ricontrollare deadline e ordinamento e rigettare i frame sperimentali prima dell'hardware.

La firma corrente di MotionExecutor in PR #49 richiede separatamente `JointTargetFrame`, `FrameValidation`, issued time/TTL, durata, execution facts e feedback. Il futuro adapter potrà fornire SOLO il frame e i metadati G5-B; non dovrà impostare `collision_clear`, `support_valid`, `path_continuous`, `configuration_valid`, approved o lease a partire da `target.valid`. Collisioni, proiezione REV01, percorso continuo tra campioni, velocità/accelerazioni, tracking, sequenzialità e latenza del bus devono essere qualificati indipendentemente. La verifica del delta G5-B non prova accelerazione o sweep collision-free.

La Safe Actuator Layer conserva ownership, autorità, q0/direction/raw resolution, limiti operativi, preflight, fault e safe-off. G5-B non mappa ID servo, tick o q0 e non crea permessi. Nessuna chiamata verso MotionService, MotionExecutor o ActuatorRuntime è stata aggiunta. Non c'è un feedback ACK nella pipeline: per l'offline ogni frame accettato viene considerato consumato; l'integrazione dovrà ricostruire/riarmare con un anchor confermato se un frame non viene applicato o la sessione cambia.

## Riproduzione offline

```bash
python3 06_Software/Matdog_Core/stabilization_g5b/run_validation.py
```

Servono g++, preferibilmente clang++ per ASan/UBSan, Python 3.12, numpy/scipy/PyYAML già usati dagli oracle MATDOG. Gli oracle storici richiedono inoltre le dipendenze già dichiarate nella loro requirements: trimesh, python-fcl, rtree e networkx. È possibile fornire quelle già installate tramite PYTHONPATH. Il runner non installa né scarica niente. Il commit storico `e1704719979789cd9c9f18741e4546725558797e` deve già essere disponibile nel clone; in caso contrario il gate storico è BLOCKED e va acquisito separatamente dal repository, nel clone isolato.

Il runner preserva il FAIL noto del vecchio test visual-zero e verifica i tre file identici a main; gli altri gate obbligatori restano fail-closed. `all_passed=false` è mantenuto quando il FAIL preesistente è presente. `required_gates_passed=true` indica esclusivamente la pipeline e le regressioni pertinenti passate, senza nascondere il FAIL. Le simulazioni e gli oracle deterministici vengono ripetuti e confrontati byte per byte; benchmark host e tempi delle suite non sono deterministici.

Risultati e limitazioni sono nel report `09_Logs/Validation_Reports/G5B_IMU_Offline_2026-10-14/REPORT.md`.
