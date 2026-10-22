# MATDOG SIMPLE V1 — candidato operativo

Branch `matdog/simple-motion-v1`, worktree `/home/matteo-manicardi/MATDOG/worktrees/matdog-simple-motion-v1`.
Base isolata: `ad0a0910124cb849056f872c83594916902dfe24`. Nessun flash, movimento, accesso a porte hardware, scrittura NVS/EEPROM, push o merge durante lo sviluppo.
Identità APP: `1.0.0-rc.1`, `MATDOG_SIMPLE_V1`, ROBOT_POWERED, OTA ingest disabilitato. Commit e SHA256 esatti sono nel `matdog_build_manifest.txt` accanto all'APP finale.

**Implementato:** un comando STAND acquisisce la posa reale, verifica profili/allarmi/calibrazione, cattura e verifica il riferimento RAM con torque OFF, abilita automaticamente i 12 servo, esegue pose coordinate, verifica il raggiungimento e mantiene STAND. WALK STANDARD è nella stessa APP: STAND automatico a 150 mm, gait esistente, IK esistente, tre passi 20/30/40 mm per ciclo di 7 s, lift 15 mm, duty 0,8, partenza e arresto coordinati. FAST escluso.

La quota nominale accetta l'intervallo di comando 0–150 mm; il workspace MATDOG C4 recuperato e verificato è **60–150 mm**, ulteriormente ristretto dai limiti live: quote inferiori vengono rifiutate prima del torque. Nessuna ridefinizione dello zero. La quota restituita è stimata dagli encoder e dall'orientamento IMU, con ipotesi di piano di appoggio; sulla fixture non misura l'altezza dal pavimento.

## Catena e riuso

`Dashboard → MotionService → SimpleMotion → gait/IK → ServoBus → ST3215`.
`SimpleMotion` possiede l'unico stato operativo: READY, MOVING, STANDING, WALKING, STOPPED, FAULT. Il Controller mantiene l'unico proprietario UART1 e l'arbitro già esistente per escludere la manutenzione concorrente. Nessun comando PRIME/ARM, permesso Motion o sessione operativa aggiuntivi. Dopo STOP, STAND riacquisisce anche pose erette, intermedie o WALK contenute nelle regioni geometriche verificate; non impone il ritorno manuale in PRONE.

Recuperati: SCServo 1.0.2 e protocollo C018; `ServoBus`, `ValidatedServoRead`, `BusTransaction`, `ServoPacket`; conversioni e popolazione canonica; Geometry V5/URDF; `Gait`, `solveGait`, IK analitica e contact IK; estrazione tilt/IK inclinata; driver BNO085 e task acquisizione; DALY; LED, Wi-Fi, autenticazione e mailbox HTTP. Restano integri `CalibrationExecutionEngine → ActuatorRuntime → ServoBus::writeGoalPosition → WritePosEx`, Full Calibration, certificati, restore e manutenzione NVS.

Rimossi dalla nuova sorgente eseguibile: directory `execution`, MotionExecutor/Coordinator/Session, Commissioning e LocalValidation, OperationalReadiness, MotionClock/scheduler precedenti, AdaptiveStand/FeedbackWalk e relativi gate, PATH_SAMPLE/PATH_BOX, retimer e monitor di tracking. `GaitTiming` recupera soltanto la legge quintica già usata per partenza/arresto STANDARD. Non è un altro gait engine.

STAND usa i waypoint dei cinque tratti geometrici recuperati (PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER, RISE). Invia **un SYNC_WRITE ai 12 ID** per posa e aspetta l'arrivo reale, con tolleranza V25 di 12 tick e speed nativa ≤50, prima del tratto seguente. Nessun confronto con un microtarget istantaneo. La deadline usa distanza, velocità/accelerazione native e il timeout V25 di 12 s. I bounds CAD verificano lo spazio delle configurazioni tra pose, anche con avanzamento indipendente dei giunti: non sono un monitor temporale degli encoder.

## Profili esclusivamente RAM

| Uso | GoalSpeed | ACC | Motivazione |
|---|---:|---:|---|
| STAND | 300 | 8 | Velocità operativa esistente, circa 0,46 rad/s; accelerazione della Full Calibration hardware conservata, circa 1,23 rad/s² alla scala nominale SDK. |
| WALK STANDARD | 1000 | 50 | Profilo STANDARD recuperato: circa 1,53 rad/s e 7,67 rad/s² nominali, sopra i picchi del riferimento (~0,84 rad/s, ~3,69 rad/s²). |
| Calibrazione/manutenzione | 160 | 8 | Ricerca hardware 24/24 del 01/10, invariata. |

LeRobot e norma-core/SO-101 usano ACC=254; non è trasferita come qualificazione al quadrupede. ACC=20 non è un vincolo del nuovo firmware. Regolazione, accelerazione e velocità rimangono interne al servo. Nessuna modifica EEPROM, PID, ID, offset, geometria o formato di persistenza. Il preflight verifica le protezioni C018 effettivamente memorizzate.

## Tempo e sicurezza

Un clock `esp_timer_get_time()`, periodo Motion **40 ms / 25 Hz**, nessun recupero di microframe arretrati. Il riferimento STANDARD ha passo massimo offline ~0,033 rad a questa cadenza. Ogni target è un gruppo di 12 giunti.

Budget di attesa UART per release: quattro feedback da ≤2 ms (8 ms), un controllo a due blocchi ogni 120 ms (≤4 ms), eventuali quattro conferme termiche dirette (≤8 ms), SYNC_WRITE 104 B a 1 Mbaud = 1,04 ms. Totale massimo nominale di queste operazioni: **21,04 ms**, prima di calcolo, overhead e scheduling. Sono budget software/tempi di linea, non misure ESP32 sul robot. DALY 8 B a 9600 baud richiede circa 8,34 ms: viene servito soltanto con almeno 12 ms liberi prima della release. Wi-Fi/configurazione lenta rimandata a torque disabilitato; il server HTTP esistente continua a funzionare. Console RX limitata per giro; output USB non bloccante.

IMU in task separato, timestamp ricezione reale corretto per età SH2 disponibile; normalizzazione del quaternion quantizzato. Correzione roll/pitch proporzionale 0,5, ampiezza ≤0,020 rad, applicata al generatore mediante IK. Ridotta/soppressa se la soluzione esce da limiti o geometria ammessa. Non è un PID servo aggiuntivo. Guadagno e comportamento caricati rimangono da qualificare sull'hardware.

STOP Dashboard su corsia prioritaria e STOP USB indipendente da HTTP; interrompe anche la manutenzione attiva. SAFE_OFF prima dell’inizializzazione lenta al boot e burst prioritario sui **13 servo installati**, seguito da lettura torque per ogni ID e retry fino a 13/13: TX completa non viene dichiarata torque OFF verificato. Persistono limiti meccanici, allarmi nativi, controlli RAM, BMS, perdita persistente delle risposte, heartbeat operatore di 3 s e task watchdog esistente di 5 s. Perdita della cadenza mentre energizzato avvia SAFE_OFF. Nessun tracking del normale ritardo dinamico.

## Verifiche offline e limiti della consegna

APP ESP32-S3 compilata con core 3.3.11, layout identico (`8f756ecb…46e7`), slot 5 MiB. Ordine di grandezza verificato: APP ~1,61 MB, RAM statica ~120 kB, stack Controller 16 KiB, task IMU 8 KiB. Dimensioni definitive nel log e manifest della build pulita. Heap, stack residuo e tempi di calcolo sul dispositivo non sono misurati offline.

- Record reale gen1: fingerprint `be8d5902bb4e37da`, restore e 12 limiti, mapping/direzioni/round-trip; zero scritture nello storage host. Moduli di persistenza, certificati e dati Geometry V5 identici alla base congelata.
- SYNC_WRITE 104 B di entrambi i profili e SAFE_OFF 13 ID/34 B confrontati byte per byte con SCServo; parser e deadline/STOP/ownership del bus verificati con i test esistenti.
- Sequenza STAND, quota 60/150, tre preset WALK, stop coordinato, IMU entro workspace, STOP, allarmi, mancato raggiungimento e comunicazione persa: test con osservazioni esplicitamente iniettate. **Non sono un modello fisico dei servo e non validano il movimento.**
- Collisioni: riuso delle prove CAD/FCL continue di entry, phase chords, height e STANDARD; corrispondenza URDF e 17 mesh verificata, 252 waypoint identici. Digest in `CollisionData.h`, esportabile con `scripts/recover_simple_collision_data.py --evidence-root <TOLERANCE_V1_20261018>`.
- Contratto/API JSON e buffer 4096 B verificati; sintassi JavaScript controllata. Nessuna prova browser/dispositivo collegato al robot.

Log e risultati: `/home/matteo-manicardi/MATDOG/build-output/simple-motion-v1-*.log` e `simple-motion-v1-offline/`. Le suite del precedente Motion Core appartengono al worktree congelato, non a questa catena sostitutiva. Test V1: `scripts/tests/run_simple_motion_tests.sh`, `run_servo_packet_tests.sh`, `run_servo_bus_deadline_tests.sh`, `run_calibration_restore_tests.sh`; record reale con `scripts/release/check_gen1_tolerance_offline.cpp`.

Rimane da eseguire esclusivamente il collaudo hardware autorizzato separatamente: transitori reali, profili sotto carico, latenza STOP, jitter/tempi ESP32 e stabilizzazione/supporto a terra. Non sono dichiarati PASS da una simulazione.

## Comandi e primo collaudo supervisionato

Dashboard: STAND con quota, WALK STANDARD 20/30/40, WALK STOP coordinato, STOP prioritario, stato, stima quota, IMU, alimentazione e feedback. Endpoint esistenti `/api/motion/status`, `/api/motion/capabilities`, `/api/motion/command`; autenticazione conservata. USB: `@MOTION STAND 150`, `@MOTION WALK STANDARD`, `@MOTION WALK STOP`, `@MOTION STATUS`, `@MOTION HEARTBEAT`, `@STOP` / `@MOTION STOP`. Un operatore USB mantiene l'heartbeat; la Dashboard autenticata lo rinnova automaticamente. Comandi Calibration & Maintenance esistenti conservati.

Dopo autorizzazione separata: verificare candidato/rollback e backup esistenti, flash solo APP con layout invariato; robot sospeso e arresto alimentazione disponibile. Al boot verificare SAFE_OFF 13/13, preflight 12/12, gen1/fingerprint, IMU/BMS e READY. Eseguire un solo STAND150, verificare feedback/GoalPosition/RAM e completamento, poi STOP e 13/13. Ripetere per una quota raggiungibile. Provare STANDARD 20 mm, WALK STOP e STOP/perdita heartbeat; misurare cadenza, STOP e risorse sul dispositivo. Solo dopo qualificazione della fixture procedere al supporto a terra e alla stabilizzazione caricata. Nessuna Full Calibration da rifare.

Riferimenti consultati: [Full Calibration dfcecb6](../../09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md), [XGO ricostruzione](https://github.com/MattRobotics/xgolite-low-level-reconstruction) (pipeline e clock, nessuna geometria trasferita), [norma-core ST3215](https://github.com/MattRobotics/norma-core/tree/main/software/drivers/st3215) e relativo esempio SO-101, [LeRobot SO follower](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py) e [configurazione motori](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py). Il firmware installato resta `ad0a091`, APP `7ada67c82992c16326e1cc039c5a70ffb98761d350b51bdab0ebe469981144a2`, confermato dal readback archiviato del 21/10.
