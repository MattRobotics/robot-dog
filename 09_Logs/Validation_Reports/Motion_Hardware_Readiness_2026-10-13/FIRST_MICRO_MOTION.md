# A — FIRST SUPERVISED MICRO-MOTION ON FIXTURE

**NO-GO oggi.** Preparazione software disponibile; questo documento non autorizza flash, torque o movimento. I gate WALK/STAND autonomo non sono prerequisiti di A. La fixture deve assumere interamente il carico e trattenere il corpo anche a torque OFF; piedi portanti e altezza assoluta non devono essere inferiti.

## Gate minimi e tipo di evidenza

| Gate critico | Offline disponibile | Misura/review locale necessaria | Stato oggi |
|---|---|---|---|
| Firmware esatto, flags 0, SHA/manifest/layout | Build e receipt | Identificare immagine installata; approvare separatamente futura immagine abilitata | Software candidato; authority 0 |
| Record corrente integro/acknowledged/current | RESTORE parser/CRC/schema/generation/geometry/identity testati | Leggere status del record installato; confrontare generation attesa e montaggio | BLOCKED misura corrente |
| 12 identità, mapping e zero | Tabella URDF↔bus↔unit↔direction; mapping provato in simulazione | Etichette unità e montaggio, q0 neutro/linkage, nessun servo scambiato | BLOCKED |
| 12 limiti del piccolo workspace osservato | Builder e proposta, test rejection | Review posizione effettiva e contatti correnti; 12 bound approvati insieme | 0/12 approved |
| Fixture, persone, kill fisico | Procedura e revoca simulata | Corpo trattenuto OFF, accesso alle gambe, cavi liberi; operatore al taglio servo | BLOCKED |
| Percorso micro specifico | Quintico di un joint, 11 hold, bound analitici; gate geometry/epoch/path | Osservare posa iniziale e fixture; certificare il piccolo box/corridoio di tutti i 12 target | Artifact specifico assente |
| Freshness 12 servo + IMU | Producer/timestamp/dropout tests | 12 read-back attuali; drift, round-trip/WCET/errori, quaternion valido/timestamp | BLOCKED |
| Prime/torque/holding/output ceiling | Sequenza vera attraverso Safe Actuator Layer, simulazione | GoalPosition/PRESENT coherence, output ceiling efficace, torque state e holding/revoca verificati sotto fixture | BLOCKED |
| Alimentazione e temperatura | Producer BMS/allarmi, domain checks | Rail servo, BMS/allarmi, max temp/voltage e limiti scoped approvati | BLOCKED |
| Velocity/accel/tracking/STOP/watchdog | Analisi target, checks, fault/revoca sim | Bound caricati per micro-test, deadline feedback/timeout/abort e STOP reale | BLOCKED |
| Permit locale esclusivo e immagine autorizzata | MotionSession/arbiter/generation/expiry | Review gates, adapter locale esplicito e separata autorizzazione operatore | Nessun grant nella candidata |

Non serve qualificare una falcata, un trasferimento ventrale→piedi, la dinamica WALK, il tubo di ingresso prone o il support polygon autonomo. Serve però che **tutti i target di prime/hold dei 12 joint** siano ammessi: un LOWER iniziale fuori limite non si rende lecito perché si intende muovere solo un HIP.

## Sessione read-only che l'operatore può preparare ora

1. Robot su fixture rigida, corpo sostenuto senza motori; documentare punto di sostegno, ritenzione, capacità, libero volume delle gambe e assenza di persone nella loro escursione. Un secondo operatore deve poter interrompere fisicamente il rail servo. Verificare l'interruttore senza comandare i servo; documentare rail che cade e supporto che tiene il corpo.
2. Identificare firmware installato, profilo, build ID e comandi disponibili. La baseline indica `b08322dedc...`; non utilizzare endpoint o RESTORE nuova presumendo che questa candidata sia installata. Registrare `@STATUS`/help della versione effettiva e mode/authority.
3. Usare soltanto comandi read-only effettivamente presenti. Per le versioni documentate: `@CALIBRATION PERSIST STATUS`, `@CALIBRATION Q0 STATUS`, `@IMU STATUS`, `@BMS STATUS`, `@SERVO READ <id>` in MAINTENANCE. ID canonici: 13,12,11,23,22,21,33,32,31,43,42,41. Evitare scan/calibration/recovery/grant, BMS KEY write, SAVE/ACK/reconcile o altri writer. Verificare prima il lessico disponibile sulla versione installata.
4. Raccogliere per ciascun ID posizione raw, torque state, GoalPosition se esposto, status errore, temperatura, tensione e timestamp host/device coerenti. Ripetere letture read-only per una finestra finita (proposta 10 s) e registrare min/max drift, latenza massima e fallimenti; non scambiare 12 letture sequenziali per una misura simultanea.
5. Registrare quaternion BNO085, timestamp ed eventuali reset; BMS freshness, tensione/allarmi e stato power. Misurare separatamente il rail servo con strumento appropriato: BMS del pacco non dimostra il rail del bus. Nessuna telemetria elettrica viene etichettata “load-bearing”.
6. Fotografare/annotare etichette fisiche delle 12 unità e mounting. Misurare pose dei linkage rispetto al modello e verificare il riferimento di zero disponibile nel record, senza forzare endstop o rifare Full Calibration. Dati insufficienti → gate resta BLOCKED.
7. Selezionare un joint lontano da endstop/URDF e certificare il piccolo workspace della posa reale; approvare posizione/rate/accel/tracking/thermal/electrical e deadline specifiche. Gli intervalli in `JOINT_LIMITS.md` sono proposte, non limiti da caricare automaticamente.
8. Compilare la scheda misure e fare review A. Solo dopo tutti i gate critici è possibile chiedere la separata autorizzazione per installazione/RESTORE in RAM/prime/torque/micro-test. La richiesta deve nominare immagine SHA, joint, escursione, durata, fixture, personale e abort. Nessuna autorizzazione a WALK è implicita.

## Proposta di prima escursione, da approvare dopo le misure

Proposta iniziale: **un solo joint, ±2 tick = ±0,003067962 rad = ±0,17578125°**, durata nominale ≥1 s, altri 11 target al PRESENT osservato. Segno scelto verso l'interno del limite misurato. Non è un limite approvato né un default runtime.

Per quintico, con δ=2 tick/T=1 s: max velocità target 1,875|δ|/T = 0,00575243 rad/s; max accelerazione target (10/√3)|δ|/T² = 0,017713 rad/s². Sono bound software, non una misura della risposta ST3215. Scegliere T più lungo se l'envelope qualificato è inferiore; qualunque tolleranza tracking deve restare inferiore al margine geometrico/meccanico residuo. Il runtime attende feedback/ACK di ogni frame: il wall-clock può superare T. Lifetime del permit e timeout per frame devono limitare esplicitamente il tempo reale.

Il target 2 tick non va usato su un joint entro 2 tick dal bound approvato. La risoluzione encoder (1 tick) e microrad del resolver vanno inclusi nella tolleranza di tracking; 1e-7 rad della route autonoma non è un threshold appropriato per questo test. Se il pose box degli altri joint non è valido, NO-GO.

## Procedura di movimento futura, esclusivamente dopo review e autorizzazione separata

1. Operatore e supervisore presenti, fixture e taglio servo ricontrollati, logging attivo, rail/tensioni/temperature nei limiti della scheda approvata; nessuna sessione calibration/discovery concorrente.
2. SAFE_OFF verificato sui 12 joint e sul collo ID51 secondo il dossier dell'installazione. Se non risponde o torque non è OFF, non procedere; usare il taglio fisico. L'esecuzione operativa muove solo i 12 joint; il collo deve restare fuori dal test.
3. RESTORE esplicita in RAM del record corrente nella versione che la supporta; aspettarsi 12 trasformazioni ammesse, limiti ancora non approvati e permit revocato. Applicare solo l'artifact scoped A approvato, vincolato a geometry/generation/epoch/workspace.
4. Nuova osservazione iniziale fresca di tutti i 12, senza altezza o carico inventati. Fissare i limiti autorizzati di δ/T/tracking/deadline e scadenza breve del permit. Nessun comando Web può armare.
5. Grant locale specifico FIXTURE_MICRO tramite l'adapter operator approvato; attesa PRIMING PRESENT mentre OFF. Expected: 12 target coerenti con gli encoder, nessuna variazione significativa del linkage. Poi ENERGIZING con output ceiling qualificato, 12 torque-on freschi successivi; ARMING → ARMED solo dopo completa osservazione. Drift o feedback incompleto → revoca/SAFE_OFF.
6. Inviare il singolo micro-corridoio certificato, rispettando joint/segno/δ/T. Expected: tracking dentro tolerance qualificata, gli altri 11 restano dentro box/hold tolerance, niente urti, rumore anomalo o movimento del corpo. Endpoint: `FIXTURE_MICRO_TRACKED`; l'ACK del browser da solo non costituisce risultato.
7. STOP tramite percorso sicuro locale/Web autenticato; fixture continua a sostenere. Expected: revoca, SAFE_OFF_REQUIRED, readback OFF dei 12, release authority. Se risposta/energia/movimento violano la deadline, taglio fisico immediato; non attendere rete o ACK. Collo sempre OFF.
8. Registrare risultati, non eseguire ritorno automatico. Una escursione inversa è un nuovo test con nuovo permit/review di stato e percorso; nessun rearm automatico. Tornare a power-safe secondo la procedura hardware locale.

Abort immediato: perdita fixture/operatore/kill, ingresso persona, salto di target/servo, drift hold, tracking oltre bound, stale uno dei 12 o IMU, perdita timestamp, status errore, temperatura/tensione/allarmi fuori envelope, timeout prime/batch/tracking, STOP non confermato, torque inatteso, cambi generation/epoch/authority o sessione. Fixture porta il peso anche durante SAFE_OFF.

## Scheda registrabile (nessun dato precompilato)

| Campo | Misura / review |
|---|---|
| Data/ora, operatore, supervisore | |
| Firmware installato e candidato SHA/commit | |
| Geometry/record digest/schema/generation/ACK/integrità | |
| 12 identità/mounting/q0 confermati | |
| 12 encoder/torque/GoalPosition/timestamp, max age/latenza/drift | |
| IMU quaternion/timestamp/reset; BMS/allarmi; rail servo misurato | |
| Fixture/ritenzione/kill e collo OFF verificati | |
| Artifact 12 limiti/workspace, hash/generation/epoch | |
| Joint/sign/δ/T, velocity/accel/tracking, temp/voltage, timeout/lifetime approvati | |
| Prime/energize/arm feedback e tempi | |
| Target vs actual dei 12; endpoint; STOP/SAFE_OFF/authority release | |
| Abort/anomalie; PASS/FAIL/BLOCKED; firma GO per eventuale prova successiva | |
