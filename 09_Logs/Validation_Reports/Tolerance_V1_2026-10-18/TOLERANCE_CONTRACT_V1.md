# MATDOG — contratto tolleranze operative V1

Data: 18 ottobre 2026. Ambito del candidato: ESP32-S3, profilo ROBOT_POWERED, STAND 60–150 mm e WALK STANDARD **su fixture supervisionata**. WALK FAST resta escluso. Nessuna qualificazione del moto a terra viene dedotta da questa revisione.

Il codice candidato separa risoluzione matematica, quantizzazione, arrivo, inseguimento e dominio geometrico. I numeri di accettazione sono implementati; la loro adeguatezza sotto carico resta una qualificazione hardware. Un certificato CAD non misura attrito, flessione, carichi o montaggio reale.

## Provenienza e letture

Baseline installata: `789e07cf27014cad3672a8666262d22b1a4039ae`, APP SHA256 `1bf9ab6f628490fc946d478e2c32e5143e7e739b97412ec1830cf2c563447ec8`, ROBOT_POWERED. L'immagine conservata è stata nuovamente hashata; corrisponde al log di flash/verify del 17/10 e alla firma del primo boot. Non è stata interrogata né scritta la scheda in questa sessione: questa è verifica delle evidenze conservate, non una nuova attestazione live.

Worktree di riferimento pulito: `~/MATDOG/worktrees/robot-dog-stand-operational-20261017`, HEAD `6d60e1f` (i commit dopo `789e07c` modificano soltanto la designazione). Sviluppo isolato, una sola linea: `~/MATDOG/worktrees/robot-dog-tolerance-v1-20261018`. Il worktree iniziale `github/robot-dog` contiene modifiche preesistenti e non è stato modificato. Nessun altro agente della sessione era attivo; non sono stati avviati subagenti.

Letti: `first-stand/ANALISI_PRIMO_STAND.md`, `first-stand/RCA_ENTRY_ALIGN.md` sotto `verification-artifacts/STAND_OPERATIONAL_20261017`; revisione precedente `09_Logs/Validation_Reports/Entry_Tube_2026-10-17/README.md` e `entry_region_tube_sweep.json` nel worktree `robot-dog-entry-completion-20261017` @ `2cde0e2`; executor/coordinatore/sessione/planner, certificati, Geometry Compiler V5, URDF, mesh collisione, calibrazione YAML storica e record gen1 realmente conservato. Nessuno di questi documenti obbligatori è risultato mancante.

**Correzione della revisione precedente:** `minimum_centre_distance_m` è il minimo fra i centri campionati, quindi un upper bound del minimo globale, non la sua dimostrazione. Il bound continuo storico di 6,047 mm supera la richiesta di 6 mm di 0,047 mm; non è una distanza fisica di 0,047 mm fra superfici. Neppure i circa 7,73 mm campionati provano un minimo globale di 7,73 mm.

## Tabella del candidato

SSOT dei budget operativi: `src/motion/OperationalTolerance.h`. I valori programmati sono confrontati in radianti semantici dopo le conversioni reali, non arrotondando prima l'errore a un numero intero di tick.

| Grandezza | Prima | Candidato / nuovo | Ambito e motivazione | Stato e singola misura ancora necessaria |
|---|---:|---:|---|---|
| Tolleranza numerica IK | `1e-9 m` | invariata `1e-9 m` | IK analitica e contatto; precisione di calcolo, non accuratezza fisica | VERIFIED OFFLINE: round trip FK/IK mirati; nessun rilassamento matematico |
| Risoluzione ST3215 | 4096 tick/giro | invariata: 1 tick = 1,533980788 mrad = 0,087890625° | conversione joint-servo, dominio raw 0–4095 | VERIFIED HARDWARE per i raw del log e conversione; specifica di risoluzione del produttore, non prova di accuratezza angolare assoluta |
| Quantizzazione / arrotondamento | implicita | ±(0,5 tick + 2 µrad) = ±0,768990394 mrad | budget conversione q→µrad→raw→µrad, separato dal residuo servo | VERIFIED OFFLINE sul resolver reale; non comprende errori fisici del sensore |
| Residuo osservato #61 | 5 tick | dato invariato: 7,669904 mrad raw-goal; massimo errore semantico replay circa 7,84 mrad | RH_UP/LH_UP fermi, velocità zero, due tentativi | VERIFIED HARDWARE limitatamente alla sessione del 17/10, senza dedurne capacità sotto carico |
| Arrivo alla fine fase / posa | 6 mrad = 3,91 tick = 0,344° a ogni frame | 10 tick + quantizzazione = **16,108798273 mrad**, circa **0,923°**, 10,5013 tick | ENTRY a fine stadio, fine fase STAND, quota finale; HOLD WALK usa lo stesso limite più velocità di assestamento | IMPLEMENTED NOT VALIDATED; misura: residuo finale peggiore dei 12 giunti nella singola prova fixture |
| Tracking intermedio | 6 mrad per-frame statici | **20 mrad = 1,145916° = 13,038 tick** | frame finiti: errore rispetto al box `[precedente,target]`, con avanzamento solo se errore sul target ≤20 mrad; WALK: distanza dall'intervallo dei riferimenti effettivamente inviati negli ultimi 150 ms | IMPLEMENTED NOT VALIDATED; misura: massimo errore e ritardo per-giunto durante ENTRY/STAND, poi WALK |
| Errore che ferma | attesa fino a timeout in TRACKING finito | uscita dal box segmento ±20 mrad: `TRACKING_DIVERGENCE` immediato al campione; HOLD oltre20: `HOLD_TRACKING_ERROR` | nessuna persistenza per attraversare il limite geometrico; WALK conserva arresto immediato fuori box e tracking persistente su 3 campioni nuovi | VERIFIED OFFLINE per il comportamento; soglia fisica non qualificata |
| Passo target STAND | 6 mrad, 0,29 rad/s ×20 ms | **4 mrad**, velocità **0,19 rad/s**, periodo minimo20 ms; accelerazione2 rad/s² invariata | riserva dell'intero box anche se il frame attraversa un vertice della polilinea | VERIFIED OFFLINE per limiti e replay; tempo reale rallentabile dal feedback, non promesso50 Hz su hardware |
| Dominio geometrico | tubo6 mrad | **40 mrad = 2,291831° = 26,076 tick** | nuovo dominio CAD più ampio; non è una tolleranza di tracking ammessa | VERIFIED OFFLINE solo nei domini esplicitati nei certificati; non prova del robot fisico |
| Riserva CAD/installazione | non distinta | **15 mrad = 0,859437°**, limite candidato combinato | riserva per mapping fisico, flessioni e installazione nelle coppie non adiacenti; non viene aggiunta come permesso di errore servo | PROPOSED FOR HARDWARE QUALIFICATION; misura: errore massimo esterno giunto/CAD, incluse inversioni di carico. Gli accoppiamenti adiacenti vicino al folded-stop richiedono verifica locale, vedi sotto |
| Modello angolare locale adiacenti ENTRY | non separato | **8 mrad =0,458366°**, circa5,22 tick | folded-side LOWER; domini SAT comprendono il limite misurato meno8 mrad, LF fino a−1,6065 rad | PROPOSED FOR HARDWARE QUALIFICATION; misura: errore esterno angolo/CAD dei LOWER durante la prova fixture. La prova CAD del dominio è VERIFIED OFFLINE |
| Bound misurati usati dalla prova ENTRY | dominio generico esteso oltre i limiti effettivi | LOWER LF/RF/RH/LH ≥ **−1598,5 /−1580,1 /−1592,4 /−1590,9 mrad** | condizioni di contenimento dei limiti caricati, arrotondate sotto i limiti reali gen1; non sostituiscono i limiti raw | VERIFIED OFFLINE contro record gen1; un limite caricato più ampio viene rifiutato dalla configurazione geometrica |
| Clearance non adiacenti |3 mm | invariata3 mm; ENTRY e box quota/WALK verso base_link6 mm | distanza fra superfici CAD, non parametro di prestazione | VERIFIED OFFLINE entro i domini delle prove; fixture, cavi e cover modificata non diventano modellati |
| Clearance più robusta desiderata | — |10–12 mm | obiettivo di percorso, non soglia sostitutiva della prova corrente | PROPOSED FOR HARDWARE QUALIFICATION / futuro planner; non ottenuta come garanzia universale sull'intera regione ammessa |
| Feedback segmenti |400 ms | invariato400 ms; campioni post-TX per avanzare | feedback reale; errore di lettura, clock futuro e torque perso hanno motivi distinti | VERIFIED OFFLINE; misura: età massima durante la prova con rete attiva |
| Feedback WALK |100 ms per-giunto, osservazione120 ms | invariati; lag150 ms; 3 campioni per mancato seguito | streaming già esistente, distinto dal tracking istantaneo sul target più recente | VERIFIED OFFLINE, non WCET hardware |
| Assessment |2 s |2 s all'ammissione; CAD statico del medesimo frame accettato resta vincolato a executor/epoch/limiti e timeout | evita di etichettare un arrivo lento come dato geometrico invecchiato. Evidenza esterna dinamica continua a scadere a2 s | VERIFIED OFFLINE: età, sequenza e contenuto continuano a essere verificati |
| Mancata convergenza | executor4 s, nascosto da assessment2 s |4 s invariati: `ARRIVAL_TIMEOUT` finale, `TRACKING_TIMEOUT` intermedio | un servo fermo non consente avanzamento indefinito: errore si accumula, arresto o timeout entro il budget | VERIFIED OFFLINE |
| Timeout batch / frame |1500 ms / segmento20 s | invariati | guasto bus / comando distinto dal tracking | invariati, regressione offline |
| Deadman |3 s | invariato3 s | proprietà del firmware; pagina o autenticazione non ne prolungano la scadenza senza heartbeat valido | VERIFIED OFFLINE; misura: tempo heartbeat perso→SAFE_OFF nella prova |
| Regolazione quota |8 s per50 mm, minimo8 s |12 s per50 mm, minimo12 s | stessa IK a contatti fissi, temporizzazione compatibile con0,19 rad/s | VERIFIED OFFLINE per60/100/140→150 mm; quota fisica ancora da misurare |

L'ineguaglianza conservativa del candidato è:

`20 + 4 + 0,768990394 + 15 = 39,768990394 mrad < 40 mrad`.

Il passo è contato interamente: non si assume che il punto medio di un box indipendente giaccia sulla polilinea. La riserva fisica di15 mrad è una **condizione di qualificazione**, non un fatto misurato. Le prove adiacenti ENTRY usano domini espliciti: i LOWER sono limitati dai bound misurati meno8 mrad di allowance angolare locale, mentre le altre articolazioni conservano il box completo. Prima del moto reale occorre qualificare questo bound fisico e la clearance locale; il report non li promuove a PASS fisici.

La prova40 mrad non limitata aveva quattro witness UPPER/FOOT sotto3 mm, sul lato folded intorno a−95°, oltre i limiti effettivamente ammessi. Il primo tentativo di restringere il lato aperto non li risolveva ed è stato scartato. Il certificato finale interseca il dominio con i limiti LOWER misurati ammessi più15 mrad di incertezza fisica per le coppie non adiacenti. Per gli adiacenti LOWER usa un bound angolare locale candidato di8 mrad. Il firmware verifica che i limiti caricati non siano più ampi di quelli usati nella prova: **non cambia** alcun limite raw. Il riconoscitore±10° e il percorso dal PRONE osservato restano invariati.

Il bound locale8 mrad non è provato dagli encoder: è PROPOSED FOR HARDWARE QUALIFICATION. Una misura esterna dell'errore angolare nei LOWER vicino al folded-stop deve dimostrarlo prima della qualificazione del moto. Il checker prova anche il piccolo tratto oltre il limite URDF necessario a includere quell'errore fisico; non si assume che un modello fisico coincida esattamente con il dato encoder. Gli altri accoppiamenti adiacenti, la cover e i cablaggi richiedono l'ispezione già dichiarata.

I limiti raw gen1 restano vincolanti anche quando il tubo CAD è più ampio. Un feedback entro20 mrad ma oltre un limite raw/URDF viene arrestato. Il compiler non allarga quei limiti.

## Capacità reali e confronti verificabili

La [specifica Feetech ST-3215-C018 A/0,20/07/2023,pp.3–4](https://www.feetechrc.com/Data/feetechrc/upload/file/20240507/6385067068652648096680943.pdf) dichiara encoder magnetico12 bit, backlash ingranaggi≤0,5°, velocità a vuoto0,222 s/60° e coppia nominale10 kg·cm distinta dallo stallo30 kg·cm. La formula360/4095 riportata nella tabella di risoluzione è una convenzione sugli estremi; MATDOG conserva4096 tick/giro, coerente con il resolver e i dati osservati. Risoluzione, velocità a vuoto e coppia di stallo non dimostrano inseguimento statico o dinamico del quadrupede.

Il profilo letto dai servo usa `CWDead=1`, `CCWDead=1` nei registri0x1A/0x1B. Questo valore di registro non dimostra una deadband meccanica di un tick. Attrito, gioco, stiction, inversione del carico e regolatore interno possono produrre residui maggiori. Nessun PID, EEPROM, TorqueLimit, offset o mapping è stato modificato.

Un [resoconto sperimentale originale Robonine](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/) riporta circa0,0151 rad di gioco su leva86 mm e circa±0,3 mm di ripetibilità su leva100 mm con precarico della sonda. Sono prove di un altro apparato, non specifiche garantite né misure MATDOG; il gioco non va sommato automaticamente al residuo encoder senza stabilire cosa misurano sensore e uscita. Non si assume che la ripetibilità con precarico descriva il robot sotto carico.

[ROS2 JointTrajectoryController](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) interpola nel tempo e separa tolleranza di percorso da tolleranza finale. L'esempio YAML usa0,05/0,03 rad: è una configurazione illustrativa, non una misura di robot. [CHAMP Hardware Integration](https://github.com/chvmp/champ/wiki/Hardware-Integration) richiede un'interfaccia verso12 giunti e distingue feedback reale da possibili pseudo-feedback negli adattatori open-loop. MATDOG continua a usare encoder reali. [DOGZILLA](https://github.com/YahboomTechnology/DOGZILLA) dichiara12 DoF, IMU e feedback angolare; [Petoi/OpenCat](https://docs.petoi.com/apis/serial-protocol/feedback-servos) documenta letture di feedback. In queste fonti consultate non è disponibile una precisione fisica comparabile che autorizzi numeri MATDOG.

Con upper90 mm e lower110 mm, una stima conservativa sagittale è `|δp| ≤200 mm·|δupper| +110 mm·|δlower|`: due errori di20 mrad possono comportare circa6,2 mm, prima del contributo HIP e degli altri errori reali. Non è un'accuratezza misurata del piede né dell'odometria. Si richiede funzionalità e stabilità, non un residuale numerico ideale.

## Geometry Compiler: significato dei parametri

| Parametro | Valore | Ruolo |
|---|---:|---|
| Clearance storica |3 mm | requisito geometrico delle superfici non adiacenti |
| Extra ventrale |3 mm oltre i3 mm | allowance cover originale per i domini ENTRY/box quota/WALK, non scansione discreta |
| Narrow-phase margin |1 mm | ricerca delle coppie vicine per distanza, non clearance ammessa |
| Griglia |5 mm | accelerazione della ricerca triangoli; non risoluzione fisica della prova |
| Sweep / parking |1° /0,5° | ricerca candidati, non certificazione continua né accuratezza servo |
| Bisezione contatto |0,0001 rad =0,1 mrad | raffinamento numerico di un bracket, non accuratezza dell'end-stop reale |
| Candidate-pair cap V5 |2.000.000 | limite risorse; il suo esaurimento resta errore/UNRESOLVED. Il checker adiacente separato usa il proprio cap100.000.000 |
| PASS / FAIL / UNRESOLVED | invariati | bound di separazione / witness o intersezione / prova non conclusa; UNRESOLVED non equivale a nessuna collisione |

Il Geometry Compiler V5 congelato resta byte per byte identico alla baseline: modificarne il sorgente romperebbe la provenienza gen1. Unica ottimizzazione consegnata: cache limitata all’ultima posa FCL di ciascun link nel certificatore offline `Region`. Stesso input di500 pose e1000 distanze:2,974 s→2,631 s, speedup1,13×, distanze identiche bit per bit. È una misura di quel tratto del certificatore, non del compiler completo. Il nuovo certificatore di percorso riusa `Region`/FCL e checker adiacente esistenti, racchiudendo ogni segmento completo in un box; non introduce un planner. Il vecchio bound Lipschitz sul tubo largo era UNRESOLVED già al segmento0 e non è stato convertito in PASS: una prova più precisa e conservativa sostituisce quel risultato inconcluso.

Cache dei risultati geometrici più ampie, broad-phase aggiuntiva, pruning e ricerca di percorsi con garanzia10–12 mm restano proposte separate. Nessuna mesh, limite meccanico o calibrazione è cambiata.

## Misure hardware minime

Una sola acquisizione sincronizzata nella prova fixture: angoli/goal/raw e velocità dei12 giunti, età per-giunto e timestamp TX, stato planner/executor, heartbeat e SAFE_OFF. Una misura esterna della posa/gioco contro il CAD deve qualificare il budget fisico, con attenzione agli accoppiamenti folded vicini al contatto. Un righello sul riferimento base_link verifica la quota150 mm (target di collaudo iniziale±10 mm, non qualificato); la fixture mantiene supporto indipendentemente dalla quota virtuale del planner. Tensione, corrente e temperatura sono raccolte dalla telemetria già disponibile.

Sequenza: STAND150 → mantenimento → STOP/SAFE_OFF; solo dopo esito stabile, una regolazione quota con ritorno a150 e WALK STANDARD30 mm per un ciclo, WALK STOP, poi STOP/SAFE_OFF. Non si prova FAST. Non servono tuning servo o un nuovo dump flash. Vedi piano dettagliato nel report patch.
