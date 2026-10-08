# MATDOG — revisione offline recovery post-ABORT e telemetria termica

Data: 3 ottobre 2026. Stato: implementato e verificato offline; candidato alla
revisione finale. Nessuna validazione hardware aggiunta. Controller spento:
nessuna seriale aperta, connessione ESP32, comando servo, flashing o reset.

## Provenienza e consegna

Firmware indicato dall'operatore: `be0c12979e5b`, ROBOT_POWERED, NVS dedicata già
configurata. Base software locale: `7b258b36c257` sul branch di persistenza.
Lavoro isolato in `/home/matteo-manicardi/MATDOG/github/robot-dog-post-abort`,
branch `fix/calibration-post-abort-thermal`; nessuna modifica al checkout
originale e nessun merge dal branch gait `feat/g5a-stabilization-feasibility`.
È stata preservata anche la correzione locale già presente del crosscheck
LF UPPER MIN: midpoint assoluto 1461 ±16 tick, indipendente da Q0.

Pacchetto locale:
`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_POST_ABORT_THERMAL_20261003`.
Contiene applicazione `.ino.bin`, ELF, manifest nativo, report, runbook, log
di verifica, evidenza CAD, rollback applicativo precedente e SHA256SUMS.
`release-manifest.json` riporta il commit esatto, stato CLEAN, profilo,
dimensione/hash e confini della scrittura applicativa. `modified-files.txt`
e `commits.txt` identificano tutti i file e il commit di consegna. Non distribuire
un'immagine merged come applicazione.

## A — causa e recovery distinto

INITIAL RECOVERY ammetteva soltanto prime vicino a Q0, entro 64 tick, sia
nell'executor sia in SafeActuatorPolicy. Quel limite protegge l'ordinario
riallineamento, ma non rappresenta le dipendenze della calibrazione interrotta.
RF LOWER 2348/Q0 1997 è a +351 tick; RF UPPER 1080/Q0 2106 è a −1026;
RH UPPER 1665/Q0 2058 è a −393. Nell'interruzione LOWER MAX, RF UPPER era il
supporto orizzontale e RH UPPER il parcheggio posteriore. Portare prima UPPER o
rear park a Q0 eliminerebbe un prerequisito del percorso LOWER.

La nuova porta esplicita è `POST_ABORT RECOVERY <LEG> CONFIRM_Q0_RECOVERY`.
Usa gli stessi executor, Geometry V5, policy, backend, sessione, lease e permit.
Non aumenta il gate ordinario dei 64 tick e non ammette obiettivi arbitrari.

L'executor conserva un witness RAM della fase e degli ultimi dodici readback
usabili, con età ≤3 s al momento dell'interruzione. Lo concede soltanto per
UPPER MIN/MAX o LOWER MIN/MAX e pose compatibili con i prerequisiti. Serve
poi SAFE_OFF verificato su tutti i dodici giunti. In una nuova sessione verifica
identità/unità/bus, Q0 e geometria ancora coerenti, rilegge i dodici giunti
torque OFF, impone drift massimo 16 tick dal witness e tolleranza 10 tick
dalle pose dei supporti. Il giunto cercato resta nei guard Geometry V5 esistenti.

Per LOWER: re-prime dei supporti alla posizione presente, uno alla volta,
eventuale rear park, HIP e UPPER orizzontale; riattivazione LOWER; ritorno
LOWER → UPPER → rear park. Per UPPER: rear park/HIP/LOWER mantenuti, ritorno
UPPER → rear park. Poi SAFE_OFF, INITIAL RECOVERY ordinario sequenziale di
tutti i dodici giunti e verifica finale Q0/torque OFF. Il witness è consumato:
un'interruzione del recovery non riutilizza automaticamente la prova originale.

La policy ammette soltanto il giunto e il target emessi dall'executor in quel
passo; il resolver deve confermare il tick. Rimangono profilo CALIBRATION_SEARCH,
RAM TorqueLimit 500, velocità/accelerazione esistenti, corrente hard ≥200 raw,
limite termico 70 °C, stato servo, readback, hold/bystander, timeout di movimento
12 s più il budget di distanza a 80 tick/s e gate 4 campioni/400 ms, errore
≤10 tick, velocità ≤4. Un SAFE_OFF non verificato viene ritentato e non consente
di dichiarare il run completo.

Le diagnostiche di rifiuto distinguono assenza di witness, fase non dimostrata,
Q0/geometria cambiati e posa incompatibile. In preflight queste condizioni
non producono goal, torque ON o torque-limit write. HIP e transizioni rimangono
non qualificati per il recovery esteso. Non sono introdotti movimenti all'avvio.

## B — analisi acquisizione prima della correzione

`ServoBus::readControlFeedback` esegue due richieste ST3215: blocco 40..49
(TorqueEnable, GoalPosition, TorqueLimit), poi 56..70 (posizione, velocità,
carico, tensione, temperatura, status, corrente). La conferma legge direttamente
PresentTemperature tramite una richiesta di un byte. Non è una temperatura
stimata dal carico o un campo EEPROM; i due blocchi non sono una lettura atomica.

È stata letta la libreria SCServo installata, senza modificarla. Il suo `Read`
svuota RX, trova FF FF, legge il numero richiesto di byte e verifica checksum,
ma non confronta ID della risposta e lunghezza dichiarata con la richiesta,
né rifiuta lo status di errore. `readSCS` rinnova il timeout a ogni byte:
un flusso incompleto può consumare più di un singolo timeout. L'ACK delle
scritture ha già controlli ID/lunghezza/checksum; resta seguito da readback.

Questi sono difetti dimostrati del percorso software. Traffico, risposte
ritardate/cross-servo o desincronizzazione sono cause plausibili dei picchi,
non una diagnosi provata dell'evento fisico. Senza trace UART non si può
attribuire retroattivamente 95/117/34 a uno specifico errore del bus né escludere
un guasto del sensore. La vecchia regola due hot su tre classificava correttamente
secondo il proprio criterio quei campioni, pubblicando 117, ma il criterio era
insufficiente contro due letture anomale consecutive.

`ValidatedServoRead`, al confine già proprietario del bus, mantiene tutte le
scritture della libreria e valida header, ID richiesto, larghezza esatta,
status zero e checksum prima di pubblicare qualsiasi dato. RX viene drenato
con limite 64 byte; flood, rumore, packet troncato o incoerente falliscono.
Il timeout copre l'intera risposta: 100 ms operativo, 20 ms diagnostico.
Non si pubblicano buffer parziali né si sostituisce una lettura di sicurezza
fallita con quella precedente. I timestamp della sequenza usano il clock
effettivo di acquisizione/update, non quello precedente al giro UART.

Il protocollo non restituisce l'indirizzo del registro o un transaction ID.
Una risposta ritardata dello stesso ID e della stessa lunghezza, arrivata dopo
il drain e con checksum valido, resta indistinguibile a livello protocollo.
Checksum a 8 bit, integrità elettrica e comportamento reale dell'UART sono rischi
residui. Ping e gli ACK informativi delle scritture restano nella libreria;
le letture di sicurezza/readback passano nel validatore.

## B — conferma adattiva e tempi durante il contatto

Il limite resta **70 °C**, senza scritture EEPROM o modifica della configurazione
persistente del servo. Un bulk >70 avvia uno stato per servo: fino a cinque
campioni totali, con massimo quattro dirette aggiuntive distanti almeno 50 ms.
Tre hot confermano subito; tre cool con ultimo diretto cool risolvono il
transiente. Mancante, invalido, sorgente bus diversa o deadline scaduta
falliscono chiusi. I campioni bulk successivi non sostituiscono quelli diretti
richiesti dal voto; ogni diretto è una nuova transazione validata.

- 95,117,34,34,34: TRANSIENT, pubblicato 34, quattro dirette, circa 200 ms.
- 150,34,34,34: TRANSIENT dopo circa 150 ms.
- 71,71,71: CONFIRMED dopo circa 100 ms, senza attendere cinque campioni.
- Campione normale: nessuna conferma aggiuntiva.
- Tre transienti risolti in 30 s, oppure otto per servo nella stessa accensione:
  REPEATED_ANOMALY latched e fail-closed. Letture normali non azzerano i conteggi.

La conferma è cooperativa, senza delay nella produzione e con massimo una
diretta aggiuntiva per tick. PENDING blocca nuovi goal/torque e l'avanzamento
della ricerca. Continuano acquisizione, corrente/status, readback attivo,
posizioni, hold e bystander; un'anomalia fatale porta al SAFE_OFF nello stesso
tick che la rileva. La deadline PENDING è 300 ms; una singola lettura diretta
può ancora occupare fino a 100 ms e il tempo reale dipende dal giro di controllo.
Non è una garanzia hard realtime di 300 ms dal fenomeno fisico.

Durante l'attesa il servo può continuare a premere verso il goal precedente,
già limitato dalla ricerca. Lo stop termico software richiede nominalmente
50 ms in più del precedente voto 2/3 in caso di calore continuo; non vengono
emessi ulteriori passi mentre attende. Le protezioni di corrente e del servo
restano attive, ma la pressione di contatto non è qualificata dai soli test host.
Tempistiche e bus sotto carico richiedono misura nella futura prova controllata.

## C — coerenza, ripresa e persistenza

Il runner esistente aggiunge post-abort, resume, persist e verify-persistence;
non è stato creato un secondo calibratore. Resume confronta build/acquisizione,
uptime, tutti i Q0 e geometria attuali, poi chiusura completa e sei contact
witness delle zampe trattenute. Duplicati o geometria/Q0 incompatibili rifiutano
la ripresa. Nel replay LF resta valido, RF viene recuperato e rifatto, poi
RH e LH completano 24/24. Il firmware conserva i Q0 promossi e i record completi
delle altre zampe; il recovery non finalizza un record di calibrazione.

Q0 CAPTURE rifiuta evidenze RAM esistenti/sessione live. Il comando esplicito
EVIDENCE DISCARD CONFIRM_NEW_Q0 scarta soltanto evidenze e witness RAM quando
inattivi, preservando Q0 promosso e NVS; nessun RESET fisico necessario.
La generazione nuova non può incorporare record delle acquisizioni precedenti,
anche se i tick misurati fossero identici.

SAVE CHECK/SAVE/ACK usano il servizio e schema record/NVS preesistenti. Il runner
verifica prima 24/24, SAFE_OFF 13/13 e authority NONE, poi generazione ACK
esatta e marker/record consistenti. Dopo un power cycle separato la verifica
è di sola lettura, richiede firma uguale e uptime diminuito, e conferma
record acknowledged integro e MOTION_AUTHORIZED=0. Non ripristina capacità
di movimento e non invia reboot. Il reboot è simulato nei test, non eseguito.

## Verifiche riproducibili e risultati

Comandi eseguiti soltanto offline, dalla worktree isolata:

```bash
bash 05_Firmware/MATDOG_Controller/scripts/tests/run_host_tests.sh
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_calibration_hw_session.py
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_static_audit_safe_actuator.py
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_calibration_search_behaviour_mutations.py --thermal-only
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_calibration_search_behaviour_mutations.py --anchors-only
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
python3 06_Software/Matdog_Core/calibration/matdog_post_abort_geometry_v5.py \
  --corners --output /tmp/matdog-recovery-geometry-final.json
MATDOG_PROFILE=ROBOT_POWERED MATDOG_OTA_INGEST_VALIDATION=0 \
  bash 05_Firmware/MATDOG_Controller/scripts/build.sh --build-path /tmp/matdog-post-abort-build
```

| Verifica | Risultato finale offline |
|---|---|
| Suite host C++ | 45 eseguibili, 456.791 controlli, zero fallimenti |
| FullLegCalibrationExecutor | 61.807 controlli: quattro zampe, 16 combinazioni UPPER/LOWER, replay RF/RH, UART/current/heat/timeout/permit, SAFE_OFF |
| Termica | 134 controlli: sequenza richiesta, heat reale, invalidi/incompleti, deadline, anomalie concentrate e sparse |
| UART reale con transport sintetico | 135 controlli: ID/length/status/checksum, noise, truncation, flood/stale RX, timeout assoluto |
| Router reale + NVS simulata | 15.055 controlli; discard/generazioni, SAVE/ACK, framing e gate |
| Stato persistenza | 209.730 controlli; store/service/backend e reboot simulati nelle suite pertinenti |
| Runner Python | 34 test: quattro zampe, retain LF/resume RF, Q0/geometria incompatibili, SAVE/ACK e reboot simulato |
| Mutation gate termico | 9/9 regressioni rilevate come fallimenti di test; nessuna compilation failure conteggiata come successo |
| Audit/mutazioni statiche e framing | PASS; 110 anchor della suite comportamentale controllati |
| Geometry V5 CAD | PASS: 179 percorsi, 26.476 pose, step ≤0,5°, caso osservato e corner ±10 tick dei supporti |
| ESP32-S3 | Compilazione CLEAN ROBOT_POWERED, ingest 0; dati esatti nel manifest del rilascio |

Il corpus completo dei 110 mutanti comportamentali non è stato eseguito:
sono stati eseguiti i nove termici, il gate di mutazione statica e i negativi
framing; gli altri anchor sono stati verificati. Non confondere questa copertura
con un'esecuzione completa del corpus.

La geometria riusa URDF/mesh/provenienza V5 e pose/corridoi esistenti. Esclude
la coppia di stop intenzionale del giunto cercato come il validatore di sequenza
esistente. Il controllo è campionato: non è una prova continua, non enumera
tutte le combinazioni di tolleranza delle dodici zampe né deforma i componenti.
I ritorni usano il piano V5 esistente e i suoi gate; giochi, cavi, attriti,
supporto esterno e deformazioni reali non sono qualificati dal CAD.

## Rollback, aggiornamento e rischi residui

Vedere [runbook di rilascio](../../05_Firmware/MATDOG_Controller/POST_ABORT_RECOVERY_RUNBOOK.md)
per istruzioni applicative future, senza bootloader, partition table o NVS.
Tabella e corridoi V5 non sono cambiati; schema NVS, default USB_ONLY, ingest 0,
protezioni termiche/corrente e assenza di autorità al boot sono conservati.

Hash locale dell'applicazione precedente be0c12979e5b ROBOT_POWERED verificato:
`7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`.
Le copie full-flash A/B esistenti da 16 MiB sono identiche, SHA256
`856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7`.
Preflight locale del pacchetto M0.4 ripetuto: PASS, HARDWARE_IO=NO.
Manifest e ricevuta locali già registrano decifratura e recupero byte-equal;
nessun accesso remoto, nuova decifratura o nuovo backup hardware in questa fase.

**Limite da riconoscere prima del futuro flashing:** sono verificati immagine
precedente, hash e procedura sui file, ma non un nuovo ripristino fisico né
il recupero da perdita USB. App1 non offre fallback automatico. Il runbook M0.4
documenta accesso manuale BOOT/EN; il dump storico precede la migrazione e un
full restore ripristinerebbe layout/NVS storici. Preferire rollback app-only
con l'applicazione precedente e i relativi gate, che conserva la NVS corrente.

Dopo lo spegnimento reale in corso, il witness precedente è perso. Questa
release risolve le **future** interruzioni compatibili della stessa accensione;
non autorizza automaticamente il rientro retroattivo dal log del 3 ottobre.
Restano da provare su hardware tempi UART/termici durante il movimento,
sicurezza del contatto, recovery fisico, SAVE/ACK e persistenza dopo power cycle.
Il rilascio è pronto per revisione finale, non è installato sul robot.
