# Handoff — unica candidata dev.4, collaudo futuro supervisionato

Questa procedura è preparata, **non eseguita**. Installazione/collaudo solo con operatore fisicamente presente, robot trattenuto e STOP/KEY OFF raggiungibili. La candidata non è una release qualificata. Nessun test da100/150 di PRONE autorizzato dai replay. Il mantenimento della coppia al terminale non prova l'appoggio.

## 1. Una installazione APP-only, poi verifiche progressive

1. Verificare APP/SHA/manifest di BUILD.md, commit sorgente e patch. Conservare rc.11 r2 originale come rollback: APP `fc0a7141f827c37cdd5bccb888951ca280bd2a981b1572de8eef7c74474170be`. Usare la procedura APP-only già qualificata, adattando esplicitamente identità attesa/sorgente/destinazione alla dev.4, non avviare invariato un vecchio flash script che installa rc.11. Verificare readback dell'APP e identità delle regioni protette/NVS/calibrazione, senza scriverle. Nessun erase completo, modificaPID/TorqueLimit/EEPROM/limiti. Qui nessuna porta è stata aperta e nessun comando di flash fornito o eseguito.
2. Dopo il futuro boot controllare versione/build, gen1 VALID_ACKNOWLEDGED, SAFE_OFF13/13, status nativi0, power/BMS/IMU/bus e feedback validi, profili servo invariati. Non iniziare Motion se una precondizione manca. Aprire dashboard, login/sessione; a torque OFF attendere «Modello caricato». Verificare ACTUAL12 e quaternion, età che cresce con polling interrotto, distinzione budget generale/Motion. Non aumentare frequenza polling.
3. Collaudo HTTP/telemetria insieme ai passi seguenti: per ogni comando osservare id/sequence, ACK e status identificato. Registrare intervalli sent→headers→body, handler Server-Timing, accepted_ms→finished_ms del Controller, dispatch/loop, heap/stack minimi, missed releases/max gap. ACK è accettazione, non fine movimento. Obiettivo preliminare: ACK ordinario sano entro1s, nessun timeout5s o UNKNOWN irrisolto; non è una prestazione già qualificata. Se un ritardo compare, esportare immediatamente la traccia e non fare retry ciechi.

## 2. Primo PRONE soltanto da STAND50

Piano rigido orizzontale, area libera, anticaduta verificato a ogni fase, corsa di arresto minima, operatore pronto a sostenere il corpo. Video laterale con riferimento di scala e vista dei contatti. Il precedente STOP da150 ha causato caduta: STOP resta immediato, la ritenuta gestisce il rischio.

- Selezionare **50 mm** prima di STAND (il selettore può mostrare150 di default). Attendere HOLD stabile; almeno12 campioni per giunto, senza timer che eseguaSTOP. Confrontare residui raw rispetto al riferimento50 `[−1,3,−26,2,0,16,2,1,10,0,−1,−4]` in ordine slot0..11, stesso segno `commanded_raw - raw`. ±4tick è solo un gate esplorativo conservativo rispetto ai dati precedenti; non garantisce il movimento. Nessun creep/fault/feedback stale o carico anomalo. Fuori gate: sostenere il corpo e interrompere, non modificare limiti o target per forzare il passaggio.
- Inviare **un solo PRONE** da dashboard oppure terminale. Attendere PRONE_LIFT→PRONE_DEPLOY→PRONE_LAND→PRONE_CONTACT_UNVERIFIED, torque ON. Rilevare margini attuali gen1/box, errore goal/actual, carico/velocità/temperatura/tensione/status, IMU e contatori. La telemetria350ms è una osservazione campionata: può non catturare il minimo tra due letture. Non chiamarla monitor di sicurezza aggiuntivo.
- Registrare scivolamento e primo appoggio ventrale visibile. Il modello prevede20,3mm sotto il ventre posteriore e circa60% peso ancora sui posteriori; nessun sensore disponibile prova il trasferimento completo. Se margini collassano, posa è inattesa, compare interferenza/perdita feedback/fault: STOP immediato, anticaduta pronto. Nessuna discesa di recovery.
- Al terminale **sostenere prima il corpo**, poi STOP e SAFE_OFF13/13. Accompagnare l'assestamento, misurare gap/angoli e non forzare finecorsa. Non togliere coppia liberamente «per vedere se regge». Questo singolo esperimento decide se PRONE50 mantiene controllo e supporto; non qualifica automaticamente100/150 né SAFE_OFF automatico.

## 3. Logging senza STOP temporizzato

Metodo preferito con dashboard: usare il polling esistente e «Scarica diagnostica Motion di questa pagina». Conserva ultimi240snapshot (~84s a350ms) e48comandi: scaricare entro tale finestra dopo la prova, prima di chiudere/ricaricare la pagina. Non contiene CSRF/cookie. Convertire offline:

```sh
python3 scripts/validation/motion_observer.py --replay-json /percorso/matdog-motion-trace.json --out /nuova/directory-prone
```

Alternativa, **dashboard chiusa** e comandi/STOP dalla console già qualificata: avviare il medesimo script con `--url` esplicito del Controller e `--out` nuovo; GET `/api/motion/status` a350ms, nessun comando. `--duration` e Ctrl-C terminano soltanto il logger, **non Motion**. Non usare contemporaneamente due poller, né `load_stand50.py` con STOP pianificato. I CSV calcolano margini dai limiti gen1/box immutati, lasciano vuoti quelli non osservabili/freschi; samples.jsonl conserva i dati completi. In questa sessione solo replay offline.

## 4. WALK STANDARD e WALK STOP, separati dal primo PRONE

Solo dopo esito favorevole di50 e riposizionamento sostenuto nella posa di ingresso ammessa. Spazio/anticaduta adatti a150mm: WALK sale a150 se necessario. Primo passo STANDARD30mm, non60. Registrare partenza, fase, comando WALK STOP e arrivo STANDING/HOLD con torque ON. Il normale arresto programmato richiede circa7–9s a regime più assestamento; nei primi istanti può richiedere circa13,5s nel test software. Questo non autorizza ACK tardivo né un ritardo indefinito. Non ripetere WALK STOP a raffica e non convertirlo in SAFE_OFF.

Verificare un arresto a regime e, solo se il primo è stabile, un arresto in avvio. Completamento deve appartenere all'id/sequence inviato e lo status restare fresco. Per stabilità/supporto/continuità insufficienti o fault usare STOP immediato con ritenuta, senza aspettare il gait. La routeSTAND non è cambiata: nessun test della variante esclusa; acquisire piuttosto video dei contatti e dello slittamento sull'attuale route. Non dedurre coefficienti d'attrito dal solo carico servo.

Verificare STOP anche con polling/comando normale in ritardo, inizialmente in posa sostenuta: invio disponibile, vecchia risposta non sovrascrive loSTOP, SAFE_OFF13/13 deriva dai readback. La corsia600ms è un budget software, non promessa misurata. KEY OFF e console sono indipendenti dalla pagina; nessun fault elettrico/bus provocato nel primo collaudo.

## 5. Decisione e congelamento

NON congelare se rimangono timeout HTTP inspiegati/UNKNOWN, riconciliazione errata, snapshot presentati freschi quando scaduti, jitter/deadline fault nuovi, stack insufficiente, gen1 diverso, slittamento destabilizzante o contattoPRONE non controllabile. Un solo flash è un obiettivo, non un motivo per tenere firmware difettoso. Un eventuale ritorno alla rc.11 richiede la normale procedura supervisionata verificata, non è eseguito qui.

Dopo qualificazione hardware positiva: fissare commit esatto compilato, SHAAPP installata verificata per readback, manifest e log reali con condizioni/durate/esiti; annotare esplicitamente ambito qualificato (PRONE50 sperimentale non equivale a discesa autonoma completa). Preparare allora backup completo recuperabile con NVS/calibrazione e dati EEPROM necessari, identificato e con procedura di ripristino verificabile. Conservare credenziali/chiavi/backup sensibili in archivio privato protetto, mai suGitHub. Solo dopo questi gate creare tag/release, conservando APP e manifest e pubblicando esclusivamente sorgenti/binari/log sanitizzati appropriati. Nessun backup, tag, release, push o merge è stato eseguito ora.

Per futura integrazione: lavorare da snapshot installato `8e20c63...`, non dal soloHEADf838dirty. La candidata è già integrata nel worktree isolato; non sostituire SimpleMotion con versioni rc.10/dev.2. Le patchA/B sono incrementali su dev.3; D è riferimento storico già presente, da non riapplicare. Con una baseline futura diversa ricalcolare il delta, conservare integralmente termica e protezioni, rieseguire i test e produrre nuova identità. Nessuna integrazione nel worktree operativo qui.
