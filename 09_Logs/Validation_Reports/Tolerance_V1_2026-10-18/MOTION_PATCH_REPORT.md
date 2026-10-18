# MATDOG — remediation Motion V1

## Esito e perimetro

Candidato offline su `~/MATDOG/worktrees/robot-dog-tolerance-v1-20261018`, ramo `fix/tolerance-operational-v1-20261018`, derivato dalla build installata789e07c attraverso il HEAD documentale6d60e1f. Un solo profilo ROBOT_POWERED, STAND60–150 mm e WALK STANDARD. Nessuna scrittura sul dispositivo, flash, NVS, EEPROM, PID, TorqueLimit o mapping. Nessun merge/push/reset/clean; nessuna modifica ai worktree precedenti.

Identità finale di sorgente/binario ed esiti dei comandi sono in `~/MATDOG/verification-artifacts/TOLERANCE_V1_20261018/CANDIDATE_FIRMWARE.md` e nel receipt del candidato. I valori ingegneristici e i loro limiti sono in `TOLERANCE_CONTRACT_V1.md`. **L'offline non dimostra stabilità o sicurezza fisica sotto carico né abilita moto a terra.**

## Cause corrette

1. **Criterio di arrivo usato universalmente per ogni frame.** Il servo fermo5 tick dal goal non soddisfaceva6 mrad. L'executor finito esistente ora separa tracking20 mrad da arrivo10 tick più conversione (16,108798 mrad). I frame intermedi rispettano un tempo minimo, richiedono nuovi campioni posteriori all'ultimo byte TX e errore sul target entro20 mrad; gli endpoint di stadio/fase/quote richiedono arrivo. Nessun secondo planner o nuova macchina a stati.
2. **Assessment statico scaduto mentre il frame era già in esecuzione.** L'età2 s resta verificata prima dell'ammissione. `noteSubmitted` lega l'assessment alla sequenza accettata. Solo evidenza `OFFLINE_CERTIFIED` con route tube revisionato evita la seconda scadenza di età durante quel frame: contenuto, sequenza, clock non futuro, supporto, geometria, lease/epoch/limiti, freschezza feedback e timeout executor restano controllati. Un assessment esterno dinamico continua a scadere. L'executor4 s può ora riportare la vera mancata convergenza.
3. **Dominio del certificatore troppo conservativo sul folded-side.** Il tubo40 mrad estendeva il LOWER fino a circa−95°, oltre ciò che i limiti raw gen1 consentono al feedback. Qui sono comparsi quattro veri witness UPPER/FOOT sotto3 mm. Un primo tentativo sul lato aperto non risolveva quei witness ed è stato scartato. La prova finale usa l'intersezione con i bound LOWER effettivamente ammessi, più la riserva fisica dichiarata (15 mrad non adiacenti,8 mrad angolari locali per adiacenti). `configureCommissioning` controlla che i limiti caricati siano contenuti nei bound usati; nessun raw limit, calibrazione, offset o percorso è cambiato. Solo la misura esterna può qualificare gli allowance fisici.
4. **Mancanza di un guard di divergenza nel ramo TRACKING finito.** Ora il box `[precedente,target]±tracking` è controllato anche mentre si attende il termine del frame. Un superamento ferma al campione, senza attendere4 s. READY continua a verificare l'hold. Non si spende il tubo geometrico40 mrad come tolleranza servo.
5. **Motivo fisico sostituito dalla conseguente perdita permit.** `MotionService::refresh` conserva il motivo executor prima di valutare `OPERATOR_GO_REQUIRED`; `MotionSession` propaga il motivo executor invece di sostituirlo con `EXECUTOR_LOST`. STOP rimane prioritario. Evidenza locale aggregata è distinta per generazione, età, presenza, potenza, installazione, profilo e perdita di scope. Clock futuro, lettura non valida, torque perso e campione vecchio hanno codici diversi. Il confronto post-TX finito usa la stessa epoca millis e resta corretto al rollover32 bit, con test dedicato.
6. **Ripetizione del planner all'ARM.** Il percorso già verificato prima della coppia viene riusato soltanto se lo start held coincide entro2 µrad di arrotondamento. Uno start diverso viene ripianificato dagli stessi controlli. Questo elimina il lavoro duplicato che consumava il margine feedback; non trasforma un campione vecchio in fresco e non modifica400 ms. L'associazione della spike hardware al ricalcolo è coerente con sorgente/log, ma il miglioramento WCET sul chip va misurato.
7. **Rinnovo amministratore concorrente possibile nel codice.** Il portale e la dashboard potevano invocare `resume()` insieme, producendo più richieste e rotazione token. Ora condividono la stessa promise; test DOM con due chiamate concorrenti→una richiesta. Un401/AUTH_FAILED esplicito può rinnovare e rispedire lo stesso id/sequence; timeout o rete incerta non ritrasmettono un POST. Il log USB precedente non dimostra che questa race abbia causato gli errori osservati.
8. **STAND riattivabile tra ACK HTTP e successivo poll.** Un ACK ACCEPTED mantiene entrambi i pulsanti STAND disabilitati fino a stato terminale del Controller; il server conserva già `COMMAND_IN_PROGRESS` e idempotenza. STOP ha la propria corsia. Non si attribuiscono i quattro STAND storici a ritrasmissioni: mancano log HTTP dei clic.

## File e funzioni

| File | Intervento |
|---|---|
| `motion/OperationalTolerance.h` | SSOT di arrivo/tracking/tubo/passo/quantizzazione e allowance fisica candidata; static_assert del budget |
| `execution/MotionExecutor.{h,cpp}` | envelope opzionale timed/arrival; `arm`, `submit`, `update`, `monitor`; nessuno stato aggiunto |
| `motion/AdaptiveStand.{h,cpp}` | `pendingFinal`, `noteSubmitted`, lifetime assessment; stesso generatore; quota12 s/50 mm |
| `motion/MotionFeedback.{h,cpp}` | semantica `checkAssessment`, motivo dominio quota; precisione FK/IK invariata |
| `execution/MotionCoordinator.{h,cpp}` | endpoint vs frame intermedio, notifica submit, riuso pre-arm, guard del budget certificato |
| `execution/CommissioningPolicy.{h,cpp}`, `core/Commissioning.cpp` | binding del solo profilo operativo, tetto tracking e budget; velocità0,19/passo4 mrad |
| `motion/StandHeightRegion.h` | box target conservativo della quota, controllato prima dell'uso; prova di tutto il volume indipendente, non soltanto campioni IK |
| `motion/OperationalWalkProfile.h` | profilo STAND rallentato; drive RAM300/20 invariato; WALK STANDARD1000/50 invariato |
| `execution/MotionSession.{h,cpp}`, `core/Controller.cpp`, `core/MotionService.cpp` | motivi precisi e conservazione causa; protezioni invariate |
| `network/PortalPage.h`, `network/MotionDashboard.h` | rinnovo single-flight, prevenzione invii STAND multipli, STOP indipendente |
| `paired_prone/certify_measured_entry_region.py` | cache limitata all’ultima posa FCL per link; compiler congelato invariato |
| `paired_prone/certify_operational_tube.py` | riuso Region/FCL per i251 segmenti, bound su tutto il box continuo; checker adiacente esistente |
| `paired_prone/restrict_measured_entry.py` | restringe il dominio, riusa soltanto PASS su superset e ricalcola ogni FAIL/UNRESOLVED; parent conservato |
| `paired_prone/certify_operational_walk_region.py` | stesso algoritmo, possibilità di provare il box quota |

Test aggiunti: `test_tolerance_v1.cpp`, fixture del frame misurato, runner mirato e verifica di contratto/certificati; checker read-only gen1. Aggiornato il modello servo operativo per quantizzare con i resolver di produzione, distinguere passo del riferimento da errore del servo e simulare stop-short5 tick. Nessuna soglia numerica FK/IK è stata rilassata.

## Avanzamento e arresto

La sequenza rimane: comando STAND autenticato→PRIME a coppia OFF→ENERGIZE→ARM→ENTRY_DISENGAGE→ENTRY_ALIGN→cinque fasi della stessa route→STAND. Una quota diversa da150 mm usa poi lo stesso componente IK a contatti fissi, nello stesso comando; il comando non completa alla quota intermedia150 mm. Nessuna conferma testuale supplementare.

Il modo finito usa ACK del feedback dopo TX come condizione di osservazione, non una prova di precisione perfetta. Il tempo minimo del frame è20 ms; campioni lenti possono dilatare l'esecuzione. Non è stata introdotta una promessa di50 Hz reali per STAND. Il WALK continua a usare lo scheduler/streaming esistenti, box certificato indipendente, feedback100 ms e riferimenti realmente inviati negli ultimi150 ms.

Un servo che resta fermo mentre il riferimento si allontana non può avanzare illimitatamente: perde il budget20 mrad o il criterio di avanzamento e interviene `TRACKING_DIVERGENCE`/`TRACKING_TIMEOUT`. Un residuo finale12 tick è ammesso come following ma non come arrivo e termina con `ARRIVAL_TIMEOUT` a4 s. HOLD WALK conserva3 s di settle timeout e velocità≤un passo registro al secondo. Feedback stale, permit/heartbeat perso, fault, limite o STOP interrompono senza un nuovo target successivo al rilevamento.

## Replay #61 e prove proporzionate

Il replay usa i12 raw/goal/target/actual effettivi del primo campione con completed_sequence60, NVS gen1 e trasformazioni raw reali. La telemetria2 Hz non contiene il precedente target#60: l'executor viene inizializzato al target già scritto#61 per riprodurre la decisione di completamento in TRACKING, non per inventare la transizione mancante. `frame61-replay-input.json` conserva riga originale e SHA del log; la fixture C++ conserva dati e hash. La route completa è provata separatamente con il modello quantizzato e stop-short5 tick.

| Prova | Prima | Candidato |
|---|---|---|
| Replay misurato#61, max errore circa7,84 mrad | completed0, rimane TRACKING; vecchio planner poi ASSESSMENT_STALE | completed61, `TIMED_TRACKED` |
| Residuo5/8/10 tick quantizzato |5 tick già oltre6 mrad | frame intermedio e arrivo ammessi entro i budget candidati |
| Residuo12 tick | oltre6 mrad | avanzamento intermedio; arrivo rifiutato e `ARRIVAL_TIMEOUT` a4 s |
| Residuo20 tick | attesa/timeout nel ramo finito | `TRACKING_DIVERGENCE`, nessun avanzamento |
| Campioni entro age ma anteriori al termine UART | criterio basato sull'inizio scrittura | nessun ACK/avanzamento fino a campioni post-TX |
| Assessment statico a2,01 s in-flight | ASSESSMENT_STALE | contenuto ancora verificato; scadenza di arrivo demandata all'executor4 s |
| Errore collisione o assessment dinamico vecchio | stop | stop preservato |
| STAND150 con banda5 tick | stallo riproducibile al criterio vecchio | STAND_VERIFIED nella simulazione, ~8,25 mrad finali; non è una misura fisica |
| Quote140/100/60 e ritorno150 | test idealizzati | produzione IK + conversione raw quantizzata, stessi limiti/tracking, WALK ammesso solo a150 |
| STOP / perdita heartbeat / stale / torque perso / dato invalido | protezioni esistenti | preservate, motivi distinti; nessuna scrittura successiva al rilevamento |
| Restore record reale gen1 | — | fingerprint `be8d5902bb4e37da`,12 trasformazioni/limiti, **zero scritture storage** |

I log conclusivi, gli eventuali tentativi intermedi falliti e il vecchio certificatore UNRESOLVED sono conservati. Solo gli esiti conclusivi sono associati al candidato; nessun conteggio di milioni di PASS è usato come argomento ingegneristico. Il vecchio test backend contava tentativi di polling come frame completati; con release temporizzata il test ora attende25 sequenze realmente completate, senza ridurne il numero. Il gate host seleziona ora `run_tolerance_v1_tests.py` al posto della matrice operativa estesa (rimasta invocabile separatamente): copre le regressioni di questo diff senza ripetere la campagna FAST esclusa dalla build. Gli altri runner host e i controlli statici restano attivi. Un test precedente attendeva STALE_FEEDBACK per un campione futuro: corretto in FEEDBACK_CLOCK_INVALID, coerente con la causa reale.

## Geometria e prestazioni

La polilinea ENTRY e il riconoscitore±10° dal PRONE osservato restano invariati. Il checker ha trovato quattro veri FAIL UPPER/FOOT nel dominio40 mrad non limitato, con LOWER intorno a−95°. Questi witness sono fuori dai limiti effettivamente consentiti: il sottoinsieme finale è l'intersezione con i bound misurati verificati dal firmware, ampliati degli allowance fisici espliciti. Le prove PASS sul dominio parent più grande restano valide per inclusione; le quattro coppie fallite e ogni dominio adiacente ampliato vengono ricalcolati. Il parent e il tentativo scartato restano conservati, senza riscriverne l'esito. La prima prova25 mrad ha richiesto refinement soltanto di `base_link/lf_lower_leg_link` e `base_link/rf_lower_leg_link`; entrambe chiuse con bound≥6 mm. La prova finale40 mrad riserva separatamente15 mrad candidati per discrepanza fisica/CAD. Gli adiacenti ENTRY LOWER includono8 mrad oltre il bound misurato ammesso, con prova continua ricalcolata anche oltre il bordo URDF ove necessario. Il valore8 mrad resta un requisito di qualificazione esterna, non una precisione fisica dimostrata.

La route dopo ENTRY ha251 box continui×120 coppie non adiacenti, più12 domini adiacenti completi. La discesa quota ha un box indipendente protetto a runtime, con120+12 coppie; il WALK STANDARD ha il box originale ampliato dal tubo,120+12 coppie. La richiesta di10–12 mm non è garantita dall'intera regione PRONE ammessa; i witness campionati sotto10 mm impediscono di prometterla senza restringere gli start o ricercare un percorso diverso. La soglia6 mm verso il corpo nell'ENTRY resta invariata.

Cache locale del certificatore:2,974 s→2,631 s su500 pose e1000 distanze, risultati bit-identici, speedup1,13× della sola porzione misurata. La prima cache FK nel compiler congelato, pur equivalente, è stata rimossa perché invalidava la provenienza gen1; il controllo canonico del bundle è nuovamente PASS. I certificati già prodotti usavano quella cache FK equivalente: il sorgente esatto resta archiviato come `scene_cache_superseded.py`, con confronto bit-identico conservato. Nessuna trasformazione o algoritmo di separazione è cambiato nel ripristino; la cache FCL consegnata è anch’essa confrontata sul medesimo input con la versione senza cache. Nessuna modifica a griglia, sweep, parking, bisezione, pair cap o classificazione. UNRESOLVED resta tale; il nuovo checker risolve con una prova differente più stretta, non rilassando un requisito.

## Limiti aperti e piano hardware minimo

Il candidato non dimostra ground support, accuratezza assoluta dei giunti, montaggio/cover/cavi/fixture, comportamento termico sotto carico o WCET con rete reale. Il budget fisico15 mrad è proposto: per i LOWER vicini al folded-stop serve dimostrare il bound locale8 mrad e la clearance reale, non applicare ciecamente lo stesso allowance15 mrad. Il record gen1 è coerente, ma un CRC/fingerprint non misura un montaggio.

Prima di eseguire moto: fixture che sostenga il robot, taglio fisico servo a portata di mano, verifica esterna delle clearance/angoli iniziali rispetto al CAD e degli accoppiamenti folded. Dopo flash autorizzato, boot senza movimento: firma binario, gen1/fingerprint e torque OFF12+collo. Nessun nuovo dump.

1. **Un STAND150 mm**, un solo clic: acquisire raw/goal/velocità dei12 giunti con tempi, età feedback, transizioni e causa eventuale stop; mantenere STAND almeno10 s. Misurare la quota esterna del riferimento base_link, errore massimo e ripetibilità/gioco contro CAD. Questo qualifica i budget, non un successo solo della UI.
2. **STOP/SAFE_OFF**: registrare tempo richiesta→ultima TX e12 readback TorqueEnable=0 (più gestione collo preesistente). Una perdita heartbeat supervisionata verifica il deadman3 s senza fare affidamento sul solo browser. Qualsiasi mancato SAFE_OFF interrompe il collaudo.
3. Solo dopo1–2 stabili: **una regolazione quota**, inizialmente140 mm e ritorno150; le quote inferiori sono offerte solo se limiti live e box certificato le contengono. Per qualificare tutto il workspace ripetere almeno agli estremi60 e150, con la stessa acquisizione.
4. **WALK STANDARD30 mm, un ciclo**, poi WALK STOP→STAND e STOP→SAFE_OFF. Misurare following rispetto ai riferimenti inviati, feedback age, intervalli frame e caduta tensione; temperatura dalla telemetria. Il cammino a terra e carico richiede una successiva prova supervisionata dopo la fixture. FAST non si esegue.

Non sono richiesti nuovi PID, EEPROM o TorqueLimit. Se il budget fisico locale o il tracking misurato non rientra, il candidato non viene qualificato per quel moto; si conserva la coppia/giunto e il margine che hanno fallito e si ripianifica, senza promuovere UNRESOLVED o allargare una soglia in campo.
