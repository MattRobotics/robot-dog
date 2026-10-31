# MATDOG — chiusura real-time del candidato Motion

Questa correzione prosegue8343b179 nello stesso worktree. Il suo binario/SHA restano archiviati; il nuovo commit, APP, manifest e dimensioni sono nel receipt esterno `verification-artifacts/TOLERANCE_V1_20261018/realtime/CANDIDATE_FIRMWARE.md`. Nessun accesso al dispositivo, flash o movimento. NVS gen1, mapping, geometria, limiti, PID, EEPROM e TorqueLimit invariati. Una sola nuova build ROBOT_POWERED dopo la patch.

## Barriere dimostrate e correzione circoscritta

8343b179 richiedeva a ogni microstep: tempo minimo, tutti i12 campioni posteriori a TX, errore sul target; anche `AdaptiveStand::checkTracking` richiedeva lo stamp collettivo post-frame. Quindi20 ms nominali non erano50 Hz effettivi. La sonda prima della patch misurava90 ms/frame e76,608 s fino a STAND con un read per ciclo da6 ms; col caso storico25 ms/read arrivava a325 ms/frame e276,300 s. Sono modelli, non nuovi dati hardware. Il primo tentativo di sonda aveva anche un errore nel test di ammissione WALK; le misure STAND sono conservate separatamente e quel test non è dichiarato PASS.

La patch usa lo stesso executor finito, lo stesso `PeriodicScheduler` già presente nel coordinatore, il clock monotono64 bit di WALK e le letture rapide esistenti. Nessuna nuova macchina a stati, planner o autorità.

- Microstep: rilascio dopo almeno20 ms reali sul clock monotono, monitoraggio prima del nuovo target, feedback asincrono age≤100 ms, errore sul target≤20 mrad. Nessun ACK collettivo post-TX. Un superamento diventa `TRACKING_LAG_LIMIT`, anziché introdurre pause ripetute per recuperare un frame.
- Il guard del box segmento rimane; limiti raw, contenuto assessment, lease, potenza, supporto, deadman e STOP rimangono attivi. Una pausa non prevista dopo un microstep completato oltre100 ms produce `FRAME_CADENCE_GAP`.
- Checkpoint: fine dei due stadi ENTRY, delle cinque fasi route e della quota mantengono campioni reali post-TX e arrivo16,108798 mrad. STAND richiede anche quiete misurata. Non si confonde completamento temporale intermedio con posa fisicamente assestata.
- `Controller` può presentare il certificato statico e inviare il frame nello stesso giro; non attende un'altra iterazione per il solo assessment.
- Lo stesso istante monotono governa executor e planner. Il test con jitter aveva dimostrato che il precedente arrotondamento millis poteva consumare un rilascio prima che fossero realmente trascorsi20 ms, producendo gap ricorrenti verso40 ms. La correzione evita questa doppia quantizzazione temporale.
- Sul solo modo fixture, WALK usa i contatti della posa STAND nominale verificata come riferimento, controllando separatamente che i giunti reali siano entro20 mrad. La precedente ricostruzione dagli encoder introduceva `FOOTPRINT_MISMATCH` da1 mm con residuo5 tick. Il feedback reale non viene sostituito; fuori fixture la verifica del footprint misurato resta invariata.

File runtime: `MotionExecutor.{h,cpp}`, `MotionCoordinator.cpp`, `AdaptiveStand.{h,cpp}`, `FeedbackWalk.cpp`, `Controller.cpp`, `MotionService.cpp`. Sono aggiunti alla telemetria `motion_timing.frames_tx`, `last_tx_complete_us`, `last_interval_us`, `max_interval_us`. Gli intervalli globali comprendono checkpoint e inattività tra comandi: non vanno interpretati tutti come jitter dei microstep. I contatori sono osservazione, non nuovi gate.

## Bus effettivo e limiti della stima

Il profilo operativo ammette polling dopo2 ms dal completamento dell'ultimo gruppo, non il limite storico20 ms. Prima della patch STAND faceva un ID per passaggio, con due richieste (control40..49 e feedback56..70). Ora ENTRY/STAND temporizzati riusano il ramo rapido WALK: fino a3 ID per passaggio, continuando soltanto se il budget iniziale di2 ms lo consente. Non è garantito che siano sempre3. Un control read aggiuntivo ruota ogni20 ms in STAND; WALK conserva100 ms. Il control state in STAND non viene riusato oltre400 ms; il feedback encoder durante il moto non oltre100 ms.

A1 Mbaud8N1 il solo filo richiede circa0,29 ms per richiesta/risposta fast,0,53 ms per le due richieste combinate,1,04 ms per SYNC_WRITE104 byte. Sono lower bound seriali: non includono risposta servo, CPU, rete, IMU e timeout. Il minimo ideale di un giro rapido è nell'ordine di12 ms includendo i gap nominali; il giro combinato nell'ordine di30 ms. La granularità millis e il lavoro del Controller impediscono di interpretarli come WCET. Nei modelli completi a4/6 ms per ciclo,3 read per passaggio danno16/24 ms per giunto; con un read a25 ms servono300 ms.

Il timeout richiesto ai read è8 ms; il blocco combinato contiene due richieste. Il budget del ciclo può terminare dopo una lettura lenta; in caso di lettura fallita il motion è invalidato. Non esiste qui una misura hardware del massimo giro né del WCET completo. Il limite100 ms arresta al successivo controllo, non dimostra una latenza indipendente dal blocco del task. Watchdog e taglio fisico restano necessari e invariati. I307–361 ms osservati il17/10 non sono accettabili per la nuova esecuzione continua.

## Misure mirate, codice e conversione raw di produzione

Sonda con servo quantizzato, residuo5 tick, τ30 ms, drive STAND300/20, latenza UART modellata1,1 ms, read420 µs. Start sintetico PRONE con offset UPPER2°, contenuto nel dominio provato. I periodi Controller sotto sono ipotesi di carico, non misure ESP32. Il caso4 ms contiene spazio per gap polling e trasferimenti; il precedente caso2 ms era ottimistico e non viene usato per qualificare il candidato.

| Modello | Microstep p50 / p99 / massimo | ENTRY | Fino a STAND | Massima age ai controlli | Esito |
|---|---:|---:|---:|---:|---|
| Ciclo4 ms,3 read |20 /20 /20 ms |2,084 s |17,260 s |12,840 ms | PASS offline |
| Ciclo6 ms,3 read |24 /24 /24 ms |2,490 s |20,664 s |18,840 ms | PASS offline |
| Ciclo4 ms + jitter0..1,2 ms,3 read |22,982 /24,538 /24,772 ms |2,385 s |19,805 s |16,310 ms | PASS offline |
| Ciclo6 ms,1 read | arresto a1,716 s | — | non completa |66 ms | rifiuto atteso TRACKING_LAG_LIMIT |
| Ciclo25 ms,1 read | arresto a0,125 s | — | non completa |125 ms | rifiuto atteso STALE_FEEDBACK |

I massimi comprensivi dei checkpoint sono40 /54 /54,008 ms nei tre scenari rapidi. Non sono pause a ogni microstep. ENTRY/STAND partono dopo ARM: PRIME/ENERGIZE/ARM, rete reale, posa iniziale diversa e hardware non sono inclusi. Non è un massimo garantito sul robot.

WALK STANDARD30 mm dopo STAND con residuo5 tick: ammissione PASS, un ciclo completo, intervallo20/20/20 ms. Non attende il feedback dei12 a ogni frame; usa lo streaming preesistente con monitor per-giunto. WALK STOP coordinato torna a STAND in21,016 s nel caso provato: è la decelerazione/chiusura sui quattro piedi prevista dal gait, non la latenza del comando di emergenza STOP/SAFE_OFF. Quest'ultimo conserva revoca e burst torque-OFF prioritari, con regressioni dedicate già presenti.

Esecuzioni: `scripts/tests/run_motion_realtime_tests.py`, `run_tolerance_v1_tests.py`, owner MotionServiceExecutor, HTTP production handler e audit read; log nel receipt esterno. La matrice estesa e tutte le prove CAD non vengono rigenerate: i target nominali e il limite4 mrad restano quelli provati, con una temporizzazione almeno altrettanto conservativa. Il controllo di binding dei quattro certificati è nuovamente PASS. I tentativi intermedi falliti restano archiviati e non sono qualificazione.

## Futura IMU

L'executor supporta aggiornamenti continui senza assestamento per-frame e WALK ha già timestamp, scheduler e monitor asincroni. Nessun ostacolo richiede un controller parallelo. **Non autorizza però ancora correzioni arbitrarie roll/pitch**: il planner oggi usa corpo/contatti nominali e box certificati. La modifica futura minima è calcolare la posa corretta e la IK a contatti mantenuti prima degli stessi controlli di limiti, rate e collisione. Occorre certificare il dominio corretto: la riserva40 mrad include già20 tracking +4 passo +quantizzazione +15 allowance fisico, quindi non è un budget libero per offset IMU. Nessun algoritmo di stabilizzazione viene aggiunto ora.

## Controllo fisico minimo prima della coppia e verdetto

15 mrad generale e8 mrad LOWER folded restano **PROPOSED FOR HARDWARE QUALIFICATION**, mai VERIFIED HARDWARE. Le prove CAD sono condizionate al modello; gen1/CRC/encoder non misurano l'angolo assoluto o le deformazioni reali.

1. Rail servo fisicamente disabilitato, fixture che sostenga il corpo indipendentemente dalle zampe,13 torque OFF secondo la verifica esistente. Controllare cablaggi, cover, serraggi, fixture e spazio di tutto l'inviluppo previsto, soprattutto UPPER/FOOT e accoppiamenti LOWER ripiegati.
2. Confrontare riferimenti fisici accessibili con CAD e angoli raw della posa iniziale; misurare bias/gioco nelle due direzioni. Serve un metodo con incertezza complessiva dichiarata capace di verificare8 mrad locali e15 mrad generali. Per ordine di grandezza,8 mrad su110 mm sono0,88 mm e15 mrad su90 mm sono1,35 mm. Non si assume che un righello, telefono o display di calibro possieda l'accuratezza richiesta. La quota150 mm da sola non verifica questi bound.
3. Misurare le clearance reali minime delle coppie accessibili e confrontarle con i requisiti3 mm /6 mm del dominio; nessuno slack di un lower bound CAD è una distanza fisica fra superfici. Una verifica statica nella posa iniziale non dimostra l'intero percorso, né il gioco/flessione sotto carico. Non forzare articolazioni contro i finecorsa per esplorarlo.

Con le sole evidenze disponibili non è dimostrata la verifica fisica8/15 mrad né la clearance reale lungo il moto: **NO-GO movimento/ENTRY finché questa condizione essenziale non è qualificata**. Il controllo statico è necessario, ma non sostituisce la verifica sotto il carico previsto. Nessun gate amministrativo è aggiunto nel firmware e nessuna misura mancante viene dichiarata superata.

Il candidato compilato e verificato offline può essere autorizzato per APP-only e controlli supervisionati a coppia OFF. Per il flash usare il medesimo USB-JTAG software ROM già disponibile: un ingresso `esptool --before usb-reset --after no-reset --no-stub ... read-mac`, poi il flasher APP-only in `no-reset`; nessun BOOT fisico richiesto e nessun nuovo dump A/B. Nessun comando è eseguito in questa fase. Procedere a STAND150/hold10 s, STOP/SAFE_OFF e infine WALK STANDARD solo se i controlli fisici sono positivi e il movimento è autorizzato.

## Integrità e dimensioni

ELF/map8343b179 contro789e07c: APP+3248 byte; crescita prevalente in pagina portale(+581), diagnostica Controller(+372), executor/update(+297), funzioni di assessment e nuovi bound. La firma nuova sostituisce quella vecchia; nessun grande duplicato evidente. Non è motivato un refactoring dimensionale. Le dimensioni della nuova build sono nel receipt.

Il diff8343b179 contro789e07c è36 file,+1457/−96 righe, **zero file eliminati**; contro6d60e1f è35 file,+843/−96. Il workspace iniziale, su un altro ramo, ha modifiche preesistenti e non è stato toccato. Il dato UI208 file/−24600 non è riproducibile con questi confronti; non viene attribuito senza prova a file generati. I build esterni e le copie ignorate non sono cancellazioni Git. Nessun documento, sorgente o certificato storico è stato eliminato.
