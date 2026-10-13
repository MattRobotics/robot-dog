# Dossier per futura validazione fisica supervisionata

**NO-GO attuale. MOTION_AUTHORIZED=0, OTA_INGEST=0.** Build e test offline non costituiscono un'autorizzazione hardware. Nessuna connessione al dispositivo, flash, torque, NVS/EEPROM write o calibrazione è avvenuta. Il firmware installato resta `b08322dedc5564c080f691dc98047511ef2b953f`. La Full Calibration 24/24 del 1 ottobre e il record gen-1 sono conservati.

La nuova RESTORE è implementata e testata in RAM, ma non è stata eseguita sul robot. LOAD, RESTORE, accettazione, approvazione dei limiti e permit sono stati separati. La validazione offline gen-1 usa record sintetici e codice di produzione: non è una lettura del record installato. Identità fisica dei 12 motori, mounting, geometria, checksum, generation e provenance devono essere verificati contro l'installazione corrente. Mai reintrodurre vecchi q0/offset; i valori provengono esclusivamente dal record corrente accettato. ID del bus e unità attese sono in `joint_readiness.json`: **0/12 operational envelopes approvati in questa release**. I limiti URDF elencati non sono limiti operativi; gli endstop di calibrazione non danno motion permit.

Prima di qualunque prova attuativa devono chiudersi questi gate software: certificazione swept delle 12 coppie adiacenti, margini di configurazione e collisione approvati, route pitch-aware nello state machine runtime, adapter di contatto CAD qualificato, producer locali di osservazioni/supporto/assessment e prime/arm/permit, trasporto operativo autenticato in RUN_STA. `Controller::updateMotion` continua correttamente a negare hardware approval/support observability. Una variabile impostata a true non chiude questi gate.

## Osservazione iniziale futura, esclusivamente read-only

Operatore fisicamente presente, nessuna coppia comandata, fixture che sostenga il corpo e stop reale dell'alimentazione servo verificato. Raccogliere timestamp coerenti, encoder/torque state dei 12 ID, quaternion IMU e validità della calibrazione. Usare soltanto comandi read-only documentati per il firmware effettivamente installato; non usare una nuova API non ancora presente come se fosse già disponibile. `OBSERVATION_TEMPLATE.json` è un template vuoto, non una misura. Nessuno script qui apre una porta seriale o un dispositivo.

Convertire le letture con il mapping gen-1 verificato, confrontare FK e ingombri delle 17 STL con il robot. Misurare piano, posizione/assetto del base_link e contatto ventrale indipendentemente: l'IMU non misura la quota assoluta. Distinguere vicinanza dei piedi dal contatto e dal carico portante. Dopo perdita di torque serve una nuova osservazione completa. Il candidato sintetico qui ha HIP inward 1°, rear UPPER 90°, LOWER −92°, front UPPER circa 83,737°, pitch circa +5,546° e base Z 11,390 mm: non è una posa reale registrata, né una presa da imporre al robot.

## Micro-test futuri, dopo chiusura gate e GO separato

1. Verifica passiva di alimentazione/BMS/temperature, identità e feedback; nessun movimento. Interruzione fisica dell'alimentazione servo provata localmente in condizioni appropriate e documentate.
2. Review dell'immagine e manifest, backup NVS verificato e confronto del layout. Procedura APP-ONLY esistente soltanto dopo autorizzazione distinta; mai erase/migrazione dei dati di calibrazione.
3. Verifica di prime/arm/SAFE_OFF con fixture e limiti/rate approvati. Rifiuto di stale, missing feedback, permit revocato e inhibit. Le latenze del bus sequenziale e dello stop devono essere misurate.
4. Piccoli segmenti PRE-ALIGN da posa osservata compatibile, con assessment indipendente. HIP rimangono neutre soltanto dopo verifica del mapping e del percorso; non presumere encoder zero.
5. Dispiegamento posteriore e anteriore coordinato, misura di sliding/friction, pitch/roll, clearance e distribuzione dei carichi. Trasferimento ventrale→piedi subordinato a evidenza di sostegno, non al COM nominale.
6. Risalita incrementale, verifica tracking sotto carico e piedi portanti, poi conferma 150 mm. C4-C viene riusato soltanto da ingresso compatibile.
7. Prima falcata WALK lenta e STOP coordinato, con stance/swing realmente osservabili e margine dinamico qualificato. Ripartenza soltanto da stato riammesso. STOP hardware/fault può richiedere SAFE_OFF immediato invece di completare una falcata.

GO richiede responsabile locale identificato, fixture adeguata, piano/supporto misurati, mapping e limiti approvati, condizioni thermal/power nei range del Controller, feedback fresco, clearance e loaded tracking, stop fisico efficace, operator GO esplicito. NO-GO/abort su perdita di supporto, urto/sliding inatteso, pitch/roll fuori dall'envelope approvato, tracking/freshness/sequence/TTL errati, servo senza risposta, fault/BMS/power/thermal o revoca permit. I valori operativi di soglia devono essere qualificati e pinned, non inventati in questo documento.

La stabilità dinamica, le forze di contatto, la qualità del pavimento, l'efficacia dell'E-stop e la risposta sotto carico restano **HARDWARE/DYNAMIC VALIDATION OPEN**. Nessuna simulazione puramente geometrica li dimostra.
