# Handoff — prima prova esclusivamente STAND50

La candidata è un risultato offline, non una release operativa. Nessuna prova da 100/150 mm è proposta. Nessun flash è stato eseguito. La rc.11 installata resta APP fc0a7141…; la nuova dev.3 è eda0cf08… con sorgente c2f11a5.

## Ostacoli e decisione

Il nuovo floor risolve i due margini critici nel replay e il fault riprodotto dai residui50. Non si promuove questo a garanzia fisica: residui durante il moto e tolleranze/attrito non sono misurati. Il distacco nominale dei piedi anteriori è solo 0,17 mm; circa 60% del peso resta sui posteriori e il ventre posteriore è a 20,3 mm. Nessun criterio encoder/IMU/carico prova da solo che il corpo possa reggersi senza coppia.

**NO GO** per uso libero, SAFE_OFF finale automatico, recupero su fault e collaudo iniziale da100/150. Una singola prova esplorativa da50 è proposta soltanto con operatore, anticaduta meccanicamente verificato per arrestare il corpo in ogni fase e logging disponibile. Se manca questa protezione, se la posa/feedback non rientra nelle condizioni verificate o compare instabilità, la prova non parte. Lo STOP rimane immediato; il rischio di caduta viene gestito dalla ritenuta fisica, non ritardando STOP.

## Preparazione futura, nessuna esecuzione qui

1. Revisione del diff completo contro snapshot 8e20c63 e SHA della candidata. Per una diversa rc.11 qualificata futura, ricostruire prima il delta; preservare la termica e rieseguire le regressioni. Non sostituire interamente SimpleMotion con quello dev.2. L'integrazione è già presente solo nel nuovo worktree, non nella baseline/APP operativa.
2. L'eventuale installazione futura richiede la normale procedura autorizzata APP-only, verifica hash/readback, gen1 e regioni protette; questa consegna non esegue né automatizza tale procedura. Nessun cambio NVS/PID/TorqueLimit. Confermare prima della prova versione, build, gen1 VALID_ACKNOWLEDGED, SAFE_OFF13/13 iniziale, bus/IMU/power validi e native status0.
3. Piano rigido orizzontale, zona libera, anticaduta inizialmente lasco ma con corsa minima verificata a50, KEY OFF e STOP accessibili; operatore pronto a sostenere il corpo. Video laterale con riferimento millimetrico, inclusi appoggi anteriori e posteriori. Registrare con la cadenza già qualificata, senza polling extra sul bus.
4. Logger: acquisire i dati senza un vecchio STOP temporizzato a fine HOLD. **Non riutilizzare invariato load_stand50.py**: il suo STOP pianificato può interrompere PRONE dopo pochi secondi. Servono acquisizione passiva/console esistenti e STOP operativo disponibile; nessuna automazione di discesa su allarme. Non ignorare fault reali; una singola lettura termica isolata si interpreta secondo la conferma r2.

## Un solo esperimento

1. Comandare STAND50 con la procedura normale; aspettare HOLD stabile. Conservare almeno12 campioni di ciascun giunto, raw/cmd/err/load, velocità, torque/goal register, età feedback, tensione/status/temp, IMU, BMS e contatori temporali.
2. Prima di PRONE confrontare i residui con il caso50 registrato: in ordine slot0..11, errore raw `[−1,3,−26,2,0,16,2,1,10,0,−1,−4]`. Il confronto ±4 tick è una precondizione conservativa proposta per limitare la prima esplorazione, non una soglia runtime nuova o una garanzia durante la discesa. Carichi stabili e diretti al goal, dentro i gate esistenti, nessun creep, feedback fresco, posa stabile. Fuori da queste condizioni: sostenere il corpo e interrompere, non tentare altri target per aggirare il limite.
3. Un solo `@MOTION PRONE` oppure un solo click PRONE, mai entrambi. Attendere la sequenza PRONE_LIFT→PRONE_DEPLOY→PRONE_LAND→PRONE_CONTACT_UNVERIFIED con torque ON. Le durate del planner non includono i dwell; non imporre uno STOP pianificato basato sulle sole durate nominali.
4. Registrare continuità, eventuali scivolamenti, errori/carichi e minimi verso box/gen1, missed release/max gap, istante del primo appoggio ventrale e quota del ventre posteriore. Verificare che la dashboard mostri età/stale e terminale non verificato; una sessione/pagina persa non deve avviare recovery. Non provocare fault bus/elettrici in questa prima prova.
5. Al terminale non togliere coppia per vedere «se regge». Sostenere fisicamente il corpo, quindi STOP e verifica SAFE_OFF13/13. Accompagnare lentamente l'eventuale assestamento finale, senza spingere giunti oltre finecorsa; misurare la corsa posteriore e gli angoli. Questa parte assistita distingue ostacolo geometrico da possibile riposo passivo ma NON qualifica un rilascio libero.

Interrompere immediatamente in caso di instabilità/interferenza, movimento inatteso, perdita di feedback/power/autorità o fault; anticaduta pronto. Non annullare STOP per privilegiare la discesa. Nessun retry automatico né risalita da posa non qualificata.

## Cosa deve decidere la prova

Confermare o respingere: la riduzione della flessione conserva carichi e controllo, i margini reali non collassano, l'appoggio anteriore previsto esiste e i posteriori mantengono supporto fino alla presa manuale. Registrare quanto assestamento posteriore è realmente possibile dopo supporto del corpo. Solo questa evidenza permette una decisione successiva; non autorizza immediatamente la discesa da100/150, il trasferimento completo al ventre o SAFE_OFF automatico.
