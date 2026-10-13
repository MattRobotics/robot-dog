# B — AUTONOMOUS PRONE STAND 150 mm + WALK

**NO-GO autonomo.** A su fixture non prova capacità di sostegno sui piedi. Nessuna prova fisica viene lanciata da questi artifact.

## Gate aggiuntivi

| Gate | Evidenza attuale | Cosa manca |
|---|---|---|
| Prone route congelata/adjacent/nonadjacent/3mm | PASS offline, digest invariato | Installazione effettiva, cablaggio, tolleranze/deflessione caricata |
| Entry/intermediate recovery | Knot matching fail-closed | Tubo di ingresso misurato e certificato a risoluzione encoder; body pose esterna |
| Limiti iniziali LOWER | Difetto provato rispetto alla proposta | Pose osservata e raccordo inward con margini; non approvare −92° sul bordo |
| LF UPPER e tutti i 12 margini caricati | Tabella review | Zero/mounting correnti, contatti e margin review, envelope fisico |
| Supporto ventrale iniziale | Richiesta esplicita del planner | Osservazione reale del corpo/piano/sostegno, condizioni di attrito |
| REAR/FRONT DEPLOY | Profili speculari e mapping individuale | Spazio reale, tracking/stop/supporto durante deployment |
| LOAD TRANSFER | Contatto geometrico dei 4 piedi | Testimoni indipendenti di reazioni/carico, perdita ventrale controllata, COM e attrito |
| RISE/C4 150 mm | Target continui verificati | Quota assoluta base_link esterna, loaded torque/holding/thermal, anti-tip/fall management |
| STAND_VERIFIED | State machine + ACK/actual/support | Conferma reale 4 piedi portanti, quota, stationary, nessuna dipendenza dal target come misura |
| GAIT_START/STOP/velocity changes | G4 conservato; tests e adapter runtime | Derivate STL stabili in tutto il dominio, path continuo realmente eseguito |
| Prima falcata swept | Solo sampled baseline | Certificato continuo di runtime adapter/clock, ground/support/contact phase; 3mm dove pertinente |
| Stance/swing/dinamica | Poligono nominale con supporti assunti | Reazioni, slittamento, COM/inerzia, sicurezza swing, stabilità caricata |
| vx/vy/yaw_rate e transizioni | API e arresto→rebase→rampa testati | Qualificazione di tutto il box ammesso e traiettorie combinate; nessuna estrapolazione della demo forward |
| STOP/fault/rearm | Fail-closed software e coda cancellata | Latenze caricate, caduta gestita/fixture di cattura, nuova authority e permit per rearm |

Sono richieste misure esterne di quota/posa e criteri di sostegno **prima** del prime/torque autonomo. Il firmware non contiene sensori di contatto/load inesistenti. Il producer locale riceve osservazioni esterne con source e timestamp, mentre sostituisce i joint del payload con encoder reali freschi. Missing evidence significa rifiuto, non fallback IMU/q0.

Scope AUTONOMOUS_STAND richiede supporto/carico osservabile e rates/holding qualificati; il scope AUTONOMOUS_WALK aggiunge dinamica qualificata. Non si deve far dipendere un primo STAND dal completamento di tutta la futura teleoperazione: si può approvare uno scope STAND circoscritto, quando i suoi gate sono davvero soddisfatti. Nella candidata nessuno scope fisico è approvato.

## Procedure future staged, dopo separata autorizzazione per ogni prova

1. Concludere A e relativa review: STOP/SAFE_OFF/prime, mapping e feedback devono essere stati misurati, non semplicemente simulati. Definire fixture di cattura che non falsifichi le misure di appoggio/carico; specificare criteri di abort e gestione caduta a torque OFF.
2. Raccogliere posa prona reale, joint encoder, pitch/roll/quota esterna e stato del sostegno ventrale. Stabilire i margini LOWER/UPPER caricati e certificare ingresso/corridoio verso la route. Pose ignota o fuori tubo → NO-GO; non comandare il candidato −92° automaticamente.
3. Approvare 12 limiti, envelope STAND, bus timing e support witnesses. Installare solo immagine SHA approvata con autorizzazione separata; RESTORE esplicita con torque OFF. Nuova osservazione iniziale valida prima di arm.
4. Permit locale AUTONOMOUS_STAND con lifetime/deadline specifiche, prime e arm osservati. Avanzare PRE_ALIGN, poi rear deploy e front deploy secondo la route; HIP neutrali dopo PRE_ALIGN. Misurare actual vs target e clearance, nessun salto di fase o riposizionamento implicito.
5. LOAD_TRANSFER solo se i criteri indipendenti dimostrano piedi portanti e trasferimento controllato. Non usare mask stance, piede vicino a Z=0, corrente servo o quaternion come certificazione del carico. Mancanza di testimoni → STOP/abort secondo piano di cattura.
6. RISE attraverso le fasi contact-locked/flat/C4. Misurare la quota 150 mm rispetto al piano di supporto, actual encoder e margini; verificare holding/thermal e supporto di ogni piede. STAND_VERIFIED richiede osservazione indipendente/stationary e tracking dell'ultimo frame. Registrare reazioni, quota e tempi nella scheda; nessun WALK implicito.
7. Dopo review STAND, risolvere i quattro switch del contact-reference STL e qualificare derivate/swept della funzione runtime reale. Includere primo swing, entry sway, ritorno all-foot STOP e primo target dopo rebase. Campioni e video restano evidenza illustrativa; non sostituiscono swept.
8. Autorizzazione separata AUTONOMOUS_WALK per **una sola prima falcata** con parametri e box velocità espliciti. Nuova osservazione all-foot/STAND, nuovo permit, GAIT_START continuo, nessun asse lateral/yaw se non incluso nel certificato. Supervisore al kill, testimoni stance/swing/carico freschi.
9. STOP a fine falcata, holding sicuro oppure SAFE_OFF con cattura secondo il piano. Fault/stale/inhibit/permit loss provocano arresto immediato del writer e SAFE_OFF, non un tentativo di completare la falcata. STOP volontario cancella qualsiasi nuova velocità accodata. Registrare latenza, tracking dei 12 e supporto durante arresto.
10. Rearm soltanto dopo stato reale recuperabile, SAFE_OFF e nuove verifiche/permit/authority; nessun ritorno automatico da fault. Variazioni vx/vy/yaw richiedono la review del dominio combinato prima della prova; la continuità software da sola non dimostra stabilità.

Per ogni stage usare scheda A estesa con: body X/Y/Z/pitch/roll misurati e timestamp, sostegno ventrale, reazioni dei quattro piedi e metodo, COM/attrito/anti-tip, stance/swing, prima falcata swept digest, limite caricato e risultati STOP/holding/cattura. Tutti i threshold fisici ancora non misurati restano vuoti. Ogni requisito critico vuoto impone NO-GO dello stage.
