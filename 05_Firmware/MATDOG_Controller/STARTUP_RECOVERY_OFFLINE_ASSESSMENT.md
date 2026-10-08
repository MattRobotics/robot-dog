# MATDOG — startup recovery e rilascio offline, 3 ottobre 2026

La precedente diagnosi **OFFLINE_BLOCKED_STARTUP_REFERENCE** è ritirata: la fonte completa della cattura RF esiste ed è verificata. Il rilascio contiene un percorso startup esplicito, limitato alla configurazione RF LOWER MAX descritta qui. Il marker finale, commit, hash e risultati di compilazione sono nel `FINAL_OFFLINE_REPORT.md` del pacchetto distribuito.

MATDOG è rimasto spento. Nessun accesso seriale, flashing, RESET o movimento. Nessun merge/push è stato eseguito. Il checkout originale, il branch gait e le modifiche precedenti sono conservati.

## Riferimento completo e provenienza

Fonte: `/home/matteo-manicardi/MATDOG/evidence/full_cal_nvs_20261003_134848/`. Il verificatore `scripts/matdog_startup_reference.py` confronta direttamente log, JSON originale ed export dopo l'interruzione; rifiuta duplicati, acquisizioni/identità/geometrie incompatibili e sessioni ambigue.

| ID | Q0 | Unità | ID | Q0 | Unità |
|---|---|---|---|---|---|
|11|2102|M33|21|1997|NEW03|
|12|2107|ELR01|22|2106|ELR03|
|13|1975|M22|23|2024|NEW01|
|31|2036|NEW05|41|2079|M41|
|32|2058|ELR02|42|2076|M42|
|33|2086|NEW06|43|2026|M43|

Il log contiene una sola connessione e un solo boot POWERON con firma be0c12979e5b, ROBOT_POWERED / ESP32-S3. CAPTURE session=1 completa 9/9 passaggi, 12/12 candidati, nove campioni e spread=0 per giunto; PROMOTE ammette 12/12 CURRENT_BOOT_CAPTURE. Seguono LF session=1, sei contatti validati, e RF session=2, interrotta in LOWER MAX per OVER_TEMPERATURE (95,117,34). L'export della stessa esecuzione conferma i sei Q0 LF/RF, geometria `3713f4ddc43b204e`, sessioni 1/2 e 6/24 contatti complessivi. RH/LH non hanno record completi: i loro Q0 sono nel capture e nel JSON, non vengono inventati contatti mancanti.

SHA256 originali:

- log: `a9129abc1200a6bdd915334d84618c98e588bb76eada36fab983a9801085ea88`
- JSON: `cf14a2b6df6511db63c719e2325f8ae53021f416ff0b113a323acab30a787881`
- export: `7650cadba0022bfb6e5902a0ba1b234c92e3f03e7f3d6aef6895e554bae628b5`

Sono hash calcolati ora e corroborazione degli originali locali, non una firma digitale storica. Non viene usata la precedente cattura con RF UPPER=2107. Il log di controllo posizioni successivo è contesto diagnostico, non prova di continuità né autorità.

## Polarità e Geometry V5/CAD

RF LOWER / NEW03 / ID21 ha direzione encoder **+1** nel profilo compilato dell'installazione (`HISTORICAL_SLOT_UNCHANGED`). Il campo URDF motorDirection=-1 non è usato dal controller. Nessuna nuova prova di polarità è stata effettuata sul robot. Con questa convenzione le posizioni osservate producono q URDF +351, +1026 e +393 tick per 21,22,32.

La prova corretta già superata viene riutilizzata, verificandone la provenienza e l'assenza di modifiche al modello/kernel/corridoi. Il precedente caso osservato con LOWER negativo è superato: il percorso corretto ha tre segmenti e 316 pose PASS. La prova delle tolleranze comprende 396 esiti PASS, 708 box continui, 281 raffinamenti, clearance conservativa minima ≥3,1 mm rispetto alla soglia V5 esistente di 3 mm. Le 36 coppie adiacenti sono coperte dal dominio V5 e da sweep massimo 0,25°; non sono una dimostrazione continua equivalente a quella delle coppie non adiacenti. Esclusioni fisse del modello conservate; nessuna esclusione nuova per il giunto attivo. Limiti URDF verificati.

| Fase | Giunto in movimento, tick URDF | Supporti particolari |
|---|---|---|
|RF LOWER|−10…367|RF UPPER 1014…1034; RH UPPER 388…408|
|RF UPPER|−10…1034|RH UPPER 388…408|
|RH UPPER|−10…408|tutti gli altri −10…10|

Tutti i giunti non indicati restano **−10…10 assoluti**, rispetto ai riferimenti della cattura, non ±10 aggiunti a un ingresso incerto. 10 tick ≈0,879°. RF HIP deve restare vicino a Q0; RF UPPER sostiene la configurazione orizzontale, RH UPPER mantiene il parcheggio a circa 35°. Il massimo 367 copre 351+16, non amplia INITIAL RECOVERY ordinario, che mantiene 64 tick. L'executor controlla le bande sia prima del torque ON sia nelle osservazioni durante il percorso.

URDF SHA256 `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`; manifest mesh `60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1`. Il pacchetto conserva hash di sorgenti geometrici e 17 mesh collisione, più l'associazione al nuovo programma controller. Non è stato ripetuto l'audit CAD generale già passato.

## Riferimenti dopo reboot e autorità

La promozione ordinaria e il witness post-ABORT sono RAM-only. Il boot LOAD è read-only e non restaura trasformazioni/autorità; la sessione originale riportava NVS READY / NO_RECORD. Il firmware nuovo include una tabella immutabile separata dei 12 riferimenti, legata alla geometria e alle unità canoniche. Questa tabella non entra in JointTransformTable, non salva NVS e non abilita altri comandi.

`@CALIBRATION STARTUP QUALIFY RF_RETURN_20261003` esegue solo letture: census e preflight freschi, identità/popolazione attuale, tre passaggi sui dodici giunti, torque OFF, posizione stabile, corrente, temperatura, status e velocità validi. Rifiuta dati mancanti/scaduti, pose sconosciute e supporti fuori banda. La qualificazione scade dopo 3 secondi ed è consumabile una volta per boot. Posa nominale: nessun comando startup alimentato.

Solo `@CALIBRATION STARTUP RECOVERY CONFIRM_SUPPORTED_RF_RETURN`, dopo GO dell'operatore e qualifica fresca, può aprire sessione RF e permesso RAM con scope startup. Il permesso non può diventare quello della calibrazione ordinaria. SafeActuatorPolicy accetta solo prime alla posizione fresca e i tre target Q0 fissi; rifiuta movimento arbitrario, contatto, giunti/target diversi e assenza del grant dell'executor.

L'executor esistente ricostruisce i sostegni uno alla volta, con prime prima di ogni torque ON, RAM TorqueLimit=500, speed/accelerazione e readback del profilo esistente; recupera **RF LOWER → RF UPPER → RH UPPER**. Controlla posizione, torque/goal/limit, corrente, temperatura, timeout, lease e permesso. Primo guasto → cleanup SAFE_OFF; nessun retry. PASS solo con dodici giunti vicini ai riferimenti e SAFE_OFF verificato; il Controller verifica anche collo51, revoca permesso e autorità e chiude con SAFE_OFF13/13, NONE, reference_admitted=0. Un cleanup non certificato non viene dichiarato SAFE_OFF13/13.

Nessuna lettura o qualifica startup parte automaticamente al boot, nessun movimento automatico. Il riferimento storico permette la conversione geometrica; le posizioni fresche decidono l'ammissione. Il vecchio log non autorizza il movimento.

## Workflow operativo unico

1. `matdog_01_flash.sh`: application-only indipendente dalla posa, con tutti i gate binario/SHA/firma/ESP32/backup/partizioni/OTA/NVS. Dopo la futura scrittura: firma nuova, MAINTENANCE, NONE, SAFE_OFF13/13, encoder validi; nessun riposizionamento.
2. `matdog_02_full_calibration.sh`: firmware/SAFE_OFF/DALY e condizioni di sicurezza, readback reali, qualifica startup, eventuale recovery fisso, verifica 12 giunti entro10 dai riferimenti, **Fresh Q0 12/12 e nuova promozione**, INITIAL RECOVERY, LF→RF→RH→LH, export24/24, SAVE CHECK/SAVE/ACK della generazione. Un solo GO copre la procedura definita. Nessun secondo calibratore.
3. `matdog_03_verify_and_finalize.sh`: vero power-cycle dell'operatore, verifica read-only della generazione ACK e record intatto, nessuna autorità dopo reboot; documentazione e merge controllato soltanto dopo PASS hardware. Non è eseguito offline.

Dopo reboot viene acquisito un nuovo riferimento coerente prima dei contatti: i sei contatti LF storici non sono mescolati al nuovo Fresh Q0. Il resume della stessa accensione conserva invece i record delle zampe già validate solo se Q0, origine e geometria sono compatibili; i relativi test restano PASS.

## Temperatura e sicurezza durante il contatto

La correzione termica del firmware precedente resta invariata. UART: frame con ID/lunghezza/checksum/provenienza validi, timeout bounded, bulk e conferme dirette distinte; non è dimostrata offline una singola causa elettrica degli spike. 95,117,34 non basta più a confermare: 95,117,34,34,34 viene classificato transitorio. Solo sospetti sopra70 attivano fino a cinque campioni e almeno tre hot; normale → una misura. Soglia70 ed EEPROM conservate. Invalidi/comunicazione fallita → fail-closed; tre transitori/30s o otto/boot bloccano. Conferma incrementale a50ms, deadline300ms; avanzamento sospeso mantenendo controlli corrente, posizione, held-role e timeout. Durante startup read-only una misura hot/invalidata rifiuta direttamente l'ammissione. Tempi reali sotto carico e il mantenimento del contatto durante la conferma richiedono verifica hardware.

## Validazione offline e limiti fisici

I nuovi test riguardano solo il cambiamento: riferimento completo e mutazioni, qualificazione fresca, scadenza/consumo, scope policy/permit, percorso osservato e SAFE_OFF, pose incompatibili e supporti fuori banda, UART/timeout/corrente/temperatura/perdita permesso, workflow quattro zampe24/24 SAVE/ACK e reboot simulato, rifiuti senza retry. Audit statico delle sole integrazioni modificate e mutazioni delle protezioni. Risultati numerici e log nel rapporto finale del pacchetto. Le verifiche precedenti termiche/UART/NVS/CAD e audit generale sono mantenute per il loro ambito, senza ripeterle.

La prova geometrica è nominale e condizionata all'installazione originale, al riferimento manuale acquisito e al rispetto delle bande. I label fisici e la polarità meccanica non sono leggibili via UART: si verifica la mappa canonica e si richiede continuità d'installazione. Non prova deformazioni, giochi, carichi, gravità, stabilità sul pavimento o tolleranza totale d'assemblaggio. ±0,15mm per pezzo non è una tolleranza completa. **Corpo e giunti passivi devono essere sostenuti contro gravità**, senza imporre una nuova posizione o forzare riduttori. Se il sostegno non può essere predisposto senza muovere il robot, o l'installazione/riferimento è cambiata, la procedura non è ammessa e si conserva SAFE_OFF. La qualificazione offline non costituisce PASS hardware. Il GO attesta queste condizioni; nessun flag supera un rifiuto firmware.

## Aggiornamento e rollback

Solo application slot risolto dal flasher canonico; nessuna scrittura bootloader/table/OTA data/NVS, nessun erase globale. NVS dedicata e schema persistente conservati; nuova failure enum aggiunta in coda per preservare i valori esistenti. Il flasher verifica anche sdkconfig/anti-rollback e intervallo erase prima di una scrittura.

Rollback be0c12979e5b, SHA256 `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`, manifest e precedente candidato125d981b02a3 conservati. Backup storico A/B16MiB verificato localmente e procedura locale di recovery esistente; nessun nuovo backup richiesto e nessun restore fisico verificato in questo incarico. Il full dump legacy non è un'applicazione e non va usato per il rollback application-only: modificherebbe layout/NVS.

Le invocazioni senza argomenti o `--offline-check` dei tre entry point fanno solo file/Git checks. La sessione hardware futura usa `--hardware-session --session-dir /percorso/nuovo`, in sequenza1→2→3; un errore consuma il tentativo e richiede diagnosi, non una ripetizione automatica.
