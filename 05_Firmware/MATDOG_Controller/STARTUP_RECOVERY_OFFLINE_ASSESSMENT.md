# MATDOG — esito della revisione startup recovery, 3 ottobre 2026

**OFFLINE_BLOCKED_STARTUP_REFERENCE.** Il flashing application-only è stato
separato dalla posa; il rientro alimentato dopo spegnimento non è autorizzabile
con le evidenze disponibili. Non è dichiarato OFFLINE_READY.

MATDOG è rimasto spento. Nessuna apertura seriale, lettura del dispositivo,
scrittura flash, EEPROM o NVS, comando dei servomotori, RESET, merge o push.
Checkout originale, branch gait, modifiche precedenti e rollback conservati.

## Correzione del flash

`matdog_01_flash.sh` delega al helper aggiornato: nessun controllo di
`qualified_path`, distanza da Q0 o attestazione di posa nella fase di flash.
Restano i controlli canonici di binario, SHA256, firma, profilo, identità ESP32,
backup, partizioni, slot OTA, intervallo di cancellazione e NVS.

Dopo la futura scrittura si verificano firma, MAINTENANCE, autorità NONE,
SAFE_OFF 13/13 e dodici encoder validi con torque OFF. Le posizioni possono
essere quelle residue osservate. Nessun riposizionamento; FLASH_OK registra
`nominal_pose_verified=false`. Il gate della posa resta nella fase 02 prima
di qualsiasi movimento o acquisizione Q0. Un errore consuma il tentativo.

## Polarità e geometria

La riga RF LOWER / NEW03 / ID 21 del profilo compilato ha direzione **+1**,
fonte `HISTORICAL_SLOT_UNCHANGED`. Il campo URDF `motorDirection=-1` non è
la convenzione encoder usata dal controller. È verificata la convenzione
software dell'installazione; non è stata eseguita una nuova prova di polarità
sul robot. Con i riferimenti della richiesta: RF LOWER +351 tick, RF UPPER
+1026 tick URDF, RH UPPER +393 tick URDF.

La precedente prova CAD del solo caso osservato usava RF LOWER **−351**:
quella porzione di evidenza è superata dalla verifica corretta, **3 segmenti,
316 pose, PASS**, senza esclusione della coppia di arresto del giunto attivo.
Le altre prove precedenti non sono state ripetute.

Sequenza candidata: tenere RF HIP vicino a zero, RF UPPER orizzontale e RH
UPPER parcheggiato a 35°; riportare RF LOWER a zero, poi RF UPPER, poi RH
UPPER. Corpo e zampe richiedono sostegno contro la gravità. Non si comanda
la popolazione intera simultaneamente.

Il nuovo controllo usa lo stesso URDF, STL, modello e kernel Geometry V5,
senza modificare corridoi o partition table. Le bande in tick URDF sono:

| Fase | Giunto in movimento | Supporti particolari |
| --- | --- | --- |
| RF LOWER | −10…367 | RF UPPER 1014…1034, RH UPPER 388…408 |
| RF UPPER | −10…1034 | RH UPPER 388…408 |
| RH UPPER | −10…408 | tutti gli altri −10…10 |

Tutti gli altri giunti rimangono nella banda assoluta −10…10. Il massimo
367 copre +351 più 16 tick; non è un nuovo limite generico di recovery.
Le bande sono **assolute rispetto al riferimento geometrico**, non ±10
aggiunti a una posizione d'ingresso già incerta. Un eventuale executor deve
controllarle durante tutto il percorso: il solo drift rispetto al readback
d'ingresso non soddisfa questa prova.

Risultato: **396 esiti PASS**, 708 box continui, 281 raffinamenti. Le coppie
non adiacenti sono coperte da limiti conservativi di movimento dei vertici
sull'intero box; clearance certificata minima **≥3,1 mm**, soglia esterna
esistente 3 mm. Per 36 esiti delle coppie adiacenti si usano il dominio V5 e
sweep a passo massimo 0,25°. Una lower bound inferiore alla soglia non è
trattata come collisione esatta; risorse esaurite o prova irrisolta bloccano.

Questa è evidenza geometrica nominale: non prova carichi/gravitazione,
supporti fisici, deformazioni o tolleranze di assemblaggio. Il dato esistente
±0,15 mm per pezzo stampato non è una tolleranza totale di assemblaggio.
Non è un'autorità di movimento e non qualifica da solo lo startup sul robot.

## Q0 dopo il riavvio: blocco concreto

`Controller::begin()` esegue LOAD read-only. `CalibrationPersistenceService`
azzera immediatamente il record decodificato; non ammette trasformazioni o
autorità. I Q0 promossi di una calibrazione parziale sono RAM-only. Non esiste
un salvataggio separato dei soli Q0; SAVE/ACK richiede l'evidenza completa.
Il witness post-ABORT è anch'esso perso al riavvio.

L'ultima evidenza locale della persistenza prima della calibrazione riporta
NVS READY / NO_RECORD, slot e marker assenti. Non è una nuova lettura NVS.
Non è disponibile una ricevuta SAVE/ACK della calibrazione RF interrotta.
Anche un record completo acknowledged, se presente, rimarrebbe evidenza
read-only e richiederebbe un'ammissione esplicita separata.

Il file locale `full-calibration-stop-20261003/q0-current.json` contiene dodici
misure, ma appartiene al precedente tentativo fermato su LF UPPER MIN:
RF UPPER **2107**. La successiva richiesta RF LOWER MAX riferisce **2106**,
e fornisce soltanto i Q0 21, 22, 32 e 23. La differenza di un tick, da sola,
non prova un pericolo: prova però che non si tratta della medesima cattura.
Non sono documentati qui tutti i dodici valori promossi di quella successiva
cattura, né la compatibilità geometrica delle altre otto righe.

Non si completa la tabella con righe della cattura LF, CR2-C, encoder 2048
o con le posizioni residue. La stabilità dei campioni di un vecchio log non
certifica da sola la corrispondenza del suo zero con le bande geometriche
attuali. Con readback freschi si conosce la posizione raw, ma manca un
riferimento qualificato completo per convertirla e dimostrare l'appartenenza
del robot al box ammesso. Un GO non ricostruisce questo dato.

**Diagnostica: STARTUP_Q0_REFERENCE_UNAVAILABLE_12_OF_12.** Serve una fonte
completa e coerente dell'acquisizione promossa RF, con identità/provenienza e
qualifica rispetto al datum CAD, oppure il ristabilimento fisico qualificato
del riferimento. Non è richiesto un nuovo backup né una calibrazione 24/24
come prerequisito teorico: un riferimento geometrico separato può essere
ammesso se completo e qualificato. La fonte mancante non è ricostruibile
offline da questi tre/quattro valori. Non viene prescritta una manipolazione
del robot per aggirare il blocco.

## Stato del workflow e test

La fase 02 restituisce il rifiuto preciso prima dell'I/O e di torque ON.
Non è stato introdotto un executor startup alimentato privo di riferimento,
né un movimento automatico al boot. Quindi startup recovery e il workflow
richiesto che lo include **rimangono bloccati**, non sono dichiarati completati.
Il runner nativo rimane unico e conserva fresh Q0/promozione/INITIAL RECOVERY,
LF→RF→RH→LH, export 24/24, SAVE CHECK/SAVE/ACK quando la posa è ammessa.
La fase 03 conserva la verifica read-only dopo power-cycle e documentazione/
merge controllato soltanto dopo PASS hardware e di persistenza.

Verifiche di questo intervento: **21 test PASS** degli script, inclusa la
simulazione completa nominale fino a SAVE/ACK e reboot, pose RF/RH residue
ammesse al solo flash, gate prima del movimento, UART/timeout/firma/encoder/
torque/MAINTENANCE/autorità/SAFE_OFF invalidi e nessun retry. **7 test PASS**
del nuovo analizzatore, tra cui 400 confronti con la distanza triangolo esatta,
collisioni, clearance non sicura, lower bound insufficienti e limite risorse.
Sintassi/whitespace e integrità del pacchetto verificati.

I guasti durante un nuovo startup alimentato non sono stati simulati: quel
percorso non è ammesso né implementato. I test precedenti di recovery
post-ABORT, temperatura, corrente, UART e NVS restano validi per il loro
ambito e non sono presentati come test dello startup. Nessun audit generico
o test firmware già passato è stato ripetuto.

## Artefatti e consegna

Firmware conservato: **125d981b02a3**, commit
`125d981b02a38673c8c54f2096632c99fd1f29ff`, ESP32-S3 ROBOT_POWERED, ingest 0,
1.129.024 byte, SHA256
`0b6c22f0470133adbcc1be5b96623def33d64b0278e29a58289adc57597cef34`.
Nessun sorgente applicativo modificato: nessuna ricompilazione necessaria.
Manifest e build canonica pulita preservati. Gli strumenti hanno un commit
separato elencato nel pacchetto, insieme a file modificati e patch.

Pacchetto: `/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_POST_ABORT_THERMAL_20261003`.
Entry point: `matdog_01_flash.sh`, `matdog_02_full_calibration.sh`,
`matdog_03_verify_and_finalize.sh`. Le invocazioni senza argomenti o con
`--offline-check` leggono soltanto file/Git. Il primo PASS significa preparazione
del flash; **non** significa OFFLINE_READY dell'intera sessione.

Rollback application-only be0c12979e5b e manifest conservati, stesso layout;
SHA256 `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`.
Backup storico A/B 16 MiB e verifica/procedura locale esistenti preservati.
Il full dump legacy non va usato come application-only: riavvolgerebbe layout
e NVS. Nessun nuovo backup richiesto; nessun restore fisico verificato qui.
In una futura scrittura si usa solo lo script canonico con i suoi gate;
nessuna tabella/bootloader/NVS, fallback BOOT/EN o riprogrammazione automatica.
La sessione completa rimane sospesa fino alla rimozione documentata del blocco.
