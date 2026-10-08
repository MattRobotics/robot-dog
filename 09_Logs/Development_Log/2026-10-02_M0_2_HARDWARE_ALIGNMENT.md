# MATDOG — M0.2: actual hardware integration and migration readiness

Data: 2026-10-02. Repository `~/MATDOG/github/robot-dog`, branch
`feat/calibration-persistence-record-store-v1`, HEAD iniziale verificato
`174aa04`. Sorgente firmware immutabile `be0c129`.

**Esito operativo: Gate B BLOCKED.** Backup Gate A verificato e preflight
strutturale PASS; recupero fisico da guasto non dimostrato e percorso powered
ancora da qualificare. L'aggiornamento è esclusivamente documentale.
Validazione documentale finale: **PASS**.
Nessun nuovo accesso al dispositivo, flash, erase, reset, OTA, boot, movimento,
push, PR o merge effettuato. Il Gate A descritto è stato eseguito dall'operatore,
non da questo incarico. Nessuna nuova acquisizione è stata richiesta o eseguita.

## Configurazione reale, distinta dall'alternativa futura

La batteria alimenta il dominio protetto DALY con **KEY ON**. Servo bus, LED
ring e TECNOIOT restano alimentati; TECNOIOT fornisce 5 V all'ESP32-S3. L'host
usa la porta esterna GPIO19/D−, GPIO20/D+ e GND, **senza VBUS**: comunica ma non
alimenta l'ESP32. La USB-C integrata non è il collegamento utilizzato. BOOT ed
EN sono sotto la cover, non accessibili con il robot assemblato.

Robot sostenuto, zampe libere, disgiuntore accessibile. L'operatore ha verificato
tutti i 13 servo con torque=0, speed=0, current=0 **prima** dell'ingresso ROM.
Non si presume che quel valore sopravviva a un reset/power-cycle dei servo.
Non sono dichiarati rail a 0 V, segnali UART/LED scollegati o un'alimentazione
USB-C isolata mai realizzati. La configurazione USB-C/rail isolati è soltanto
un'alternativa futura da qualificare, non il preflight standard M0.2.

Nessuna progettazione elettrica è stata modificata. Il runbook e i riferimenti
canonici ora distinguono questa installazione, i risultati del Gate A e i
limiti di recupero. I vecchi comandi W/primo boot USB sono conservati in un
**template sospeso**, collassabile, non applicabile al robot assemblato.

## Gate A: evidenze e backup effettivi

Directory originale preservata:

`/home/matteo-manicardi/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594`

| Evidenza locale | Esito |
|---|---|
| `read-a-16m.bin` | 16.777.216 byte |
| `read-b-16m.bin` | 16.777.216 byte, inode distinto da A |
| Confronto integrale A/B | Identici byte per byte |
| SHA-256 A e B | `856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7` |
| `fullflash.sha256.txt` | Entrambe le righe corrispondono ai file e allo SHA atteso |
| `read-a.log` | Read 16777216 bytes, 1541.7 s; `Staying in bootloader` |
| `read-b.log` | Read 16777216 bytes, 2145.3 s; `Staying in bootloader` |
| `rom-entry.log`, `flash-id.log` | ESP32-S3 v0.2, MAC 14:c1:9f:22:75:94, PSRAM 8 MiB, flash 16 MiB; USB-Serial/JTAG |
| `security-info.log` | Flags 0, Secure Boot disabled, Flash Encryption disabled, SPI_BOOT_CRYPT_CNT=0 |

La dichiarazione dell'operatore conferma acquisizioni indipendenti senza W/erase
e permanenza ROM durante le acquisizioni. I file/log sono stati letti e
fotografati tramite size/SHA prima dell'analisi; gli originali sono conservati
integralmente. Non si è sostituito il backup con quello storico del 29 settembre.
Il transcript runtime/servo pre-ROM non è nella directory: il valore legacy
`dfcecb670d05`, ROBOT_POWERED, app0/3 MiB, OTA UNDEFINED/ingest DISABLED e i
13 torque-off sono evidenza fornita dall'operatore, non un log fabbricato.

La verifica del backup è **YES**. Non viene estesa automaticamente a tutti i
requisiti di B/R: copia su secondo supporto e relativo receipt non sono attestati
da questi file. Non è stata inventata una copia o richiesta una terza lettura.

## Analisi locale e preflight M0

Nuovo bundle di analisi, separato dai backup e dai bundle M0/M0.1:

`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_2_HARDWARE_ALIGNMENT_174aa04_20261002T190017Z`

`migration_m0.py archive` ha letto A/B e prodotto nove estratti in `regions`.
`migration_m0.py plan` invariato ha verificato il backup fresco con runtime
osservato `dfcecb670d05 @0x10000` e il candidato USB originale, producendo
`usb-only-file-plan`. Entrambi: **FILE_CHECKS=PASS, AUTHORIZATION_GRANTED=NO**.
Il piano USB è una regressione/preflight di file, **non il piano operativo M0.2**.

| Regione / controllo | Esito sul backup fresco |
|---|---|
| Tabella legacy `[0x8000,0x8C00)` | SHA canonico `ace02503447d0f470692e65fa76002f2d77a92dc81cd3813d8aa66718d716da9` |
| Settore tabella `[0x8000,0x9000)` | 4096 byte, SHA `0bcf1787e46f4bf1ce9ad28bd22c2257e715987e913add5008c0265d3feb2fcd` |
| Bootloader `[0,0x4E00)` | Byte-identico al riferimento 19.968 B, SHA `31b3c1be45dc5a76aa85c82540d6787b675e711eaf11021a5eea7e36f469c6de`; padding fino a 0x8000 FF |
| otadata `[0xE000,0x10000)` | SHA `f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0`; entrambi i CRC validi, stati UNDEFINED; seq 1/0, selezione app0 |
| Config bootloader | Rollback enabled, anti-rollback disabled; due slot in entrambi i layout e offset app0 invariato |
| App0 legacy `[0x10000,0x310000)` | BUILD_ID `dfcecb670d05\0` presente; checksum 0x46 e validation hash validi da image-info offline |
| Default NVS `[0x9000,0xE000)` | Contiene dati, archiviati/preservati; SHA `af47fce13caca986d34b3fc4b55f48e1fc40b8f8a36050b7c7bb0c05e6d2675c` |
| App1 legacy `[0x310000,0x610000)` | Tutti FF |
| FFAT legacy `[0x610000,0xFF0000)` | Tutti 10.354.688 byte FF |
| Nuovo app1 `[0x510000,0xA10000)` | Tutti 5.242.880 byte FF |
| Destinazione matdog_nvs `[0xFE0000,0xFF0000)` | Tutti 65.536 byte FF; SHA `71189f7fb6aed638640078fba3a35fda6c39c8962e74dcc75935aac948da9063` |
| Coredump `[0xFF0000,0x1000000)` | Tutti FF, archiviato/preservato |

Nessuna precondizione strutturale del backup è fallita. Le sovrapposizioni
FFAT/app1/NVS hanno quindi gli stessi presupposti M0, senza perdita di dati
non autorizzata. Nessuna W/erase aggiuntiva è stata proposta per prepararli.
Il risultato USB ipotetico ha SHA
`8e97bbed6bf9a1e82cc8b44a9f7575fad0b8be80edf291c7d45e0ffd5029ffe1`;
è esplicitamente un risultato del piano file-only USB, non un target powered.

Entrambi i manifest/immagini originali sono stati verificati offline:
Manifest V2, fonte CLEAN be0c129, BUILD_ID be0c12979e5b, FQBN/OPI, OTA ingest 0,
hash immagine/tabella, checksum e validation hash. Il manifest ROBOT_POWERED
passa il verificatore generico già esistente. Lo stesso profilo dato al
`migration_m0.py plan` invariato restituisce invece **STOP=PROFILE_MISMATCH**;
nessuna directory/piano powered viene creata. Anche `writer.prepare` rifiuta
il candidato robot **prima della costruzione della porta**. L'eventuale hint
`MATDOG_FLASH_PROFILE` del verificatore generale non è un override M0.1 e non
va usato per tentare un bypass.

## Ingresso ROM e writer: due fasi distinte

Il Gate A reale ha usato esptool **5.3.1**:

`--before usb-reset --after no-reset --no-stub --connect-attempts 1 read-mac`

È il reset iniziale esplicitamente autorizzato. Il codice esatto esptool 5.3.1
attestato in M0.1 seleziona `USBJTAGSerialReset` (`loader.py:846`), la cui sequenza
DTR/RTS è in `reset.py:145`. Non è una semplice lettura senza effetti B.
Il log conferma l'identità e `Staying in bootloader`; non prova BOOT tenuto basso.

Le successive letture usano **no-reset/no-reset/no-stub**. Il writer M0.1,
invariato byte per byte da 174aa04, accetta una ROM già raggiunta via quel reset:
DTR/RTS inattivi prima dell'open, una `connect(mode=no-reset, attempts=1)`,
IS_STUB false, contatori immagine/blocco **1/1**, nessun FLASH_END o reset in
chiusura; SerialException/protocol error sono STOP senza reconnect/retry/chaining.
Il reset iniziale non è incapsulato né ripetuto nel writer o in un handler.

I due test selezionati sul vero main/API con trasporto simulato 303a:1001
confermano i binding 1/1, i rifiuti di reset/finish/stub e la scrittura singola
su ROM pronta con l'artefatto USB originale. Non fingono un ingresso/recupero
fisico USB dopo un guasto. I 16 test/83 invocazioni di fault injection M0.1
rimangono valide per lo stesso codice, che non è stato modificato.

## Recupero fisico: non pronto

Un errore senza perdita della ROM lascia tecnicamente raggiungibili letture e
R1/R2; serve comunque nuova autorizzazione, nuovo stato letto e confrontato,
nessuna retry automatica. Questo non copre il guasto di alimentazione.

Dopo un'interruzione W app0, non si presume un'immagine bootabile o CDC della
app; app1 è vuoto e non offre fallback. Dopo un'interruzione della tabella, il
bootloader può rifiutarla. La ROM non dipende dalla validità della tabella per
programmare, **ma deve poter essere raggiunta**. Con BOOT/EN coperti, il solo
usb-reset già riuscito su firmware valido non dimostra quella raggiungibilità
quando firmware/USB non tornano disponibili. La USB esterna senza VBUS non
mantiene né ripristina alimentazione. Il disgiuntore arresta la potenza ma non
seleziona la ROM; può anche far perdere la precedente evidenza torque-off.

**RECOVERY_PATH_READY=NO; Gate B BLOCKED.** Per sbloccare in un incarico futuro:
accesso fisico di manutenzione ai BOOT/EN reali e percorso ROM indipendente
qualificato, oppure un metodo alternativo di recupero dimostrato nelle
condizioni di guasto pertinenti alla configurazione assemblata. Non basta
ripetere il reset sul legacy funzionante o citare una simulazione host. Non è
stato chiesto/eseguito qui alcun test fisico, power-cycle, apertura cover o W.

Il backup completo resta adatto al ripristino byte-identico device-specific,
anche con tabella parziale, **se** la ROM è accessibile e security/power restano
compatibili. R1/R2 sono verificati come percorsi software; non sono certificati
come recupero fisicamente disponibile nell'installazione odierna.

## Candidato appropriato e adattamento minimo ancora necessario

**ROBOT_POWERED è il candidato da qualificare per il cablaggio reale.**
USB_ONLY non spegne rail o servo. `HardwareProfile.h`/`BuildConfig.h` vi
dichiarano false servo/battery/LED, mentre sono live: diagnostica non adeguata.
`ServoBus::begin` e `DalyBms::begin/update` inizializzano UART/polling anche in
USB_ONLY; non offrono isolamento. `LedRing::begin` USB_ONLY lascia GPIO47 INPUT
e non manda OFF al ring alimentato. Non si certifica lo stato fisico di quel ring.

In entrambi i profili `Controller::begin` pone authority NONE, revoca permit,
carica NVS senza RESTORE e non avvia census/torque/movimento. Nessuno dei due
profili invia automaticamente SAFE_OFF a tutti i servo. ROBOT_POWERED esprime
le rail reali; il LED riceve clear/show OFF al boot e successivamente la policy
può pilotarlo. Il primo boot powered non è dichiarato sicuro soltanto per quei
fatti software: restano recupero, alimentazione e stato reale servo da qualificare.

| Identità / geometria | USB_ONLY, riferimento storico | ROBOT_POWERED, candidato da qualificare |
|---|---|---|
| Application SHA | `f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e` | `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0` |
| Manifest SHA | `b6dc29ca350af8073c3d9977cd5be89a593d637ef8279a283f1bfa49c55dc5d8` | `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48` |
| Bytes | 1.120.352 | 1.123.472 |
| W app0 | `[0x10000,0x121860)` | `[0x10000,0x122490)` |
| Erase/readback | `[0x10000,0x122000)`, 274 settori | `[0x10000,0x123000)`, 275 settori |

Stessi BUILD_ID `be0c12979e5b`, tabella V1 SHA
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`, FQBN e
OTA ingest 0. Non distinguere il profilo dal solo BUILD_ID.

Modifiche minime future, documentate ma **non attuate**:

1. Selezione attesa esplicita ROBOT_POWERED nel piano e nel writer; pinning di
   manifest/immagine robot già prodotti, senza adattare un manifest in-place.
   Non inferire il profilo automaticamente dal file né usare override/force.
2. Adattare `migration_m0.py verified_artifact` e caller CLI a quel profilo
   nominativo, riusando Manifest V2/layout. Writer closed-set per i soli artefatti
   attestati; tutti i controlli M0.1 e gate separati conservati.
3. Readback app di `0x113000`, padding/snapshot/hash powered ricalcolati sul
   backup fresco; tabella sempre un settore e app0 sempre prima della tabella.
4. Test positivi/negativi dei due profili, hash/offset/gate e fault injection
   reale API anche sul candidato robot, senza ridurre contatori 1/1 o recupero.
5. C powered dedicato: banner/rail coerenti, authority NONE, persistence NO_RECORD,
   OTA disabilitato, >=120 s senza reset, diagnostica DALY/LED e letture nominative
   dei 13 servo. `@SERVO READ` richiede MAINTENANCE: il cambio modo necessario non
   va inventato nella whitelist né considerato consenso ad authority/torque.

Nessuna modifica firmware/artefatti o ricompilazione necessaria. L'eventuale
primo boot non autorizza torque, movimento, calibrazione, SAVE, ACK, RECONCILE,
RESTORE o promozione di trasformazioni. Il LOAD/STATUS NVS ha lo stesso percorso
READONLY approvato in entrambi i profili: nessun motivo per cambiare il firmware
al fine di ottenere FF. La destinazione attuale è effettivamente interamente FF.
Un primo boot non è una lettura globale della flash: normali attività core/NVS/
OTA possono cambiare altre regioni; vanno archiviate e riconciliate prima di
qualunque dichiarazione di rollback byte-identico.

## File modificati e validazione

- `05_Firmware/MATDOG_Controller/FLASH_LAYOUT_M0_RUNBOOK.md`: hardware reale,
  backup concluso, ingresso USB distinto, B bloccato, confronto/adattamento
  candidato, checklist corrente; template W/boot USB sospeso.
- `04_Electronics/README.md`: sessione assemblata e limite di recupero, senza
  variazione del progetto elettrico.
- `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`: alimentazione di servizio,
  BOOT/EN coperti e limite del disgiuntore/recupero.
- Questo rapporto: evidenze, esiti, condizioni STOP e gate finali.

| Verifica | Esito |
|---|---|
| Backup A/B e SHA atteso | PASS, senza acquisizioni aggiuntive |
| Archive / preflight strutturale M0 USB | PASS, AUTHORIZATION_GRANTED=NO |
| Manifest e image-info originali USB/ROBOT, app legacy fresca | PASS su file locali |
| Piano/prepare powered sui tool invariati | STOP atteso PROFILE_MISMATCH; nessun piano powered/porta |
| Test M0/layout/manifest/backup/OTA | PASS: 42+57+83+25+40 = **247** |
| Writer binding + native-ROM candidate | PASS: **2 test** API reali, trasporto simulato |
| Sintassi runbook | PASS: **20** blocchi bash controllati con bash -n |
| Audit statico | PASS, 203 file sorgente; suite host/mutation richieste eseguite |
| Scope firmware/tool e integrità input | PASS, soli quattro Markdown; 8 file/log backup e 3.603 file bundle M0 invariati, 16 export precedenti identici |

Log/JSON nel nuovo bundle: `backup-verification.json`, `region-analysis.json`,
`regions/report.json`, `usb-only-file-plan/report.json`,
`logs/archive.log`, `logs/usb-only-file-plan.log`,
`logs/robot-powered-plan-refusal.log`, `logs/manifest-*.log`,
`logs/image-info-*.log`, `logs/test_*.log`,
`logs/writer-native-ROM-compatibility.log`, `logs/static-audit.log`,
`documentation-and-scope-check.json`, `protected-inputs-before.json`.
I log di analisi sono fuori dalla directory originale del backup.

## Rischi residui, STOP e gate finali

STOP prima di ogni W se recupero non pronto, candidate/profile/writer non
qualificati, power/USB instabili, periferiche/stato servo incerti, backup/receipt
non riconciliati o stato dispositivo cambiato dopo l'acquisizione. STOP su
qualsiasi errore trasporto/verifica; nessun reset, secondo tentativo, passaggio
alla W successiva o recovery automatico. Mai usare un boot come test implicito.
Il nuovo app1 non è fallback; W app/tabella non atomiche; gli effetti di una
reale interruzione di alimentazione non sono provati host. La disponibilità
fisica della ROM e la sicurezza delle rail powered restano il vincolo principale.

Il correttivo documentale può essere committato localmente quando i controlli
passano; questo non modifica l'esito BLOCKED della migrazione. Nessuna
concessione a flash/hardware né richiesta di accesso ulteriore in questa fase.

```text
GATE_A_BACKUP_VERIFIED=YES
M0_RUNBOOK_HARDWARE_ALIGNED=YES
RECOVERY_PATH_READY=NO
GATE_B_MIGRATION_READY=NO
HARDWARE_FLASH_AUTHORIZED=NO
```
