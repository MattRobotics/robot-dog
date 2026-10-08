# MATDOG M0 — Esito della pianificazione flash offline

Data: 2026-10-02. **M0 completato offline. Nessuna operazione hardware autorizzata
o eseguita; migrazione e primo boot restano da autorizzare separatamente.**

Il deliverable operativo è il
[runbook completo](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_M0_RUNBOOK.md):
mappa prima/dopo e intersezioni, comandi numerati con esiti e STOP, backup/preflight,
due scritture condizionate, primo boot diagnostico, matrice di recupero/rollback
regionale e completo, checklist e gate distinti A/B/C/R.

## Stato iniziale e perimetro delle modifiche

- Branch verificato: `feat/calibration-persistence-record-store-v1`.
- HEAD completo verificato: `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`.
- Working tree iniziale pulito; nessun AGENTS.md applicabile trovato negli antenati
  o nel repository. Nessuna delega/sub-agent usata.
- Nessuna modifica a P3a, calibrazione, motion control, OTA, partition CSV,
  Manifest V2 o strumenti esistenti di flashing.
- Aggiunti solo runbook, questo rapporto, checker locale `scripts/migration_m0.py`
  e test sintetici `scripts/tests/test_migration_m0.py`.
- Nessun push, PR o merge. Eventuale nuovo HEAD del commit M0 è documentale;
  gli artefatti continuano a provenire **esclusivamente da be0c129**.

Il modello richiesto nel task era GPT-5.6 Sol con ragionamento High; la scelta
del modello è della sessione e non è stata modificata né certificata da queste
operazioni. Nessuna attestazione della build firmware dipende dal modello.

## Ricognizione e decisioni

`partitions.csv` e il binario prodotto dal core coincidono con il layout V1
pinnato da `matdog_layout.py`. Esaminati `build.sh`, `flash_app_only.sh`,
`upload.sh`, writer/parser/verificatore Manifest V2, `backup_gate_logic.py`,
`verify_application_partition.py`, `ota_partition_logic.py`, bootloader/SDK,
`CalibrationRecordNvsBackend`, servizio di persistence, boot LOAD e comandi STATUS.

Il wrapper app-only legge il dispositivo e rifiuta la tabella legacy prima
di risolvere lo slot; resta invariato. `upload.sh` è uno stub di rifiuto.
Il Manifest V2 lega fonte/CLEAN, profilo, OTA ingest, FQBN, firmware, hash tabella
e dimensione slot; non prova da solo il BUILD_ID incorporato o la versione
del core. Il checker M0 aggiunge il controllo BUILD_ID, mentre la provenienza
del core e la pulizia del checkout sono verificate e archiviate separatamente.
Non è stato costruito un nuovo flasher o un nuovo sistema di manifest.

Il layout legacy ha app0/app1 da 3 MiB, con app1 a `0x310000`; il nuovo ha
slot da 5 MiB e app1 a `0x510000`. I 64 KiB della nuova NVS a `0xFE0000`
sono interamente parte della **vecchia FFAT**. Il contratto esistente vieta
di scrivere/cancellare quella regione. Se il nuovo backup contiene anche
un byte non-FF lì, il piano arresta la migrazione senza proporre un erase.
Questo percorso richiede inoltre FFAT legacy e nuovo app1 interamente FF.

Il bootloader generato dal core coincide con quello storico; legge il nuovo
layout dalla tabella e non necessita di riscrittura, a condizione che il nuovo
backup ne provi ancora l'identità. Rollback enabled, anti-rollback esplicitamente
disabled. Il normale percorso conserva otadata stabile con slot app0:
numero slot e indirizzo app0 non cambiano. Non inizializzare otadata a tutti FF;
il seed del core è un riferimento valido soltanto per eventuale recupero separato.
Nessuna W su otadata o NVS è necessaria nel percorso ammesso.

La sequenza futura ha **due W**: app0 e, dopo tutti i controlli sull'app,
partition table. Per il candidato USB: immagine `[0x10000,0x121860)`, erase
implicito `[0x10000,0x122000)` = 274 settori; tabella 3.072 B e un settore
`[0x8000,0x9000)` = 1 settore. Ogni W ha verify-flash e readback indipendente;
due full readback finali provano che tutte le altre regioni sono preservate.
Nessun erase esplicito, boot intermedio o write durante il preflight.

## Artefatti verificati

Bundle esterno al repository, persistente:

```text
/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_FLASH_LAYOUT_be0c129_20261002T150453Z
```

Due cloni locali separati, entrambi detached al commit approvato e puliti prima
e dopo la build. Eseguito l'esistente `build.sh --clean`, con build-path nuovi,
profilo esplicito e flags effettivi per BUILD_ID/profilo/OTA ingest 0.
Le credenziali locali Wi-Fi/HMAC non sono copiate: valgono i fallback vuoti
del sorgente. Le 16 esportazioni già presenti nel checkout originario sono
state fingerprintate e riconfrontate integralmente: **tutte intatte**.
Il vecchio manifest DIRTY/stale non è usato come candidato.

| Verifica | USB_ONLY | ROBOT_POWERED |
|---|---|---|
| SOURCE_COMMIT | `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a` | identico |
| SOURCE_STATE | CLEAN | CLEAN |
| BUILD_ID effettivo | `be0c12979e5b`, incorporato terminato NUL | identico |
| Hardware profile | USB_ONLY, esplicito | ROBOT_POWERED, esplicito |
| MATDOG_OTA_INGEST_ENABLED | 0, flag effettivo e manifest | 0, flag effettivo e manifest |
| Firmware byte | 1.120.352 | 1.123.472 |
| Firmware SHA-256 | `f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e` | `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0` |
| Manifest V2 SHA-256 | `b6dc29ca350af8073c3d9977cd5be89a593d637ef8279a283f1bfa49c55dc5d8` | `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48` |
| ELF SHA-256 | `c5903e74e86339f5f4c4b28de9c5cdcbe1a7e08542b38d56f01077362598b073` | `9939533aa1e562fe61ed1fc582a1202101714ee59f9e349fc8dc4c35d3ebdd20` |
| App slot | 5.242.880 B, 21.4% usato | identico, 21.4% usato |
| Checksum/digest immagine | validi | validi |

Tabella comune: 3.072 B, SHA
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`.
Bootloader comune solo per confronto: 19.968 B, SHA
`31b3c1be45dc5a76aa85c82540d6787b675e711eaf11021a5eea7e36f469c6de`.
Seed core: 8.192 B, SHA
`f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0`.
Archivio autonomo del sorgente approvato `approved-source.tar`, SHA
`d8096e874f08e74e5eb71d53fe9942264cf1edc5842287668e260ee774c3f611`.

Core **ESP32 3.3.11**, IDF v5.5.5 `b774170ff46`, Arduino CLI **1.5.1**, esptool
del core **5.3.1**. FQBN esatto nel runbook e nei manifest: 16 MiB, qio,
USB hwcdc, PSRAM opi. Gli effettivi include `qio_opi`, header SDK OPI e
`BOARD_HAS_PSRAM` sono verificati; il generico sdkconfig esportato non è usato
come unica prova della PSRAM. Header immagine DIO 80 MHz coerente con
`boards.txt` qio/build.flash_mode=dio. Nessuna riscrittura dell'header richiesta.

Il candidato del primo boot è **USB_ONLY**, con rail robot fisicamente isolati.
ROBOT_POWERED è solo confronto offline. Né la build né questa decisione
costituiscono autorizzazione a caricare il firmware.

## Backup storico esaminato, non sostitutivo del nuovo backup

Il file `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-29_185351_nostub.bin`
resta intatto: 16.777.216 B, SHA
`7290a3271fa11e0a73a438bc963795d2a52c22828c7fad53ebb9144a1971a684`.
Companion manifest coerente. Tabella legacy SHA
`ace02503447d0f470692e65fa76002f2d77a92dc81cd3813d8aa66718d716da9`.

| Regione nel backup storico | Evidenza locale |
|---|---|
| Bootloader | Identico al riferimento, padding FF fino a `0x8000` |
| Default NVS | Contiene dati; hash `f085a42376cfc2058d6aef95e4f5a339a2095f7da009aa96f4ba36a540e92f75` |
| otadata | Identico al seed core; seleziona app0, stato UNDEFINED |
| App0 | Immagine ESP32-S3 con checksum/digest validi; non è il candidato M0 |
| App1 legacy | Tutti i 3 MiB FF |
| FFAT legacy | Tutti i `0x9E0000` byte FF |
| Destinazione matdog_nvs | Tutti i 65.536 B FF; hash `71189f7fb6aed638640078fba3a35fda6c39c8962e74dcc75935aac948da9063` |
| Coredump | FF nel backup storico; il runbook ne preserva anche dati eventualmente nuovi |

**Nessuna inferenza sullo stato del 2 ottobre.** L'immagine storica non prova
letture indipendenti nuove, firmware attuale, slot attuale o FFAT ancora vuota.
La simulazione sui suoi file è archiviata come `historical-offline-exercise.json`,
con ambito esplicito HISTORICAL_FILES_ONLY e nessuna autorizzazione. La stessa
otadata risolve app0 sia col legacy sia con la V1, con dimensione ora 5 MiB;
staging e snapshot atteso passano i controlli puramente locali.

## Test offline e risultati

| Suite / controllo | Esito |
|---|---|
| `test_migration_m0.py` | **42 test PASS**, nessun skip |
| `test_matdog_layout.py`, generatore reale core 3.3.11 | **57 test PASS** |
| `test_build_manifest.py` | **83 test PASS** |
| Test Python `*_logic.py` backup/OTA | **65 test PASS** |
| `static_audit.py`, baseline e finale | **PASS**, 203 sorgenti; include suite C++ host esistente con 44 eseguibili |
| Due clean build reali, USB_ONLY e ROBOT_POWERED | **PASS** |
| Manifest V2 + BUILD_ID + table + confini + `image-info`, entrambi | **PASS** |
| Sintassi dei blocchi shell del runbook (`bash -n`, nessuna esecuzione) | **15 blocchi PASS** |
| Sintassi del reader Python nel runbook (`compile`, nessuna esecuzione) | **PASS** |
| Comparazione delle 16 esportazioni precedenti e backup storico | **PASS, invariati** |
| Esercizio file storici: staging, tabella nuova, selezione OTA preservata | **PASS offline** |

Le fixture M0 sono immagini da 16 MiB e tabelle sintetiche con entry/MD5/padding
che riproducono esattamente entrambi gli hash pinnati; non sono firmware
bootabili. Solo il digest del bootloader sintetico viene iniettato nei test
della logica; la CLI non offre override del digest reale.

| Scenario richiesto / aggiuntivo | Risultato verificato |
|---|---|
| Legacy corretto | Piano app0, table; otadata/NVS/bootloader preservati |
| Layout inatteso o padding tabella corrotto | STOP `INSTALLED_LAYOUT_UNEXPECTED` |
| Slot selezionato 1 | STOP `ACTIVE_SLOT_UNEXPECTED`; nessuna conversione silenziosa |
| Runtime e otadata discordanti | STOP `RUNNING_SLOT_MISMATCH` |
| Manifest V1, fonte errata/DIRTY, FQBN/profilo/ingest errati | Rifiuti esistenti specifici |
| BUILD_ID dichiarato senza incorporazione o layout marker assente | STOP `BUILD_ID_NOT_PROVEN` / marker refusal |
| NVS destinazione non vuota, anche un byte finale | STOP `DESTINATION_NVS_NOT_EMPTY` |
| Dati FFAT fuori dalla NVS / residui app1 nuovo | STOP `FFAT_NOT_EMPTY` / `NEW_APP1_NOT_EMPTY` |
| Hash firmware o backup non corrispondente | STOP specifico; nessun output di piano |
| Immagine oltre 5 MiB | STOP `APPLICATION_TOO_LARGE` |
| Immagine entro 5 MiB ma oltre lo staging legacy verificato | STOP `STAGING_EXCEEDS_LEGACY_APP0` |
| otadata entrambi FF o CRC invalidi | STOP `OTADATA_AMBIGUOUS` |
| NEW, PENDING anche nel settore perdente, stato ignoto | STOP `OTADATA_UNSTABLE` |
| Backup assente, corto, senza SHA atteso, stesso file due volte o letture discordanti | STOP; nessun piano |
| Bootloader/config ignoti o differenti | STOP; nessun tentativo di aggiornamento |
| MAC errato / formato errato | STOP; nessuna directory di piano creata |
| W parziale, byte app/tabella sbagliato o danno default NVS/matdog_nvs/coredump | STOP `SNAPSHOT_BYTE_MISMATCH` |
| Output archivio già esistente | STOP senza sovrascrittura |
| Autorizzazione assente o regione NVS nel contratto di migrazione | Rifiuto del contratto esistente |
| Archiviazione/piano CLI sintetici validi | File/SHA corretti, `authorization_granted=false`, `hardware_io=false` |

Durante la prima costruzione della fixture è stata corretta una slice Python
che allungava l'immagine sintetica di un byte; aggiunta l'asserzione della
dimensione esatta. I 42 risultati finali si riferiscono alla fixture corretta.
La modifica non ha interessato firmware o dati reali.

## Prove conservate e rischi residui

Nel bundle: `previous_exports.json`, `artifact-summary.json`,
`toolchain-and-provenance.json`, `approved-source.tar`,
`historical-offline-exercise.json`, sorgenti/compile directories separati,
export completi, log di build/verifica/test, header/flags effettivi e sorgenti
Espressif del commit esatto usati nella ricognizione. Il runbook mantiene
ogni comando hardware condizionato ad autorizzazione; non è stato lanciato
alcun comando con `--port`, né serial reader, flash/erase/reset/OTA.

Restano da provare sul dispositivo autorizzato: stato attuale e identità,
qualità delle due acquisizioni, isolamento/back-power, accessibilità BOOT/EN,
USB e ri-enumerazione, primo boot/NVS reale e resistenza a interruzioni di
potenza. I backup file non autenticano da soli MAC/orari/indipendenza.
La tabella non è aggiornata atomicamente con l'app; ROM e backup verificati
sono il percorso di recupero. App1 nuovo vuoto significa nessun fallback OTA.
Init del core può intervenire sulla default NVS; un boot non è globalmente
read-only. Rimangono i warning preesistenti SCServo, incluso ReadMode bounds.
Non sono stati corretti in M0, per rispettare il freeze del codice.

**A** deve autorizzare identità/letture/backup e sole transizioni diagnostiche
previste; **B** deve approvare il backup fresco e gli hash/piano delle due W;
**C** deve approvare solo primo boot e diagnostica; **R/R2** un recupero concreto
con regioni e motivazione. Nessuna delle quattro autorizzazioni è conferita
da questo rapporto. Nessun SAVE, ACK, riconciliazione, RESTORE, ammissione dei
JointTransform, calibrazione o movimento è stato eseguito o autorizzato.

M0 termina qui.
