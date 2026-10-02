# MATDOG M0 — Migrazione flash e recupero, validati offline

Data: 2026-10-02. **M0.2: backup Gate A verificato; Gate B BLOCKED.**
Base firmware immutabile: `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`.
Branch `feat/calibration-persistence-record-store-v1`, HEAD di ingresso M0.2
`174aa04`. P3a e il writer M0.1 restano invariati.

Il Gate A hardware descritto sotto è stato eseguito dall'operatore e riferito
nella sessione; i log/file locali lo documentano. **M0.2 esegue soltanto L**:
nessun nuovo accesso hardware, backup, reset, boot, W o erase. Le istruzioni
hardware riportano un ingresso già avvenuto o template futuri sospesi; non sono
un'autorizzazione a ripeterli. Il backup positivo non abilita B, C, R o R2.

Classi: **L** = file locali; **R** = lettura del dispositivo, senza scrittura flash;
**W** = scrittura con cancellazione dei settori indirizzati; **B** = boot/reset/transizione
ROM, che può attivare le normali scritture del firmware/core. R non significa
assenza di interazione hardware. M0/M0.1 e questo correttivo M0.2 hanno eseguito esclusivamente L.

## Stato vincolante M0.2 — installazione realmente usata

| Elemento | Configurazione del Gate A reale |
|---|---|
| Alimentazione | Batteria → DALY BMS, **KEY ON** → rail servo/LED e TECNOIOT → **5 V ESP32-S3** |
| Collegamento host | USB di servizio esterna: **GPIO19/D−, GPIO20/D+, GND**; **nessun VBUS** |
| USB-C integrata | Non utilizzata nelle operazioni descritte; non è la procedura standard M0.2 |
| Periferiche | Servo bus, LED ring, DALY e TECNOIOT rimangono alimentati e collegati; nessuna misura 0 V o isolamento dei segnali è qui dichiarata |
| Accesso ROM fisico | **BOOT ed EN sotto la cover, non accessibili** con robot assemblato |
| Sicurezza meccanica | Robot sostenuto, zampe libere, disgiuntore accessibile |
| Stato servo pre-ROM | Tutti i **13** verificati dall'operatore: torque=0, speed=0, current=0; non è una garanzia dopo power-cycle/reset periferico |
| Chip verificato | ESP32-S3 v0.2; MAC `14:c1:9f:22:75:94`; flash 16 MiB; PSRAM 8 MiB |
| Security | Secure Boot e Flash Encryption disabilitati |
| Legacy osservato | `dfcecb670d05`, **ROBOT_POWERED**, app0 `0x10000`, size `0x300000`, OTA UNDEFINED, ingest DISABLED |

Cablaggio coerente con [elettronica](../../04_Electronics/README.md) e
[alimentazione/porta di servizio](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).
Nessuna modifica alla progettazione elettrica è proposta o eseguita.
L'alimentazione USB-C con rail isolati resta soltanto un'**alternativa futura**
che richiederebbe una diversa configurazione fisica qualificata, non un requisito
retroattivo per dichiarare valido questo backup.

Il backup reale è in
`~/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594`: due file distinti
`read-a-16m.bin` e `read-b-16m.bin`, ciascuno 16.777.216 byte, identici byte per
byte, SHA comune
`856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7`.
I file e `fullflash.sha256.txt` sono stati verificati localmente; non ripetere
acquisizioni né altre operazioni hardware in M0.2. I log esistenti riportano
identità, security e `Staying in bootloader`; la permanenza in ROM fra le due
acquisizioni e la verifica dei 13 servo sono confermate dall'operatore.
Non inventare log runtime/servo assenti dalla directory.

```text
GATE_A_BACKUP_VERIFIED=YES
M0_RUNBOOK_HARDWARE_ALIGNED=YES
RECOVERY_PATH_READY=NO
GATE_B_MIGRATION_READY=NO
HARDWARE_FLASH_AUTHORIZED=NO
```

## Recupero fisico attuale — blocco prima di ogni W

L'ingresso `usb-reset` con firmware legacy funzionante è dimostrato. **Non è
una dimostrazione del recupero dopo una W app0/tabella interrotta.**

| Stato dopo un guasto | Percorso disponibile / limite |
|---|---|
| ROM ancora attiva, alimentazione e USB conservate | Letture e possibile R1/R2 tecnicamente raggiungibili, solo con nuova R/R2; il writer non li avvia da un errore. Non provato fisicamente con una W fallita |
| Reset/brownout/power loss durante W app0 | App parziale può non partire; il nuovo app1 è vuoto. Non assumere CDC del firmware, torque sicuro dopo reset dei servo o fallback OTA |
| Tabella parziale/corrotta dopo reset | Il bootloader può rifiutare la tabella; la ROM può ripristinarla **se raggiungibile**. La recuperabilità dei byte non prova la raggiungibilità della ROM |
| USB assente/bloccata o robot disalimentato | La porta esterna non fornisce 5 V; BOOT/EN non accessibili. Disgiuntore utile per arresto elettrico, ma non equivale a ingresso ROM e può far perdere lo stato torque-off |

**Gate B resta BLOCKED.** Per rimuovere questo blocco occorre rendere disponibili
BOOT/EN con accesso di manutenzione alla cover (senza ridisegnare l'elettronica),
identificare i controlli reali e qualificare ingresso ROM/recupero indipendente
dall'applicazione; oppure dimostrare un metodo alternativo sulla configurazione
assemblata anche quando l'app/tabella non sono utilizzabili. Il solo usb-reset
su legacy valido, l'uscita `Staying in bootloader`, un file di backup, i test
host e la disponibilità teorica della ROM non soddisfano questo requisito.
Nessuna prova hardware del recupero è richiesta o autorizzata da M0.2.
Non provare un power-cycle, usb-reset ripetuto, UART0 improvvisata o una W
come esperimento di recupero. Se USB/ROM si perde: STOP, conservare evidenze,
mettere in sicurezza il robot e predisporre accesso fisico con incarico distinto.

## Mappa completa e contratto

Tutti gli intervalli sono `[inizio, fine)`; la fine è esclusa. Flash `0x1000000`
= 16.777.216 B; settore 4.096 B. Non usare il numero di MiB nel nome legacy
come misura della FFAT: la CSV effettiva assegna `0x9E0000`.

| Regione | Prima, legacy | Dopo, V1 | Dimensione prima → dopo | Azione normale |
|---|---|---|---:|---|
| Bootloader + spazio riservato | `0x000000–0x008000` | identico | 32.768 → 32.768 B | Preservare tutti i byte |
| Partition table e padding di settore | `0x008000–0x009000` | identico intervallo | 4.096 → 4.096 B | Sostituire tabella; padding atteso FF |
| `nvs`, data/02 | `0x009000–0x00E000` | identico | 20.480 → 20.480 B | Preservare; backup separato verificato |
| `otadata`, data/00 | `0x00E000–0x010000` | identico | 8.192 → 8.192 B | Preservare se selezione app0 stabile |
| `app0`, app/10 | `0x010000–0x310000` | `0x010000–0x510000` | 3.145.728 → 5.242.880 B | Scrivere solo l'immagine e i settori che copre |
| `app1`, app/11 | `0x310000–0x610000` | `0x510000–0xA10000` | 3.145.728 → 5.242.880 B | Nuovo app1 deve essere FF; nessuna scrittura |
| `ffat`, data/81 | `0x610000–0xFF0000` | `0xA10000–0xFE0000` | 10.354.688 → 6.094.848 B | Vecchia FFAT interamente FF; nessuna scrittura |
| `matdog_nvs`, data/02 | non esiste | `0xFE0000–0xFF0000` | 0 → 65.536 B | Deve essere FF; preservare; nessun format |
| `coredump`, data/03 | `0xFF0000–0x1000000` | identico | 65.536 → 65.536 B | Preservare, anche se contiene un dump |

La tabella binaria canonica occupa `0xC00` B: `[0x8000,0x8C00)`.
Il resto del settore `[0x8C00,0x9000)` deve essere FF dopo la scrittura.
Il bootloader noto occupa `[0,0x4E00)`; `[0x4E00,0x8000)` è FF nel riferimento.
Non esistono altri intervalli non coperti della flash.
Nel commit approvato FFAT compare nel contratto di layout, senza mount/format
del filesystem nel runtime; non serve prepararvi un filesystem per il primo boot.

| Intervallo comune ai due layout | Proprietario legacy → proprietario V1 | Conseguenza |
|---|---|---|
| `0x010000–0x310000` | app0 → app0 | Indirizzo di ingresso invariato |
| `0x310000–0x510000` | primi 2 MiB app1 → ultimi 2 MiB app0 | Archiviare; eventuali vecchi byte qui non sono un nuovo slot bootabile |
| `0x510000–0x610000` | ultimo 1 MiB app1 → primo 1 MiB app1 nuovo | Deve essere FF nel percorso M0 |
| `0x610000–0xA10000` | primi 4 MiB FFAT → ultimi 4 MiB app1 nuovo | Deve essere FF |
| `0xA10000–0xFE0000` | FFAT → FFAT con base diversa | Non è un filesystem preservato mediante semplice cambio tabella |
| **`0xFE0000–0xFF0000`** | **ultimi 64 KiB FFAT → matdog_nvs** | **Esaminare tutti i 65.536 byte del nuovo backup; un solo byte non-FF impone STOP** |

Gli slot crescono di 2 MiB ciascuno. La FFAT perde `0x410000` B = 4.0625 MiB:
4 MiB per gli slot e 64 KiB per la NVS dedicata. Le regioni invarianti sono
bootloader/spazio riservato, default NVS, intervallo otadata e coredump;
la tabella cambia contenuto pur mantenendo lo stesso indirizzo.

**Percorso M0 ammesso:** layout legacy esatto, bootloader noto, slot realmente
in esecuzione app0, otadata stabile, FFAT legacy interamente FF e nuovo app1
interamente FF. Byte residui nel futuro app0 oltre l'immagine restano archiviati
e invariati; non si cancellano 5 MiB per comodità. Le build qui selezionate
occupano meno dei 3 MiB legacy, quindi lo staging non tocca il vecchio app1.

Se FFAT, app1 nuovo o destinazione NVS non sono vuoti, **STOP prima di ogni W**.
Archiviare comunque il backup. Decidere separatamente conservazione/esportazione
dei dati e una nuova procedura: non consentire un'eccezione implicita o un erase
aggiuntivo. `matdog_layout.py` protegge esplicitamente la NVS dedicata anche
durante la migrazione. Questo runbook non modifica quel contratto.

## Artefatti e provenienza

Directory immutabile della sessione offline, esterna al repository:

```text
/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_FLASH_LAYOUT_be0c129_20261002T150453Z
```

`source-usb` e `source-robot` sono cloni locali separati, detached al commit
approvato; ciascuno ha eseguito `scripts/build.sh --clean` con build-path nuovo.
Gli export originali sono stati fotografati in `previous_exports.json` e non
sono stati sovrascritti. Gli artefatti di ciascun profilo sono sotto
`source-<profilo>/05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/`.

| Artefatto | Byte | SHA-256 |
|---|---:|---|
| USB_ONLY application `.ino.bin` | 1.120.352 | `f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e` |
| ROBOT_POWERED application `.ino.bin` | 1.123.472 | `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0` |
| Partition table, entrambi | 3.072 | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |
| Bootloader generato, entrambi, solo riferimento | 19.968 | `31b3c1be45dc5a76aa85c82540d6787b675e711eaf11021a5eea7e36f469c6de` |
| Core `boot_app0.bin`, solo riferimento/recupero | 8.192 | `f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0` |
| Manifest V2 USB_ONLY | 632 | `b6dc29ca350af8073c3d9977cd5be89a593d637ef8279a283f1bfa49c55dc5d8` |
| Manifest V2 ROBOT_POWERED | 637 | `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48` |

Entrambe: `SOURCE_COMMIT=be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`,
`SOURCE_STATE=CLEAN`, `BUILD_ID=be0c12979e5b` realmente presente come stringa
terminata NUL; `OTA_INGEST_ENABLED=0`. L'effettivo comando di compilazione
contiene **`-DMATDOG_OTA_INGEST_ENABLED=0`**, il profilo esplicito e il build ID.
I manifest V2 sono stati verificati con gli strumenti esistenti; ELF, mappe,
compile_commands, log e hash di tutti gli export sono conservati nel bundle.

FQBN esatto:

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi
```

Core installato e utilizzato **3.3.11**, ESP-IDF **v5.5.5**, commit `b774170ff46`;
esptool **5.3.1** del medesimo pacchetto. L'include effettivo è `qio_opi/include`,
con `CONFIG_SPIRAM_MODE_OCT=1` e `BOARD_HAS_PSRAM`; il generico sdkconfig esportato
non basta a provare OPI, perché contiene impostazioni PSRAM generiche differenti.
Il controllo usa anche compile_commands e l'header effettivo. `image-info`
conferma ESP32-S3, 16 MB, checksum e digest validi. L'header dell'immagine è DIO
80 MHz: `boards.txt` del core associa proprio QIO a `build.flash_mode=dio` e
`build.boot=qio`; non è un motivo per riscrivere l'header o il bootloader.

Non sono stati copiati gli header locali Wi-Fi/HMAC: le build usano i fallback
vuoti del sorgente approvato. Ciò è esplicito e registrato; non alterare questi
artefatti aggiungendo credenziali prima del flash. L'`App version` IDF
`ee57070` è del lib-builder e **non identifica MATDOG**: usare BUILD_ID,
SHA del file, manifest e banner MATDOG. Il manifest già esportato nel checkout
originario è DIRTY, riferito a `ff0543c…`, e non descrive il binario attuale:
non è un candidato M0, pur restando conservato.

## Candidato primo boot nella configurazione assemblata

**ROBOT_POWERED è il candidato coerente da qualificare**, già compilato da
be0c129; non è ancora abilitato nel piano/writer M0.1 e non è autorizzato al flash.
USB_ONLY non è un profilo di sicurezza per questo cablaggio: dichiara servo,
batteria e LED non alimentati, mentre sono alimentati. Non spegne quei rail.

| Comportamento del sorgente approvato | USB_ONLY | ROBOT_POWERED |
|---|---|---|
| Aspettative servo/batteria/LED | false/false/false, diagnostica di disponibilità non rappresentativa | true/true/true, corrisponde ai rail reali |
| ServoBus / DALY | UART inizializzate anche qui; il polling DALY non è un isolamento elettrico | Stessi trasporti, aspettative powered |
| LED GPIO47 | INPUT, nessun frame: non spegne né osserva un ring già alimentato; stato precedente non attestato | Boot clear/show OFF, poi normale policy di stato può cambiare i LED |
| Startup servo | Nessun ping/census/torque/movimento automatico | Identica assenza di startup torque/movimento; **nessun torque-off automatico da presumere** |
| Authority/persistence | NONE, permit revocato; LOAD senza RESTORE | Identico percorso; il profilo non autorizza attuazione o SAVE |

Riferimenti esatti: `HardwareProfile.h`, `BuildConfig.h`,
`Controller::begin/update`, `ServoBus::begin`, `DalyBms::update`, `LedRing::begin`.
USB_ONLY non mostra un torque-on automatico nei sorgenti, ma **non è qualificato
come primo boot sicuro e diagnosticamente adeguato su questi rail powered**.
ROBOT_POWERED descrive l'impianto correttamente; la sicurezza del primo boot
resta da qualificare, con B/C distinti e recupero fisico pronto.

**Modifiche minime future, non eseguite in M0.2:**

1. Piano esplicitamente ROBOT_POWERED, non inferito dal manifest o da un env
   ereditato; Manifest V2 esistente, SHA applicazione
   `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`,
   manifest `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48`,
   stessa tabella/FQBN/BUILD_ID/OTA=0. Nessuna nuova build o modifica di artefatti.
2. `migration_m0.py`/writer: selettore atteso esplicito e pin del candidato robot,
   mantenendo USB_ONLY soltanto per la sua alternativa qualificata; riusare
   Manifest V2/layout e lasciare invariati contatori 1/1, divieti di reset,
   reconnect, retry, chaining e tutte le verifiche. Attualmente la richiesta
   ROBOT_POWERED è correttamente rifiutata con `PROFILE_MISMATCH`.
3. Immagine robot 1.123.472 byte (`0x112490`): W
   `[0x10000,0x122490)`, erase/readback `[0x10000,0x123000)` = **275 settori**;
   lettura app `0x113000`, nuovi padding/snapshot/hash locali sul backup fresco.
   La tabella resta 3072 byte e un settore. Il piano USB da 274 settori non va riusato.
4. Test positivi ROBOT_POWERED con artefatto già attestato e negativi profilo/hash/
   offset errati; fault injection reale API per entrambi i profili e R1/R2,
   senza ridurre protezioni. Nessun mock del firmware per certificare recupero fisico.
5. Piano C powered dedicato: banner ROBOT_POWERED con rail YES, authority NONE,
   startup torque/motion/scan DISABLED, persistence NO_RECORD, OTA ingest DISABLED;
   diagnostica DALY/LED e letture di stato dei 13 servo nominate esplicitamente.
   `@SERVO READ` richiede MAINTENANCE: l'eventuale cambio modo deve essere nominato
   nel futuro gate C e non concede authority/torque. Non dichiarare servo sicuri
   dai soli status cached. Nessun torque, movimento, calibrazione, SAVE, ACK,
   RECONCILE, RESTORE o promozione di trasformazioni è autorizzato dal primo boot.

Il writer corrente **non viene adattato né forzato** in questo correttivo
solo documentale. Non usare flash_app_only, una CLI W o un manifest editato
per superare il rifiuto. La fonte be0c129 resta immutabile.

## Bootloader e otadata effettivi

Il backup fresco del 2 ottobre conferma il bootloader generato dal core
corrente, byte per byte, più padding FF fino a `0x8000`, già osservato nel
backup storico. Il preflight locale è PASS: `migration_m0.py plan` rifiuta
ogni differenza, senza proporre un aggiornamento del bootloader.
Entrambi i layout hanno due sottotipi OTA contigui, app0 a `0x10000`, otadata a
`0xE000` e tabella MD5 a `0x8000`; il bootloader legge indirizzi e dimensioni
dalla tabella. Non incorpora gli offset legacy degli slot.

Configurazione provata: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`,
`# CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set`, partition table offset
`0x8000`, MD5 abilitato. La selezione deriva da CRC, stato e
`(ota_seq - 1) % 2`; non dall'etichetta, da una supposizione su app0 o dal
solo numero di sequenza. NEW e PENDING_VERIFY impongono STOP in **entrambi**
i settori. Anche il settore perdente PENDING può essere riscritto al boot.
La logica di base resta quella di `ota_partition_logic.py`, corrispondente al
[bootloader Espressif del commit esatto](https://raw.githubusercontent.com/espressif/esp-idf/b774170ff46/components/bootloader_support/src/bootloader_utility.c)
e alle sue [regole di validità/CRC](https://raw.githubusercontent.com/espressif/esp-idf/b774170ff46/components/bootloader_support/src/bootloader_common_loader.c).

Seed core e otadata storica coincidono: settore 0 `seq=1`, stato
`0xFFFFFFFF` (UNDEFINED), CRC `0x4743989A`; settore 1 `seq=0`, stato
UNDEFINED, CRC `0xFFFFFFFF`. La selezione è app0. **UNDEFINED non è NEW**:
non attiva il percorso di conferma/rollback pending. Non sostituire questo
seed con otadata interamente FF o con una struttura inventata.

Per il percorso normale si conserva l'otadata nuova acquisita, purché provata
stabile e coerente con app0 realmente in esecuzione. Il numero di slot e
l'indirizzo di app0 restano uguali: **nessuna W su otadata è necessaria**.
La migrazione non crea un vecchio firmware di fallback in app1; il recupero
dipende dal backup locale dello stesso dispositivo, non dall'OTA rollback.

## Runbook operativo numerato

**Ogni comando deve terminare con successo prima del successivo.** Un exit code
non zero, un log incompleto o un risultato diverso dall'atteso impone STOP:
restare in ROM, conservare log e immagini, non riavviare l'applicazione, non
ritentare scritture automaticamente. Nei blocchi shell usare `set -euo pipefail`.

### 1. L — Preparare percorsi e verificatori

```bash
set -euo pipefail
REPO="$HOME/MATDOG/github/robot-dog"
SCRIPTS="$REPO/05_Firmware/MATDOG_Controller/scripts"
M0_ROOT="$HOME/MATDOG/verification-artifacts/MATDOG_M0_FLASH_LAYOUT_be0c129_20261002T150453Z"
USB="$M0_ROOT/source-usb/05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3"
ROBOT="$M0_ROOT/source-robot/05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3"
# APP/MANIFEST USB sono soltanto riferimenti del checker invariato/template storico.
# NON selezionano un candidato operativo M0.2; B resta BLOCKED.
APP="$USB/MATDOG_Controller.ino.bin"
TABLE="$USB/MATDOG_Controller.ino.partitions.bin"
MANIFEST="$USB/matdog_build_manifest.txt"
ESPTOOL="$HOME/.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool"
M01_ROOT="$HOME/MATDOG/verification-artifacts/MATDOG_M0_1_HARDENING_ebb6078_20261002T162810Z"
WRITE_PY="$M01_ROOT/venv/bin/python"
M0_WRITER="$SCRIPTS/migration_m0_write.py"
# L: verifica versione e hash dei sorgenti API, senza aprire porte.
"$WRITE_PY" -I -c 'import sys; sys.path.insert(0, sys.argv[1]); import migration_m0_write as w; w.load_tool(); print("M0_WRITER_TOOL=PASS")' "$SCRIPTS"
FQBN="$(PYTHONPATH="$SCRIPTS" python3 -c 'import matdog_layout; print(matdog_layout.PINNED_FQBN)')"
```

Queste assegnazioni non accedono al dispositivo. Conservare il bundle, compresi
i due cloni sorgente; non costruire nel checkout originario né usare il suo
path `build/` come scorciatoia. Non passare un `.merged.bin` a nessun comando W.

### 2. L — Conservare e controllare il backup storico

```bash
HIST="$HOME/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-29_185351_nostub.bin"
test "$(stat -c%s "$HIST")" -eq 16777216
test "$(sha256sum "$HIST" | cut -d' ' -f1)" = 7290a3271fa11e0a73a438bc963795d2a52c22828c7fad53ebb9144a1971a684
```

Atteso: dimensione e SHA coincidono. Il file e il companion manifest restano
conservati. Non usarlo come backup fresco, né inferire da esso la FFAT attuale.
Il backup fresco del 2 ottobre è già in una directory distinta, verificata al passo 8; non sovrascrivere né ripetere le acquisizioni.

### 3. L — Verificare entrambi gli artefatti esistenti, senza abilitarli

I controlli USB sono regressioni del checker invariato; i controlli ROBOT
attestano il candidato coerente ancora da qualificare. Nessuno dei due PASS
concede B/C o seleziona automaticamente un binario da flashare.

```bash
test "$(GIT_OPTIONAL_LOCKS=0 git -C "$M0_ROOT/source-usb" rev-parse HEAD)" = be0c12979e5b4b8ddc9dd21772d0d106e4358f4a
test -z "$(GIT_OPTIONAL_LOCKS=0 git -C "$M0_ROOT/source-usb" status --porcelain)"
test "$(stat -c%s "$APP")" -eq 1120352
test "$(sha256sum "$APP" | cut -d' ' -f1)" = f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e
test "$(sha256sum "$TABLE" | cut -d' ' -f1)" = 8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7
test "$(sha256sum "$MANIFEST" | cut -d' ' -f1)" = b6dc29ca350af8073c3d9977cd5be89a593d637ef8279a283f1bfa49c55dc5d8
test "$(sha256sum "$USB/sdkconfig" | cut -d' ' -f1)" = 534f3457aaf3ffbc82c45dc21d6b3ac57388c82d0d9a24bd2b8e6cadc3e48d9d
python3 "$SCRIPTS/build_manifest.py" verify --manifest "$MANIFEST" --binary "$APP" \
  --head be0c12979e5b4b8ddc9dd21772d0d106e4358f4a --tree-state CLEAN \
  --expected-fqbn "$FQBN" --requested-profile USB_ONLY --requested-ota-ingest 0
python3 "$SCRIPTS/matdog_layout.py" check-build --partitions "$TABLE" --binary "$APP" --fqbn "$FQBN"
python3 "$SCRIPTS/matdog_layout.py" check-write --offset 0x10000 --partition-size 0x500000 --image-size 1120352
"$ESPTOOL" --chip esp32s3 image-info "$APP"
```

Atteso: manifest V2, fonte CLEAN, hash USB e tabella della sezione artefatti;
layout V1, slot da 5 MiB, immagine 1.120.352 B, checksum/digest validi.
Controllare anche `logs/effective-compiler-usb.json` e il BUILD_ID incorporato.
STOP per ogni incompatibilità, immagine vuota/oversize o toolchain diversa.
`image-info` opera su file: non apre la porta.

Attestazione locale del candidato powered già prodotto, senza cambiare APP del
checker USB o attivare writer:

```bash
test "$(GIT_OPTIONAL_LOCKS=0 git -C "$M0_ROOT/source-robot" rev-parse HEAD)" = be0c12979e5b4b8ddc9dd21772d0d106e4358f4a
test -z "$(GIT_OPTIONAL_LOCKS=0 git -C "$M0_ROOT/source-robot" status --porcelain)"
test "$(stat -c%s "$ROBOT/MATDOG_Controller.ino.bin")" -eq 1123472
test "$(sha256sum "$ROBOT/MATDOG_Controller.ino.bin" | cut -d' ' -f1)" = 7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0
test "$(sha256sum "$ROBOT/matdog_build_manifest.txt" | cut -d' ' -f1)" = d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48
python3 "$SCRIPTS/build_manifest.py" verify --manifest "$ROBOT/matdog_build_manifest.txt" \
  --binary "$ROBOT/MATDOG_Controller.ino.bin" \
  --head be0c12979e5b4b8ddc9dd21772d0d106e4358f4a --tree-state CLEAN \
  --expected-fqbn "$FQBN" --requested-profile ROBOT_POWERED --requested-ota-ingest 0
"$ESPTOOL" --chip esp32s3 image-info "$ROBOT/MATDOG_Controller.ino.bin"
```

### 4. Gate A — Backup già completato, non autorizzazione alla migrazione

L'operatore ha autorizzato ed eseguito l'ingresso **usb-reset** iniziale, le
letture ROM successive **no-reset** e due acquisizioni complete. Il loro
confronto/SHA è verificato offline al passo 8; i file originali si conservano.
Non chiedere né effettuare nuovi accessi hardware per questo incarico.
Il positivo Gate A backup non concede W, erase, OTA, boot del candidato,
attuazione, B/C o recovery. Nessuna nuova transizione ROM/legacy è parte di M0.2.

### 5. B/R — Registrare alimentazione e stato fisico reali

Durante il Gate A il robot ha usato batteria/DALY **KEY ON**, servo bus e LED
alimentati, TECNOIOT→5 V ESP32 e la sola porta esterna USB19/20/GND senza VBUS.
La USB-C integrata non è stata usata. I segnali UART servo GPIO17/18, DALY
GPIO15/16 e LED GPIO47 restano collegati a periferiche alimentate; BNO085 è
sulla 3V3 ESP32. Non attribuire al setup isolamento o misure 0 V mai effettuati.

I 13 servo erano verificati torque=0, speed=0 e current=0 **prima** del reset
USB. Robot sostenuto, zampe libere, disgiuntore accessibile. Questa evidenza
non prova il comportamento dopo brownout/power-cycle e non autorizza torque
o movimento. BOOT/EN sono sotto la cover, non disponibili; il recupero fisico
resta il blocco di B indicato sopra. Non aprire cover o modificare alimentazione
in questo incarico. Conservare quanto riferito dall'operatore senza creare
fotografie, misure o log servo fittizi.

Un solo proprietario USB per le eventuali sessioni hardware future; porta by-id
identificata, MAC atteso `14:c1:9f:22:75:94`. Nessuna nuova apertura della porta
in M0.2. La precedente configurazione USB-C isolata è solo alternativa futura,
non il preflight standard per l'installazione assemblata.
### 6. R/B — Evidenza runtime legacy già acquisita

L'operatore ha osservato `dfcecb670d05`, ROBOT_POWERED, app0 `0x10000`, size
`0x300000`, OTA UNDEFINED e ingest DISABLED prima dell'ingresso ROM. Il backup
fresco contiene il BUILD_ID NUL-terminato e una app0 con checksum/digest validi.
I log locali disponibili non includono l'intero transcript runtime/servo:
la sua provenienza è la dichiarazione dell'operatore, non un log inventato.

Usare `CURRENT_OFFSET=0x10000`, `CURRENT_BUILD_ID=dfcecb670d05` e il MAC
verificato soltanto per l'analisi locale di questa acquisizione. Non avviare
nuovamente legacy o reader. Per un'altra sessione non inferire lo slot realmente
eseguito dalla sola selezione otadata. Pending, fallback o divergenze sono STOP.
### 7. B/R — Ingresso USB ROM già verificato; nessun reset durante M0.2

Comando **già eseguito dall'operatore**, esptool 5.3.1:

```bash
PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
SESSION="$HOME/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594"
# TRASCRIZIONE del Gate A: NON rilanciare in M0.2.
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 115200 \
  --before usb-reset --after no-reset --no-stub --connect-attempts 1 read-mac
```

`usb-reset` è un **reset iniziale B esplicitamente autorizzato**, non R pura:
`USBJTAGSerialReset` del 5.3.1 applica la sequenza DTR/RTS per ingresso ROM.
Il log `rom-entry.log` conferma chip/MAC e `Staying in bootloader`; non implica
GPIO0 tenuto fisicamente basso né un percorso cold-recovery dimostrato.
BOOT/EN non sono stati azionati manualmente.

Le successive letture già effettuate hanno usato `no-reset/no-reset/no-stub`.
Per eventuali operazioni future già autorizzate, mantenere questo contesto
ROM separato dal reset iniziale (non invocato da M0.2):

```bash
ESP=("$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 115200 \
  --before no-reset --after no-reset --no-stub --connect-attempts 1)
```

Il writer M0.1 usa la stessa API/USB nativa 303a:1001 e può connettersi una volta
in `no-reset` a una ROM già entrata con usb-reset. Non contiene il reset iniziale,
non lo ripete su errore, non invia FLASH_END e non esce automaticamente dalla ROM.
I contatori 1/1 e tutti i rifiuti restano invariati. Non richiede un nuovo reset
per ogni invocazione; questo è compatibilità del percorso software, **non**
prova di recupero fisico o autorizzazione W. Il commento M0.1 su BOOT trattenuto
riguarda il template isolato, non una garanzia ottenuta sull'hardware assemblato.
Un errore, un reset inatteso o la perdita ROM non autorizza reconnect/reset/retry.
Security/flash confermati in `security-info.log`/`flash-id.log`; nessun force/eFuse.
### 8. L — Verificare la doppia acquisizione completata, senza ripeterla

Il dispositivo è rimasto in ROM durante le due acquisizioni riferite
`BACKUP_IDENTICI=YES`, `GATE_A_DOPPIA_ACQUISIZIONE=PASS`. Controlli M0.2 soltanto
su file preesistenti:

```bash
SESSION="$HOME/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594"
A_IMAGE="$SESSION/read-a-16m.bin"
B_IMAGE="$SESSION/read-b-16m.bin"
test "$(stat -c%s "$A_IMAGE")" -eq 16777216
test "$(stat -c%s "$B_IMAGE")" -eq 16777216
cmp "$A_IMAGE" "$B_IMAGE"
sha256sum --check "$SESSION/fullflash.sha256.txt"
NEW_BACKUP_SHA256=856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7
```

File distinti/inode distinti, confronto completo e SHA comune verificati.
Il confronto locale conferma i dati; l'indipendenza delle acquisizioni e la
permanenza ROM derivano dai log esistenti e dalla dichiarazione dell'operatore.
Conservare **tutti** i file/log originali senza sovrascriverli. Non acquisire una
terza lettura, non ripetere backup o reset in questa fase. Un file mancante,
size/hash differente o mismatch avrebbe imposto STOP, non una maggioranza.
### 9. L — Estratti offline del backup fresco

Gli esiti M0.2 sono nel nuovo bundle
`~/MATDOG/verification-artifacts/MATDOG_M0_2_HARDWARE_ALIGNMENT_174aa04_20261002T190017Z`.
Le directory `regions` e `usb-only-file-plan` sono già prodotte: non sovrascriverle.
Per riprodurre l'analisi scegliere una nuova directory di output, senza hardware:

```bash
OFFLINE_ROOT="${OFFLINE_ROOT:?scegliere una nuova directory locale per la riproduzione}"
python3 "$SCRIPTS/migration_m0.py" archive --backup "$A_IMAGE" --repeat "$B_IMAGE" \
  --backup-sha256 "$NEW_BACKUP_SHA256" --out "$OFFLINE_ROOT/regions"
"$ESPTOOL" --chip esp32s3 image-info "$OFFLINE_ROOT/regions/legacy_app0.bin" \
  | tee "$OFFLINE_ROOT/installed-app0-image-info.log"
"$ESPTOOL" --chip esp32s3 image-info "$USB/MATDOG_Controller.ino.bootloader.bin" \
  | tee "$OFFLINE_ROOT/bootloader-reference-image-info.log"
```

Atteso: `FILE_CHECKS=PASS`, `AUTHORIZATION_GRANTED=NO`; tabella legacy canonica
SHA `ace02503447d0f470692e65fa76002f2d77a92dc81cd3813d8aa66718d716da9`.
`regions/report.json` registra offsets, dimensioni e SHA dei nove estratti:
intera regione bootloader, settore tabella, default NVS, otadata, entrambi gli
slot legacy, FFAT, excerpt destinazione NVS e coredump. Gli estratti coprono
ogni byte della flash; quello NVS di destinazione è volutamente anche parte FFAT.
Verificare checksum/digest validi dell'app installata e confrontare il suo
BUILD_ID con il log runtime e gli eventuali artefatti storici disponibili.
Un semplice `strings` o l'App version IDF non certifica il firmware attuale.

Conservare anche i log runtime/security, CSV legacy del core, tabella canonica,
manifest del candidato, SHA e tool/versioni. Copiare l'archivio su un secondo
supporto e rileggerlo completamente: `cmp` e SHA contro l'originale. Il secondo
supporto è ridondanza di conservazione, non una terza acquisizione dal chip.
Registrare un recovery receipt con sessione, MAC, seriale USB, orari/metodi A/B,
BACKUP_SIZE, BACKUP_SHA256 e verifica del secondo supporto. Il formato KEY=VALUE
compatibile con `backup_gate_logic.py` non basta da solo a provare identità,
freschezza o indipendenza: gli allegati e la firma dell'operatore sono necessari.

### 10. L — Preflight strutturale, non un piano operativo powered

Il checker invariato accetta solo USB_ONLY. È stato eseguito sul nuovo backup
come verifica delle precondizioni/layout e regressione del candidato storico:
**FILE_CHECKS=PASS non seleziona USB_ONLY per il robot assemblato**.
La verifica del manifest ROBOT_POWERED con `build_manifest.py` è PASS; il piano
ROBOT_POWERED con `migration_m0.py` invariato dà invece il rifiuto atteso
`STOP=PROFILE_MISMATCH`, senza produrre un piano powered.

```bash
CURRENT_OFFSET=0x10000
CURRENT_BUILD_ID=dfcecb670d05
OBSERVED_MAC=14:c1:9f:22:75:94
python3 "$SCRIPTS/migration_m0.py" plan --backup "$A_IMAGE" --repeat "$B_IMAGE" \
  --backup-sha256 "$NEW_BACKUP_SHA256" --sdkconfig "$USB/sdkconfig" \
  --bootloader "$USB/MATDOG_Controller.ino.bootloader.bin" \
  --running-offset "$CURRENT_OFFSET" --installed-build-id "$CURRENT_BUILD_ID" \
  --expected-mac 14:c1:9f:22:75:94 --observed-mac "$OBSERVED_MAC" \
  --binary "$APP" --manifest "$MANIFEST" --out "$OFFLINE_ROOT/usb-only-file-plan"
```

Atteso: `FILE_CHECKS=PASS`, `AUTHORIZATION_GRANTED=NO`, slot 0, azioni
`PRESERVE` per bootloader/otadata e `PRESERVE_ERASED` per matdog_nvs.
Il checker legge di nuovo integralmente le due acquisizioni, verifica il backup
con `backup_gate_logic.py`, riusa Manifest V2/layout/OTA, controlla BUILD_ID,
tutti i byte FFAT/NVS/app1 nuovo, e prepara esclusivamente file locali nuovi.
Non contatta il chip, non crea un flasher e non autentica la dichiarazione umana
di MAC o acquisizione indipendente. Le prove dei passi precedenti restano obbligatorie.

`plan/app0_sector_padded.bin` è solo per readback, non il firmware Manifest V2.
`plan/target_partition_sector.bin` è tabella più 1.024 byte FF. Il report indica
SHA del risultato finale previsto e intervalli effettivi. Qualunque STOP
interrompe il percorso; un nuovo backup o un'eccezione richiede nuova revisione.

### 11. Gate B — BLOCKED, nessuna migrazione pronta

Le precondizioni del backup/layout sono PASS ma non bastano. Mancano un accesso
fisico BOOT/EN utilizzabile o un recupero alternativo dimostrato da stato guasto,
e un piano/writer/primo boot qualificati ROBOT_POWERED. Anche la copia su secondo
supporto e il recovery receipt prima di B non sono attestati dai soli tre file
di backup; non inventarne il completamento. **Nessuna W in M0.2.**

Il Gate A positivo autorizzava letture/reset iniziale, non W o primo boot del
candidato. Non impostare `AUTHORIZED_BACKUP_SHA256` come effetto di un PASS,
non concedere B/C/R/R2 e non provare a migrare per verificare la recuperabilità.
### 12. L — STOP operativo; controlli locali consentiti

Riconciliare soltanto evidenze, manifest, hash e report locali. Non aprire la
porta per confermare nuovamente MAC o ROM. Backup e preflight di M0.2 non attestano
uno stato futuro dopo un boot, power-cycle o modifica: allora serviranno una
nuova valutazione della freschezza e gate distinti, non la ripetizione di backup
in questo incarico. Nessun `flash_app_only.sh`, upload, CLI W o override profilo.

## Template sospeso M0/M0.1 — solo alternativa USB isolata futura

**Le sezioni seguenti fino a R2 non sono il piano operativo della configurazione
assemblata.** Conservano il template tecnico USB_ONLY per una diversa
configurazione fisica isolata con BOOT/EN accessibili, da qualificare separatamente.
Non usare questi comandi, le loro attese 274 settori o la whitelist USB per il
candidato ROBOT_POWERED. Le protezioni del writer restano riferimenti obbligatori
anche per l'eventuale adattamento powered; non sono un modo di sbloccare Gate B.
<details>
<summary>Template USB_ONLY sospeso — da non eseguire sulla configurazione assemblata M0.2</summary>

### M0.1 — Contratto di ogni W (B oppure R/R2)

Tutte le W sotto usano `migration_m0_write.py`, mai la CLI `write-flash`.
Il pacchetto API 5.3.1 nel nuovo bundle M0.1 è attestato contro tutti i 26 moduli
nel binario congelato M0; il writer verifica versione e hash dei sorgenti
esptool/pyserial. Usa Python isolato (`-I`), non il pacchetto esptool di sistema.
Non modificare né reinstallare nel bundle M0 originale. Ambiente riproducibile:
sdist ufficiale e SHA in `scripts/migration_m0_esptool531.json`, dipendenze in
`$M01_ROOT/requirements.lock.txt`, prova di equivalenza nel bundle M0.1.

Dopo B, oppure dopo la distinta autorizzazione R/R2, predisporre il contesto:

```bash
WRITE_CONTEXT=(--backup "$A_IMAGE" --backup-repeat "$B_IMAGE" \
  --backup-sha256 "$AUTHORIZED_BACKUP_SHA256" --port "$PORT" \
  --mac 14:c1:9f:22:75:94 --binary "$APP" --manifest "$MANIFEST")
```

Il flag `--gate` registra il gate richiesto, **non concede autorizzazione**.
I passi ordinari 13/14 richiedono B. Una R distinta può nominare la riparazione
V1 con il medesimo candidato/tabella approvati: solo in quel caso usare
`app0` oppure `table` con `--gate R`, stessi SHA/offset/manifest e verifiche
indipendenti, app0 prima della tabella. Non è rollback byte-identico del legacy.
Un seed otadata diverso dall'estratto del backup non è accettato: l'eventuale
riparazione V1 con seed richiede il piano separato citato nella matrice, fuori
dal writer M0.1; nessun bypass CLI o nuova W è implicitamente autorizzato.
Ogni invocazione accetta una sola operazione e un solo artefatto/range fisso.
Prima della porta controlla backup A/B, hash, manifest e byte esatti autorizzati.
DTR/RTS sono inattivi prima dell'open; una sola connessione ROM `no-reset`,
nessuno stub. Prima di W impone e verifica `ESPLoader.WRITE_FLASH_ATTEMPTS=1`
e `esptool.loader.WRITE_BLOCK_ATTEMPTS=1` sui binding realmente utilizzati.
Un secondo FLASH_BEGIN, blocco duplicato, reset/finish o comando non ammesso
è rifiutato. Errore di protocollo o trasporto avvelena la sessione e propaga STOP;
SerialException non raggiunge la riconnessione/reset della libreria.

Nessuna compressione, encryption, diff/skip, erase-all o modifica degli header;
attach ROM diretto senza fallback XMC/reset NOR della CLI. La configurazione
volatile SPI/watchdog e la scansione delle risposte ROM non sono W in flash.
Il SYNC produce più risposte a una sola richiesta; `command()` può leggere fino
a 100 risposte per correlare un ACK, senza ritrasmettere FLASH_DATA. Anche il
primo errore durante SYNC impone STOP, senza il ciclo di retry del sync.
Il ROM scrive blocchi da 1024 byte con checksum esptool e padding FF invariati.
Verifica MD5 obbligatoria, seguita dalle verifiche indipendenti del runbook.

**STOP non certifica i byte scritti**: il settore potrebbe essere cancellato e
un blocco parziale già programmato. Conservare log, restare in ROM, acquisire
nuove letture indipendenti dello stato e riconciliarle prima di **qualsiasi W**,
anche su un'altra invocazione. Rinnovare B o R/R2 per la decisione risultante.
Non ritentare, non concatenare alla fase successiva, non avviare recovery da un
handler d'errore. I blocchi 13, 14 e R1/R2 sono passi separati per l'operatore,
non un file shell da lanciare integralmente.

### 13. W/R — Scrivere app0, poi verificare prima di proseguire

```bash
"$WRITE_PY" -I "$M0_WRITER" app0 "${WRITE_CONTEXT[@]}" --gate B \
  --offset 0x10000 --artifact "$APP" \
  --sha256 f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e \
  2>&1 | tee "$SESSION/write-app0.log"
"${ESP[@]}" verify-flash 0x10000 "$APP" | tee "$SESSION/verify-app0.log"
"${ESP[@]}" read-flash 0x10000 0x112000 "$SESSION/app0-readback.bin" \
  | tee "$SESSION/readback-app0.log"
cmp "$SESSION/app0-readback.bin" "$SESSION/plan/app0_sector_padded.bin"
sha256sum "$SESSION/app0-readback.bin" | tee "$SESSION/app0-readback.sha256.txt"
```

Atteso: immagine `[0x10000,0x121860)`, cancellazione implicita dei **274 settori**
`[0x10000,0x122000)`; padding finale FF. Scrittura/hash interno, verify-flash
indipendente e readback con `cmp` devono tutti riuscire. La vecchia tabella è
ancora installata; non bootare questo stato intermedio. STOP a ogni errore;
non scrivere la tabella per cercare di correggere un'app non verificata.

### 14. W/R — Scrivere la tabella per ultima, poi verificare

```bash
"$WRITE_PY" -I "$M0_WRITER" table "${WRITE_CONTEXT[@]}" --gate B \
  --offset 0x8000 --artifact "$TABLE" \
  --sha256 8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7 \
  2>&1 | tee "$SESSION/write-table.log"
"${ESP[@]}" verify-flash 0x8000 "$TABLE" | tee "$SESSION/verify-table.log"
"${ESP[@]}" read-flash 0x8000 0x1000 "$SESSION/table-readback.bin" \
  | tee "$SESSION/readback-table.log"
cmp "$SESSION/table-readback.bin" "$SESSION/plan/target_partition_sector.bin"
```

Atteso: W `[0x8000,0x8C00)`, un solo settore cancellato `[0x8000,0x9000)`.
Nessuna W su otadata, bootloader, default NVS, FFAT, coredump o matdog_nvs.
Non impartire un erase separato: i soli settori cancellati normalmente sono
quelli coperti dalle due scritture. STOP per errore, padding/hash diverso o log
incompleto; restare ROM, passare alla matrice di recupero.

### 15. R/L — Verifica globale prima di ogni boot

```bash
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-write-a.bin" | tee "$SESSION/post-write-a.log"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-write-b.bin" | tee "$SESSION/post-write-b.log"
cmp "$SESSION/post-write-a.bin" "$SESSION/post-write-b.bin"
python3 "$SCRIPTS/migration_m0.py" check-snapshot --backup "$A_IMAGE" \
  --backup-sha256 "$AUTHORIZED_BACKUP_SHA256" --snapshot "$SESSION/post-write-a.bin" \
  --binary "$APP" --manifest "$MANIFEST" --stage table
"${ESP[@]}" read-mac | tee "$SESSION/read-mac-after-write.log"
```

Atteso: immagini complete identiche e `SNAPSHOT_CHECK=PASS`; ogni byte coincide
con il backup originale salvo app e settore tabella come pianificati, compresi
otadata, default NVS, bootloader, app0 non riscritto, app1 nuovo, FFAT, NVS
dedicata e coredump. Questo controlla anche danni fuori dagli intervalli attesi.
Il SHA finale deve coincidere con `plan/report.json`; la tabella installata è
esattamente la V1. La stessa otadata seleziona app0 ora da 5 MiB.
Qualunque differenza impone STOP. Anche un PASS lascia il dispositivo in ROM:
**pronto per valutare C, primo boot ancora non autorizzato**.

### 16. Gate C — Autorizzare separatamente il primo boot diagnostico

Richiedere una autorizzazione nuova per: singolo boot USB_ONLY isolato;
monitoraggio per almeno 120 s; soli comandi del passo 17; successivo ritorno
manuale in ROM e letture del passo 18. Spiegare che un boot può avere normali
effetti flash del core/OTA e che non è un'operazione di sola lettura globale.
Non è consenso a SAVE, ACK, RECONCILE, RESTORE, Q0 PROMOTE, calibrazione,
authority/permit, servo traffic, movimento o OTA. Nessuna concessione automatica
di C per il solo fatto che la verifica del flash è riuscita.

### 17. B/R — Primo boot e diagnostica, senza mutazioni dell'operatore

Preparare il monitor/log; rilasciare BOOT, eseguire una sola transizione EN
autorizzata, attendere enumerazione. Aprire la porta con DTR/RTS inattivi,
115200 baud. Questo reader limitato alla whitelist è eseguibile **solo dopo C**:

```bash
MATDOG_M0_PORT="$PORT" MATDOG_M0_LOG_PATH="$SESSION/first-boot.log" python3 - <<'PY'
import os, serial, time
s = serial.Serial(port=None, baudrate=115200, timeout=0.2)
s.dtr = False
s.rts = False
s.port = os.environ["MATDOG_M0_PORT"]  # solo il by-id già verificato
commands = [b"@STATUS\n", b"@OTA STATUS\n", b"@CALIBRATION PERSIST STATUS\n",
            b"@CALIBRATION STATUS\n", b"@ACTUATOR STATUS\n", b"@AUTHORITY STATUS\n"]
with open(os.environ["MATDOG_M0_LOG_PATH"], "xb") as log:
    try:
        s.open()
        start = time.monotonic()
        index, next_command = 0, start + 3
        while time.monotonic() - start < 120:
            data = s.read(max(1, s.in_waiting))
            if data:
                log.write(data)
                log.flush()
            now = time.monotonic()
            if index < len(commands) and now >= next_command:
                log.write(b"\nHOST_DIAGNOSTIC_COMMAND " + commands[index])
                if s.write(commands[index]) != len(commands[index]):
                    raise RuntimeError("STOP: scrittura comando diagnostico incompleta")
                index += 1
                next_command = now + 3
    finally:
        s.close()
PY
```

Richiede pyserial, già usato dagli strumenti di sessione esistenti. Conservare
le risposte complete; se il distanziamento non basta o una risposta manca,
STOP e richiedere solo la diagnostica mancante, senza mutazioni/reset automatici.
Il banner può essere perso dalla CDC non bloccante: in tal
caso non dichiarare completata l'identificazione del profilo; STOP e predisporre
una nuova acquisizione diagnostica con autorizzazione esplicita al reboot.

| Evidenza | Risultato atteso / STOP |
|---|---|
| Banner identità | `build : be0c12979e5b`, USB_ONLY; servo_power/battery/led_rail `NO` |
| Partizione realmente in esecuzione | `app0 @ 0x010000`, size `0x500000`; divergenza = STOP |
| Tabella | Hash V1 del readback e slot/label coerenti nel firmware |
| Reset | Un solo boot atteso; nessun PANIC, WDT, BROWNOUT, riavvio o fault fatale per >=120 s |
| Boot persistence | `nvs=READY verdict=NO_RECORD available=0 motion_authorized=0` |
| PERSIST STATUS | `NVS=READY ESP_ERROR=0 PARTITION=matdog_nvs`, `LOAD_COUNT=1 LOAD_STATUS=NOT_FOUND`, entrambi i verdict `NO_RECORD` |
| Storage inizialmente vuoto | `MARKER=ABSENT`, slot A/B `ABSENT`, `CLASS=NEVER_INITIALIZED_OR_ERASED`, generazioni zero |
| Scritture servizio | `WRITE_STATE=OPEN`, uncertain/block flags zero; ACK e reconciliation non richiesti |
| Nessun restore | `RESTORE=NOT_IMPLEMENTED`, `CALIBRATION_AVAILABLE=0 MOTION_AUTHORIZED=0` |
| JointTransform | `limits_admitted=0 transforms_admitted=0`; nessun Q0 PROMOTE |
| Motion/authority | authority `NONE`; permit/operator/token `NO`; startup torque/motion/scan `DISABLED`; nessun executor/session attivo |
| OTA | `OTA_INGEST=DISABLED`; state di update inattivo, counters/stream zero; nessun NEW/PENDING |
| Wi-Fi | credenziali assenti nel candidato; non aggiungerle o avviare OTA come diagnostica |

`STORAGE_SAVE_ALLOWED=1` può essere normale con storage vuoto: è una possibilità
del record store, **non** autorizzazione al SAVE né al movimento. Non eseguire
neppure SAVE CHECK in questa sessione. LOAD è automatico al boot e STATUS lo
osserva; non esiste un comando operativo PERSIST LOAD da inventare.
NO_RECORD non è calibrazione valida e non distingue storicamente una partizione
mai usata da una completamente cancellata; qui la provenienza è il backup fresco.

`CalibrationRecordNvsBackend::begin()` usa il label esatto, non formatta e
riporta errori senza erase/retry. LOAD e STATUS non salvano marker/record e non
ammettono trasformazioni. Il core Arduino può invece cancellare la **prima**
NVS se corrotta: l'ordine `nvs` prima di `matdog_nvs` e il backup default NVS
sono essenziali. L'IDF può recuperare pagine NVS corrotte durante init;
la destinazione FF e il namespace assente evitano quel caso nel percorso M0.
I sorgenti esatti di [storage NVS](https://raw.githubusercontent.com/espressif/esp-idf/b774170ff46/components/nvs_flash/src/nvs_storage.cpp)
e [page manager](https://raw.githubusercontent.com/espressif/esp-idf/b774170ff46/components/nvs_flash/src/nvs_pagemanager.cpp)
sono archiviati nel bundle. Non promettere un boot universalmente privo di effetti flash.

### 18. B/R/L — Ritorno in ROM e chiusura diagnostica

Solo nel perimetro C registrato, tornare manualmente in ROM; nessun ulteriore
boot automatico. Ripetere MAC e leggere tabella, otadata e NVS dedicata:

```bash
"${ESP[@]}" read-mac | tee "$SESSION/read-mac-post-boot.log"
"${ESP[@]}" read-flash 0x8000 0x1000 "$SESSION/table-post-boot.bin"
"${ESP[@]}" read-flash 0xE000 0x2000 "$SESSION/otadata-post-boot.bin"
"${ESP[@]}" read-flash 0xFE0000 0x10000 "$SESSION/matdog-nvs-post-boot.bin"
cmp "$SESSION/table-post-boot.bin" "$SESSION/plan/target_partition_sector.bin"
cmp "$SESSION/otadata-post-boot.bin" "$SESSION/plan/otadata.bin"
cmp "$SESSION/matdog-nvs-post-boot.bin" "$SESSION/plan/destination_matdog_nvs.bin"
```

Atteso: tabella identica, otadata stabile preservata, destinazione ancora FF
per il namespace mai creato e le sole operazioni diagnostiche. Una differenza
richiede STOP, dump e analisi: non cancellare per fare coincidere il risultato.
Se sono avvenuti crash o interventi di recovery NVS, acquisire un nuovo full dump
prima di ogni recupero e spiegare ogni differenza da quello pre-boot.

### 19. L — Registrare l'esito, senza estendere il perimetro

Solo tutte le verifiche hardware completate permettono di dichiarare la sessione
di migrazione/boot diagnosticamente riuscita, con log e SHA. Distinguere
`MIGRATION_WRITES_VERIFIED`, `FIRST_BOOT_DIAGNOSTIC_PASS` e un eventuale
`ROLLBACK_VERIFIED`; nessuno dei tre è stato ottenuto in M0. Resta vietato
interpretarli come autorizzazione al movimento, al profilo alimentato o a P3b.

## Matrice di recupero e rollback — gate R separato

Alla prima anomalia restare/ritornare in ROM con rail isolati, verificare MAC e
security, salvare lo stato guasto prima di modificarlo. Un reset per recupero
richiede autorizzazione R se non già incluso esplicitamente nella sessione.
Verificare sempre SHA/dimensioni degli estratti contro il receipt del **nuovo**
backup dello stesso dispositivo. Il contratto vieta full restore nella
migrazione normale; il recupero completo è un'azione diversa, motivata e
autorizzata R. Mai usare un backup come immagine generica per un altro ESP32.

| Evento | Recupero minimo possibile in ROM | Condizione prima di boot / STOP |
|---|---|---|
| Errore W app0 | Leggere area danneggiata; riscrivere candidato identico solo con R, oppure ripristinare `legacy_app0.bin` | Verify e readback; tabella attuale deve essere nota; niente scrittura tabella per mascherare errore app |
| Verifica app/table fallita | Seconda lettura indipendente per distinguere trasporto da contenuto; confrontare file e alimentazione | Nessun retry W automatico; se discordanza persiste, ripristino della regione o STOP |
| Perdita di alimentazione | Rientro manuale ROM, MAC/security; dump stato corrente e determinazione di quali passi sono completi | Non proseguire da un log precedente incompleto; ripiano o rollback autorizzato |
| Tabella danneggiata | Ripristinare settore tabella legacy del backup, insieme all'app legacy se app0 era stato sostituito | Tabella per ultima; confronto integrale prima del boot; ROM non dipende dalla tabella |
| Nuova app non avvia | Boot log/crash dump; recuperare legacy app0 e settore tabella, otadata originale se alterata | App1 nuovo è vuoto: non promettere fallback o rollback OTA |
| otadata incoerente | Archiviare entrambi i settori; ripristinare `otadata.bin` originale | CRC/stati/selezione verificati offline; non scegliere la seq più alta senza validità |
| otadata originale non utilizzabile dopo un guasto | Seed core noto solo come riparazione V1 separata, app0 candidato già verificato | Non è rollback del firmware originale; autorizzazione R specifica e prova del boot successivo |
| NVS dedicata non inizializzabile | STATUS/errori e dump 64 KiB; verificare label, geometria e contenuto | Nessun erase automatico, SAVE/ACK/reconcile; preferire rollback legacy; modifica NVS fuori dal contratto M0 |
| Default NVS cambiata dal core / coredump nuovo | Salvare evidenza; per rollback byte-identico restaurare anche le regioni effettivamente cambiate con R | Non dichiarare identico un rollback limitato a app/table se questi byte differiscono |
| USB assente dopo boot | Controllare cavo/rail/by-id; ingresso manuale BOOT+EN in ROM con R | Nessuna scansione seguita da W su tty arbitrario; UART0 3V3 solo con collegamento verificato e nuova autorizzazione |
| Corruzione estesa / bootloader differente o danneggiato | Ripristino completo del backup fresco, solo stesso chip e security compatibile | Ultima opzione R; se backup non verificato o identità incerta, STOP |

### R1. W/R — Ripristino delle singole regioni

Esempio di rollback dopo le due W normali: legacy app0 per prima, otadata solo
se la lettura ne dimostra l'alterazione, settore tabella legacy per ultimo.
BOOT deve restare basso. Ogni riga W va trattata come passo distinto:

```bash
"$WRITE_PY" -I "$M0_WRITER" r1-app0 "${WRITE_CONTEXT[@]}" --gate R \
  --offset 0x10000 --artifact "$SESSION/plan/legacy_app0.bin" \
  --sha256 "$(sha256sum "$SESSION/plan/legacy_app0.bin" | cut -d' ' -f1)" \
  2>&1 | tee "$SESSION/write-r1-app0.log"
"${ESP[@]}" verify-flash 0x10000 "$SESSION/plan/legacy_app0.bin"
"${ESP[@]}" read-flash 0x10000 0x300000 "$SESSION/recovery-app0.bin"
cmp "$SESSION/recovery-app0.bin" "$SESSION/plan/legacy_app0.bin"

# SOLO se necessario e incluso nell'autorizzazione R:
"$WRITE_PY" -I "$M0_WRITER" r1-otadata "${WRITE_CONTEXT[@]}" --gate R \
  --offset 0xE000 --artifact "$SESSION/plan/otadata.bin" \
  --sha256 "$(sha256sum "$SESSION/plan/otadata.bin" | cut -d' ' -f1)" \
  2>&1 | tee "$SESSION/write-r1-otadata.log"
"${ESP[@]}" verify-flash 0xE000 "$SESSION/plan/otadata.bin"
"${ESP[@]}" read-flash 0xE000 0x2000 "$SESSION/recovery-otadata.bin"
cmp "$SESSION/recovery-otadata.bin" "$SESSION/plan/otadata.bin"

"$WRITE_PY" -I "$M0_WRITER" r1-table "${WRITE_CONTEXT[@]}" --gate R \
  --offset 0x8000 --artifact "$SESSION/plan/partition_sector.bin" \
  --sha256 "$(sha256sum "$SESSION/plan/partition_sector.bin" | cut -d' ' -f1)" \
  2>&1 | tee "$SESSION/write-r1-table.log"
"${ESP[@]}" verify-flash 0x8000 "$SESSION/plan/partition_sector.bin"
"${ESP[@]}" read-flash 0x8000 0x1000 "$SESSION/recovery-table.bin"
cmp "$SESSION/recovery-table.bin" "$SESSION/plan/partition_sector.bin"
```

Il writer ammette inoltre `r1-default-nvs` `[0x9000,0xE000)` e
`r1-coredump` `[0xFF0000,0x1000000)` **solo con R nominativa** se le letture
ne dimostrano l'alterazione: artefatto esatto dal backup A/B, SHA registrato,
verify-flash e readback/cmp separati. Altre regioni richiedono R2; nessun range
libero. Queste W sono recovery, mai parte delle due W ordinarie.

Se necessario e incluso nella distinta R, **prima del settore tabella finale**:

```bash
# SOLO default NVS alterata e inclusa nella R nominativa:
"$WRITE_PY" -I "$M0_WRITER" r1-default-nvs "${WRITE_CONTEXT[@]}" --gate R \
  --offset 0x9000 --artifact "$SESSION/plan/default_nvs.bin" \
  --sha256 "$(sha256sum "$SESSION/plan/default_nvs.bin" | cut -d' ' -f1)" \
  2>&1 | tee "$SESSION/write-r1-default-nvs.log"
"${ESP[@]}" verify-flash 0x9000 "$SESSION/plan/default_nvs.bin"
"${ESP[@]}" read-flash 0x9000 0x5000 "$SESSION/recovery-default-nvs.bin"
cmp "$SESSION/recovery-default-nvs.bin" "$SESSION/plan/default_nvs.bin"
```

```bash
# SOLO coredump alterato e incluso nella R nominativa:
"$WRITE_PY" -I "$M0_WRITER" r1-coredump "${WRITE_CONTEXT[@]}" --gate R \
  --offset 0xFF0000 --artifact "$SESSION/plan/coredump.bin" \
  --sha256 "$(sha256sum "$SESSION/plan/coredump.bin" | cut -d' ' -f1)" \
  2>&1 | tee "$SESSION/write-r1-coredump.log"
"${ESP[@]}" verify-flash 0xFF0000 "$SESSION/plan/coredump.bin"
"${ESP[@]}" read-flash 0xFF0000 0x10000 "$SESSION/recovery-coredump.bin"
cmp "$SESSION/recovery-coredump.bin" "$SESSION/plan/coredump.bin"
```

I blocchi condizionati non sono uno script da eseguire in blocco. Nessun boot
intermedio; STOP immediato a ogni W/R/cmp fallito. Una riparazione per mantenere
V1 può riscrivere il candidato o la sua tabella invece dell'estratto legacy,
ma non è un rollback e deve avere un piano di coerenza esplicito.

Dopo il recupero regionale leggere due full dump indipendenti, `cmp` fra loro
e contro `$A_IMAGE`; verificare MAC, tabella legacy, otadata originale e SHA.
Se divergono da `$A_IMAGE`, non dichiarare rollback byte-identico: identificare
le altre regioni cambiate e autorizzarne il recupero prima di procedere.
Il boot legacy successivo è un altro B autorizzato; confermare firmware/profilo,
slot effettivo, stato OTA e assenza di reset. Solo allora registrare
`ROLLBACK_VERIFIED`, con le prove, non per il solo exit code del flash.

### R2. W/R — Ripristino completo, distinto dalla migrazione

Se più regioni sono cambiate/corrotte, la riparazione locale non è verificabile
o il bootloader stesso è danneggiato, l'operatore può autorizzare R2. Confermare
di nuovo MAC **del backup**, security/assenza di encryption e il SHA approvato;
un'immagine flash non ripristina eFuse e non è portabile fra dispositivi.
Il ripristino riscrive anche bootloader e NVS: questa è la ragione per una
autorizzazione e una giustificazione separate. Non usare `erase-flash`.

```bash
test "$(stat -c%s "$A_IMAGE")" -eq 16777216
test "$(sha256sum "$A_IMAGE" | cut -d' ' -f1)" = "$AUTHORIZED_BACKUP_SHA256"
"$WRITE_PY" -I "$M0_WRITER" r2-full "${WRITE_CONTEXT[@]}" --gate R2 \
  --offset 0x0 --artifact "$A_IMAGE" --sha256 "$AUTHORIZED_BACKUP_SHA256" \
  2>&1 | tee "$SESSION/write-r2-full.log"
"${ESP[@]}" verify-flash 0x0 "$A_IMAGE"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/recovery-full-a.bin"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/recovery-full-b.bin"
cmp "$SESSION/recovery-full-a.bin" "$SESSION/recovery-full-b.bin"
cmp "$SESSION/recovery-full-a.bin" "$A_IMAGE"
```

Atteso: dimensione, SHA e **tutti i byte** del backup originale; ogni errore
impone STOP in ROM. Successivo boot legacy diagnostico autorizzato e verificato
come R1. Il file qui è il backup device-specific, non un `.merged.bin` di build.

</details>

## Checklist attuale M0.2 — separazione dei gate

- [x] Hardware reale documentato: batteria/DALY KEY ON, TECNOIOT→ESP32, rail powered;
      porta USB esterna senza VBUS; USB-C non usata; BOOT/EN non accessibili.
- [x] Robot sostenuto, zampe libere, disgiuntore accessibile; 13 torque/speed/current
      zero pre-ROM riferiti dall'operatore, senza estendere la validità post-reset.
- [x] Ingresso usb-reset già verificato; successive letture no-reset; identità/security note.
- [x] A/B distinti, completi, identici, SHA fresco atteso; nessuna nuova acquisizione.
- [x] Regioni archiviate offline: tabella legacy, bootloader noto/padding, otadata
      stabile/app0, FFAT/app1 nuovo/destinazione NVS interamente FF.
- [x] App legacy dfcecb670d05 presente, checksum/digest validi; source be0c129 immutabile.
- [x] Manifest/immagini offline USB e ROBOT attestati; rifiuto powered del checker
      attuale preservato. Il PASS USB locale non è un piano di migrazione hardware.
- [ ] Recupero fisico disponibile/qualificato indipendente da app/tabella: **BLOCKED**.
- [ ] Secondo supporto riletto e receipt device-specific per un eventuale B/R futuro.
- [ ] Piano/writer/test powered e readback da 275 settori qualificati senza bypass.
- [ ] B distinto con recupero pronto e candidato/backup nominativi: **non concesso**.
- [ ] C powered distinto con diagnostica e ritorno ROM realizzabili: **non concesso**.
- [ ] Eventuale R/R2 distinta, stesso dispositivo, verifica integrale prima di
      dichiarare rollback: **nessun recupero eseguito o certificato fisicamente**.

## Validazione offline e limiti residui

Il nuovo `scripts/migration_m0.py` è il minimo controllo mancante: legge solo
file e scrive estratti/report in una **nuova directory**. Riusa i quattro moduli
esistenti; non modifica build.sh, upload.sh, flash_app_only.sh, OTA o P3a.
Non garantisce la verità di MAC/orari/runtime dichiarati: lo provano i log e
l'operatore. La validità interna dell'immagine ESP è un gate separato di
`esptool image-info`, oltre a SHA/Manifest, non una seconda implementazione
di image parser. Nessun PASS locale autorizza hardware.

Test sintetici eseguibili senza robot:

```bash
python3 "$SCRIPTS/tests/test_migration_m0.py"
MATDOG_M0_USB="$USB" "$WRITE_PY" -I "$SCRIPTS/tests/test_migration_m0_write.py"
python3 "$SCRIPTS/tests/test_matdog_layout.py"
python3 "$SCRIPTS/tests/test_build_manifest.py"
python3 "$SCRIPTS/tests/test_backup_gate_logic.py"
python3 "$SCRIPTS/tests/test_ota_partition_logic.py"
python3 "$SCRIPTS/static_audit.py"
```

Risultati e tracciabilità effettivi sono nel
[rapporto M0](../../09_Logs/Development_Log/2026-10-02_M0_FLASH_LAYOUT_OFFLINE.md).
I test M0 coprono legacy corretto, layout inatteso/corrotto, slot1/fallback,
manifest/profilo/OTA/core configuration incompatibili, NVS/FFAT/app1 non vuoti,
hash sbagliati, oversize/staging fuori dal legacy, otadata ambiguo/instabile,
backup assente/non verificato/discordante e danni fuori regione nel readback.

Per il correttivo e i test reali API con trasporto simulato vedere il
[rapporto M0.1](../../09_Logs/Development_Log/2026-10-02_M0_1_MIGRATION_WRITE_HARDENING.md).
Il criterio del passo 18 resta invariato: 64 KiB `matdog_nvs` interamente FF
nel flusso USB_ONLY/LOAD/STATUS approvato; nessuna modifica al firmware NVS.

Rischi residui M0.2: identità/stato riferiti al Gate A, non a un futuro boot;
nessun recupero da guasto dimostrato con BOOT/EN coperti; servo powered e stato
torque-off non garantito dopo reset dei servo; USB senza VBUS e alimentazione
comune alle rail; indipendenza delle letture attestata dall'operatore/log;
stabilità elettrica/USB e ritorno ROM dopo guasto da qualificare; durability NVS e interruzioni reali di potenza non provate offline;
app1 vuoto non offre fallback; flash/table W non atomiche; primo boot può
eseguire recovery core/NVS; credenziali volutamente assenti; warning SCServo
preesistenti; perdita del banner CDC possibile. Una condizione fuori dal
percorso M0 richiede una nuova decisione concreta, non un bypass.

Per stato del backup reale, analisi regionale, confronto profili e verifiche:
[rapporto M0.2](../../09_Logs/Development_Log/2026-10-02_M0_2_HARDWARE_ALIGNMENT.md).

**Chiusura M0.2:** backup Gate A verificato localmente e runbook allineato;
**recupero non pronto, Gate B BLOCKED, flash non autorizzato**. Il Gate A hardware
è stato effettuato dall'operatore; questo correttivo non ha effettuato accessi
al robot, flash, erase, OTA, reset, movimento, push, PR o merge.
