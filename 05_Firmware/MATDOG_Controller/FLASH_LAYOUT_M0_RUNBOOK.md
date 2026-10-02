# MATDOG M0 — Migrazione flash e recupero, validati offline

Data: 2026-10-02. Stato: **M0 completato offline; nessuna autorizzazione hardware**.
Base approvata: `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`, branch di ingresso
`feat/calibration-persistence-record-store-v1`, inizialmente pulito. P3a resta invariato.
Questo documento pianifica una sessione futura sullo stesso ESP32-S3; non ne attesta
lo stato attuale. I comandi hardware sotto sono istruzioni condizionate, **non eseguite**.

Classi: **L** = file locali; **R** = lettura del dispositivo, senza scrittura flash;
**W** = scrittura con cancellazione dei settori indirizzati; **B** = boot/reset/transizione
ROM, che può attivare le normali scritture del firmware/core. R non significa
assenza di interazione hardware. M0 ha eseguito esclusivamente L.

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

**Candidato del primo boot: USB_ONLY.** Servo power, batteria/DALY e rail LED
sono attesi spenti; questo coincide con l'isolamento fisico richiesto e riduce
le aspettative di periferiche alimentate. Il profilo non interrompe fisicamente
l'alimentazione e non sostituisce l'isolamento. ROBOT_POWERED attende tutti quei
rail attivi: è una build di confronto offline, non il candidato del primo boot.
Questa scelta **non autorizza alcun caricamento**.

## Bootloader e otadata effettivi

Il backup storico del 29 settembre contiene il bootloader generato dal core
corrente, byte per byte, più padding FF fino a `0x8000`. La compatibilità sul
dispositivo futuro va riconfermata dal backup nuovo: `migration_m0.py plan`
rifiuta ogni differenza, senza proporre un aggiornamento del bootloader.
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
APP="$USB/MATDOG_Controller.ino.bin"
TABLE="$USB/MATDOG_Controller.ino.partitions.bin"
MANIFEST="$USB/matdog_build_manifest.txt"
ESPTOOL="$HOME/.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool"
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
Il nuovo backup userà directory e nomi diversi, senza sovrascrittura.

### 3. L — Rivalidare il candidato e il sorgente esatto

```bash
test "$(git -C "$M0_ROOT/source-usb" rev-parse HEAD)" = be0c12979e5b4b8ddc9dd21772d0d106e4358f4a
test -z "$(git -C "$M0_ROOT/source-usb" status --porcelain)"
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

### 4. Gate A — Autorizzazione separata a backup/preflight

Prima di qualsiasi R/B richiedere una registrazione dell'operatore con sessione,
dispositivo/MAC atteso, condizioni elettriche, letture diagnostiche consentite,
ingresso manuale in ROM e due acquisizioni complete. L'autorizzazione A non
consente W, erase, OTA, primo boot del candidato o attuazione. M0 non la concede.

### 5. B/R — Isolare e identificare fisicamente

ESP32 alimentato **solo USB stabile**; robot sostenuto meccanicamente. Scollegare
batteria, caricabatterie, alimentatore esterno, ramo servo e rail LED; evitare
retorni da step-down o altri controller. Verificare con misura che i rail degli
attuatori siano a 0 V e scarichi. Isolare i segnali UART servo GPIO17/18 e i
segnali LED/DALY verso periferiche spente per evitare back-power dai GPIO; BNO085
sulla 3V3 ESP32 può restare collegato. Documentare foto/schema e misure.
Non affidarsi al profilo software, alla chiave DALY o a un comando SAFE_OFF come
prova di isolamento. Nessun comando servo è parte di questo preflight.

Un solo proprietario USB; chiudere monitor, viewer, servizi di calibrazione,
upload automatici e altri processi seriali. Usare il by-id, non un ttyACM
ipotizzato. MAC storico atteso `14:c1:9f:22:75:94`; un diverso dispositivo
impone STOP e nuova identificazione, non modifica automatica del valore atteso.

### 6. R/B — Rilevare firmware e slot realmente in esecuzione

Se il firmware legacy è già in esecuzione, aprire un reader senza reset
intenzionale e richiedere solo `@STATUS` e `@OTA STATUS`. Registrare banner,
BUILD_ID, profilo, `OTA_IMAGE running=... @...`, stato OTA/reset e log grezzo.
Aprire con DTR/RTS inattivi impostati **prima** dell'open; un reset inatteso
impone STOP. Non usare una utility di calibrazione come reader.

Se si parte direttamente in ROM, il dump prova lo slot *selezionato*, non quello
che era realmente in esecuzione: non inventare quest'ultima osservazione.
Occorre autorizzare esplicitamente anche un boot diagnostico del firmware
legacy isolato, salvare prima uno snapshot ROM, acquisire il runtime, tornare
in ROM e acquisire la coppia finale di backup **dopo** quel boot. Conservare
anche lo snapshot precedente. Nessuna migrazione senza questa evidenza.

Registrare `CURRENT_OFFSET` e `CURRENT_BUILD_ID` dai dati ottenuti. Lo slot
deve essere app0 legacy `0x10000`, dimensione `0x300000`. Pending verification,
rollback, fallback o divergenza fra runtime e selezione del dump impongono STOP.

### 7. B/R — Entrare in ROM e controllare identità/security

Ingresso manuale: GPIO0/BOOT basso durante EN/reset, come da
[selezione boot ESP32-S3 Espressif](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html).
Mantenere la condizione di ROM anche in caso di reset accidentale fino al gate C.
Non cortocircuitare linee sconosciute. Se BOOT/EN o l'isolamento non sono
accessibili, STOP: non sostituire la procedura con reset automatici.

```bash
PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
SESSION="$HOME/MATDOG/backups/esp32/m0-$(date -u +%Y%m%dT%H%M%SZ)-14c19f227594"
mkdir "$SESSION"
ESP=("$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 115200 \
     --before no-reset --after no-reset --no-stub)
"${ESP[@]}" read-mac | tee "$SESSION/read-mac-before.log"
"${ESP[@]}" flash-id | tee "$SESSION/flash-id.log"
"${ESP[@]}" get-security-info | tee "$SESSION/security-info.log"
```

Atteso: chip ESP32-S3, MAC esatto, flash rilevata 16 MiB. Registrare USB serial,
chip revision, JEDEC ID, data/ora e tool/versione; eFuse security coerente con
secure boot/flash encryption disabilitati e ROM read/write disponibile.
STOP se security è ignota/attiva, il chip/flash non coincidono, porta assente
o readback non consentito. Non usare `--force`, non programmare eFuse.
Le opzioni impediscono i reset impliciti e usano ROM senza stub, secondo
[le opzioni esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/advanced-options.html).
Un errore di connessione non autorizza un reset o una diversa modalità.

### 8. R — Due acquisizioni complete indipendenti

```bash
A_IMAGE="$SESSION/read-a-16m.bin"
B_IMAGE="$SESSION/read-b-16m.bin"
test ! -e "$A_IMAGE" && test ! -e "$B_IMAGE"
"${ESP[@]}" read-flash 0x0 0x1000000 "$A_IMAGE" | tee "$SESSION/read-a.log"
"${ESP[@]}" read-mac | tee "$SESSION/read-mac-between.log"
"${ESP[@]}" read-flash 0x0 0x1000000 "$B_IMAGE" | tee "$SESSION/read-b.log"
test "$(stat -c%s "$A_IMAGE")" -eq 16777216
test "$(stat -c%s "$B_IMAGE")" -eq 16777216
cmp "$A_IMAGE" "$B_IMAGE"
sha256sum "$A_IMAGE" "$B_IMAGE" | tee "$SESSION/fullflash.sha256.txt"
NEW_BACKUP_SHA256="$(sha256sum "$A_IMAGE" | cut -d' ' -f1)"
```

Due processi/read-flash realmente distinti, con log e orari; mai una copia,
hardlink o un rehash della prima acquisizione. Nessun boot/W fra le letture.
Atteso: 16 MiB ciascuna, byte identici e SHA identico; file leggibili integralmente.
Errori USB, acquisizione parziale o un solo byte diverso impongono STOP.
Archiviare i tentativi falliti con nome separato; risolvere connessione/alimentazione
e rifare **due** acquisizioni, senza scegliere per maggioranza né concatenare
pezzi senza una verifica indipendente completa. Il precedente NO_STUB_FULL
evita una fragilità già documentata delle letture con stub/ri-enumerazione.

### 9. L — Estrarre, verificare, archiviare

```bash
python3 "$SCRIPTS/migration_m0.py" archive --backup "$A_IMAGE" --repeat "$B_IMAGE" \
  --backup-sha256 "$NEW_BACKUP_SHA256" --out "$SESSION/regions"
"$ESPTOOL" --chip esp32s3 image-info "$SESSION/regions/legacy_app0.bin" \
  | tee "$SESSION/installed-app0-image-info.log"
"$ESPTOOL" --chip esp32s3 image-info "$USB/MATDOG_Controller.ino.bootloader.bin" \
  | tee "$SESSION/bootloader-reference-image-info.log"
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

### 10. L — Preparare il piano tecnico sul nuovo backup

```bash
CURRENT_OFFSET="${CURRENT_OFFSET:?richiesto offset dal runtime legacy}"
CURRENT_BUILD_ID="${CURRENT_BUILD_ID:?richiesto BUILD_ID dal runtime legacy}"
OBSERVED_MAC="${OBSERVED_MAC:?richiesto MAC da read-mac}"
python3 "$SCRIPTS/migration_m0.py" plan --backup "$A_IMAGE" --repeat "$B_IMAGE" \
  --backup-sha256 "$NEW_BACKUP_SHA256" --sdkconfig "$USB/sdkconfig" \
  --bootloader "$USB/MATDOG_Controller.ino.bootloader.bin" \
  --running-offset "$CURRENT_OFFSET" --installed-build-id "$CURRENT_BUILD_ID" \
  --expected-mac 14:c1:9f:22:75:94 --observed-mac "$OBSERVED_MAC" \
  --binary "$APP" --manifest "$MANIFEST" --out "$SESSION/plan"
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

### 11. Gate B — Autorizzare separatamente la migrazione

Richiedere accettazione esplicita di: stesso dispositivo/sessione; receipt e
**SHA del nuovo backup**; firmware USB SHA della tabella artefatti; tabella V1
SHA; perdita della semantica/posizione FFAT; app1 nuovo vuoto senza fallback;
sole due W del piano; nessun erase/bootloader/otadata/NVS write; stop e recupero.
Gli elementi di `MIGRATION_PRECONDITIONS` devono essere tutti esplicitamente
soddisfatti. Una autorizzazione al backup, al build o a un precedente flash
non soddisfa questo gate. Registrare `AUTHORIZED_BACKUP_SHA256` dal receipt
approvato, non assegnarlo automaticamente dal file che si sta per usare.

### 12. R/L — Ultimo controllo, senza scritture automatiche

Rivalidare MAC, sorgente/manifest del passo 3, receipt e copie backup;
`test "$NEW_BACKUP_SHA256" = "$AUTHORIZED_BACKUP_SHA256"`.
La coppia di backup deve appartenere all'attuale sessione ROM congelata:
se c'è stato un boot o una modifica nel frattempo, STOP e nuovi backup/piano/gate B.
Confermare visivamente BOOT basso/ROM, rail isolati, cavo e alimentazione stabili.
Non eseguire `flash_app_only.sh`: continua a rifiutare il legacy; il suo wrapper
usa reset ordinari e non governa questa transazione. `upload.sh` resta uno stub
di rifiuto. Nessun preflight chiama W come effetto collaterale.

### 13. W/R — Scrivere app0, poi verificare prima di proseguire

```bash
"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x10000 "$APP" | tee "$SESSION/write-app0.log"
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
"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x8000 "$TABLE" | tee "$SESSION/write-table.log"
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
"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x10000 "$SESSION/plan/legacy_app0.bin"
"${ESP[@]}" verify-flash 0x10000 "$SESSION/plan/legacy_app0.bin"
"${ESP[@]}" read-flash 0x10000 0x300000 "$SESSION/recovery-app0.bin"
cmp "$SESSION/recovery-app0.bin" "$SESSION/plan/legacy_app0.bin"

# SOLO se necessario e incluso nell'autorizzazione R:
"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0xE000 "$SESSION/plan/otadata.bin"
"${ESP[@]}" verify-flash 0xE000 "$SESSION/plan/otadata.bin"
"${ESP[@]}" read-flash 0xE000 0x2000 "$SESSION/recovery-otadata.bin"
cmp "$SESSION/recovery-otadata.bin" "$SESSION/plan/otadata.bin"

"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x8000 "$SESSION/plan/partition_sector.bin"
"${ESP[@]}" verify-flash 0x8000 "$SESSION/plan/partition_sector.bin"
"${ESP[@]}" read-flash 0x8000 0x1000 "$SESSION/recovery-table.bin"
cmp "$SESSION/recovery-table.bin" "$SESSION/plan/partition_sector.bin"
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
"${ESP[@]}" write-flash --flash-mode keep --flash-freq keep --flash-size keep 0x0 "$A_IMAGE"
"${ESP[@]}" verify-flash 0x0 "$A_IMAGE"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/recovery-full-a.bin"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/recovery-full-b.bin"
cmp "$SESSION/recovery-full-a.bin" "$SESSION/recovery-full-b.bin"
cmp "$SESSION/recovery-full-a.bin" "$A_IMAGE"
```

Atteso: dimensione, SHA e **tutti i byte** del backup originale; ogni errore
impone STOP in ROM. Successivo boot legacy diagnostico autorizzato e verificato
come R1. Il file qui è il backup device-specific, non un `.merged.bin` di build.

## Checklist del preflight e autorizzazioni

- [ ] A registrata, con sessione, MAC, R e transizioni ROM/legacy diagnostico previste.
- [ ] Solo USB; batteria/caricatore/alimentatore/rail attuatori isolati; misura 0 V,
      segnali verso periferiche spente isolati, robot sostenuto.
- [ ] Porta by-id e chip/MAC/seriale/JEDEC/revision verificati; security nota e compatibile.
- [ ] Nessun altro proprietario USB; ROM stabile, nessun reset implicito/stub.
- [ ] Firmware/profilo e slot realmente in esecuzione registrati e riconciliati col dump.
- [ ] Due acquisizioni complete indipendenti, 16 MiB, byte/SHA identici, log/orari leggibili.
- [ ] Estratti e default NVS verificati; copia del backup su secondo supporto riletta.
- [ ] Backup 29 settembre preservato; nuovo receipt legato allo stesso dispositivo.
- [ ] Tabella legacy esatta; bootloader interamente uguale al riferimento; config OTA nota.
- [ ] CRC/stati entrambi i settori noti e stabili; app0 selezionato e realmente eseguito.
- [ ] Tutta la FFAT legacy, nuovo app1 e tutti i 64 KiB destinazione NVS verificati FF.
- [ ] Candidato CLEAN commit approvato, BUILD_ID incorporato, USB_ONLY, OTA ingest 0,
      FQBN/core/OPI, firmware/table/manifest SHA e intervalli verificati.
- [ ] Piano locale PASS e nessuna W nel preflight; perdita semantica FFAT accettata.
- [ ] B registrata con SHA backup/candidato/table, sole due W, isolamento e recupero.
- [ ] Dopo W: verify-flash, readback e snapshot globale PASS; ancora ROM.
- [ ] C distinta, diagnostica e transizioni autorizzate; nessuna concessione a storage/motion.
- [ ] Eventuale R/R2 distinta, regioni e motivo dichiarati, stesso dispositivo, verify e boot
      verificati prima di ogni dichiarazione di rollback riuscito.

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

Rischi residui: stato presente del chip sconosciuto; indipendenza delle letture
non dimostrabile da due file soli; affidabilità elettrica/USB e accesso ROM da
provare; durability NVS e interruzioni reali di potenza non provate offline;
app1 vuoto non offre fallback; flash/table W non atomiche; primo boot può
eseguire recovery core/NVS; credenziali volutamente assenti; warning SCServo
preesistenti; perdita del banner CDC possibile. Una condizione fuori dal
percorso M0 richiede una nuova decisione concreta, non un bypass.

**Chiusura M0:** documentazione, build, analisi storica e test locali completati.
Autorizzazioni A/B/C/R mancanti e volutamente separate; nessun accesso al robot,
flash, erase, OTA, reset, movimento, push, PR o merge eseguito.
