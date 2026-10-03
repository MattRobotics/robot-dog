# MATDOG M0.4 — preparazione software e procedura ROBOT_POWERED

Data: 2026-10-03. Checkpoint di ingresso `2757604400db34e3107abacd994ca6fb9d0bb515`,
branch `feat/calibration-persistence-record-store-v1`. Sorgente firmware
immutabile `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`, BUILD_ID `be0c12979e5b`.

Questo è il percorso corrente per batteria → DALY KEY ON → servo/LED e
TECNOIOT → ESP32-S3, USB GPIO19/20/GND verso ASUS senza VBUS. Non richiede
nuovo cablaggio, isolamento delle rail o USB-C. La perdita completa di USB è
un rischio accettato dall'operatore; il recupero manuale può richiedere apertura
cover e BOOT/EN. JTAG e prova preventiva di ogni guasto non sono gate.
I gate di alimentazione, periferiche, integrità e corretta scrittura restano
obbligatori. Il nuovo app1 è vuoto e non offre fallback.

**La preparazione software non autorizza alcun accesso hardware.** Nessun
comando hardware qui riportato è stato eseguito in M0.4. Le sezioni successive
al controllo locale richiedono incarichi e autorizzazioni distinti: preflight
hardware, due W nominate, primo boot, eventuale recovery. `--gate B` registra
il contesto; non concede l'autorizzazione. Non eseguire l'intero documento come
script, né concatenare fasi W o recovery.

## Pacchetto operativo locale

`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_4_ROBOT_POWERED_2757604_20261003`

Contiene `artifacts/` (copie identiche degli artefatti approvati), `tools/`
(writer e verificatori con dipendenze), `plan/` (estratti, padding, report),
manifest operativo `package-manifest.json`, `package-files.sha256`,
`context.env`, `preflight-local.sh`, attestazioni di provenienza e log dei test.
Il venv API 5.3.1 già qualificato M0.1 è riutilizzato nel suo percorso originale;
il pacchetto dichiara questa dipendenza e non richiede installazioni né build.
I backup A/B originali sono soltanto letti, senza nuove copie integrali nel
pacchetto e senza modifiche. Non usare un `.merged.bin`.

| Artefatto | Byte | SHA-256 |
|---|---:|---|
| ROBOT_POWERED `MATDOG_Controller.ino.bin` | 1.123.472 | `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0` |
| Tabella `MATDOG_16M_2x5M_NVS_V1` | 3.072 | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |
| Manifest V2 ROBOT_POWERED | 637 | `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48` |
| Backup A e B, ciascuno | 16.777.216 | `856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7` |

Il manifest conserva FQBN N16R8/OPI, flash 16M, hardware CDC, OTA ingest 0,
layout V1 e slot 5 MiB. Il writer ammette solo le coppie app/manifest esatte
archiviate per USB_ONLY e ROBOT_POWERED; il profilo powered deve essere nominato
esplicitamente. Non esiste override per firmware arbitrari. La verifica del
profilo non dipende dal solo BUILD_ID condiviso tra le due immagini.

## Preflight locale, eseguibile senza robot

```bash
bash /home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_4_ROBOT_POWERED_2757604_20261003/preflight-local.sh
```

Verifica SHA del pacchetto/tool, A/B, manifest, layout, geometria e `prepare`
del writer per app0 e tabella, senza chiamare `execute`, costruire una porta o
effettuare reset. Atteso `FILE_PREFLIGHT=PASS; HARDWARE_IO=NO;
AUTHORIZATION_GRANTED=NO`, checksum e validation hash immagine validi.
Il piano file-only già generato resta distinto dallo stato hardware corrente.

## Primo avvio qualificato nel software

`Controller::begin` azzera authority a NONE, non apre sessioni, revoca permit
e motion authorization; lega geometria senza ammettere limiti/trasformazioni.
La persistence inizializza soltanto `matdog_nvs` e fa LOAD in lettura, scarta
il record decodificato e non fa RESTORE/promozione. Calibrazioni nella NVS
legacy non vengono importate. Record non confermati producono verdetti di
blocco; anche un record confermato e disponibile non autorizza movimento.
La NVS dedicata è FF nel backup: primo LOAD atteso READY/NO_RECORD,
`motion_authorized=0`. Non vengono fatti erase/format su errori NVS.

ServoBus inizializza UART senza scan, census, torque-on o target automatici;
anche update non avvia scan da solo. Il firmware non applica SAFE_OFF
automaticamente: lo stato fisico dei 13 servo va verificato prima della ROM.
DALY inizializza UART/polling, senza scritture di configurazione al boot; il
ring alimentato riceve OFF iniziale e poi segue la policy di stato. Il profilo
dichiara servo/batteria/rail LED disponibili, coerentemente con l'hardware.
Nessuna di queste conclusioni sostituisce il preflight powered fisico.

## Successivo preflight hardware — da autorizzare separatamente

1. Robot sostenuto, zampe libere, disgiuntore accessibile; batteria/DALY KEY ON
   e TECNOIOT stabili, nessun undervoltage, brownout, riavvio o perdita USB.
   Registrare tensione batteria, uscita TECNOIOT 5 V e stabilità nelle condizioni
   di carico reali. Non dichiarare rail a 0 V o isolamento che non esistono.
2. Se il legacy è in esecuzione, aprire la CDC con DTR/RTS inattivi e senza
   reset automatico; salvare un transcript completo di `@STATUS`, `@MODE STATUS`,
   `@AUTHORITY STATUS`, `@CALIBRATION STATUS`, `@OTA STATUS`, `@BMS STATUS`,
   `@LED STATUS`. Atteso runtime `dfcecb670d05`, ROBOT_POWERED, app0 `0x10000`,
   authority NONE, nessuna sessione/permit/motion attivi e OTA ingest disabilitato.
3. Verificare MAINTENANCE. Se occorre, il preflight autorizzato deve includere
   esplicitamente `@MODE MAINTENANCE`, che non assegna authority né torque.
   Una alla volta: `@SERVO READ 11`, `12`, `13`, `21`, `22`, `23`, `31`, `32`,
   `33`, `41`, `42`, `43`, `51` (sempre con prefisso `@SERVO READ`). Richiedere
   risposta valida da tutti e torque=0, speed=0, current=0. Una condizione
   diversa impone STOP: non aggiungere SAFE_OFF, attuazione o correzioni alla
   whitelist. Chiudere il monitor senza reset prima dell'ingresso ROM.
4. Se il robot è già in ROM, non bootarlo per cercare un transcript: usare
   l'evidenza corrente di sicurezza periferiche e continuità di alimentazione.
   Se manca o è invalidata da un power-cycle periferico, STOP e preflight
   separato. I dati del 2 ottobre da soli non attestano lo stato di oggi.

Predisporre una nuova directory di sessione senza toccare i backup originali:

```bash
set -euo pipefail
source /home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_4_ROBOT_POWERED_2757604_20261003/context.env
SESSION="/home/matteo-manicardi/MATDOG/verification-artifacts/m0-4-hardware-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$SESSION"
bash "$PKG/preflight-local.sh" | tee "$SESSION/local-preflight.log"
```

Solo quando autorizzato e se non già in ROM: **una** chiamata ordinaria di
ingresso `usb-reset`; tutte le operazioni successive usano `no-reset`.

```bash
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 115200 --before usb-reset \
  --after no-reset --no-stub --connect-attempts 1 read-mac 2>&1 | tee "$SESSION/rom-entry.log"
```

Non richiedere BOOT tenuto basso durante la procedura ordinaria. Nessun reset
è nel writer. Se ingresso fallisce, STOP senza retry/reset/fallback automatico.
Per ROM già raggiunta saltare quel blocco e registrare come è stata raggiunta.

```bash
"${ESP[@]}" read-mac 2>&1 | tee "$SESSION/read-mac.log"
"${ESP[@]}" flash-id 2>&1 | tee "$SESSION/flash-id.log"
"${ESP[@]}" get-security-info 2>&1 | tee "$SESSION/security-info.log"
```

Atteso ESP32-S3 v0.2, MAC `14:c1:9f:22:75:94`, flash 16 MiB e PSRAM 8 MiB;
security flags 0, Secure Boot/Flash Encryption disabilitati, crypt counter 0,
nessun secure-download. Porta by-id indicata in `context.env`, nessuna porta
alternativa indovinata. Incongruenze o log incompleto impongono STOP.

Confrontare lo stato corrente con A/B verificati. Queste sono **nuove letture
di stato**, salvate nella sessione; nessun backup originale viene sovrascritto.

```bash
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/current-a.bin" 2>&1 | tee "$SESSION/current-a.log"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/current-b.bin" 2>&1 | tee "$SESSION/current-b.log"
cmp "$SESSION/current-a.bin" "$SESSION/current-b.bin"
cmp "$SESSION/current-a.bin" "$A_IMAGE"
cmp "$SESSION/current-b.bin" "$B_IMAGE"
sha256sum "$SESSION/current-a.bin" "$SESSION/current-b.bin" | tee "$SESSION/current.sha256"
python3 "$SCRIPTS/migration_m0.py" plan --profile ROBOT_POWERED \
  --backup "$SESSION/current-a.bin" --repeat "$SESSION/current-b.bin" --backup-sha256 "$BACKUP_SHA" \
  --sdkconfig "$SDKCONFIG" --bootloader "$BOOTLOADER" --running-offset 0x10000 \
  --installed-build-id dfcecb670d05 --expected-mac 14:c1:9f:22:75:94 \
  --observed-mac 14:c1:9f:22:75:94 --binary "$APP" --manifest "$MANIFEST" --out "$SESSION/plan"
```

File di stato distinti e acquisizioni realmente indipendenti, size 16.777.216,
SHA esatto atteso, `FILE_CHECKS=PASS`. Verificare copia di sicurezza su secondo
supporto con receipt prima della W: M0.4 non ne inventa l'esistenza.
Se lo stato è cambiato (anche NVS/core), non aggiornare l'hash autorizzato in
modo opportunistico: STOP, archiviare e riconciliare le differenze, verificare
eventuale nuovo A/B e rigenerare il piano dopo revisione. Nessuna W finché
integrità, power, sicurezza periferiche e stato corrente non sono attestati.
Il preflight positivo non concede W; richiedere autorizzazione nominativa alle
due operazioni sotto, con questi artefatti e questo stato.

## Sequenza controllata di migrazione — preparata, non eseguita

Ogni blocco è una fase separata. STOP a ogni errore; conservare i log, restare
in ROM se raggiungibile. Prima di qualsiasi nuova W dopo errore servono nuove
letture indipendenti, riconciliazione e nuova autorizzazione. Nessun retry,
erase-all, reset, stub, OTA o restore automatico. Perdita totale USB: manutenzione
manuale BOOT/EN con incarico separato; non forzare la fase successiva.

**W1 app0, quindi verifica indipendente.**

```bash
"$WRITE_PY" -I "$M0_WRITER" app0 "${WRITE_CONTEXT[@]}" --gate B \
  --offset 0x10000 --artifact "$APP" --sha256 "$APP_SHA" 2>&1 | tee "$SESSION/write-app0.log"
"${ESP[@]}" verify-flash 0x10000 "$APP" 2>&1 | tee "$SESSION/verify-app0.log"
"${ESP[@]}" read-flash 0x10000 0x113000 "$SESSION/app0-readback.bin" 2>&1 | tee "$SESSION/readback-app0.log"
cmp "$SESSION/app0-readback.bin" "$SESSION/plan/app0_sector_padded.bin"
```

W `[0x10000,0x122490)`; erase `[0x10000,0x123000)`, **275 settori**; padding
FF di `0xB70` byte. MD5 interno e secondo verify API sono obbligatori, poi
verify-flash indipendente e readback/cmp. Non bootare sul layout intermedio.
Prima della tabella verificare anche l'intera flash:

```bash
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-app-a.bin" 2>&1 | tee "$SESSION/post-app-a.log"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-app-b.bin" 2>&1 | tee "$SESSION/post-app-b.log"
cmp "$SESSION/post-app-a.bin" "$SESSION/post-app-b.bin"
python3 "$SCRIPTS/migration_m0.py" check-snapshot --profile ROBOT_POWERED \
  --backup "$A_IMAGE" --backup-sha256 "$BACKUP_SHA" --snapshot "$SESSION/post-app-a.bin" \
  --binary "$APP" --manifest "$MANIFEST" --stage app
```

SHA globale atteso `ff0ae739c3cdbb3106cf410a90c55d96ce3a43607bc6c655aba913dfe293e4dd`.
Non scrivere tabella se qualsiasi byte fuori range o padding differisce.

**W2 tabella V1 per ultima, quindi verifica indipendente.**

```bash
"$WRITE_PY" -I "$M0_WRITER" table "${WRITE_CONTEXT[@]}" --gate B \
  --offset 0x8000 --artifact "$TABLE" --sha256 "$TABLE_SHA" 2>&1 | tee "$SESSION/write-table.log"
"${ESP[@]}" verify-flash 0x8000 "$TABLE" 2>&1 | tee "$SESSION/verify-table.log"
"${ESP[@]}" read-flash 0x8000 0x1000 "$SESSION/table-readback.bin" 2>&1 | tee "$SESSION/readback-table.log"
cmp "$SESSION/table-readback.bin" "$SESSION/plan/target_partition_sector.bin"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-table-a.bin" 2>&1 | tee "$SESSION/post-table-a.log"
"${ESP[@]}" read-flash 0x0 0x1000000 "$SESSION/post-table-b.bin" 2>&1 | tee "$SESSION/post-table-b.log"
cmp "$SESSION/post-table-a.bin" "$SESSION/post-table-b.bin"
python3 "$SCRIPTS/migration_m0.py" check-snapshot --profile ROBOT_POWERED \
  --backup "$A_IMAGE" --backup-sha256 "$BACKUP_SHA" --snapshot "$SESSION/post-table-a.bin" \
  --binary "$APP" --manifest "$MANIFEST" --stage table
"${ESP[@]}" read-mac 2>&1 | tee "$SESSION/read-mac-final.log"
```

W `[0x8000,0x8C00)`; erase `[0x8000,0x9000)`, **un settore**, padding FF 1024 B.
SHA globale finale atteso `f02f742f13c224da722c2c2dbb94a4d7a423bd16df1b99a39b61f0d5ff87bb40`.
Nessuna cancellazione separata. Tutti i byte fuori dalle due finestre devono
coincidere con il backup: bootloader, NVS legacy `[0x9000,0xE000)`, otadata,
resto app0, app1, FFAT, matdog_nvs `[0xFE0000,0xFF0000)` e coredump.
La tabella cambia metadati; non cancella automaticamente le partizioni.
Il checker rifiuta qualsiasi dato occupato nell'app1 legacy, nuova app1,
FFAT/destinazione NVS e padding tabella. La sostituzione dell'app0 legacy è
l'unica perdita intenzionale di contenuto, coperta da A/B.

## Gate C distinto — primo boot futuro

Fermarsi in ROM dopo W2; nessun reset/boot finale automatico. Solo con nuova
autorizzazione al primo boot powered si stabilisce la transizione esplicita
di avvio, senza power-cycle dei servo e senza inserirla nel writer.
La scelta di quella transizione non fa parte delle due W autorizzabili qui.
Confermare banner `be0c12979e5b`/ROBOT_POWERED, tre rail YES, app0/5 MiB,
layout V1, persistence READY/NO_RECORD e `motion_authorized=0`, authority NONE,
permit revocato, nessuna sessione/torque/movimento e OTA ingest DISABLED.
Osservare almeno 120 s senza reboot/brownout/perdita USB, acquisire BMS/LED e
le 13 letture servo come sopra (MAINTENANCE esplicita se necessaria).
Vietati SAVE, ACK, RECONCILE, RESTORE, PROMOTE, torque e movimento.
Un nuovo boot può cambiare regioni core/NVS/OTA: non dedurre rollback
byte-identico dai confronti pre-boot; un recovery necessita stato e gate propri.

```text
M0_4_SOFTWARE_READY=YES
ROBOT_POWERED_QUALIFIED=YES
WRITER_POWERED_READY=YES
HARDWARE_PREFLIGHT_REQUIRED=YES
HARDWARE_FLASH_AUTHORIZED=NO
```

QUALIFIED indica la qualificazione software dell'artefatto esatto. Alimentazione,
stato periferiche, identità/security e confronto della flash corrente rimangono
verifiche hardware successive. Nessun impedimento software noto; nessun
preflight fisico corrente o autorizzazione alla scrittura è attestato da M0.4.
