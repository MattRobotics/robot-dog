# MATDOG — Procedura APP-only separata dalla prova Motion

Stato al 16/10/2026: **piano verificato staticamente, esecuzione NO_GO** finché il gate host non riconosce il candidato congelato. Questa procedura non è stata eseguita e non autorizza un flash. La decisione corrente è nella [review](FINAL_PREFLASH_REVIEW.md). Nessun comando qui è stato inviato al dispositivo.

## 1. Identificazione e piano offline già verificati

Directory candidata unica:

`~/MATDOG/verification-artifacts/MOTION_POST_VALIDATION_20261016/build-powered/artifacts/ROBOT_POWERED-ota0/`

| Campo | Valore |
|---|---|
| File applicazione | `MATDOG_Controller.ino.bin` |
| Sorgente compilato | `652c2e2b526694839d11351a6ed6fd947e6f7588` CLEAN |
| SHA-256 app | `ac972a5ef802988af6a515903a5afd5589d3c2d0e04400f99f69670d8ff499d7` |
| Dimensione app | 3885168 B = `0x3b4870` |
| Manifest / ELF / SDK | `matdog_build_manifest.txt`, `MATDOG_Controller.ino.elf`, `sdkconfig` nella stessa directory |
| Profilo | ROBOT_POWERED, OTA ingest 0, boot motion 0, runtime-governed |
| Layout | `MATDOG_16M_2x5M_NVS_V1` |
| SHA-256 tabella | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |
| Target storico installato | app0, offset `0x010000`, dimensione `0x500000` = 5242880 B |
| Write range, end esclusivo | `0x010000..0x3c4870` |
| Erase range, end esclusivo | `0x010000..0x3c5000` |
| Resto slot app0 | fino a `0x510000`; non necessario cancellarlo per scrivere l'immagine più corta |
| MATDOG NVS | `0xfe0000..0xff0000`, fuori dal range |

Solo l'app `.ino.bin` è candidata: non usare `.merged.bin`, bootloader, tabella partizioni o boot_app0. Nessun erase globale o scrittura otadata/NVS. `check-write` e `build_manifest.py verify` sono già stati eseguiti offline; ricevuta in `final-preflash-review/integrity.json`. Il digest ELF incorporato e la ricostruzione locale esptool sono coerenti. Non ricompilare il candidato per far coincidere HEAD: BUILD_UTC cambierebbe anche il digest.

## 2. Blocco da risolvere prima di collegarsi

`scripts/flash_app_only.sh` non ha un dry-run generale. **Non invocarlo con `--dry-run`**: il parametro non è una garanzia di mancato accesso hardware. La presente verifica è statica, senza porta e senza invocare lo script di flash.

Il percorso ordinario seleziona la directory build locale e richiede sorgente/HEAD corrispondenti. L'opzione `MATDOG_FLASH_CANDIDATE_DESIGNATION` usa `verify_designated_candidate.py`, che oggi accetta soltanto le differenze e le ricevute della precedente release: rifiuta il candidato 652c2e2 dopo i cambi audit/test di b0b7e51, richiede supervised-capable 1 e cerca la qualificazione del 13/10. Il file `designation_probe_NOT_APPROVED.json` è soltanto un test negativo conservato, **non una designazione valida**.

Prima di qualsiasi flash occorre un adeguamento host isolato e testato: vincolare digest app/ELF/manifest, inventario dei sorgenti compilati, commit fonte e differenze ammesse di soli test/audit/documenti; collegare con hash le regressioni già concluse e la loro risoluzione composta. Preservare dirty-tree refusal, controllo profilo/FQBN/OTA/layout, backup gate, MAC, selezione partizione, fencing e verifica post-write. Regressioni negative devono rifiutare alterazioni di sorgente compilato, binario, manifest, prove, profilo e worktree dirty. Non introdurre un bypass generico e non dichiarare un test generale nuovamente PASS se è stata usata la verifica composta documentata.

Questa riparazione non richiede cambiare il firmware o aumentare i timeout. L'eventuale CSP Twin corretta, invece, produrrebbe un successore con un nuovo digest: non etichettarlo `ac972…`.

## 3. Precondizioni della futura sessione autorizzata

1. Decisione esplicita sul flash, dopo chiusura del blocco host. Motori/rail servo fisicamente disalimentati; batteria e fonte di alimentazione in stato adatto alla programmazione, senza usare il SOC come prova di disponibilità. Flash separato dalla fixture con attuazione.
2. Confermare worktree isolato pulito, provenienza 652c2e2, hash identici alla tabella. Conservare i tre commit precedenti e tutti gli artefatti. Individuare un solo proprietario della seriale; arrestare bridge/logger/monitor prima dell'accesso ROM. Non far competere processi sulla USB.
3. Riutilizzare il backup valido già verificato:

   `~/MATDOG/private-preflight/20261013T180138Z/fullflash_A.bin`

   16777216 B, SHA-256 **`9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83`**. Il backup conserva lo stato del 13/10, non viene presentato come copia aggiornata della calibrazione del 16/10. Il recupero APP-only sotto usa il binario baseline esatto e preserva la NVS corrente; non serve un nuovo backup per questa procedura. Manifest/expected hash esplicito del backup restano richiesti dal gate esistente.
4. Solo dopo autorizzazione e ingresso supervisionato in ROM: verificare MAC `14:c1:9f:22:75:94`, identità ESP32-S3/flash, tabella installata e otadata con `verify_application_partition.py`. Questo è un controllo futuro di lettura, non svolto durante la review. Il target storico è app0, ma non sostituire la lettura corrente con l'assunzione dell'offset.
5. Rifiutare se tabella o selezione attiva sono ambigue/diverse. Il tool deve derivare offset/size dal dispositivo e verificare il range col layout. SDK candidato: rollback enable, anti-rollback non impostato; ciò non prova che esista un'altra app valida per rollback automatico. Nessuna modifica efuse, bootloader, tabella o otadata per «far passare» il controllo.

## 4. Scrittura e verifica futura

Piano di argv, soltanto documentale, subordinato a tutti i gate precedenti:

```text
esptool 5.3.1
--chip esp32s3 --port <porta verificata ed esclusiva> --baud 115200
--no-stub --connect-attempts 1 --before no-reset --after no-reset
write-flash
0x010000 <directory candidata>/MATDOG_Controller.ino.bin
```

Il gate deve confermare che l'offset verificato sia ancora `0x010000`; se cambia, questa pianificazione va rivalutata. Il writer ammesso contiene **una sola coppia offset/file**. Il piano non è un invito a saltare il runner verificato usando direttamente esptool.

Successivamente: `verify-flash` indipendente sulla medesima coppia con `--no-stub`, un solo tentativo, `--before no-reset --after no-reset`. Conservare stdout/stderr/exit e risultato del confronto device/file; archiviare nuovamente SHA-256 locale e manifest. La verifica del tool confronta il contenuto flash con il file; il digest SHA-256 dichiarato identifica il file candidato, non un inesistente readback SHA-256 prodotto in questa review. Se la verifica fallisce, rimanere in ROM e investigare/recuperare, senza avviare la qualificazione Motion.

Non eseguire reset/boot automatico dopo write/verify. Un primo boot distinto e supervisionato, ancora con servo rail OFF, deve registrare build ID `652c2e2b5266`, GIT_SHA completo, ROBOT_POWERED, OTA ingest disabilitato, app0 e reset reason; osservare sessione/executor OFF, assenza di priming/energizing/motion. Con rail servo OFF readiness può essere bloccata: non forzare READY per questa verifica. Il codice permette auto-RESTORE/preflight/binding/RUN/authority, ma questi non autorizzano torque; un movimento richiede un comando esplicito e i gate live.

Ricevuta finale futura: autorizzazione, UTC e locale/offset, boot nonce se disponibile, source commit, SHA app/ELF/manifest, profilo, layout, MAC, target/size/range, backup/digest, risultato write e verifica indipendente, stato ROM e successivo boot separato. Questo è un PASS di programmazione soltanto, mai un hardware Motion PASS.

## 5. Recupero della precedente applicazione

Il firmware precedente esatto è già disponibile e verificato, senza leggere o scrivere il dispositivo:

`~/MATDOG/worktrees/robot-dog-opus-audit-20261014/05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin`

SHA-256 **`92472cd00f6ef9745361fcb1452d63c2ddda520b7eb445a4ceea98e485ac0cd5`**, 3999408 B, sorgente `2c2c13bf52c125194d692f666606c0bd3600adfc`, manifest CLEAN ROBOT_POWERED/OTA ingest 0. Layout e schemi cal corrispondono. Conservare il file e il worktree originale; non modificarli per attivare un gate del nuovo candidato.

Se l'app nuova non fa boot o fallisce i controlli: rail servo OFF, ingresso manuale supervisionato in ROM, esclusività seriale, ripetere identità/layout/partizione e usare una distinta designazione di rollback del **binario precedente** con hash e manifest verificati. Scrivere soltanto l'app attiva verificata e fare `verify-flash` indipendente, poi eventuale boot separato. Non presumere che l'OTA rollback ripristini 2c2c13b: la scrittura APP-only sovrascrive l'app attiva e il boot precedente riportava `rollback_possible=NO`/`UNDEFINED`.

Non ripristinare l'intera flash per un guasto della sola applicazione: toccherebbe calibrazione/configurazione e richiederebbe un'altra procedura e autorizzazione. Il backup completo esistente resta recupero di emergenza distinto. Nessuna cancellazione NVS/EEPROM, migrazione o modifica dei profili è parte di questa procedura.

La prova con robot alimentato e possibilità di movimento è esclusivamente nella [procedura fixture](FIXTURE_QUALIFICATION.md), da iniziare dopo un'ulteriore decisione operativa.
