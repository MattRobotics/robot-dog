# MATDOG — APP-only e rollback, 17/10/2026

Designazione e qualificazione host offline completate; consultare REPORT.md e CANDIDATE.json. La qualificazione statica è COMPOSTA e preserva il primo FAIL e la sua risoluzione. Questa procedura non è stata eseguita su hardware. Un GO offline è una raccomandazione per una decisione esplicita, non autorizza autonomamente il flash o il movimento.

## File e verifiche offline

Candidato ROBOT_POWERED da sorgente pulito `8e1673b371400cacff9f24e344009bc20ae32b63`:

`~/MATDOG/verification-artifacts/RECOVERY_FINAL_20261017/build-powered/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`

SHA-256 `af13420cd45858b1af89781b430fa49af97e114ddb168873cf5c66b3e0037711`, dimensione 3887408 B. Usare soltanto questo `.ino.bin`: non merged, bootloader, tabella o boot_app0. Manifest, ELF e SDK nella stessa directory. `CANDIDATE.json` associa sorgente, tooling qualificato, tutti i digest e i log di questa campagna; deve essere committato, con checkout pulito. Verifica file-only ammessa senza dispositivo:

```sh
python3 05_Firmware/MATDOG_Controller/scripts/verify_designated_candidate.py \
  09_Logs/Validation_Reports/Release_Candidate_2026-10-17/CANDIDATE.json --field qualification
```

Il flasher non ha un dry-run generico: non invocarlo con un presunto `--dry-run`. Le prove automatiche usano esplicitamente un esptool fittizio e un file normale come porta, non la USB del robot.

Backup esistente valido, nessun nuovo backup richiesto:

`~/MATDOG/private-preflight/20261013T180138Z/fullflash_A.bin`, 16777216 B,
SHA-256 `9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83`.

È uno snapshot del 13/10, non una copia aggiornata della calibrazione del 16/10. L'APP-only e il recupero app sotto mantengono la NVS corrente.

## Futura fase di flash, solo dopo autorizzazione esplicita

1. Rail servo fisicamente OFF, fonte di programmazione stabile. Nessuna prova Motion in questa fase. Lasciare STOP/taglio fisico disponibili per il successivo boot supervisionato.
2. Verificare i digest e il checkout pulito. Individuare un unico proprietario USB. L'operatore deve chiudere ordinatamente bridge/logger/monitor prima della sessione ROM: nessuno script di questa consegna termina processi seriali o forza l'apertura di una porta occupata.
3. Entrare manualmente/supervisionatamente in ROM. Il runner verifica MAC `14:c1:9f:22:75:94`, SDK rollback/anti-rollback, tabella installata e selezione attiva da otadata. Rifiutare qualunque ambiguità o layout diverso; non migrare, cancellare o modificare OTP/efuse/otadata per superare il gate.
4. Il layout atteso è `MATDOG_16M_2x5M_NVS_V1`, tabella SHA `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`. App0 storico e pianificato `0x010000`, slot `0x500000`; rileggere lo stato corrente, non assumere il target dalla ricevuta vecchia. Range nuovo end esclusivo `0x010000..0x3c5130`, erase arrotondato `..0x3c6000`, entro app0 `..0x510000`; MATDOG NVS `0xfe0000..0xff0000` esclusa.
5. Dal worktree Codex, con la designazione finale committata, il futuro comando è:

```sh
MATDOG_FLASH_CANDIDATE_DESIGNATION="$PWD/09_Logs/Validation_Reports/Release_Candidate_2026-10-17/CANDIDATE.json" \
MATDOG_FLASH_PROFILE=ROBOT_POWERED MATDOG_FLASH_OTA_INGEST=0 \
MATDOG_FLASH_BACKUP="$HOME/MATDOG/private-preflight/20261013T180138Z/fullflash_A.bin" \
MATDOG_FLASH_BACKUP_SHA256=9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83 \
05_Firmware/MATDOG_Controller/scripts/flash_app_only.sh
```

Il comando è documentato, non eseguito. Non sostituire il runner con un comando esptool diretto che salti i gate. Il runner fa una sola scrittura con una sola coppia offset/app, poi `verify-flash` indipendente; usa `--no-stub`, un tentativo, `--before no-reset --after no-reset` e resta in ROM. Qualunque mismatch va investigato senza boot automatico. Conservare log, hash locale/manifest, risultato write/verify, MAC, tabella, otadata e range. Non chiamare readback il semplice SHA di un file: il confronto dispositivo/file avverrà solo nella futura verifica reale.

## Primo boot separato e qualificazione STAND

Dopo write e verifica riusciti, un boot supervisionato distinto avviene ancora con rail servo OFF. Registrare firma `8e1673b37140`, GIT_SHA completo, ROBOT_POWERED, OTA ingest 0, app0/reset reason e sessione/executor OFF. READY può restare bloccato con i servo disalimentati: non forzarlo. Il codice permette preflight, auto-RESTORE, binding e grant authority, ma nessuno abilita automaticamente la coppia; la sequenza PRIME/energize/ARM richiede il comando STAND esplicito e tutti i gate live.

Solo una successiva autorizzazione avvia la prova fisica sulla fixture: alimentazione adeguata con cella minima/pack/allarmi freschi, taglio fisico pronto, RESTORE/preflight riconosciuti, presenza autenticata finita e 12 RAW al gate. Acquisire `MOTION_ADMISSION`, epoch/generation/lease, timestamp/età signed di tutti i giunti, heap libero/minimo/largest e stack high-water prima/durante entry/STAND/STOP. Un POST non confermato non si ripete automaticamente. STOP è prioritario nel loop dopo ricezione; HTTP single-task può ritardarne l'arrivo, quindi non sostituisce il taglio fisico. Perdita presenza oltre 3 s deve portare SAFE_OFF; il Twin non rinnova il deadman. Non eseguire WALK/FAST o operazioni persistenti concorrenti. PRONE→STAND non è hardware PASS in questa consegna.

## Rollback APP-only verso 2c2c13b

File già verificato, da preservare nel worktree Motion originale:

`~/MATDOG/worktrees/robot-dog-opus-audit-20261014/05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin`

SHA-256 `92472cd00f6ef9745361fcb1452d63c2ddda520b7eb445a4ceea98e485ac0cd5`, 3999408 B; manifest CLEAN `2c2c13bf52c125194d692f666606c0bd3600adfc`, layout e schemi cal record 1/marker 2 compatibili.

Non applicare al rollback la designazione del nuovo candidato. La baseline ha `SUPERVISED_VALIDATION_CAPABLE=1`: non falsificarlo per farla passare nel gate V3 ordinario del candidato a capability 0. Usare il runner originale dal worktree pulito a 2c2c13b e il suo percorso normale HEAD/build (già corrispondenti), con gli stessi backup/hash espliciti e profilo ROBOT_POWERED/OTA ingest 0, **senza** `MATDOG_FLASH_CANDIDATE_DESIGNATION` ereditata. Il runner originale mantiene il suo audit di sicurezza, il controllo MAC/layout/partizione e la verifica indipendente: nessun bypass. La sua eventuale esecuzione futura può rieseguire i gate della baseline; questa review non li ha lanciati né ha modificato quel worktree.

Il recupero avviene con rail servo OFF e ingresso ROM manuale, dopo nuova decisione esplicita. Verificare ancora il target attivo; ripristinare soltanto la precedente app, una coppia offset/file, seguito da verify-flash e boot separato. Non presumere rollback OTA automatico: si sovrascrive l'app attiva e il boot precedente riportava rollback_possible=NO/UNDEFINED. Non scrivere il full backup per un guasto della sola applicazione: toccherebbe configurazione/calibrazione e richiederebbe una diversa procedura/autorizzazione. Nessuna scrittura NVS/EEPROM/BMS o cambio di limiti/profili fa parte del rollback.
