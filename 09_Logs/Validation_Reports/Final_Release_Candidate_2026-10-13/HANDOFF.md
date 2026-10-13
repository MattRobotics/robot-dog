# MATDOG — handoff definitivo, 13 ottobre 2026

**Aggiornamento finale 13 ottobre, 19:29 UTC: [FLASH_SUCCESS / post-flash PASS](FLASH_CLOSEOUT.md).** Candidato originale installato una sola volta APP-ONLY dopo conferma fisica e recupero GitHub verificato. Robot in MAINTENANCE, authority NONE, 13 servo OFF, Motion session OFF; validazione Motion ancora separata. Le precedenti indicazioni NO-GO sono il quadro storico prima del completamento.

**Closeout operativo reale: NO-GO prima del flash.** [A/B correnti verificati, GPG pinentry scaduto/cancellato e stato ROM](REMOTE_BACKUP_CLOSEOUT.md). Nessun flash, movimento o scrittura NVS. L’ultima osservazione 13 OFF resta storica pre-ROM.

**A — SOFTWARE RELEASE READY / PASS.** Un solo candidato designato, clean source `b147f3884b227ec819e98a6f00c214a0aac2278e`, `ROBOT_POWERED`, `MATDOG_SUPERVISED_VALIDATION=1`, startup `MOTION_AUTHORIZED=0`, `OTA_INGEST=0`. App **3,643,120 byte**, SHA256 **`5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`**. Manifest e verifica indipendente PASS: checksum/SHA ESP32-S3, source embedded, layout 16 MiB / 2×5 MiB, pin e tutti i 1.831.776 byte della tabella WALK effettivamente presenti nel binario. RAM statica 137.508 byte, residui 190.172. Warning non bloccanti documentati nel receipt; nessuna chiamata al ReadMode(-1) della libreria.

Binario locale unico:

```text
/home/matteo-manicardi/MATDOG/verification-artifacts/FINAL_RC_20261013/final_candidate/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin
```

Il medesimo percorso contiene manifest, partition table, sdkconfig, log, provenance/toolchain, sorgente originale archiviato e receipt. I prodotti merged/bootloader generati dalla toolchain **non sono candidati al flash**; il workflow ammette esclusivamente l’app indicata. Binario e input TLS privati restano locali, con permessi riservati. [BUILD_RECEIPT](BUILD_RECEIPT.json) · [designation](designated_candidate.json) · [manifest pubblico](matdog_build_manifest.txt).

**B — REMOTE PREFLIGHT READY / PASS software.** Applicare la [correzione seriale multilinea](PREFLIGHT_SERIAL_FIX.md) prima del nuovo tentativo; 16 test offline PASS, app e manifest invariati. I 23 comandi diagnostici sono compatibili col sorgente installato storico `b08322d…`; gli script sono verificati offline e non hanno aperto USB. Il collegamento attuale ASUS↔robot resta da osservare. La connessione remota dell’ASUS deve essere indipendente dalla radio ESP32; porta USB by-id/MAC devono coincidere. Preflight solo letture, nessun nuovo RESTORE sul vecchio firmware, nessun torque/grant/flash.

**C — FLASH NO-GO ora.** Prima del GO: identità MCU/app/layout/slot correnti; authority NONE e 13 OFF; gen1 acknowledged integro e semanticamente valido; BNO085/DALY/elettrico/termico/alimentazione reviewati; due backup completi 16 MiB con SHA uguali; copia indipendente realmente recuperata/decrittata e verificata; review privata fresca. Il tool lascia nel ROM e non esegue flash. `review_preflash.py` può emettere technical GO ma mantiene `flash_authorized=false`. Occorre poi autorizzazione esplicita del proprietario per **questo SHA256**. APP-ONLY una volta, script esistente, nessun erase generale/NVS/otadata/partition table write. [Procedura completa](REMOTE_PREFLIGHT.md).

**D — SUPERVISED MOTION NO-GO fino ai gate reali.** La stessa immagine contiene RESTORE esplicito, 12 limiti scoped, profilo RAM-only con review/digest, nonce/epoch/scadenza, prime PRESENT/holding/output/readback, grant locale, STAND con NEXT, WALK_START separato, STOP/watchdog/revoca/SAFE_OFF. Nessun grant al boot/reconnect, nessuna authority Web/LAN. Console fisica USB/TTY, SSH rifiutato. Non occorre un secondo flash per qualificare progressivamente gli scope previsti.

Sequenza personale minima, in presenza, nella **stessa sessione supervisionata**:

1. Verificare fixture/ritenzione/free-volume/configurazione reale e power cut fisico della rail, incluse alimentazioni residue. Personale e area sicura; robot disarmato.
2. Osservare 13 OFF, 12 identità/encoder/direzioni/posa, IMU/BMS/power/thermal. RESTORE gen1 esplicito e confronto SHA con l’installazione corrente; nessuna Full Calibration ripetuta.
3. Review dei **12 limiti, oggi 0/12 approvati**; inset/contatti/LF UPPER a margine ridotto, incertezza totale, drive units, ceiling, timing bus worst-case, freshness/tracking/STOP. Nessun valore sintetico è un’approvazione.
4. Caricare card scoped RAM-only e ammettere la posa misurata. Variante finale fixture LOWER front **−88,5°**, rear **−89°**; la versione −88,9°/−89,4° è storica. Posa fuori limiti/corridoio: NO-GO, nessun movimento di acquisizione verso una posa inventata. Intermedi ammessi soltanto nel corridoio certificato comune.
5. Nuova autorizzazione locale: prime actual raw con OFF, verifica goal/speed/ACC, holding/output e dodici readback post-arm senza salto. Presenza held, STOP e power cut pronti. Incongruenze/stale/drift/fault: abort e 13 OFF verificati.
6. STAND per segmenti: PRE_ALIGN → posteriori → anteriori → trasferimento virtuale → RISE → C4. NEXT autorizza un solo segmento; verificare actual/target, direzioni, tracking, collisioni reali/power/temp ad ogni pausa. C4 sospesa rappresenta i target nominali 150 mm, **non prova quota reale o appoggio portante**.
7. Solo dopo review STAND: nuovo scope/grant WALK, prima falcata sway **30 mm**, lift/advance 5 mm, vy/yaw zero, clock ≥9. RF LOWER nominale +28,44766°; precedente 40 mm oltre limite escluso. Limiti/feedback/tracking attuali devono approvare l’intera falcata ±tube prima del grant. STOP/expiry revocano; renew non avvia un’altra falcata. Tempo nominale minimo 954 s più pause/bus: pianificare la supervisione, non è WCET.
8. STAND portante e WALK caricato richiedono inoltre supporto/forze/altezza assoluta da observer qualificato, massa/COM/attrito/compliance e dinamica/tracking caricato. Fixture, IMU, encoder e corrente non dimostrano questi fatti. Prona originale −92° resta NO-GO con le proposte correnti; suffisso canonico da knot ≥114/C4 soltanto dopo relativo ingresso/supporto qualificato.

[Sessione passo-passo e risultati registrabili](SUPERVISED_SESSION.md) · [admission reale](RUNTIME_ADMISSION.md) · [12 limiti/fonti/margini](JOINT_LIMITS.md).

**Funzioni e prove:** LED SOC 30/15, rilascio 33/18, UNKNOWN/stale e charging continuo 2→60 PASS offline; GPIO47/12/RMT DMA/KEY/MOS preservati. LAN MAINTENANCE read-only, ADMIN_AP 192.168.4.1, RUN_STA TLS/autenticato e recovery/STA_STOP PASS offline; credenziali/trust e radio sul robot da verificare. Dashboard 48 azioni/6 schermate/Twin 17 STL e desktop/mobile PASS browser/DOM; handler C++ reali PASS. Planned/authority assente rimangono disabilitati. [Galleria GitHub](visuals/README.md) · [48 azioni](DASHBOARD_ACTIONS.md) · [test PASS e gate BLOCKED](VERIFICATION.md) · [receipt integrato](TEST_RECEIPT.json). Nessuna prova fisica inventata.

Il commit sorgente compilato resta distinto dal successivo HEAD documentale della Draft PR #47 e dalle build/installazioni storiche. Nessun flash, movimento, grant, scrittura hardware o merge eseguito da Codex.

Quando il robot verrà acceso, eseguire **sull’ASUS** soltanto:

```bash
python3 /home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog/05_Firmware/MATDOG_Controller/scripts/release/motion_remote_preflight.py --execute-readonly --confirm-asus-independent-maintenance-link
```

## Completamento archivio — 13 ottobre, 19:12 UTC

[Copia GitHub indipendente recuperabile PASS](RECOVERY_ARCHIVE.md), con AES-256, download e decifratura verificati. Il precedente NO_GO GPG resta storico e preservato. Issue #46 aggiornata, ancora aperta; release stabile subordinata ai test hardware Motion. Nessuna nuova lettura integrale, build o calibrazione. Identità ROM corrente e partizione app0 riconfermate con sole letture senza reset. Il flash resta subordinato alla singola conferma fisica finale e alla review tecnica documentata.
