# MATDOG — recovery post-ABORT e rilascio applicativo, 3 ottobre 2026

Questa procedura accompagna il branch `fix/calibration-post-abort-thermal`.
Implementazione e verifiche sono offline. Nessuna operazione hardware di questa
procedura è stata eseguita durante la preparazione del rilascio.

## Acquisizione, interruzione e ripresa

Usare sempre `scripts/calibration_hw_session.py`, con lo stesso `--evidence-dir`.
Conservare `q0_candidates.json`, log, export e ricevuta SAVE/ACK. I normali gate
del firmware, la presenza dell'operatore, il supporto del robot e la possibilità
di interrompere l'alimentazione restano necessari nella futura sessione hardware.
Nessuna fase del runner invia un comando RESET.

1. `--phase prepare --no-flash`: verificare firma della nuova applicazione,
   MAINTENANCE, health READY, authority NONE e SAFE_OFF 13/13.
2. Predisporre fisicamente la posa nominale Q0 nella futura sessione controllata.
   `--phase q0 --confirm-q0-pose`: Fresh Q0 12/12, promozione 12 trasformazioni,
   checkpoint con build, acquisizione, uptime e dodici tick. La nuova acquisizione
   non può essere avviata sopra record RAM esistenti o una sessione live.
3. `--phase recover`: INITIAL RECOVERY ordinario, un giunto alla volta, PASS
   12/12 a Q0 e torque OFF. Il limite di 64 tick rimane invariato. Questa fase
   lascia LF session + permit pronti, con torque disabilitato.
4. `--phase legs --confirm-operator-go`: LF → RF → RH → LH, sei contatti validi
   per zampa, ritorni e SAFE_OFF, export finale 24/24. `--phase all` può eseguire
   il flusso iniziale completo; termina con evidenze RAM, senza SAVE implicito.
5. In caso d'interruzione: ABORT, revoca permit/sessione, SAFE_OFF e export.
   Non ripetere Q0 CAPTURE e non scartare i record delle zampe completate.
6. Nella **stessa accensione**, `--phase resume --confirm-operator-go` controlla
   checkpoint e Q0 promosso attuale, geometria e chiusura dei record. Salta le
   zampe complete compatibili. Per la zampa fallita richiede POST_ABORT RECOVERY;
   per quelle ancora da iniziare usa INITIAL RECOVERY ordinario. Un rifiuto
   interrompe il runner: nessun fallback verso movimenti più permissivi.
7. `--phase post-abort --legs RF --confirm-operator-go` consente di fermarsi
   dopo il solo recovery RF. La sessione risultante può essere riutilizzata da
   `resume`. Comando nativo:
   `@CALIBRATION POST_ABORT RECOVERY RF CONFIRM_Q0_RECOVERY`.
8. Dopo export verificato 24/24: `--phase persist` esegue SAFE_OFF 13/13,
   authority NONE, SAVE CHECK, SAVE e ACK della generazione esatta. Verifica NVS
   READY, record acknowledged integro e nessun pending/uncertain, quindi scrive
   `persistence_ack.json`. Non ripetere SAVE dopo un ACK incerto: diagnosticarlo
   attraverso la procedura di persistenza già esistente.
9. Spegnimento/riaccensione controllati **in una successiva sessione hardware**;
   poi `--phase verify-persistence`, tempestivamente dopo l'avvio. È una fase
   di sola lettura: confronta build, uptime diminuito, generazione acknowledged,
   record integro, authority NONE e `MOTION_AUTHORIZED=0`. Il confronto uptime
   è conservativo: dopo un'attesa maggiore dell'uptime registrato rifiuta la
   verifica anziché inferire un reboot. Non impartisce né reboot né RESET.

Per iniziare deliberatamente una diversa acquisizione: chiudere le sessioni,
SAFE_OFF, esportare le evidenze che si desidera conservare, quindi
`@CALIBRATION EVIDENCE DISCARD CONFIRM_NEW_Q0`. Cancella soltanto le evidenze e
il witness RAM; Q0 promosso e NVS non vengono cancellati. Il successivo Q0 CAPTURE
e la sua nuova promozione costituiscono una nuova acquisizione; le vecchie zampe
non possono essere mescolate con essa. Non serve un RESET fisico.

## Quando il recovery automatico è ammesso

Serve un witness dell'executor nella stessa accensione, con tutti i dodici
SAFE_OFF verificati. Sono ammesse esclusivamente interruzioni in UPPER MIN/MAX
o LOWER MIN/MAX con la posa coerente con i prerequisiti della fase. Il witness
non viene salvato in NVS e viene consumato dal tentativo di recovery.

- LOWER: mantenere HIP a Q0, UPPER nella posa orizzontale e, per LF/RF, il rear
  UPPER nel parcheggio V5 a 35°. Riattivare i supporti alla posizione presente
  verificata, uno alla volta; poi LOWER a Q0, UPPER a Q0, rear park a Q0.
- UPPER: mantenere HIP/LOWER a Q0 e l'eventuale rear park; poi UPPER a Q0 e
  ripristino rear park.
- Infine SAFE_OFF e INITIAL RECOVERY ordinario dei dodici giunti, sequenziale,
  con verifica Q0/torque OFF. Il recovery non produce nuovi contatti.

Il giunto cercato deve essere dentro i guard esistenti. I supporti devono
essere entro 10 tick dalle pose previste; i readback freschi prima del movimento
devono restare entro 16 tick dalla posa testimoniata, e rispettare anche il
vincolo di 10 tick dei supporti. I hold continuano a rispettare la posa geometrica
prevista. Prime alla posizione presente precede RAM TorqueLimit 500 e torque ON;
ogni movimento conserva il profilo CALIBRATION_SEARCH e il gate di assestamento
4 campioni/400 ms, errore ≤10 tick e velocità ≤4.

Rifiuti espliciti: `POST_ABORT_NO_WITNESS`, `POST_ABORT_PHASE_UNPROVEN`,
`POST_ABORT_Q0_CHANGED`, `POST_ABORT_POSE_MISMATCH`. HIP, transizioni e pose non
dimostrabili rimangono SAFE_OFF. Perdita UART, corrente ≥200 raw, temperatura
confermata >70 °C, readback incoerenti, timeout e perdita di autorità/permit
interrompono il tentativo. SAFE_OFF non verificato continua a essere ritentato;
non equivale a recovery completato.

Dopo il reale spegnimento del 3 ottobre il witness è perso: questo firmware
non autorizza retroattivamente i tick 2348/1080/1665 usando un log dell'operatore.
Non ci sono movimenti all'avvio né ripristino automatico dell'autorità dalla NVS.

## Conferma termica

Il limite resta 70 °C. Solo un campione bulk >70 avvia una conferma: fino a
quattro letture dirette aggiuntive, distanti almeno 50 ms. Tre hot confermano;
tre cool, con ultimo campione diretto cool, classificano il transiente.
`95,117,34,34,34` pubblica 34 °C, TRANSIENT. Una sola anomalia seguita da tre
cool richiede tre letture; un surriscaldamento ripetuto richiede soltanto due
dirette. Sui campioni normali non vengono fatte letture aggiuntive.

Durante PENDING non vengono emessi nuovi goal o comandi di torque. Il servo
può continuare verso il precedente goal limitato; corrente, posizione, readback
e supporti vengono osservati. Deadline PENDING 300 ms; una transazione UART
operativa è limitata a 100 ms. Errori/invalidità, tre anomalie risolte in 30 s
o otto nella stessa accensione provocano fail-closed. I campioni normali non
azzerano il conteggio. Il latch termina al riavvio, che comunque non abilita
movimento. Tempi effettivi e contatto meccanico attendono verifica hardware.

## Aggiornamento applicativo futuro — non eseguire in questa revisione

Il pacchetto locale è in
`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_POST_ABORT_THERMAL_20261003`.
`release-manifest.json` lega commit pulito, profilo ROBOT_POWERED, ingest 0,
dimensione/hash della `.ino.bin` e layout. Verificare SHA256SUMS e revisione finale
prima della successiva operazione hardware. Per una ricompilazione offline:

```bash
cd /home/matteo-manicardi/MATDOG/github/robot-dog-post-abort
MATDOG_PROFILE=ROBOT_POWERED MATDOG_OTA_INGEST_VALIDATION=0 \
  bash 05_Firmware/MATDOG_Controller/scripts/build.sh
```

Il solo percorso canonico del futuro aggiornamento è `scripts/flash_app_only.sh`
da checkout pulito e build manifest coerente. La seguente invocazione è una
istruzione per la **futura sessione autorizzata**, non è stata eseguita:

```bash
MATDOG_FLASH_PROFILE=ROBOT_POWERED MATDOG_FLASH_OTA_INGEST=0 \
MATDOG_FLASH_BACKUP=/home/matteo-manicardi/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594/read-a-16m.bin \
MATDOG_FLASH_BACKUP_SHA256=856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7 \
  bash 05_Firmware/MATDOG_Controller/scripts/flash_app_only.sh
```

Lo script verifica identità, tabella effettiva del dispositivo, OTA selection,
slot, confini write/erase e digest post-write; scrive una sola coppia
offset/applicazione. Non assumere il solo offset storico `0x10000` senza questi
gate. Non usare upload.sh, immagini merged, scritture bootloader/partition-table,
erase-flash o comandi di cancellazione NVS. La tabella rimane
`MATDOG_16M_2x5M_NVS_V1`, SHA256
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`;
`matdog_nvs` resta `0xfe0000..0xff0000`.

## Rollback disponibile e limite della verifica

Applicazione precedente be0c12979e5b ROBOT_POWERED disponibile nel pacchetto
`MATDOG_M0_4_ROBOT_POWERED_2757604_20261003/artifacts`, hash
`7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`.
La sua copia è inclusa in `rollback/` nel nuovo rilascio. Per il rollback
applicativo usare il commit/build/manifest precedente e i medesimi gate del
percorso app-only: preserva tabella e NVS. Non aggirare il controllo HEAD con
la vecchia immagine dentro il checkout nuovo.

Le due immagini storiche full-flash 16 MiB hanno SHA256 identico
`856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7`.
È stato ripetuto il preflight **file-only** del pacchetto precedente: PASS,
HARDWARE_IO=NO. Manifest/receipt registrano già recupero cifrato byte-equal;
sono stati ispezionati localmente, senza una nuova verifica remota o decifratura.

Prima di un futuro flashing resta da riconoscere questo limite: il ripristino
fisico completo e il recupero da perdita USB non sono stati provati in questo
incarico; app1 non è un fallback automatico. L'eventuale accesso manuale BOOT/EN
è descritto nel runbook M0.4. Il dump storico è antecedente alla migrazione:
un ripristino completo riporterebbe layout e NVS storici e richiede una procedura
separata. Non è un rollback applicativo. Nessun nuovo backup hardware è richiesto
o acquisito in questa fase.


## Offline release-stage admission, 2026-10-03

See SESSION_AUTOMATION.md and the packaged INITIAL_POSE_PLAN.md. The prior boot's
RAM witness is lost on power-off. Do not invoke post-ABORT recovery from historical
RF/RH positions alone. The packaged three-stage automation rejects the current
unqualified initial pose before hardware I/O; no-argument invocation is file-only.
No cover removal, physical RESET, BOOT/EN request or forced reducer movement is
a fallback in this mandate. Physical square/jig Q0 and a qualified placement
procedure must be established before a future OFFLINE_READY.


## Startup after power-off — separate explicit path

The same-boot witness is never reused after reboot. The release-stage command
02 handles only the fixed qualified RF LOWER MAX pose using verified reference
20261003_134848 and fresh 3x12 readbacks, current population and all protections.
It primes and restores21→22→32, then closes SAFE_OFF13/NONE before Fresh Q0.
Body/passive joints require gravity support and unchanged installation. A pose
outside the absolute certificate or invalid telemetry is refused; no automatic
retry and no boot movement. The precise gates and residual physical assumptions
are in STARTUP_RECOVERY_OFFLINE_ASSESSMENT.md and the packaged final report.
