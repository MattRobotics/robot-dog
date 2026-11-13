# Ground Contact Sensorless — chiusura Fase 1, 13/11/2026

**Esito: diagnostica passiva implementata nel firmware operativo; Ground Contact Calibration motorizzata PARZIALE / NON ANCORA QUALIFICATA.** Nessun flash, movimento hardware o misura nuova sul robot. Nessun detector di contatto viene dedotto dalle HOLD.

## Provenienza e recuperabilità

- Repository: `MattRobotics/robot-dog`.
- Worktree dedicato: `/home/matteo-manicardi/MATDOG/worktrees/matdog-ground-contact-phase1-20261113`.
- Branch: `codex/ground-contact-phase1-20261113`.
- Baseline verificata: `c3b67d0e112f9416fa4f6663a9245a8ed5724e43`, ramo `integ/motion-vnext-battery-p1-dev7-20261111`, coincidente con il remoto al controllo del 13/11. Il vecchio checkout principale `7b258b3` era sporco ed è rimasto intatto.
- Commit del codice: [`5472b65dfa2835f7a410a5261bbd6018b991fa95`](https://github.com/MattRobotics/robot-dog/commit/5472b65dfa2835f7a410a5261bbd6018b991fa95), pubblicato e verificato con `git ls-remote`. Il commit successivo aggiunge soltanto questo rapporto.
- Immagine HW storicamente documentata: `1.0.2-dev.7`, sorgente `b5610620d0cef40cdc34bd62bf6256857bc9cfde`, APP SHA256 `28744982e293d5a56a0246c7ccacd083d1a0040f70ece29da4ac72fbfe6053d4`, sessione `MATDOG_1_0_2_DEV7_HW_20261111T1849Z`. Nessuna lettura del dispositivo in questa sessione: identità attualmente installata **UNKNOWN**, non dedotta dal branch.
- Tutte le modifiche iniziali appartenevano a questa lavorazione, UID locale 1000. Alla ripresa nessuna compilazione/test precedente risultava ancora attiva; Claude risultava attivo, senza accessi di scrittura da questa sessione al suo worktree. Non è una garanzia sulle attività future di altri processi.
- Copia preventiva completa, inclusi file nuovi: `~/MATDOG/verification-artifacts/MATDOG_GROUND_CONTACT_PHASE1_20261113/recovery-20261113T202135Z/` (`uncommitted-files.tar.gz`, patch, stato Git, HEAD, hash per file). Nessun materiale preesistente sovrascritto.

## Funzioni effettive

Nella dashboard esistente: **Calibration & Maintenance → Ground Contact Calibration**. Selezione LF/RF/LH/RH, disponibilità/freschezza, comando `GROUND_CONTACT.OBSERVE` via `/api/motion/command`, risposta in `/api/motion/status`, STOP tramite il percorso esistente e download della traccia della pagina. Full Calibration dei 24 contatti e relativo stato persistente restano distinti e invariati.

Il comando acquisisce **12 sweep nuovi**, non 12 cicli di probing, con limite operativo di 5 secondi. Quantità e timeout limitano memoria/lavoro: non sono soglie hardware né criteri di accuratezza. Un risultato per volta resta in RAM; il successivo comando lo sostituisce. La traccia scaricata conserva le risposte osservate dal browser, non ogni singola lettura bus.

Acquisizione soltanto da STAND/HOLD con coppia ON e disponibilità Motion/IMU. Riutilizza il polling esistente: due letture FAST per 20 ms, nominalmente circa 8,33 Hz per giunto. Nessuna lettura servo supplementare, nessun writer, nessun aumento della frequenza. I calcoli avvengono soltanto durante la richiesta e con almeno 5 ms di slack dichiarato dal Motion; la durata reale sul dispositivo rimane da osservare.

Ogni sweep richiede che tutti i 12 timestamp siano avanzati rispetto allo sweep precedente e al comando diagnostico. Si espongono timestamp delle tre articolazioni selezionate, IMU, ultimo invio del riferimento e sfasamento massimo della popolazione. I campioni **non sono simultanei**; i timestamp FAST datano la ricezione host, non il campionamento interno del servo. Il filtro conservativo richiede Speed zero: non dimostra immobilità al di sotto della risoluzione.

Il tracking è `PresentPosition RAW − commandedRaw`, in tick e nel segno RAW, non coppia articolare. Si usa il riferimento realmente accettato dal writer operativo; se un nuovo frame cambia RAW o drive durante la finestra, l'osservazione viene rifiutata con `GROUND_REFERENCE_CHANGED`. Non si usa il Goal letto dal blocco CONTROL, più vecchio, come comando contemporaneo.

La FK usa gli angoli misurati già convertiti dalla gen-1. Il punto più basso include larghezza finita e raccordo del battistrada canonico, lungo la verticale BNO085. `lowest_mm` è una coordinata verticale relativa all'origine del corpo, **non quota del terreno né distanza dal pavimento**. `target_lowest_mm` è la FK del riferimento Motion. Il Jacobiano mostrato è quello del riferimento di contatto esistente, in mm/rad: nessuna conversione di Load in N o N·m.

La variazione rispetto alla prima osservazione può essere accompagnata dalla traslazione verticale mediana inferita dagli altri tre piedi e dal loro disaccordo. Vale soltanto nell'ipotesi che restino fermi: **non verifica il sostegno** e non separa cedimento comune, slittamento, deformazioni e bias del modello. Gli intervalli minimo/massimo sono dispersione osservata, non intervalli di accuratezza o confidenza della quota terreno.

`ground_valid=false`, `probing_available=false`; release, touchdown, correzione e incertezza terreno sono `null`. La UI dichiara MISURA TERRENO NON VALIDA / NON DISPONIBILE. Nessun pulsante di probing motorizzato è presente. STOP cancella l'osservazione e non provoca ripartenza; un nuovo comando Motion la può sostituire senza essere subordinato al suo successo. Un rifiuto diagnostico non diventa un fault o un gate STAND/WALK.

## Evidenza HOLD recepita, senza nuova identificazione

Fonte di Claude Code: `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/report/REPORT_C_IDENT_HOLD.md`, SHA256 `1af08d252895c81d32768fa0dafb0254aeccb1304f3e15822c1c00e009d238d3`; sorgente nel suo worktree `06_Software/Matdog_MuJoCo/REPORT_C_IDENT_HOLD.md`. Analisi offline dei log HW dev.7 dell'11/11. Il rapporto non è duplicato né modificato, gli script di identificazione non sono stati rieseguiti.

Nelle HOLD, PresentLoad segue quasi deterministicamente il regolatore P (banda morta ±1 tick, pendenza 8‰/tick, punch 8‰); Current non aggiunge informazione sufficientemente indipendente. Load, Current e tracking **non costituiscono tre prove indipendenti** del contatto. Il valore C=0,0336 rad/N·m resta compatibile, non misurato indipendentemente; non si identificano separatamente HIP/UPPER/LOWER. Attrito statico e forze interne possono assorbire variazioni del comando senza movimento degli encoder.

Conseguenza implementata: nessuna classificazione di contatto o soglia assoluta di Load/Current. Una futura identificazione degli eventi richiederà movimento encoder effettivo, carico/scarico, direzione e ripetibilità. Un'eventuale stima di attrito/cedevolezza resta secondaria e futura, non requisito della diagnostica.

## Limite fisico e perimetro invariato

La verifica geometrica mirata ha riusato COM e margine del programma offline `walk_support_check.cpp` e il target STAND canonico; non è una nuova campagna MuJoCo. Margini del COM nel triangolo dei tre punti di appoggio, omettendo la zampa indicata:

| STAND nominale | LF | RF | LH | RH |
|---|---:|---:|---:|---:|
| 100 mm | +6,187 mm | +5,734 mm | −5,734 mm | −6,187 mm |
| 120 mm | +5,532 mm | +5,079 mm | −5,079 mm | −5,532 mm |

Sono margini **geometrici offline a contatti puntuali**, con COM/posa nominali: non forze misurate né qualificazione hardware della stabilità. Non autorizzano neppure il sollevamento anteriore sulla sola positività del margine. Non è stato aggiunto alcun trasferimento del peso. La procedura completa 3–5 cicli release/touchdown per zampa resta non implementata.

Il diff rispetto a `c3b67d0` lascia byte-identici `SimpleMotion.cpp/.h`, gait, box/limiti, mapping e calibrazione, servo/bus, NVS, URDF e dati geometrici generati. Nei file FK/IK sono aggiunte soltanto funzioni diagnostiche di proiezione/accesso al Jacobiano; i corpi delle funzioni preesistenti non sono cambiati. Il ramo STOP in MotionService aggiunge solo la cancellazione della finestra passiva prima del medesimo `motion_->stop()`. Browser senza IK o autorità servo.

## File modificati

Prefisso `05_Firmware/MATDOG_Controller/`:

| File | Motivo |
|---|---|
| `src/simple/GroundContactDiagnostic.h` (nuovo) | Finestra passiva finita, coerenza temporale, FK e statistiche RAM |
| `src/core/MotionService.cpp`, `.h` | Comando/ACK/stato, cancellazione, buffer JSON limitato a 7168 B |
| `src/core/MotionJson.cpp` | Parametro `leg` esclusivo del comando diagnostico |
| `src/core/CommandRouter.cpp` | Buffer USB allineato allo stesso limite JSON; nessun cambiamento ai comandi STOP |
| `src/motion/MotionContract.cpp`, `.h` | Intent diagnostico e validazione LF/RF/LH/RH |
| `src/motion/FootContact.cpp`, `.h` | Proiezione diagnostica del battistrada canonico, senza modifica del modello |
| `src/motion/TiltedContactIk.cpp`, `.h` | Accesso diagnostico allo stesso Jacobiano analitico |
| `src/network/MotionDashboard.h` | Scheda essenziale, solo intenti e dati del Controller |
| `scripts/tests/test_ground_contact.cpp` (nuovo) | Timestamp, rifiuti, STOP, assenza di IO aggiuntivo e budget JSON |
| `scripts/tests/run_simple_motion_tests.sh` | Selezione mirata `--ground-contact` |
| `scripts/tests/test_simple_motion.cpp` | Buffer capabilities della fixture aggiornato e return esplicito per riuso del rig |
| `scripts/tests/test_simple_dashboard_contract.cpp` | Fixture generata dal servizio operativo |
| `scripts/tests/test_simple_dashboard_dom.cjs` | Scheda, comando, dati, rifiuto stale e STOP |

A questi si aggiunge esclusivamente questo rapporto di chiusura. Nessuna modifica alla stringa di versione, agli algoritmi di SimpleMotion, alla Full Calibration o ai sorgenti di Claude.

## Verifiche e candidato

**Build operativa finale PASS — VERIFIED OFFLINE**, da commit pulito `5472b65dfa2835f7a410a5261bbd6018b991fa95`, build ID `5472b65dfa28`, UTC `2026-11-13T20:26:54Z`, profilo `ROBOT_POWERED`, OTA ingest 0, TLS privata assente come nella baseline documentata. La stringa versione ereditata resta `1.0.2-dev.7`: NON identifica la vecchia APP hardware; build ID e SHA distinguono questo candidato.

- APP: `~/MATDOG/verification-artifacts/MATDOG_GROUND_CONTACT_PHASE1_20261113/firmware-final/MATDOG_Controller.ino.bin`, **1.772.128 byte**.
- APP SHA256: `d8582e7d57d37e0b8afb9b1fb0f59e105d8a8de788aaaabe8302cf431f6c7a68`.
- Manifest SHA256: `7c764540746f386f61176021475df68f72041b81dd1c52fb801e7d7b07b9f8d8` (`firmware-final/matdog_build_manifest.txt`).
- Layout verificato `MATDOG_16M_2x5M_NVS_V1`, tabella SHA256 `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`; slot 5.242.880 B, occupazione 33,8%. Nessuna scrittura della tabella sul robot.
- Compilatore: 141.508 B globali, 186.172 B residui dichiarati; questo non misura heap/stack o timing sul dispositivo.
- Avvisi della libreria SCServo esistente (parametro inutilizzato e indice array in `SMS_STS.cpp`) conservati nel log; libreria non modificata. Nessun errore di build.
- **VERIFIED OFFLINE:** suite operativa SIMPLE completa con ASan/UBSan PASS dopo l'adeguamento del buffer della fixture capabilities: STAND/quote, WALK/joystick, STOP, allarmi, comunicazione/deadline e PRONE. `simple_tests_closeout.log`. Nessuna qualificazione fisica implicita.

- **VERIFIED OFFLINE:** test mirato Ground Contact con ASan/UBSan: quattro zampe, segni, FK/Jacobiano, campioni asincroni, timestamp comando/feedback, cache non contata due volte, rifiuto per cambio riferimento, stale/IMU/velocità, STOP/replay, numero di letture/frame/enable identico al controllo senza diagnostica. `ground_tests_closeout.log`.
- **VERIFIED OFFLINE:** fixture dashboard PASS (88 controlli), scheda Ground Contact PASS, cockpit DOM PASS (21 buste/92 comandi), joystick modalità PASS; contratto parser/origin/sessione/CSRF/MotionService/SimpleMotion PASS (6112 controlli). `offline-final/dashboard.log` e `results.json`. Socket HTTP, thread mailbox e STOP lane sono rappresentati dal test e ancorati al sorgente, non eseguiti su ESP32.
- **VERIFIED OFFLINE:** controllo diff dei percorsi protetti, `protected-diff.txt`; nessuna modifica agli algoritmi locomotori o alla persistenza.
- **Non completato:** il wrapper dashboard esteso si è fermato nel viewer 3D per pacchetto locale `three` assente. DOM/cockpit/joystick erano PASS; il contratto essenziale è stato completato separatamente. Nessun cambiamento o installazione nel viewer. I test browser grafici opzionali non sono stati eseguiti.
- **Correzioni durante verifica:** include IMU mancante nella prima compilazione host; test capabilities da 2048 a 4096 B; buffer USB da 6144 al limite comune 7168 B. Tentativi falliti conservati, non presentati come PASS.
- **IMPLEMENTED NOT VALIDATED:** acquisizione e UI sulla build ESP32-S3 operativa; esecuzione reale, latenza addizionale, utilità dei dati e tasso di finestre accettate.
- **UNKNOWN:** touchdown/release, quota terreno assoluta, precisione, forza trasmessa, supporto effettivo, sicurezza del probing e trasferibilità dinamica a WALK/TROT.

## Artefatti, primo collaudo e handoff

Radice persistente: `~/MATDOG/verification-artifacts/MATDOG_GROUND_CONTACT_PHASE1_20261113/`. Conserva copia iniziale, build precedente, build finale/manifest, log e fixture offline, verifica geometrica `stand_probe_support.cpp/.txt`, patch finale comprensiva dei file nuovi, bundle Git incrementale (prerequisito: baseline `c3b67d0`), copia finale dei file modificati, ricevuta di pubblicazione e `SHA256SUMS`. Le immagini e i log restano locali; su GitHub vengono pubblicati codice, test e questo rapporto, senza credenziali, NVS o dump.

Dopo un eventuale flash **separatamente autorizzato**, controllare build ID/SHA, raggiungere STAND con il percorso operativo normale e tenere STOP disponibile. Aprire la scheda, scegliere una zampa e premere *Acquisisci finestra in STAND*: il comando non deve generare movimento. Leggere 12/12 campioni oppure il motivo esatto di rifiuto; scaricare la diagnostica. Ripetere manualmente sulle altre zampe; provare STOP durante una finestra e verificare che non riparta. Non sollevare/forzare manualmente zampe con coppia ON. Monitorare timing Motion e disponibilità della risposta; non dedurre un errore terreno dalla dispersione.

**Handoff per Claude Code:** patch isolata sulla base operativa `c3b67d0`, senza incorporare vNext/MuJoCo. Per l'integrazione futura rivedere principalmente MotionService, MotionContract, MotionJson e MotionDashboard; SimpleMotion e gait sono invariati. Nessun merge eseguito. Non usare la UI passiva come qualifica del detector. Residui: (1) integrazione e collaudo supervisionato della diagnostica sul prossimo flash autorizzato; (2) evidenza di un sostegno/trasferimento del carico sicuro prima di autorizzare probing; (3) solo dopo, cicli release/touchdown con movimento osservato e ripetibilità, senza interpretare le HOLD come misura indipendente di forza.
