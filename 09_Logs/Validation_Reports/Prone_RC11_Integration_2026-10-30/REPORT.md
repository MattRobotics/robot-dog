# MATDOG — PRONE su rc.11 r2, 30/10/2026

**IMPLEMENTED NOT VALIDATED.** Integrazione offline completata e compilata come `1.0.1-dev.3`. Nessun flash, boot, accesso a dispositivi o movimento. La rc.11 installata e i suoi artefatti sono invariati. PRONE termina a torque ON in `PRONE_CONTACT_UNVERIFIED`: non è un ritorno autonomo a corpo completamente appoggiato e non autorizza SAFE_OFF automatico.

## Identità e isolamento

- Worktree: `/home/matteo-manicardi/MATDOG/worktrees/matdog-prone-rc11-integration-20261030`.
- Branch: `codex/prone-rc11-integration-20261030`.
- Baseline fotografata: `8e20c63b87926a598637b17ca8e2b6fe842d9bcc`, ricostruita dal worktree rc.11 r2 dirty su f838f67. Il solo HEAD f838f67 **non** identifica i sorgenti installati.
- Verificati 207 C++/header contro le copie effettivamente compilate, rimuovendo soltanto il prefisso Arduino `#line 1`. Anche lo sketch INO coincide, escluse le righe/prototipi generati da Arduino. Markdown ed esempi `.local.h.example` non sono unità compilate.
- APP rc.11 r2 originale: `fc0a7141f827c37cdd5bccb888951ca280bd2a981b1572de8eef7c74474170be`, BUILD_UTC 2026-10-29T07:11:07Z; hash verificato sul file di build e collegato al report/readback del collaudo 30/10.
- Dev.2 importata selettivamente da `6c5cffc`, delta rispetto alla sua fotografia rc.10 `7118f7b`. Nessun merge/cherry-pick/reset/clean.
- Prima integrazione: `65b6564`; correzione cinematica e sorgente compilato: **`c2f11a55f9c46ef125a701c50e96f5bc8bc5e9e3`**, CLEAN. I commit successivi contengono soltanto documentazione/helper offline.
- 164 file protetti identici alla baseline: attuatori, calibrazione, servo, FK/IK/route, CAD/URDF, collision box, planner, sketch/layout. Sette soli file `src/` cambiano. Nessuna modifica a gen1, PID, EEPROM, TorqueLimit, NVS o limiti.
- `provenance.json`, `baseline-source-sha256.json`, `protected-files.json`: prove e digest. Stato/diff del worktree rc.11 confrontati prima e dopo; invariati. Worktree dev.2 preservato.

## Evidenza nuova e decisione sui margini

Il report hardware del 30/10 attesta STAND150 completato con rc.11 r2, HOLD 5,8 s, due picchi termici filtrati, nessun fault. Allo STOP il corpo è caduto ed è stato trattenuto dall'anticaduta. Questo è un risultato hardware preesistente, non una prova della nuova candidata.

Il replay usa 23 gruppi HOLD completi da ciascuna registrazione:

- 50 mm: `MATDOG_RC9_LOAD_TEST2_20261028T212801Z/01-load-stand50/joints_timeseries.csv`;
- 100 mm: `MATDOG_RC10_HW_20261029T055148Z/02-stand100/joints_timeseries.csv`;
- 150 mm: `MATDOG_RC11_HW_20261030T060305Z/02-stand150/joints_timeseries.csv`.

Percorsi completi, hash, errori, carichi e raw sono conservati in `scripts/offline_prone/hardware_residuals.json`. Si trasforma correttamente il carico **firmato** e l'errore secondo la direzione encoder. Il port host ha encoder positivo e q0=2048; i limiti rad sono quelli gen1. Ogni errore/carico HOLD registrato è mantenuto costante durante una risposta servo sintetica istantanea, con IMU piatta. Non si riproducono tempi, attrito, isteresi, elasticità o carichi reali della discesa; non si assume che il carico resti davvero costante.

**La semplice integrazione dev.2 non è stata accettata.** Da 50 mm 23/23 casi vanno in `PRONE_MEASURED_OUTSIDE_BOX` durante LIFT. Da 100/150 arrivano 46/46, ma restano RF_LOWER ≈2 tick da gen1 e LF_LOWER ≈1 tick dalla box (150). Il log originale include margini stampati dopo FAULT con phase IDLE: questi valori, riferiti erroneamente a ENTRY, non sono utilizzati; il fault e la fase precedente sono l'evidenza di blocco.

Correzione minima implementata **solo in beginProne**: i LOWER anteriori nei via di rientro hanno floor `−1.5246163880149817 rad`, cioè +0,020 rad / 13,04 tick rispetto al target più flesso della route STAND. Il floor interviene solo dove necessario: il primo target 50/53 mm resta identico; le giunzioni LIFT→DEPLOY→LAND condividono il medesimo target modificato. HIP rimane ±0,17 rad. UPPER, posteriori, box, calibrazioni e route STAND sono invariati. La correzione non insegue il sag encoder e non modifica i guadagni.

| Origine replay | Arrivi al terminale | Minimo gen1, tutti i giunti | Minimo box, tutti i giunti | LF_LOWER: gen1 / box | RF_LOWER: gen1 / box |
|---|---:|---:|---:|---:|---:|
| STAND50 | 23/23 | 17,00 tick, RF_LOWER | 10,009 tick, LF_LOWER | 19,00 / 10,009 | 17,00 / 20,009 |
| STAND100 | 23/23 | 15,00 tick, RF_LOWER | 5,675 tick, LH_UPPER | 24,00 / 15,009 | 15,00 / 18,009 |
| STAND150 | 23/23 | 18,00 tick, RF_LOWER | 9,213 tick, LF_UPPER | 23,00 / 14,009 | 18,00 / 19,462 |

Minimi sull'intera traiettoria, feedback quantizzato e compensazione Controller inclusi, non solo sulla posa finale. Valori completi per giunto/fase in `margin-summary.json`. Il test finale richiede margini >4 tick: equivale a conservare la box e i limiti anche con una perturbazione geometrica indipendente ±4 tick **attorno ai campioni del replay**. È una sensibilità offline, NON un limite misurato dell'incertezza. A 100 mm rimangono solo 1,675 tick dopo tale perturbazione; ±8 tick non è coperto. Non promuovere automaticamente 100/150 alla prova fisica.

## Traiettoria e posa finale

Controller SimpleMotion esistente, stesso ViaPlan e conversioni servo. Da 100/150: HEIGHT fino a **53 mm**, poi LIFT inverso, DEPLOY inverso con il floor anteriore corretto e chiusura HIP ±0,17. Da 50: direttamente LIFT inverso. Il precedente problema HEIGHT100→50 rimane: 23/23 timeout, mentre 100→53 completa in entrambe le box. Non è stato allargato kHeight.

Dopo il passaggio a 53 mm, durate nominali senza dwell: LIFT 2,495 s, DEPLOY 3,183 s, HIP 2,812 s. Velocità/accelerazione .55/1.10 rad/s e rad/s², ultima fase .10/.20; profilo servo RAM 480/13 invariato. Continuità e inviluppo del planner verificati, massimo osservato 8 tick/frame nel Controller. Appartenenza continua alle box tramite inviluppo dei via del planner, oltre al campionamento a 10 ms; nessuna nuova prova FCL necessaria perché le box esistenti non cambiano.

Posa nominale finale, HIP/UPPER/LOWER in rad:

| Zampa | HIP | UPPER | LOWER |
|---|---:|---:|---:|
| LF | +0,170000 | 1,461526 | −1,524616 |
| RF | −0,170000 | 1,461526 | −1,524616 |
| RH | −0,170000 | 1,570796 | −1,553343 |
| LH | +0,170000 | 1,570796 | −1,553343 |

Nuovo controllo geometrico circoscritto al delta, con tutte le mesh collision URDF: appoggio rigido statico ventrale anteriore + due piedi posteriori, pitch 4,689°, quota origine corpo 9,626 mm, ventre posteriore **20,322 mm** dal piano. Il floor LOWER riduce il distacco nominale dei piedi anteriori da 2,27 mm a **0,170 mm**: non è un margine robusto per provare il contatto ventrale reale. Possibile contatto quasi simultaneo dei piedi, deformazione o ventre leggermente sollevato. Il terminale conserva quindi torque ON e non afferma appoggio.

Con massa 2,8 kg e CAD scalato uniformemente: ventre 10,814–10,901 N, posteriori **16,557–16,645 N**, circa 60,3–60,6% del peso. Margine statico nominale 68,08 mm; minimo lungo gli 89 campioni CAD 68,03 mm, minima distanza di link non-foot/non-base dal piano 1,689 mm. Questi sono appoggi di equilibrio calcolati, non una prova di assenza di scivolamento durante il moto.

Sui 69 endpoint Controller: contatto ventrale previsto nel modello, margine statico minimo 39,91 mm; gap posteriore 19,712 mm (50), 19,136–19,260 mm (100), 20,805–20,864 mm (150). Non si riapre lo studio posteriore già concluso: nessuna nuova evidenza consente di trasferire automaticamente quel peso al ventre. Nessun assestamento passivo o SAFE_OFF finale viene qualificato da questi numeri.

## Protezioni e dashboard

La finestra termica r2, l'isteresi, il vincolo di continuità ≤250 ms, i contatori e l'allarme confermato ≥0,5 s sono conservati senza modifiche. Diff di SimpleMotion senza righe termiche alterate; 67 test r2 più 10 casi nuovi PRONE. Status e tensione mantengono il fault immediato, STOP conserva priorità e SAFE_OFF; nessun fault avvia una discesa di recupero.

Non si aggiungono task, server o polling sul bus. Dashboard: PRONE sperimentale, stato terminale esplicito, età snapshot e stale veritieri, /status 503 del budget Motion presentato come telemetria generale sospesa, STOP indipendente dal polling pendente. Sessione/CSRF restano richiesti. I test HTTP eseguono parser/policy/service e DOM, con ordine HTTPD/mailbox ancorato al sorgente: non esercitano socket o concorrenza ESP32. Nessuna nuova misura WCET/latency hardware; budget letture host invariato.

## Build e risultato

- **Build exit 0**, `1.0.1-dev.3`, sorgente CLEAN `c2f11a55f9c46ef125a701c50e96f5bc8bc5e9e3`.
- BUILD_UTC **2026-10-30T06:36:58Z**; ROBOT_POWERED, OTA ingest 0, MOTION_AUTHORIZED 0, SUPERVISED_VALIDATION_CAPABLE 0, BOOT_MOTION_AUTHORIZED 0. Questi flag non autorizzano il collaudo.
- APP **1.629.408 byte**, SHA256 **`eda0cf08ceb2c6c46a729ac8caf8a62ced06f6ad3428c6d80a90ec28e28ecb0d`**.
- Partition table invariata: `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`.
- Due warning esterni SCServo già noti: ACC inutilizzato e indice −23 in ReadMode. Non corretti né dichiarati innocui; static audit completo non rieseguito.
- Build separata: `/home/matteo-manicardi/MATDOG/build-output/matdog-prone-rc11-dev3-20261030`.

Pacchetto: `/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_PRONE_RC11_INTEGRATION_20261030`. Contiene APP/manifest, log, geometria, replay originali e corretti, provenienza e diff. `01-integration-vs-installed-rc11.patch` è il delta firmware completo su 8e20c63; `02-front-lower-correction-only.patch` isola la correzione rispetto a 65b6564 (non applicarla due volte). Vedere RESULTS e HANDOFF per esiti e criteri della sola prima prova STAND50.
