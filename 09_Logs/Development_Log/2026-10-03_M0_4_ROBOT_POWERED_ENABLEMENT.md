# MATDOG M0.4 — ROBOT_POWERED migration enablement

Data: 2026-10-03. Branch `feat/calibration-persistence-record-store-v1`.
Checkpoint ripreso: `2757604400db34e3107abacd994ca6fb9d0bb515` (M0.2), checkout
inizialmente pulito. Commit M0.4: il commit locale che contiene questo rapporto;
SHA completo registrato dopo il commit nel receipt `checkpoint.json` del bundle
e nel rapporto conclusivo. Nessun push, PR o merge.

**Esito: preparazione software PASS.** ROBOT_POWERED qualificato nel software,
writer pronto per quella coppia immagine/manifest esatta, pacchetto operativo
preparato. Nessun accesso USB, flash, erase, reset, OTA, JTAG o movimento.
I test del writer sostituiscono esclusivamente trasporto e identificazione
fisica con simulazioni locali. Nessuna ricompilazione firmware.

## Modifiche introdotte

- `migration_m0.py`: profilo atteso esplicito ROBOT_POWERED per plan e
  check-snapshot; scelta chiusa USB_ONLY/ROBOT_POWERED, senza inferenza dal
  manifest. Il default USB_ONLY conserva il rifiuto di un candidato powered
  non nominato. SHA globale intermedio nel report e controllo condiviso dei
  dati legacy nelle aree che cambiano destinazione.
- `migration_m0_write.py`: pin per dimensione, SHA app e SHA manifest di ciascun
  candidato archiviato; controllo del profilo conservato. Controlli aggiuntivi
  prima della porta su app1 legacy intera e padding del settore tabella, oltre
  a nuova app1, FFAT e destinazione matdog_nvs. Nessun flag force/generico.
- Test: stessa matrice dell'API esptool reale sotto i due profili, compresi i
  due candidati originali. Rifiuti per profilo errato, app valida ma non
  approvata, manifest riprodotto con byte diversi e dati legacy occupati;
  plan e check-snapshot CLI powered in entrambe le fasi.
- Nuovo `FLASH_LAYOUT_M0_4_ROBOT_POWERED_RUNBOOK.md`; rimando in apertura del
  runbook storico M0.2. Il percorso corrente usa batteria/DALY/TECNOIOT e USB
  esistente senza VBUS, usb-reset una volta per entrare ROM, poi no-reset.
  Accetta il rischio straordinario USB già deciso dall'utente e non pone JTAG
  o dimostrazione di recupero da ogni guasto come prerequisiti.

Restano invariati firmware `.ino`/`src`, partition CSV, build, Manifest V2,
NVS backend, OTA, calibrazione e movimento. I controlli del writer restano
ESP32-S3/MAC nominativo, flash 16 MiB, security compatibile, backup A/B
distinti e verificati, layout/hash esatti, offset e erase fissi, byte/padding
esatti, un solo FLASH_BEGIN e sequenze FLASH_DATA uniche. Contatori immagine
e blocco 1/1, nessun retry/reconnect/reset/stub/finish, verifica API obbligatoria
e indipendente prima della W successiva, nessun recovery automatico.

## Qualificazione del candidato archiviato

Fonte CLEAN `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`, BUILD_ID
`be0c12979e5b`; clone source-robot al commit esatto e firmware corrente
byte-identico nei sorgenti. Tutti i 12 artefatti del riepilogo ROBOT originale
sono stati riverificati per size/SHA; tutte le **64** unità sketch compilate
recano ROBOT_POWERED e OTA ingest 0. Manifest V2, FQBN N16R8/OPI e layout V1
passano i verificatori esistenti. Image-info offline: chip ID 9, flash 16MB,
revision v0.2 ammessa, checksum `0x90` valido, validation hash valido e SHA ELF
coincidente con l'ELF archiviato. Nessun motivo tecnico per ricompilare.

| Selezione | Byte | SHA-256 |
|---|---:|---|
| ROBOT_POWERED `MATDOG_Controller.ino.bin` | 1.123.472 | `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0` |
| Tabella `MATDOG_16M_2x5M_NVS_V1` | 3.072 | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |
| Manifest V2 ROBOT_POWERED | 637 | `d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48` |

Primo avvio: authority NONE, permit reset e motion authorization revocata,
nessuna sessione automatica né trasformazione ammessa dalla semplice bind
geometria. LOAD usa soltanto matdog_nvs in lettura e scarta il record;
nessuna importazione della NVS legacy o RESTORE/promozione automatica.
Anche un record valido confermato e disponibile non dà authority; un pending
o un record non confermato non abilita il percorso motion. ServoBus begin
non invia torque-on/target/census; DALY fa polling senza configurazioni
automatiche; LED powered riceve OFF iniziale e poi segue la policy.
READY/NO_RECORD e motion_authorized=0 sono attesi sulla matdog_nvs vuota.
Il software non spegne automaticamente il torque già presente nei servo:
serve evidenza fisica corrente dei 13 servo e alimentazione continua.

## Integrità e dati legacy

A/B originali del Gate A: directory
`/home/matteo-manicardi/MATDOG/backups/esp32/m0-20261002T173142Z-14c19f227594`,
entrambi 16.777.216 byte, file distinti, identici, SHA
`856435baa1402086bd38dee0a4c1e0c6ee13e8d814cfc7ddc8bed30dbed5afb7`.
**3.611 file** (8 backup/log originali + 3.603 bundle M0 originale) verificati
senza alcuna variazione di inventario, dimensione o SHA.

Il piano powered passa sul backup reale: bootloader e legacy table esatti,
otadata stabile seleziona app0, runtime noto dfcecb670d05 @0x10000. App1
legacy `[0x310000,0x610000)`, espansione app0, nuova app1, FFAT e destinazione
NVS sono FF; anche `[0x8C00,0x9000)` è FF. La tabella scrive 3072 B e cancella
solo `[0x8000,0x9000)`, prima della NVS legacy occupata (15.747 byte non-FF).
I metadati della tabella non provocano erase delle partizioni. Nessun dato
occupato viene cancellato fuori dall'app0 legacy deliberatamente sostituita
e dalla tabella. Default NVS, otadata, bootloader e coredump sono preservati
byte per byte nelle simulazioni; controllo globale richiesto sull'hardware.

App powered: W `[0x10000,0x122490)`, erase `[0x10000,0x123000)` (275 settori),
readback `0x113000`, padding FF `0xB70`. Tabella: W `[0x8000,0x8C00)`, erase
un settore fino a `0x9000`. App0 prima della tabella, nessun boot intermedio.
Snapshot globale dopo app atteso
`ff0ae739c3cdbb3106cf410a90c55d96ce3a43607bc6c655aba913dfe293e4dd`;
finale dopo tabella
`f02f742f13c224da722c2c2dbb94a4d7a423bd16df1b99a39b61f0d5ff87bb40`.

## Validazione ed evidenze

Bundle separato, senza modifiche ai bundle precedenti:

`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_4_ROBOT_POWERED_2757604_20261003`

| Verifica locale | Esito |
|---|---|
| M0 migration / layout / manifest / backup / OTA | PASS: 46 + 57 + 83 + 25 + 40 = **251 test** |
| Writer, entrambi i profili e candidati originali | PASS: **40 test**, nessuno saltato; **202 invocazioni**, 24 successi e 178 STOP attesi |
| Vincoli API simulata | Max 1 open e 1 FLASH_BEGIN per invocazione, sequenze uniche, **0 reset** |
| Audit firmware | PASS, **203 sorgenti**, suite C++ host/persistence/policy/LED powered e mutation richieste incluse |
| Snapshot su backup reale | PASS, 2 stati attesi e 10 corruzioni rifiutate |
| Preflight file-only del pacchetto | PASS, prepare app0/table e tool pinning; HARDWARE_IO=NO, AUTHORIZATION_GRANTED=NO |
| Sintassi documenti | PASS, 20 + 8 blocchi bash; Python AST e git diff --check PASS |
| Artefatti e input protetti | PASS, hash/dimensioni/provenienza esatti, originali invariati |

Log in `logs/*-tests-final.log`, `logs/writer-tests.log`,
`logs/static-audit.log`, `logs/image-info-powered.log`,
`logs/local-preflight.log`, `logs/powered-plan.log`. Attestazioni:
`validation-summary.json`, `writer-fault-injection.json`,
`candidate-qualification.json`, `compile-profile-qualification.json`,
`physical-ranges-and-preservation.json`, `protected-inputs-verification.json`.
Pacchetto: `artifacts`, `tools`, `plan`, `package-manifest.json`,
`package-files.sha256`, `context.env`, `preflight-local.sh`, `RUNBOOK.md`.
Il venv M0.1 e il binario esptool 5.3.1 originali rimangono dipendenze esplicite
verificate, con lock/provenienza/equivalenza archiviati nel pacchetto.

## Preflight hardware successivo e impedimenti

Procedura esatta e comandi separati nel
[runbook M0.4](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_M0_4_ROBOT_POWERED_RUNBOOK.md).
Il controllo locale ripetibile è:

```bash
bash /home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_4_ROBOT_POWERED_2757604_20261003/preflight-local.sh
```

Preflight fisico, solo dopo autorizzazione distinta: alimentazione stabile e
robot sostenuto, stato runtime/authority/OTA, MAINTENANCE e letture dei 13
servo nominativi con torque/speed/current zero; chiusura monitor senza reset;
un usb-reset per ROM se necessario; no-reset per identità/security/flash-ID
e due letture complete di stato in nuova directory, confrontate con A/B e
SHA atteso; piano powered rigenerato e copia di sicurezza/receipt verificati.
Se già ROM non bootare per ottenere un transcript: servono evidenze correnti
di sicurezza periferiche e continuità di alimentazione, altrimenti STOP.

Nessun impedimento software concreto residuo. **Restano non attestati oggi**
preflight hardware, confronto dello stato corrente e receipt su secondo
supporto: prima della W occorre soddisfarli. Differenze rispetto al backup
richiedono riconciliazione e piano aggiornato dopo revisione, senza cambiare
opportunisticamente il SHA approvato. I dati M0.2 non attestano il robot attuale.
Nessuna nuova autorizzazione hardware è stata acquisita o presunta; la sequenza
W1/verifiche/W2/verifiche è preparata ma non eseguita. Primo boot e recovery
mantengono gate separati, senza automatismi.

```text
M0_4_SOFTWARE_READY=YES
ROBOT_POWERED_QUALIFIED=YES
WRITER_POWERED_READY=YES
HARDWARE_PREFLIGHT_REQUIRED=YES
HARDWARE_FLASH_AUTHORIZED=NO
```

ROBOT_POWERED_QUALIFIED si riferisce al candidato software esatto; non dichiara
il preflight powered fisico già passato o la scrittura effettiva autorizzata.
