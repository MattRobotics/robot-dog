# MATDOG — M0.1: migration write and recovery hardening

Data: 2026-10-02. Branch `feat/calibration-persistence-record-store-v1`.
HEAD iniziale verificato: `ebb6078`. Sorgente firmware immutabile:
`be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`.

**VERDICT: PASS** — entrambi i finding corretti, verifiche offline completate.
Nessun hardware collegato/aperto dai controlli; nessuna operazione hardware,
flash, erase, reset, OTA, movimento, push, PR o merge eseguita.

## File del correttivo

| File | Intervento |
|---|---|
| `05_Firmware/MATDOG_Controller/FLASH_LAYOUT_M0_RUNBOOK.md` | Sostituisce tutte le W CLI con invocazioni singole del writer; mantiene verify/readback indipendenti, gate e ordine; precisa USB-C e isolamento fisico |
| `05_Firmware/MATDOG_Controller/scripts/migration_m0_write.py` | Writer M0 dedicato, operazioni/range chiusi, preflight file prima della porta, ROM senza retry/reset/fallback |
| `05_Firmware/MATDOG_Controller/scripts/migration_m0_esptool531.json` | Versioni e SHA dei sorgenti esptool/pyserial, binario congelato M0 e sdist ufficiale |
| `05_Firmware/MATDOG_Controller/scripts/tests/test_migration_m0_write.py` | Invocazione CLI reale del writer, API esptool reale, trasporto SLIP/flash simulati, fault injection e artefatto USB originale |
| Questo rapporto | Percorsi auditati, risultati, invarianti e limiti |

`migration_m0.py`, i verificatori esistenti, il rapporto M0 e il bundle M0
originale non sono stati modificati. Nessuna modifica a P3a, `.ino`, `src/`,
partition table/CSV, build/upload, protocollo NVS, gait, calibrazione o motion.
Il correttivo non entra nelle translation unit Arduino e non richiede build.

## Chiusura dei due finding

**HIGH — retry/reconnect impliciti: corretto.** Le W ordinarie e R1/R2 passano
ora solo dal writer M0.1, con una sola operazione/artefatto per processo.
Il writer impone `ESPLoader.WRITE_FLASH_ATTEMPTS = 1` e
`esptool.loader.WRITE_BLOCK_ATTEMPTS = 1`, controllandoli sul loader istanziato,
sul modulo e sui globals della funzione `flash_block` effettivamente invocata.
Il binding `SerialException` di `cmds.write_flash` è verificato contro pyserial.
Non è sufficiente impostare le due variabili: i percorsi ulteriori sotto sono
stati esaminati e limitati. Nessun file del pacchetto Espressif è patchato.

**LOW — porta USB/alimentazione: corretto.** Il passo 5 e la checklist nominano
la USB-C integrata ESP32-S3 per alimentazione **e** comunicazione. La porta
esterna GPIO19/20/GND senza VBUS non alimenta la scheda. Sono richiesti batteria
e caricatore scollegati, assenza di fonte esterna, ramo TECNOIOT→ESP32 isolato,
rail servo/LED misurati 0 V, GPIO17/18, GPIO15/16 e GPIO47 isolati da periferiche
spente, BNO085 sulla 3V3 senza seconda fonte, BOOT/EN accessibili.
USB_ONLY non è protezione elettrica. Riferimenti: `04_Electronics/README.md`
(host link e dominio TECNOIOT) e `MATDOG_POWER_STATES_AND_CHARGING.md`
(backfeed dal caricatore e connettore esterno senza VBUS).

## Identità del codice esptool realmente esaminato

Nuovo bundle, distinto e separato da quello M0:

`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_M0_1_HARDENING_ebb6078_20261002T162810Z`

Il binario congelato 5.3.1 del core Arduino ha SHA
`4be9c2060450b6a5add9d909e353b405bab40a4781ddd79cf9fa9eb92f3f6a40`.
Lo sdist ufficiale PyPI 5.3.1, conservato in `sources/esptool-5.3.1.tar.gz`, ha
SHA `125781f36e6a2d08c484524a45f340694675368b5eeead9d0cb21b2034a91d98`.

Non ci si è basati soltanto sulla stringa di versione: dal PYZ del binario M0
sono stati letti **tutti i 26 moduli esptool inclusi**. Con CPython 3.13 isolato
è stato confrontato ricorsivamente il bytecode con la compilazione dei sorgenti
sdist: istruzioni, costanti/code object annidati, nomi, variabili, closure,
argomenti, stack, flag ed exception table corrispondono. Si escludono solo
metadati di percorso/riga; si considerano le modalità di ottimizzazione.
Esito `ALL_ESPTOOL_BYTECODE_MATCH`, prova in
`esptool-frozen-source-equivalence.json`. Il writer usa questi stessi sorgenti
non modificati via API nel venv Python 3.12, pyserial 3.5, senza il pacchetto
esptool 4.7 del Python di sistema. Prima della porta verifica i SHA di tutti
27 file `.py` distribuiti esptool e 28 file `.py` pyserial; il modulo CLI
`__main__` aggiuntivo è pinning preventivo e non è invocato.

La provenienza del download è in `package-provenance.json`; le dipendenze sono
in `requirements.lock.txt`. L'installazione e CPython di confronto sono soltanto
nel nuovo bundle. L'eseguibile M0 originale e i suoi artefatti restano integri.

## Percorso W e call graph degli errori

Invocazione runbook: `venv/bin/python -I scripts/migration_m0_write.py ...`.
`main → execute → prepare → load_tool → M0ROM.connect(no-reset, attempts=1)
→ flash_spi_attach(0) → cmds.write_flash → flash_begin → flash_block
→ check_command → command → trasporto`, quindi `cmds.verify_flash` obbligatorio
sullo stesso collegamento. Le verifiche **indipendenti** CLI e readback/cmp del
runbook restano passi successivi separati, prima della W successiva.

`prepare` riusa `verified_backup`, `legacy_table`, `verified_artifact` e i
controlli del layout. A/B devono essere file distinti, completi e identici,
con SHA approvato. Per W normale confronta manifest V2, BUILD_ID, profilo,
OTA=0, hash USB esatto e tabella V1. Per R1/R2 confronta ogni byte dell'artefatto
con la regione del backup originale. Offset, lunghezza e fine erase sono fissi.
Nessun `.merged.bin`, range libero, artefatto assente/hash errato viene accettato.
Il MAC previsto è fisso; il MAC ROM viene controllato nuovamente prima di W,
insieme a ESP32-S3, security compatibile, assenza di encryption e flash 16 MiB.

Riferimenti alle linee dei sorgenti 5.3.1 pinning nel venv:

| Percorso reale | Rischio originale / trattamento M0.1 |
|---|---|
| `loader.py:399,1130` | Due tentativi immagine e tre blocco; entrambi imposti/verificati a 1 |
| `cmds.py:1417,1483` | SerialException con reopen/connect di default: a tentativo 1/1 rilancia immediatamente; ramo reconnect irraggiungibile |
| `loader.py:755,865` | Ciclo sync e connessioni/reset: una connessione `no-reset`; primo errore di `command` diventa STOP e bypassa i catch FatalError/retry/fallback |
| `loader.py:562,637` | Una sola trasmissione della richiesta; lettura fino a 100 frame per correlare risposta, nessuna ritrasmissione W. Status non valido/timeout avvelena la sessione |
| `loader.py:1181` | Fallback security-info 20→12 byte dopo FatalError: errore del comando convertito in STOP, nessun fallback silenzioso |
| `cmds.py:1308,1359,1550` | Skip/diff e full reflash dopo mismatch: `skip_flashed=False`, `diff_with=[]`, NOR, nessun ciclo alternativo raggiungibile |
| `cmds.py:1520` | Flash finish/boot per stub: stub rifiutato, finish/reset/run_stub rifiutati; ROM non invia FLASH_END |
| `cmds.py:1573,2179` | write_flash può saltare MD5 non supportato: guard MD5 converte anche il digest ROM non supportato in STOP immediato; verify_flash API aggiuntiva obbligatoria propaga errore/mismatch, mai PASS silenzioso |
| `cmds.py:1650` | Attach CLI con recupero XMC/reset volatile NOR e catch che continuano: non utilizzato dal writer; attach ROM diretto, errore → STOP |
| `cmds.py:1808` | Configurazione SPI `keep`: 16 MiB già verificati, header invariati; parametri volatili, nessun erase aggiuntivo |
| `targets/esp32s3.py:311,331,346` | Disabilitazione watchdog volatile originale preservata; watchdog/hard reset vietati, nessun reset di chiusura |

M0ROM aggiunge controlli prima della trasmissione: un solo FLASH_BEGIN con
size/offset/blocksize/numero blocchi attesi; FLASH_DATA con sequenza unica,
byte/padding e checksum esatti. Un errore avvelena il collegamento e non consente
ulteriori comandi. Le eccezioni di protocollo diventano `Stop`, quelle di
trasporto si propagano; il main chiude soltanto la porta e termina con codice 2.
Nessuna chiamata a reset_chip, flash_finish, reconnect, writer successivo o
recovery in un handler. L'utente deve acquisire nuove letture indipendenti dello
stato, riconciliarle e rinnovare il gate prima di **qualsiasi** altra W.

Il SYNC ROM produce otto risposte a una richiesta; una scansione degli ACK o
una serie di blocchi consecutivi della stessa immagine non è un nuovo tentativo.
I test contano le richieste FLASH_BEGIN e la sequenza FLASH_DATA, non le risposte.
DTR/RTS sono impostati inattivi prima dell'open, `exclusive=True`, baud 115200.
Nessuna auto-detection, URL seriale, stub, compressione, NAND, encryption,
force, erase-all, modifica header, scrittura multipla o recupero automatico.

## Operazioni, erase e recupero

| Operazione | Gate | Byte W | Settori interessati |
|---|---|---|---|
| app0 candidato | B (oppure R nominativa di riparazione V1) | `[0x10000,0x121860)` | `[0x10000,0x122000)`, 274 settori |
| Tabella V1 | B (oppure R nominativa di riparazione V1) | `[0x8000,0x8C00)` | `[0x8000,0x9000)`, 1 settore |
| R1 legacy app0 | R | `[0x10000,0x310000)` | stesso range |
| R1 otadata | R, solo se necessario | `[0xE000,0x10000)` | stesso range |
| R1 tabella legacy completa | R, per ultima | `[0x8000,0x9000)` | stesso range |
| R1 default NVS / coredump | R nominativa, solo se alterati | `[0x9000,0xE000)` / `[0xFF0000,0x1000000)` | stessi range |
| R2 backup completo | R2 distinta | `[0,0x1000000)` | tutti i settori, senza comando erase-flash |

Le due W della migrazione restano app0 **prima** della tabella, senza W su
bootloader, otadata, default NVS, matdog_nvs, FFAT o coredump. Gli interventi R1/R2
sono recovery distinto; nessuna autorizzazione è generata dal software o dal
flag `--gate`. Il gate C/primo boot resta separato. Una riparazione V1 non è
rollback legacy; un seed otadata non identico al backup è rifiutato e richiede
il piano separato fuori dal writer M0.1, senza bypass CLI.

**Percorso di recupero: verificato offline.** ROM può operare anche con tabella
parzialmente scritta. R1 riscrive byte originali con tabella per ultima; R2 usa
l'intero backup device-specific A/B. Restano obbligatorie le due acquisizioni
indipendenti post-recovery e il confronto **integrale** col backup per dichiarare
rollback byte-identico. Full restore non ripristina eFuse né trasferisce security.
Una W fallita potrebbe avere cancellato settori o programmato parte di un blocco:
nessun exit code STOP attesta lo stato della flash, nessun recovery è automatico.

Nessun byte/header/checksum dell'immagine approvata è cambiato. Il test con
l'artefatto USB originale verifica 1.120.352 byte invariati, padding FF fino a
`0x122000`, erase esatto e nessuna differenza fuori range. Il trasporto conserva
pacchetti ROM da 1.024 byte e checksum esptool. R2 completo simulato confronta
ogni byte dei 16 MiB, non soltanto un header o hash parziale.

## Verifiche finali

| Controllo | Risultato |
|---|---|
| Test M0 esistenti | PASS: migration 42, layout 57, manifest 83, backup gate 25, OTA logic 40; **247** totali |
| Fault injection writer finale | PASS: **16 test**, **83 invocazioni** (11 successi, 72 STOP attesi); massimo 1 open e 1 FLASH_BEGIN, sequenze uniche, zero asserzioni reset |
| Comandi runbook | PASS: **18** blocchi bash via `bash -n`; tutte le 8 W esplicite usano il writer, nessuna W CLI |
| Sintassi Python / diff | AST PASS per writer/test; `git diff --check` PASS |
| Audit statico | PASS, 203 file sorgente esaminati; suite host e mutation richieste eseguite |
| Bundle M0 / export precedenti | PASS: stessi **3.603** file, size/SHA/inventario invariati; **16** export precedenti identici |
| Firmware approvato | Diff vuoto per `.ino`, `src/`, partition CSV e build.sh rispetto a be0c129; ricompilazione non richiesta |

I test eseguono il vero main/parser/preflight e l'API pinning 5.3.1, inclusi
connect, command/check_command, flash_begin/flash_block e verifiche. Si simulano
solo il trasporto seriale/SLIP, la flash e l'identificazione fisica MAC/flash-ID/
USB (VID/PID nativo 303a:1001, inclusa la disabilitazione watchdog volatile). Nessun mock del writer, di write_flash o dei due writer ROM. Gli artefatti
sintetici non sono firmware; un caso usa anche il candidato USB originale.

Fault coperti: errore open/sync/security/begin prima della programmazione,
SerialException sul primo blocco e dopo tre blocchi già inviati, timeout, status
non valido e 100 risposte non correlate, MD5 interno errato/timeout, verifica
aggiuntiva fallita o non supportata, errori R1/R2, artefatto mancante, hash errato,
offset/range/byte non autorizzati, gate/porta/MAC/backup/manifest errati, drift
dei contatori e tool versione/hash errati. I test verificano che il guasto abbia
raggiunto la fase prevista, massimo un open/FLASH_BEGIN, nessun seq duplicato,
nessun reset/finish/erase-all e nessuna alterazione fuori range.

Log, contatori per invocazione e attestazioni sono nel bundle M0.1:
`logs/writer-tests-final.log`, `writer-fault-injection-final.json`,
`logs/test_*.log`, `logs/static-audit-system.log`,
`integrity-and-syntax.json`, `scope-and-command-audit.json`.
Il primo audit nel venv nuovo falliva perché mancava NumPy, non per difetti
firmware; la riesecuzione con l'ambiente numerico esistente del progetto
(NumPy 1.26.4, SciPy 1.11.4) è PASS. Nessun sorgente geometria è stato modificato.
Il pacchetto NumPy installato transitoriamente nel nuovo venv è stato rimosso;
il writer e i suoi pinning non richiedono librerie numeriche.

## NVS e limiti residui

Il criterio del passo 18 **resta corretto**: nel flusso approvato USB_ONLY con
inizializzazione della partizione vuota, LOAD e STATUS su namespace assente,
64 KiB `matdog_nvs` restano FF. Il correttivo non modifica firmware o NVS.
La revisione M0 ha seguito i sorgenti **ESP-IDF 5.5.5 b774170ff46** archiviati nel
bundle, non stub: `nvs_page.cpp` (load/initialize/writeItem),
`nvs_pagemanager.cpp` (load/activatePage), `nvs_storage.cpp`
(createOrOpenNamespace con canCreate=false). Attivazione e numero di sequenza
sono stato RAM; l'header è scritto da initialize sul percorso writeItem,
non dal LOAD/STATUS di questo backend READONLY. Resta vietato fare erase o
modificare il firmware per forzare una coincidenza documentale.

Restano da verificare sul dispositivo, con gate distinti, stato corrente,
identità/security, acquisizioni fresche e loro reale indipendenza, isolamento
misurato, stabilità alimentazione/USB, comportamento del driver e accesso ROM.
Le chiamate software non impongono reset; eventuali transitori fisici di porta
non sono dimostrabili dal simulatore e impongono STOP. BOOT resta basso durante
W. Nessuna promessa di atomicità app/tabella, fallback app1 o resistenza a una
reale interruzione di potenza è dedotta dai test offline.

Commit correttivo locale unico solo dopo completamento positivo dei controlli;
nessun altro commit, push, PR o merge. Nessuna autorizzazione hardware ottenuta
o presunta. Chiusura limitata a M0.1.

`M0_RUNBOOK_READY=YES`

`GATE_A_READONLY_READY=YES`

`HARDWARE_FLASH_READY=NO`
