# MATDOG Wi-Fi / provisioning / OTA — V3 offline qualification, 2026-10-04

## Esito e limiti

Candidato software **provvisorio** nel worktree indipendente
`/home/matteo-manicardi/MATDOG/worktrees/robot-dog-wifi-ota-shelly`, branch
`feat/controller-wifi-ota-shelly-v1`. Base immutabile
`1a5e0085098eeb907319f6f1a00693869444d9cf`.

La sessione della Full Calibration ha evidenza hardware 24/24 del 2026-10-01, ma
non dimostra il successivo SAVE/ACK/LOAD della persistenza e il firmware post-abort.
Gli ultimi report locali descrivono OFFLINE_READY. **CAL_PERSIST_BASE = BLOCKED**:
nessun commit è stato inventato o promosso come base hardware validata. È consentito
sviluppo provvisorio; riconciliare la base validata prima della promozione finale.
Nessun merge, push, PR, flash, OTA reale, seriale, movimento, backup del dispositivo,
erase NVS/PHY o cambio della toolchain condivisa è stato eseguito.

## Inventario G0

| Checkout / worktree | HEAD rilevato | Destinazione |
|---|---|---|
| `github/robot-dog` | `7b258b36c257bd455f135aee2667d035c4929544` | Sessione persistenza, dirty: lasciata intatta |
| `github/robot-dog-post-abort` | `1a5e0085098eeb907319f6f1a00693869444d9cf` | Ultima integrazione locale, base provvisoria |
| `robot-dog-post-abort-flash-125d981` | `125d981b02a38673c8c54f2096632c99fd1f29ff` | Checkout detached preesistente |
| `robot-dog-post-abort-flash-1a5e008` | `1a5e0085098eeb907319f6f1a00693869444d9cf` | Checkout detached preesistente |
| `verification-artifacts/wifi-rebuild-20261003` | `a06314e8f3647bfb0f66d0fa827fcd24f65ffaca` | Diagnostica: nessun import di erase/marker PHY |
| `worktrees/robot-dog-gait-engine` | `e1704719979789cd9c9f18741e4546725558797e` | Branch gait indipendente |

Il commit di inventario è `1bf46222fa7bcccd495908d25fc266ef3bd580e8`.
Implementazione: `96656917813408eb0fda208913ac9b83f1b7b7c5`.
Sorgente finale, catena di commit, diffstat e hash degli artefatti sono vincolati nella
ricevuta generata dopo il freeze, conservata in
`05_Firmware/MATDOG_Controller/build/verification/FINAL_RECEIPT.md`.
Questo documento è versionato prima delle build CLEAN: la ricevuta chiude la provenienza
senza modificare HEAD dopo la compilazione finale ROBOT_POWERED.

## Matrice V3

| Gate | Stato software | Evidenza / limite hardware |
|---|---|---|
| G0 inventario | PASS, base provvisoria identificata | CAL_PERSIST_BASE hardware BLOCKED |
| G1 rete / sleep / roaming | IMPLEMENTED, host-tested | RF, roaming reale, jitter, leak, sleep USB TO_TEST; adapter NO_SLEEP |
| G2 SoftAP / HTML / NVS | IMPLEMENTED, host-tested | Provisioning, browser reale, recovery, power loss e isolamento calibrazione TO_TEST |
| G3 OTA | PARTIAL: TLS/HMAC e client implementati; ingest 0 | Reboot remoto BLOCKED/unimplemented; risorse TLS, E2E e rollback TO_TEST |
| G4 offline | Suite host, audit/mutation, DOM, TLS loopback e link | Build finali CLEAN USB_ONLY poi ROBOT_POWERED e metriche: ricevuta |
| G5 documentazione | PASS: docs canoniche, log, report, runbook | Nessuna autorizzazione hardware implicita |

## Implementazione

La radio è posseduta da un worker core-0 (8192 B stack, priorità 1, tick 50 ms).
Il Controller copia snapshot con mutex a zero attesa. Una coda di un elemento riserva
`config_busy` prima di accettare modifiche. Anche RUN e comandi di calibrazione/persistenza
rispettano la prenotazione. Callback HTTP/eventi Wi-Fi non chiamano attuatori.
Il worker può subire attese del driver; le latenze del Controller, flash cache, heap e
stack si misurano sul robot, non si deducono dalla separazione dei task.

Due profili, DHCP/statico validato, fallback, backoff 2–60 s, scansioni asincrone a massimo
12 risultati e roaming opzionale senza pin BSSID. Default HT20 e roaming OFF;
HT40/roaming sono da qualificare. AP WPA2 `MATDOG-<MAC suffix>`, 192.168.4.1,
nessuna password universale. Prima password e token admin devono essere impostati via
USB fisico in una futura sessione autorizzata. Non esiste AP aperto come scorciatoia.
Il portale offline ha sei pagine e mostra anche l'indirizzo STA per il passaggio manuale.
Le modifiche richiedono socket AP, Host/Origin esatti, login con token 64-hex, cookie
HttpOnly/SameSite, CSRF, peer binding, TTL 5 min e throttling. GET non restituisce segreti.
HTTP AP si affida alla rete WPA2 privata; chi conosce la password AP condivide tale
confine. NVS e chiavi firmware non sono protetti da una nuova flash-encryption/secure-boot:
questa task non modifica eFuse o policy di sicurezza hardware.

NVS standard `nvs/md_net_v1`, record 512 B versionato con CRC32 e readback. ACTIVE è
l'autorità al boot; PENDING non è promosso automaticamente. La modifica web viene provata
su associazione fresca e IP, mentre l'AP conserva accesso alternativo. Una vecchia
associazione allo stesso SSID non conferma una nuova password. Errori/timeout/stato critico
ripristinano ACTIVE; errori storage bloccano scritture senza format/erase/riparazione.
Si prova il profilo selezionato, non tutti i profili modificati insieme. AP identity reload
attende l'uscita dei client. Password vuota conserva quella della stessa rete; enrollment
open-STA non è esposto dalla UI V1. Il backend calibrazione e i suoi A/B/marker non cambiano.

HWCDC 3.3.11 non dimostra una sessione USB aperta tramite traffico/SOF/DTR/isConnected.
Il gate prudente previsto dall'handoff è applicato: **effective modem-sleep OFF**, anche
se la preferenza AUTO/ON è salvata. OTA/provisioning/AP mantengono OFF.

TLS opzionale per-device, STA-only 443, un socket, stack 28672 B, handshake 5 s,
admission con largest internal block >=100000 B. Listener HTTP: due socket, stack 24576 B.
Lifecycle start/stop su task dedicato; mailbox con CAS e correlazione atomica impedisce
late response/doppia consegna. Identità e HMAC reali assenti dal candidato distribuito.
Un certificato localhost temporaneo ha verificato il link della libreria HTTPS, poi è
stato rimosso: il relativo artefatto dirty è solo una prova di link e viene sostituito.
La soglia heap è provvisoria: **nessuna certificazione della sostenibilità TLS sul robot**.

OTA resta `HttpTransport → OtaSession → OtaManager → OtaPolicy → OtaEspBackend`.
HTTP challenge/upload rifiutati; TLS richiede NVS configurato, quiet MAINTENANCE,
assenza di calibrazione/motion e HMAC nonce valido. Timeout/link loss/troncamento/hash
errato abortiscono. Singolo writer inattivo, hash e layout invariati. Commit resta
COMMITTED_PENDING_REBOOT, inhibit mantenuto; nessun reboot implicito o endpoint remoto
nuovo. Reboot remoto autenticato richiede il proprio gate e resta BLOCKED.
Il client verifica CA+SAN+pin DER su ogni socket e il pacchetto canonico; default check-only,
nessun resume/redirect/TLS insecure. Il manifest non è una firma digitale.

## Verifiche offline ed audit

- Suite host completa del firmware: zero errori sul run accettato. I totali e i risultati
  per suite sono nel log durevole `build/verification/host-tests.log`.
- `test_network_v3`: **5279** controlli: bit corruption/CRC, namespace/write/readback,
  trial/rollback/PENDING al boot, IP, sleep/roam/AP, parser/session/CSRF e race mailbox.
- `test_wifi_runtime_v3`: **108** controlli nell'adattatore WifiManager reale con radio,
  FreeRTOS e NVS simulati: stesso SSID/password diversa non commette su link vecchio,
  quiet guard, scan stop, rollback, AP reload/client e reboot senza auto-write.
- Client OTA: **11** unittest, incluse connessioni TLS reali solo su loopback:
  CA/SAN/pin errati rifiutati e pin rivalidato su riconnessione.
- Audit V3: quattro test, mutation su namespace, ACTIVE/PENDING, pin BSSID, scan critical,
  auth/CSRF/peer/TLS, correlazione mailbox, segreti CLI, writer estraneo e sleep untrusted.
  Audit globale conserva i divieti preesistenti, inclusi erase e percorsi verso ServoBus.
- DOM jsdom: sei tab, due form profili, selezione secondary per AP/performance,
  password write-only, SSID ostile trattato come testo, assenza asset esterni. Nessun
  browser abilitato dal servizio di computer-use: layout visivo/mobile reale TO_TEST.
- Compile/link HTTPS con identità di prova: PASS offline, nessun TLS/hardware E2E.
- `git diff --check` e default USB_ONLY / OTA ingest 0 verificati.

Sono state riparate incongruenze ereditate nei test/audit della base: link mancante di
StartupRecoveryQualification, token fixture con nuovo campo startup_recovery_only,
vecchi pin source che non riconoscevano qualifiedRecovery e l'eccezione producer startup
nel solo Controller.updateStartupQualification. Il codice degli algoritmi di calibrazione
non è stato cambiato; mutation/rifiuti e audit originali continuano a essere eseguiti.

Il toolchain pinnato è ESP32 Arduino 3.3.11, FQBN completo nel manifest, 16 MiB custom,
PSRAM OPI, native HWCDC. Core condiviso non modificato. Layout invariato
`MATDOG_16M_2x5M_NVS_V1`, due slot app da 5242880 B, table SHA256
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`.
Le warning preesistenti SCServo (`SMS_STS::ReadMode` bounds / parametro ACC) restano
visibili nei log e non costituiscono una compilazione priva di warning. Libreria condivisa
non modificata; valutarle nella sessione proprietaria, prima di nuovi percorsi servo.

## Riproduzione software

Dalla directory firmware, senza collegare il robot:

```bash
bash scripts/tests/run_host_tests.sh
python3 scripts/static_audit.py
MATDOG_PROFILE=USB_ONLY MATDOG_OTA_INGEST_VALIDATION=0 scripts/build.sh --jobs 2
# Archivia USB_ONLY prima del secondo profilo, perché il percorso output è condiviso.
MATDOG_PROFILE=ROBOT_POWERED MATDOG_OTA_INGEST_VALIDATION=0 scripts/build.sh --jobs 2
python3 scripts/ota_tls_client.py --manifest build/esp32.esp32.esp32s3/matdog_build_manifest.txt --expected-source FULL_COMMIT_SHA
```

Il test opzionale DOM usa jsdom installato in una directory build isolata e NODE_PATH;
non è una dipendenza del firmware. Nessuno di questi comandi include `--upload`.
Le build finali devono partire da CLEAN HEAD e la ricevuta deve corrispondere al manifest.

## Fonti primarie di progetto

L'adozione di due profili, password write-only, scan/status e controlli roaming riprende
il modello documentato da [Shelly WiFi](https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/WiFi/).
L'organizzazione locale segue il [manuale interfaccia Shelly](https://kb.shelly.cloud/knowledge-base/shelly-1-gen3-web-interface-guide), senza copiare branding/assets.
Il costo TLS e la necessità di qualificare heap/stack derivano dalla documentazione
[Espressif HTTPS server](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/protocols/esp_https_server.html)
e dagli header/librerie effettivamente installati. I valori adottati non provano latenze
su MATDOG: serviranno misure nel runbook.
