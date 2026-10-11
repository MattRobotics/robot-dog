# MATDOG — Appendice evidenze closeout hardware non-motion — 2026-10-11

## Recupero provenienza log APP-ONLY

È stata recuperata la trascrizione **integrale** dell'output terminale Asus del comando `scripts/flash_app_only.sh`, come incollata dall'operatore nella conversazione ChatGPT il 2026-10-11. Non è un file di logging automatico originariamente registrato sul filesystem Asus: la provenienza deve restare dichiarata esattamente così. Il file originale trascritto, archiviato **solo localmente**, è `LOCAL_ONLY/operator_terminal_flash_app0_2026-10-11.txt` e ha SHA256 `c81d532bb6d6d7f34ab6745e5348d70ec3d2751445136b91da4eb0194e0878ea`.

Questa trascrizione comprende il blocco esptool di scrittura, la successiva verifica flash e l'esito finale riportato dal comando:

```text
SOURCE_COMMIT         = b08322dedc5564c080f691dc98047511ef2b953f
APPLICATION_SHA256    = ccbce2ff6ffee9a5e132bad8646f379fb31fdd305c0d577df992e85e2ebf4b43
APPLICATION_OFFSET    = 0x010000 (partition 'app0')
LAYOUT_ID             = MATDOG_16M_2x5M_NVS_V1
Verification successful (digest matched).
APPLICATION_ONLY_FLASH = PASS
FLASHED_HARDWARE_PROFILE = ROBOT_POWERED
FLASHED_OTA_INGEST_ENABLED = 0
```

Nel testo integrale sono riportati anche il range erase (settori contenuti in `app0`) e il range della partizione `matdog_nvs` esclusa dalla scrittura. Le righe `FAIL` presenti nel blocco `USB_FRAMING_MUTATION` sono **difetti storici intenzionalmente rilevati dai test di mutazione** (`USB_FRAMING_MUTATIONS = PASS`): non sono errori dell'operazione flash.

## Controlli post-flash riportati dall'operatore

La trascrizione dei comandi seriali `@STATUS`, `@AUTHORITY STATUS`, `@CALIBRATION PERSIST STATUS` e `@WIFI STATUS`, incollata dall'operatore nella stessa conversazione, è archiviata localmente con SHA256 `5fbffd80e2d191d9bd8a13ceed959bc791983560d3a5409646f33ebce8e17dd7`. Include:

- `SYSTEM health=READY power_state=RUN mode=MAINTENANCE authority=NONE profile=ROBOT_POWERED`.
- `BNO085`, `DALY`, `SERVO` rilevati `ONLINE` e `PASS`; censimento servo 13 presenti rispetto ai 13 attesi (17 ID canonici; 4 assenti by design).
- `AUTHORITY_MODE operating_mode=MAINTENANCE motion_allowed=NO`.
- `BOOT_VERDICT=VALID_ACKNOWLEDGED`, `ACKNOWLEDGED_GENERATION=1`, `ACKNOWLEDGED_RECORD_INTACT=1`, `SLOT_A=VALID`, `MOTION_AUTHORIZED=0`.
- `WIFI_ADMIN phase=ADMIN_AP`, `WIFI_AP active=YES`, `portal=LISTENING`, `WIFI_FAULT site=NONE`.

## Verifica STA reale riportata dall'operatore

Trascrizione seriale successiva alla prova di connessione nella stessa conversazione, conservata solo localmente con SHA256 `82bfce717b1ea62a979e53f5e0da3ed78ec324c324537a52827e05844e4f5384`. Il risultato conferma `WIFI_STA_TEST outcome=CONNECTED associated=YES attempts=1 elapsed_ms=7900 rollback=NO`, `WIFI_CONFIG_TX result=COMMITTED`, `WIFI_FAULT site=NONE` e ripristino `WIFI_ADMIN phase=ADMIN_AP`. L'identità SSID/IP della rete privata è omessa da questa versione pubblicabile. La connessione STA **non resta attiva** in modalità MAINTENANCE: è il comportamento previsto dalla policy AP-only.

## Giudizio di tracciabilità

La **mancanza del file di log originale scritto automaticamente sul disco Asus** va distinta dalla mancanza del **contenuto dell'evidenza**: la trascrizione originale completa del terminale è stata recuperata, archiviata e vincolata tramite digest. Non dichiarare che il dispositivo è stato ritestato o che il log è stato acquisito in tempo reale dal sistema di audit. Non eseguire nuovi flash o operazioni hardware per la sola normalizzazione documentale. Con questi allegati la prova flash riportata dall'operatore è documentalmente ricostruibile; qualsiasi requisito interno di registrazione nativa automatica è un miglioramento del processo da applicare nei flash futuri, **non una prova hardware ancora da effettuare**.


## Audit d'importazione nel repository

Codex ha verificato SHA256SUMS: **5/5 PASS**, e confrontato questa appendice
con tutti e tre i testi LOCAL_ONLY, senza eseguire comandi hardware.
Audit privacy della versione pubblica: nessun SSID domestico, IP router, MAC,
percorso nominativo, credenziale, cookie o token. I tre originali e il manifest
del pacchetto restano fuori Git in
`~/MATDOG/verification-artifacts/closeout-20261011/flash-evidence-supplement/`.
Gli hash attestano integrità degli allegati ricevuti, non certificano indipendentemente
l'autenticità dell'output dell'operatore. Provenienza: testo originale incollato
nella conversazione, non log automatico nativo Asus e non nuova misura Codex.

Riscontro flash: applicazione 1 226 384 byte, write range
`0x010000..0x13b690`, erase range esclusivo `0x010000..0x13c000`;
`matdog_nvs 0xfe0000..0xff0000` esclusa. Partition table SHA256
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`
è dichiarata corrispondente a manifest e tabella installata dall'output originale.
Post-flash: `SLOT_B=ABSENT`, `DEGRADED=0`, `RESTORE=NOT_IMPLEMENTED` e
`AUTHORITY_COUNTERS grants=0`; nessuna trasformazione ammessa da storage.
Il dato `RECONCILIATION_REQUIRED=0` è presente nel preflight storico E4 e
riferito nell'input E1; la trascrizione finale fornita non stampa quel campo.
Non confondere tre fonti/momenti diversi.

[Report finale](../MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md).
