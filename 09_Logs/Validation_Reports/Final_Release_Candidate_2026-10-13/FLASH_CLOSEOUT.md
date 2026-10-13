# Singolo flash APP-ONLY — FLASH_SUCCESS

13 ottobre 2026, verifica completata alle **19:29:18 UTC** (21:29 Europe/Rome).

Installato il candidato originale, senza rebuild: source **`b147f3884b227ec819e98a6f00c214a0aac2278e`**, `ROBOT_POWERED`, 3.643.120 byte, SHA256 **`5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`**. Manifest/designation originali invariati.

Dopo recupero GitHub verificato e review tecnica positiva, ricevuta l'unica conferma finale diretta: operatore presente, fixture, alimentazione stabile, disgiuntore accessibile, continuità rail/configurazione dopo i 13 OFF, passphrase conservata e autorizzazione per l'esatto SHA. Eseguito una sola volta il flasher canonico ROM 115200/no-stub: app0 `0x010000`, slot 5 MiB, fine scrittura `0x3896f0`, fine erase di settore `0x38a000`. Nessun erase integrale, bootloader/table/otadata/NVS write, migrazione o secondo flash.

## Evidenze PASS

- [Archivio GitHub cifrato indipendente](RECOVERY_ARCHIVE.md): A/B correnti riutilizzati, AES-256/S2K3/SHA-256/MDC, download e decifratura byte-identici. Nessuna nuova acquisizione integrale. Release precedente invariata, archivio NON FIRMWARE e non Latest; issue #46 aggiornata e aperta.
- Gate canonici source/profile/manifest/backup/MAC/layout/slot/range e doppia verifica esptool PASS. Campagna firmware congelata riutilizzata con inventario sorgente e SHA dei log, host repairs/test/audit qualificati separatamente.
- Readback indipendente dell'app completa (3.643.120 byte) con SHA256, checksum ESP e SHA appended PASS. La digest seriale `APP_SHA256=ASSOCIATED_MANIFEST` è distinta dalla prova concreta sui byte riletti.
- **Dopo la scrittura e prima del boot**, letture di `0x0..0x10000` e `0xfe0000..0xff0000` byte-identiche alle rispettive aree del backup A: bootloader, table, otadata, NVS default e `matdog_nvs` preservati.
- Un'unica sequenza di riavvio normale documentata USB-JTAG, nessun power cycle o retry; diagnostica successiva read-only. Source/build runtime `b147f3884b22`, profilo/capability/startup coerenti; 12/12 SERVO PREFLIGHT, census 13/13, tutti i 13 torque=0. Gen-1 `VALID_ACKNOWLEDGED`, `CONSISTENT`, record intact, nessuna riconciliazione richiesta.
- BNO085 ONLINE/PASS, 398 RV, zero runtime reset; DALY ONLINE/PASS, age 729 ms, 11,5 V, SOC 79,9%, allarmi zero. Questi sono snapshot hardware post-flash effettivi.

## Stato lasciato e limiti

Applicazione nuova in esecuzione: health READY, **MAINTENANCE**, authority **NONE** (zero grants), Motion session **OFF**, planner IDLE, limiti approvati **0**, `REVIEWED_PROFILE_REQUIRED`. `OTA_INGEST=0`, boot motion non autorizzato. RESTORE runtime non eseguito, calibrazione persistente gen-1 preservata. Nessun torque ON, STAND, WALK o altro movimento comandato.

Il PASS riguarda installazione, integrità e diagnostica. La [sessione fisica supervisionata](SUPERVISED_SESSION.md) resta distinta e richiede review/admission corrente. Fixture, carico e stabilità non sono dichiarati validati da queste letture.

[Receipt completo privo di segreti](FLASH_CLOSEOUT.json) · [Handoff](HANDOFF.md) · [Build originale](BUILD_RECEIPT.json). Log, readback e backup plaintext restano privati in `/home/matteo-manicardi/MATDOG/private-preflight/20261013T180138Z`; ciphertext e manifest sono gli unici asset della Release di recupero. Il report preflight originale e il fallimento GPG restano immutati, con esiti storici preservati.
