# Archivio corrente di recupero — 13 ottobre 2026

**INDEPENDENT_RECOVERY_PASS.** [GitHub Release dedicata](https://github.com/MattRobotics/robot-dog/releases/tag/backup-esp32-recovery-20261013-20261013T191201Z-14c19f227594), archivio **NON FIRMWARE**, non Latest. Sono caricati soltanto ciphertext e manifest. La release precedente del 2 ottobre e i suoi asset sono invariati.

Riutilizzati gli A/B della sessione privata `20261013T180138Z`, senza nuove acquisizioni. Entrambi: 16.777.216 byte, SHA256 `9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83`. Firmware contenuto `b08322dedc5564c080f691dc98047511ef2b953f`; layout `MATDOG_16M_2x5M_NVS_V1`, MAC `14:c1:9f:22:75:94`.

GPG 2.4.4, AES-256, S2K iterato/salato SHA-256 (count 65011712), MDC verificato. Un agent privato separato, pinentry-gnome3 sul desktop ASUS e timeout 600 s hanno risolto il precedente timeout; nessuna passphrase ricevuta da Codex, salvata, passata in argv/env/file o registrata. Cache simmetrica disabilitata. L'agent privato è stato terminato dopo la procedura; l'agent personale non è stato modificato.

Decifratura locale e della copia riscaricata da GitHub: GOODMDC, DECRYPTION_OKAY, dimensione, SHA256 e confronto di tutti i byte PASS. Ciphertext: 829.386 byte, SHA256 `8e4d978eaacc8fc8d773e5b56efa60f23727b119deeda82d4bd38c4dfec14eb6`. [Manifest pubblico](RECOVERY_ARCHIVE_MANIFEST.json) · [Receipt con identificativi e URL remoti](RECOVERY_ARCHIVE_RECEIPT.json).

Il report preflight originale e il tentativo GPG fallito restano immutati. Questo receipt chiude il gate della copia indipendente, senza dichiarare freschi i readback pre-ROM e senza autorizzare flash o Motion. La passphrase rimane recuperabile sotto controllo dell'operatore.

## Review del candidato congelato

Il flasher canonico riutilizza la campagna già completata solo per il candidato designato: verifica inventario dei 426 file rispetto al commit compilato, SHA della test receipt e di tutti i log/certificati, esiti PASS e riparazioni host delimitate dai sette digest originali/correnti. I quattro audit APP-ONLY pertinenti sono stati rieseguiti dopo la modifica al gate host; test negativo di alterazione sorgente/log/receipt PASS. Non esiste un flag generico di esclusione test; senza designation resta il percorso audit completo. Nessun rebuild o modifica a firmware/manifest/calibrazione.
