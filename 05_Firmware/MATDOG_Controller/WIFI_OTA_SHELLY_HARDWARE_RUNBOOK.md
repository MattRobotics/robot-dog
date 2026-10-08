# MATDOG Wi-Fi / OTA V3 — runbook hardware futuro

**TO_TEST, non eseguito.** Questo documento non autorizza flash, seriale, provisioning,
reboot, OTA, movimento, erase o merge. Il candidato consegnato è ROBOT_POWERED,
ingest 0, base provvisoria. Ogni fase hardware richiede una successiva autorizzazione.

## Entrata e baseline

1. Acquisire il PASS hardware della persistenza dalla sessione proprietaria: commit
   preciso, manifest CLEAN, profile, SHA app/table e prove SAVE → ACK → LOAD dopo reboot.
   Confrontare con base 1a5e008. Portare solo l'integrazione necessaria nel branch rete;
   risolvere conflitti offline e rieseguire host/audit/build prima di qualsiasi installazione.
2. Verificare [report V3](../../09_Logs/Validation_Reports/2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md)
   e ricevuta finale; iniziare da USB recovery affidabile e layout esatto. Non modificare
   eFuse/secure boot/flash encryption, GPIO, VBUS, antenne o alimentazione con questa procedura.
3. Definire misure baseline no-motion: loop Hz/jitter, BNO085/DALY/ServoBus cached health,
   stack, free/min/largest internal heap, reset reason, uptime e marker/CRC/generazione
   della calibrazione tramite procedure della sessione proprietaria. Non usare dump di
   credenziali o erase. Non confondere SAFE_OFF storico con stato corrente.

## Prima configurazione protetta

4. In una sessione USB fisica autorizzata e quiet MAINTENANCE impostare una password AP
   univoca 8–63 char ed un token admin casuale 64-hex. Non usare credenziali esemplificative
   e non salvarle in terminal history/log/screenshots. Comandi previsti:
   `@WIFI AP KEY <unique-passphrase>` e `@WIFI ADMIN KEY <random-64-hex-token>`.
   L'ACK QUEUED non basta: rilevare COMMITTED e provisioned dal successivo stato.
5. Collegarsi al WPA2 MATDOG-MACsuffix a 192.168.4.1, verificare assenza AP aperto, login,
   sei pagine offline desktop/mobile senza Internet, password write-only, scansioni/status
   e collegamento manuale all'indirizzo STA. Autorizzare prove di provisioning separatamente
   dalla lettura dello stato. Origin/Host diverso, CSRF/cookie assente/errato, sessione
   scaduta e traffico su socket STA devono rifiutare modifiche.
6. Provare primary e poi secondary separatamente; DHCP e statico con mask/gateway/DNS
   coerenti, nessuna collisione con 192.168.4.0/24. Usare la UI per TESTING/COMMITTED.
   Provare password errata, SSID assente e interruzione alimentazione solo con autorizzazione
   specifica e robot in condizioni sicure: ACTIVE deve restare noto, PENDING ignorato al boot,
   USB e AP devono consentire recupero. Non fare reset factory/NVS per risolvere errori.
7. Provare AP timeout, permanent policy, nessun timeout con client/trial, cambio SSID/key AP
   con STA valida e deferred reload dopo uscita dei client. AP/STA possono cambiare canale
   insieme: verificare recupero del client senza interpretarlo automaticamente come crash.
8. Rileggere marker/CRC/generazione calibrazione con la procedura autorizzata: namespace rete
   su standard nvs, nessun cambiamento a matdog_nvs, nessuna nuova acquisizione/motion.

## Radio e isolamento runtime

9. Documentare router/AP e versione/configurazione, SSID, BSSID, canale, RSSI e topology.
   Eseguire almeno 100 ping per scenario e riportare loss, median/p95/max RTT insieme a
   misure firmware; il ping non è prova di sicurezza del controllo.
10. Prove autorizzate: perdita/ripristino AP, primary non disponibile, fallback secondary,
    stesso SSID multi-BSSID, switch fra AP con diversi canali. Roaming resta OFF fino alla
    misura baseline; abilitarlo solo per il suo test. Verificare intervallo/hysteresis/dwell,
    assenza oscillazioni e no scan in RUN/calibrazione/OTA. Non avviare motion per il solo
    test rete: usare stati simulati/offline finché il gate motion non è autorizzato.
11. Confrontare HT20/HT40, throughput e jitter, retry counters, worker max/stack,
    snapshot max, heap/min/largest e stabilità per cicli ripetuti. Retain HT20 se regressivo.
12. Provare USB cable assente, collegato/porta chiusa e sessione host: configurato AUTO/ON/OFF,
    effettivo OFF in tutti i casi finché manca prova trusted-session. AP/provisioning/OTA
    impongono OFF. Non promuovere isConnected, DTR o SOF a prova di sessione.

Letture disponibili: `@STATUS`, `@WIFI STATUS`, `@WIFI PROFILES STATUS`, `@WIFI AP STATUS`,
`@WIFI SLEEP STATUS`, `@WIFI ROAM STATUS`, `@WIFI MODE STATUS`, `@WEB SERVER STATUS`,
`@OTA STATUS`, `@CALIBRATION PERSIST STATUS`. Anche l'apertura seriale è hardware I/O:
questi comandi non sono stati inviati dalla task offline.

## OTA — gate separato ancora bloccato

13. Prima di abilitare ingest, implementare/revisionare e testare il riavvio remoto esplicito,
    autenticato e quiet previsto dalla V3. Attualmente non esiste tale endpoint. Non inventare
    una chiamata restart o un reset automatico nello script client come sostituto.
14. Provisionare identità TLS per-device e HMAC univoco tramite percorso fisico autorizzato,
    distribuzione CA/SAN/pin tramite canale indipendente; nessuna chiave d'esempio/master.
    Misurare handshake, free/min/largest heap, stack e loop jitter con il firmware identità
    attiva; admission 100 KB/stack 28672 B/timeout 5 s sono impostazioni iniziali da qualificare.
15. Solo dopo PASS TLS, USB recovery, base calibrazione e reboot, costruire un candidato CLEAN
    separato con override ingest 1; validare esatto manifest/profile/FQBN/app/table/commit.
    Non usare il file TLS link probe o una build dirty. Il candidato consegnato ingest 0
    deve rifiutare challenge/upload anche se il listener è disponibile.
16. Autorizzare esplicitamente prove OTA device: CA/pin/SAN errati, HTTP, HMAC/nonce/replay,
    size/hash/build incompatibili, safe-state refusal, body troncato, timeout/link loss;
    nessun errore deve commettere un nuovo boot target e ogni errore deve liberare l'inhibit.
17. Il client `scripts/ota_tls_client.py` senza `--upload` verifica soltanto il pacchetto.
    Un upload reale richiede autorizzazione distinta, CA, pin DER e HMAC file privato:
    nessun resume di nonce/session, TLS insecure, redirect o reboot automatico.
18. Su successivo firmware qualificato osservare COMMITTED_PENDING_REBOOT, invoke soltanto
    il percorso reboot esplicito approvato, verificare build/profile/health, boot guard,
    rollback nelle prove negative e retention marker/CRC della calibrazione. Non confondere
    commit inattivo con immagine eseguita. Ripetere una sessione nuova dopo guasti.

## Uscita

Registrare sorgente, toolchain locale, manifest/bin/table hash, passi autorizzati,
timestamp/log sanitizzati, misure e verdict PASS/FAIL/INCONCLUSIVE per ogni scenario.
Fermarsi su fault storage, reset anomalo, degrado timing, perdita recovery, modifica
calibrazione inattesa o qualunque gate non rispettato; nessun erase/flash diagnostico
come fallback. Merge/release richiedono autorizzazione successiva e PASS della base.
Jetson handover è solo un contratto documentato: nessuno switch automatico da provare.
