# Dev.16 — manifest delle evidenze hardware (07/11/2026)

Origine: diagnostiche `matdog-motion-trace (23).json`–`(32).json` allegate alla conversazione MATDOG. SHA256 calcolati sui **byte originali** senza modifiche. Derivati pubblicati su GitHub: [REPORT.md](REPORT.md), [TRACE_WINDOW_METRICS.csv](TRACE_WINDOW_METRICS.csv).

| Traccia | Byte | SHA256 |
|---|---:|---|
| 23 | 2.319.825 | `88fabe447faf93a16d227b0a53398bc9e30d7062f222aa52f81bc8ece78b0b0c` |
| 24 | 1.071.898 | `1704c00ca9878cffa129f834df691d11001f4c07b4cd1b1aea9d8f1e1ea1a5dc` |
| 25 | 2.304.933 | `9348faed6978f6107afb7d1195b6c10d90add2cc47b881ce3950f6d1ce8bd6e0` |
| 26 | 2.405.118 | `ae23b9b85c7be61dddbcccf1a0b7730eabf0b54d717d3c7801f5ff154e285f4c` |
| 27 | 2.488.541 | `11fdadd4e95eace8e3cf577d906cdb9f2df54c500a6e330a49b2275b81f2cca8` |
| 28 | 2.451.503 | `8cec95c066ae8f0ad3d6b2921d77ad480259cc3e026085b3eaa0a2985862a505` |
| 29 | 2.448.956 | `02abe77b42a63a13b89ba0d742d437de3a52602abbd0a996ac29ab9d03e5cb53` |
| 30 | 2.450.580 | `b94f012fea9186221d86d95be310e7e4a235a8a7e088a9e14cdd36ef3aed4b62` |
| 31 | 2.431.452 | `cce900ca20701bb6bafd4b58aaae5b1df9c76507f9627e6be4739cd9b286cad3` |
| 32 | 2.428.297 | `7e846cd5096651bf415a87bff0487a42b6b22b08712830158f1f07c37dae32f7` |

**Archivio ZIP delle 10 tracce + CSV originale di estrazione:** `MATDOG_DEV16_HW_2026-11-07_TRACES_23-32.zip`, **632.404 byte**, SHA256 `3e21a255b0acbcaad570c78439574ee40e47f8d2b187b79ea827e7a9c760f408`. Integrità ZIP verificata, 11 elementi. Archivio disponibile come artefatto di conversazione, **non ancora pubblicato nel repository GitHub**: questa lista non prova che esista sull'ASUS. Non affermare diversamente fino a upload/re-fetch e verifica SHA. Non comprendere screenshot con indirizzi, utente/browser o altre informazioni personali in una pubblicazione pubblica senza controllo.

**Altre evidenze documentate dal dialogo (non file sul branch):**
- Flash/boot via Claude Code: APP SHA256 `73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`; log originali conservati sull'ASUS in `~/MATDOG/verification-artifacts/MATDOG_DEV16_FLASH_PREP_20261107/` secondo l'agente; non materializzati qui.
- Ping 60/60, RTT 1,930/27,890/111,961 ms; `curl /status` con 503 e risposte lente/incomplete; `curl /api/motion/status` 20/20 HTTP 200 e ~25–141 ms, output del terminale riportato dall'operatore, non importato come log raw in questo dossier.
- Osservazioni visive dell'operatore: PRONE senza cadute, STAND stabile e consumo complessivo 0,4–0,5 A, WALK45 più diritto di joystick, STOP eseguiti, difetti di UI/reflow/Twin cold refresh, inutili transizioni STOP→STAND150→WALK120; spegnimento finale con KEY OFF/BMS discharge MOS anche per alimentazione ESP32-S3.

**Regola di lettura:** i JSON salvano buffer limitati e talvolta sovrapposti; le etichette `VERIFIED HARDWARE` riguardano solo ciò che è stato realmente osservato/registrato. Le ipotesi su I-cache, cause dei timeout o miglioramento 5× rimangono `HYPOTHESIS / UNKNOWN`. Nessun risultato equivale a release stabile autorizzata.

**Aggiornamento 07/11/2026 sera (sessione dev.17, solo aggiunta; risultati e classificazioni sopra invariati):** l'operatore ha fornito l'archivio nella conversazione; SHA256 ricalcolato **`3e21a255b0acbcaad570c78439574ee40e47f8d2b187b79ea827e7a9c760f408`**, 632.404 byte, 11 elementi, `testzip` senza errori, SHA dei 10 JSON coincidenti con la tabella sopra. Scansione del contenuto: nessun indirizzo IP o MAC, email, percorso locale, user agent, SSID, token o cookie (solo telemetria, tempi e SHA256 nel CSV). Pubblicato byte-identico in [`raw/MATDOG_DEV16_HW_2026-11-07_TRACES_23-32.zip`](raw/MATDOG_DEV16_HW_2026-11-07_TRACES_23-32.zip). Uso nella candidata dev.17: [Dev17_Candidate_2026-11-07/REPORT.md](../Dev17_Candidate_2026-11-07/REPORT.md).
