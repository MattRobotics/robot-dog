# MATDOG Dashboard — immagini 2026-10-11

**Simulazione illustrativa — non telemetria reale.** Entrambi i PNG pubblicati
sono mockup generati ricevuti nello ZIP operatore, non screenshot hardware e
non fonti per PASS, dati runtime, sicurezza o identità del firmware installato.

| File | Descrizione |
|---|---|
| [01 Maintenance](01_mockup_maintenance_illustrative.png) | Illustrazione Dashboard; vecchia build ab17813, SOC32% charging e valori di scena, non stato finale b08322d. Le soglie visibili non provano implementazione. |
| [02 Diagnostics](02_mockup_diagnostics_illustrative.png) | Illustrazione Diagnostics; JSON e dati storici/di scena, sessione indicata non prova timeout, nessun dato domestico o segreto visibile. |

Originali esaminati visivamente, copiati senza trasformazioni. L'indirizzo
192.168.4.1 visibile è l'AP robot previsto e autorizzato, non l'IP domestico.

Esclusi da commit/push e conservati soltanto in
`~/MATDOG/verification-artifacts/closeout-20261011/visuals/`:

- `dashboard_mockup_wifi_sta_success.png`: mockup generato, contiene SSID
  domestico e IP DHCP privato. NON prova hardware.
- `LOCAL_ONLY_wifi_sta_actual_2026-10-11.jpg`: screenshot reale operatore,
  contiene SSID e IP domestici. Conferma visivamente build breve b08322d,
  connessione riuscita e configurazione COMMITTED; copia originale non pubblicata.

Nessuna redazione pixel affidabile prodotta nel closeout: scelto il percorso
privacy-safe esplicitamente consentito di esclusione dei due file sensibili.
Anche ZIP/input originali e log integrali restano fuori Git.
[Report hardware](../../../09_Logs/Validation_Reports/MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md).
