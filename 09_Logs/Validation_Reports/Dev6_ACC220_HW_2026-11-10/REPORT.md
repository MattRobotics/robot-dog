# MATDOG `1.0.2-dev.6` ACC 220 (`450d2c7`) — validazione hardware del 10/11/2026

> **Sintesi per `main`.**
> - **Rapporto originale:** [`REPORT_HW_SESSION.md`](REPORT_HW_SESSION.md) è la copia byte-identica (SHA256 `6b55f7d5…eeab`) del rapporto di chiusura della sessione firmware. L'originale è sull'ASUS, in `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_HW_20261110/`.
> - **Percorsi:** i percorsi relativi citati nel rapporto originale (`pre/`, `flash/`, `h0/`, `usb/`, `terra/`, `closure/`) si riferiscono a quella cartella. I log grezzi restano sull'ASUS e non sono copiati qui.
> - **Evidenze:** dimensioni e SHA256 in [`EVIDENCE_MANIFEST.md`](EVIDENCE_MANIFEST.md).

**Esito: VERIFIED HARDWARE, nel perimetro descritto sotto.**
- Fixture Q1–Q8 qualificata.
- Prima prova a terra positiva ma **fuori protocollo**: non è una qualifica G0–G5.
- **14 STOP d'emergenza verificati su 14:** 8 sulla fixture e 6 a terra, di cui 2 durante la discesa PRONE.
- **0 fault Motion.** Registro 85 = 220 su tutti i 12 ST3215.

**Non è una nuova release stabile.** La Stable storica resta [`v1.0.1`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1) (`1.0.1-dev.17`), immutabile.

## Firmware installato

| | |
|---|---|
| Versione | `1.0.2-dev.6` (build di sviluppo) |
| Commit sorgente | `450d2c793dcb76d08ea341eb5c4b1e50215cee40`, integrato in `main` con la [PR #67](https://github.com/MattRobotics/robot-dog/pull/67) |
| Build | `450d2c793dcb`. Manifest: `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `ROBOT_POWERED`, OTA ingest 0, build UTC 2026-11-10T12:16:01Z |
| APP | 1 751 040 B, SHA256 `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256`. Lo stesso valore compare nel manifest, nel binario conservato e in `flash/flash.log` |
| Flash | 10/11, 19:16–19:18Z, APP-only su `app0` (0x010000–0x1bb800), verify-flash PASS. Non toccati: NVS, otadata, tabella delle partizioni (`8f756ecb…`), bootloader, EEPROM dei servo |
| Identità sul robot | Preflight H0 e 511 `/status`: build `450d2c793dcb`, un solo boot |
| Rollback (non usato) | `1fbee7b` (APP `39840bd4…`), con script e asset verificati (12/12) |

## Prove (dal rapporto originale)

**Fixture, 19:20–19:43Z**

| Prova | Esito |
|---|---|
| Q1 STAND 120 | STAND_REACHED in 10 s |
| Q2 SLOW | Drive 1000/50, 20 mm/s; WALK STOP → STANDING |
| Q3 e Q3b STANDARD | Drive 1800/180, 30 mm/s; 2 STOP verificati; riprese con WALK e con STAND |
| Q4 FAST | Drive 1950/220, 32 mm/s; 2 STOP verificati; riprese |
| Q5 cambi di profilo in camminata | Sempre WALKING; transizioni complete in 4–13 s; superati una selezione sostituita da un'altra e un WALK STOP durante un cambio |
| Q6 STOP nelle transizioni | 4 STOP verificati; selezione rifiutata con STOP attivo (`STOP_LATCHED`); ripresa |
| Q7 joystick in avanti | Fondo scala 30 / 32 / 20 mm/s (STANDARD / FAST / SLOW); al rilascio → STANDING |
| Q8 SAFE_OFF | Mask 8191, coppia OFF, ottenuto con PRONE |

**A terra, 19:43–19:58Z, fuori protocollo**
- STAND a 150 mm per 7,5 min.
- Camminate:
  - SLOW in avanti, all'indietro e in sterzata;
  - STANDARD per 11 s;
  - FAST per 34 s.
- Margine minimo delle anche: 5,3 tick in SLOW avanti, 4,3 in SLOW indietro, 7,3 in STANDARD e FAST.
- **6 STOP su 6 verificati** in 2,0–2,4 s: SLOW 1, STANDARD 2, FAST 1, PRONE 2.
- PRONE completo fino a PRONE_REST, poi SAFE_OFF.

**Perché la prova a terra è fuori protocollo**
- Batteria a 10,6–10,2 V, sotto gli 11,0 V richiesti.
- Anticaduta non confermata per iscritto.
- Robot portato a terra in STAND, a coppia ON.
- Sequenza libera.
- Alle 19:57:21Z il criterio termico del runbook è scattato e non è stato applicato. Il servo 13 leggeva 80 °C, confermati dalla rilettura; il firmware ha chiuso la finestra senza fault.

## Non verificato e aperto

- **Qualifica a terra G0–G5:** con batteria ≥ 11,0 V, anticaduta confermata e margine delle anche ≥ 4 tick.
- **Prove a terra mancanti:** STOP a fine swing anteriore, mantenimenti lunghi dopo STOP, camminate lunghe, frenata con ACC 180/220 sotto carico.
- **Comandi non provati sull'HW:**
  - cambio di profilo con la leva tenuta (verificato solo offline);
  - joystick laterale, in rotazione e all'indietro in STANDARD e FAST (non implementato: vedi l'handoff nella cartella originale);
  - SAFE_OFF dalla dashboard (pulsante assente).
- **Avviso batteria a coppia ON:** non compare, perché la presentazione LED resta sospesa con Motion attivo.
- **Diagnostica senza fault:**
  - D1: 131 release perse e 2 657 overrun;
  - D2: un HTTP 503 durante la camminata sterzata;
  - D3: letture di temperatura spurie, tutte filtrate dal firmware.

## Gemello MuJoCo (offline, non hardware)

Il gemello MuJoCo V1 (milestone M5) ha riprodotto in simulazione:
- tutti gli 8 STOP della fixture, verificati anche nel gemello;
- 4 dei 6 STOP a terra. I 2 durante il PRONE non sono riproducibili; nel gemello i 3 STOP raggiunti finiscono in fault.

È una discrepanza residua del modello sui carichi a terra, **non un risultato hardware**: [`REPORT_M5.md`](../../../06_Software/Matdog_MuJoCo/REPORT_M5.md).

## Integrità

- **`SHA256SUMS`** della cartella originale: 136 file, tutti OK alla verifica di consolidamento (10/11/2026, circa 21:45Z). Il manifesto delle 21:02Z è conservato in `closure/`.
- **`EVIDENCE.sha256`: assente.** Non è stato generato in questa integrazione: la chiusura spetta al responsabile firmware.
