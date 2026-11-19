# MATDOG — BMS DALY: aggiunta all'handoff del 18/11/2026

Aggiunta scritta dalla sessione BMS (Fable) alla chiusura. L'handoff principale è `2026-11-18_MATDOG_SESSION_CLOSE_HANDOFF.md` e non è stato modificato. Tempi in ora italiana.

## Stato alla chiusura

- **A/B firmware DALY: INCONCLUSIVE** (2 avvii a freddo su 3 previsti). Indicazione a favore di una regressione firmware, non dimostrata.
- **dev.5 `df43feb`: pulito 2 su 2.** Spegnimenti di 77,5 e 88,6 min; 356/356 e 357/357 campioni DALY validi in 12 min.
- **`30c894a`: 1 avvio a freddo con silenzio** di circa 92 s (spegnimento di 268 min, registrazione partita a 260 s).
- **`30c894a` ripristinato e verificato** alle 22:38: digest APP `03c863cc…`, `30c894a0d430`, NVS gen 1 `VALID_ACKNOWLEDGED`, calibrazione 12/12 con coppie OFF, SAFE_OFF 8191, DALY 118/118 validi.
- **Robot spento dall'operatore alle 22:46:32** (journal del PC: `usb 5-2: USB disconnect`).
- Il BMS non è risolto.

## Decisione dell'operatore per il 19/11

Si riprende con **un'acquisizione DALY dall'accensione, senza flash**. È il controllo B mancante: `30c894a`, avvio a freddo, registrato dal primo secondo.

Nessuna prova è stata avviata dopo la chiusura. Nessuna modifica a firmware o hardware.

## Procedura per l'acquisizione (sola lettura)

Da avviare **prima** del KEY ON, con la USB del robot collegata al PC e nessun altro programma sulla porta seriale.

```bash
mkdir ~/MATDOG/verification-artifacts/MATDOG_BMS_DALY_QON_20261119
cd ~/MATDOG/verification-artifacts/MATDOG_BMS_DALY_QON_20261119
python3 -I ~/MATDOG/verification-artifacts/MATDOG_BMS_DALY_AB_FABLE_20261118/tools/ab_recorder.py B2_30c894a_cold 12
```

- Il registratore aspetta la porta, poi invia solo letture di stato per 12 minuti (`@BMS STATUS` ogni 2 s).
- Durante i 12 minuti: robot fermo, nessun comando, nessuna dashboard.
- Lettura: `python3 -I …/tools/ab_read.py B2_30c894a_cold`.
- La cartella nuova serve a non alterare la cartella A/B sigillata.
- La durata dello spegnimento si calcola dal journal del PC (spegnimento alle 22:46:32 del 18/11).

## Come leggere l'esito

| Esito di B | Significato | Passo successivo |
|---|---|---|
| SILENCE | `30c894a` mostra il silenzio dove dev.5 non lo ha mostrato | bisect tra dev.5 e dev.14 (dev.7, dev.10, dev.12) |
| CLEAN | la differenza di ieri può dipendere dalle condizioni, non dal firmware | A/B alternati nella stessa sessione |
| INCONCLUSIVE | registrazione incompleta | ripetere al successivo avvio a freddo |

Lo spegnimento di stanotte sarà molto più lungo dei 77–89 min degli avvii di dev.5: se B è pulito, serve anche un avvio di `30c894a` dopo 75–90 min.

## Dove sono i dati

- Originale sigillato: `~/MATDOG/verification-artifacts/MATDOG_BMS_DALY_AB_FABLE_20261118/` (63 file, SHA256SUMS `3ab9f106…`, verificato alla chiusura).
- Archivio: `~/MATDOG/backups/verification-artifacts/session-20261118/MATDOG_BMS_DALY_AB_FABLE_20261118.tar.gz` (`542546c0…`, contenuto verificato).
- RCA V1 e V2 (solo su Asus): `MATDOG_BMS_DALY_RCA_FABLE_20261118/` e `MATDOG_BMS_DALY_RCA_V2_FABLE_20261118/`, con archivi in `~/MATDOG/backups/verification-artifacts/` (`80a04511…` e `b2eb0e51…`).
- GitHub: `09_Logs/Validation_Reports/BMS_DALY_AB_2026-11-18/` (rapporto, registro della sessione, SHA256SUMS), issue #70.
