# MATDOG — A/B firmware sul silenzio DALY, 18/11/2026

Sessione hardware autorizzata dall'operatore. Tempi UTC (ora italiana = UTC + 1). Dettaglio degli eventi: `SESSION_LOG.md`.

## Esito

**INCONCLUSIVE secondo il criterio dichiarato (2 avvii a freddo su 3), con indicazione a favore di una regressione firmware.**

- `1.0.1-dev.5` (`df43feb`): 2 avvii a freddo, 2 puliti, 0 silenzi.
- `30c894a` (installato): 1 avvio a freddo oggi, con silenzio DALY di circa 92 s.
- La prova è stata chiusa dopo il secondo avvio per decisione dell'operatore. Il terzo non è stato fatto.
- Il robot è di nuovo su `30c894a`, verificato. Nessun comando Motion inviato con dev.5.

La mia previsione scritta nel rapporto V2 (dev.5 avrebbe mostrato il silenzio) non si è verificata in questi due avvii.

## Rilievi

| Run | Firmware | OFF prima | Registrazione | DALY |
|---|---|---|---|---|
| `B0_30c894a_cold_1749` | `30c894a` | 268 min | da 260 s (robot già acceso) | FAULT da 217,4 a 308,8 s di uptime (storico LED), TIMEOUT fino a 307 s, poi 395 campioni validi |
| `A1_dev5_cold` | dev.5 `df43feb94cdb` | 77,5 min | da 0 a 720 s | 356 su 356 validi, buco massimo 4 s |
| `A2_dev5_cold` | dev.5 `df43feb94cdb` | 88,6 min | da 0 a 720 s | 357 su 357 validi, buco massimo 2 s |

Entrambi gli avvii A sono veri KEY OFF/ON (`reset_reason=POWERON`, disconnessione ed enumerazione USB nel journal del PC), non reboot dell'ESP.

## Quanto pesa

- A novembre, con le immagini recenti, 12 avvii a freddo su 14 hanno avuto il silenzio; con oggi 13 su 15.
- Nella stessa fascia di spegnimento degli avvii A (60–130 min) a novembre i silenzi sono 5 su 5 (68,6, 84,8, 122,4, 125,0 e 126,8 min).
- Con la frequenza di novembre, due avvii puliti di fila hanno probabilità (2/14)² ≈ 2 %. Con tre sarebbe stata 0,3 %.

## Limiti

- Due avvii, non tre: il criterio "forte evidenza" non è raggiunto.
- Il controllo B di oggi ha uno spegnimento più lungo (268 min contro 77–89 min) e non è registrato dall'accensione.
- Manca un avvio a freddo di `30c894a` registrato dall'inizio nelle stesse condizioni degli avvii A (stessa sera, 75–90 min).
- In A1 e A2 l'ESP non è stato resettato nella finestra; in 6 dei 12 silenzi di novembre c'era un reset dell'ESP nella finestra.

## Passo successivo proposto

1. Controllo B a costo zero: registrare dall'accensione il prossimo avvio a freddo di `30c894a` (stesso registratore, sola lettura).
2. Se B mostra il silenzio: bisect mirato tra dev.5 e dev.14 (dev.7, dev.10, dev.12), con le stesse regole di questa sessione.
3. Se B risulta pulito: ripetere prima A/B alternati, perché la differenza potrebbe dipendere dalle condizioni della serata e non dal firmware.

## Flash e ripristino

| Passo | Ora | Esito |
|---|---|---|
| Controlli prima del flash | 18:07–18:08 | identità `30c894a0d430`, SAFE_OFF 8191, coppie OFF, DALY valido; layout `8f756ecb…`, app0 `0x010000`, digest dell'installato = `03c863cc…` |
| Flash APP-only dev.5 (`c8b27d67…`, 1 634 240 B) | 18:08:29–18:09:13 | `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS` |
| Controlli prima del rollback | 21:37 | identità `df43feb94cdb`, SAFE_OFF 8191, coppie OFF, DALY valido; digest dell'installato = `c8b27d67…` |
| Flash APP-only `30c894a` (`03c863cc…`, 1 779 312 B), script qualificato invariato | 21:37:47–21:38:37 | `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS` |
| Avvio dopo il rollback (`B1_30c894a_postrollback_boot`, 4 min) | 21:38:48 | `1.0.2-dev.7` `30c894a0d430`, NVS gen 1 `VALID_ACKNOWLEDGED`, calibrazione `be8d5902bb4e37da` 12/12 con coppie OFF, census 13/13, READY/IDLE, SAFE_OFF 8191, DALY 118/118 validi |

- Scritta solo la partizione app0 (`0x010000`). NVS, calibrazioni, bootloader, tabella delle partizioni e registri DALY non toccati.
- Comandi inviati al robot: solo letture di stato (`@BMS STATUS`, `@STATUS`, `@SYSTEM SOURCE_SIGNATURE`, `@MOTION STATUS`, `@CALIBRATION PERSIST STATUS`, `@LED STATUS`, `@MODE STATUS`).
- Worktree operativo: HEAD `30c894a`, pulito, nessun file scritto. Nessun push, nessun merge, nessuna modifica ai sorgenti.

## File

- `SESSION_LOG.md`, `SHA256SUMS.txt`, `preflight_artifacts.txt`
- `B0_30c894a_cold_1749/`, `A0_dev5_postflash_boot/`, `A1_dev5_cold/`, `A2_dev5_cold/`, `B1_30c894a_postrollback_boot/` (ognuno con `serial.log` e `session.json`)
- `pre_flash_read_30c894a/`, `pre_rollback_read_dev5/`, `flashlog_A_dev5/`, `flashlog_B_rollback/`
- `tools/` (registratore, lettore, ciclo), `flash/` (script e controlli)
