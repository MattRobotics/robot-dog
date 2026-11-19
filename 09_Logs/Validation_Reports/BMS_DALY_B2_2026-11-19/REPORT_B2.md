# MATDOG — controllo B DALY del 19/11/2026

Un avvio a freddo di `30c894a` (`1.0.2-dev.7`, `30c894a0d430`) registrato dall'accensione, sola lettura, stesso registratore e lettore del 18/11 (`tools_used.sha256`). Nessun flash, nessun comando Motion.

## Esito: CLEAN

- KEY OFF 18/11 21:46:32Z (journal), KEY ON 19/11 05:20:11.730Z: spegnimento di 453,7 min. `reset_reason=POWERON`.
- 358 campioni `@BMS STATUS` da 4 a 718 s, 358 `comm=OK`, buco massimo 2 s.
- Primo campione valido a 4,0 s (`last_ok_uptime_ms=3204`, `rx_bytes=129`, `first_byte_ms=18`).
- `fail_streak=0` in tutti i 358 campioni: nessun timeout, nessuna risposta tronca, nessun silenzio.
- `@STATUS` ogni 30 s: sempre READY, uptime continuo fino a 699 s (nessun reset). SAFE_OFF 8191, coppie OFF.
- Una riga `COMMAND=REFUSED REASON=MOTION_ACTIVE_USE_STOP` a 3,8 s, durante il boot, in risposta alla riga vuota iniziale del registratore; presente anche nel boot del 18/11 dopo il rollback.

## Confronto

| Run | Firmware | OFF prima | Esito |
|---|---|---|---|
| B0 18/11 | `30c894a` | 268 min | silenzio ~92 s (217–309 s), record dal 260 s |
| A1 18/11 | dev.5 | 77,5 min | CLEAN 356/356 |
| A2 18/11 | dev.5 | 88,6 min | CLEAN 357/357 |
| B2 19/11 | `30c894a` | 453,7 min | CLEAN 358/358 |

## Lettura

- `30c894a` può avviarsi a freddo senza silenzio: è il terzo avvio pulito su 16 delle immagini recenti (13 silenzi).
- L'A/B resta INCONCLUSIVE e l'indicazione a favore della regressione si indebolisce: 2 puliti con dev.5 contro 1 silenzio e 1 pulito con `30c894a` (Fisher unilaterale p = 0,5).
- Le condizioni non sono equivalenti: spegnimento di 454 min contro 77–89 min, mattina contro sera.
- Nessuna causa dimostrata. Il problema non è risolto.
