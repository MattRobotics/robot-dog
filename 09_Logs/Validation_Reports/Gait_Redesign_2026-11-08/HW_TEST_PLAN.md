# Collaudo supervisionato unico — WALK STANDARD v2 (`1.0.2-dev.2`)

**Da eseguire solo dopo autorizzazione esplicita del flash APP-only.** Stable installata `1.0.1-dev.17` (APP `14e055a0…`), rollback dev.16. Profilo drive `{1000,50}` **invariato**: nessuna nuova velocità servo da qualificare. Ciò che resta da qualificare è il comportamento fisico della nuova legge a velocità di giunto più alte (1,27 rad/s contro 0,82) e con lo swing trapezoidale.

| Fase | Azione | Osservabili | Criterio di arresto |
|---|---|---|---|
| H0 | Flash APP-only (`scripts/flash_app_only.sh`, manifest), boot in sola lettura | `FW_VERSION=1.0.2-dev.2`, NVS gen-1 `6d03b9dd…` invariato, census 13/13, IDLE 60 s senza overrun | qualunque differenza di identità/NVS |
| H1 | Robot sospeso (piedi liberi): STAND 150 → WALK 50 mm 2 cicli → WALK STOP | `timing.work_hist`, `overrun_releases`, `missed_releases`; lag `goal`/`raw` per giunto; nessun fault | FAULT, missed > 0 ripetuti |
| H2 | A terra, supervisione manuale: WALK 50 → WALK STOP; WALK 75 → WALK STOP; WALK 100 → WALK STOP (3 cicli ciascuno) | video laterale/frontale; strisciamento piede al distacco; ribaltamento/oscillazione di rollio; temperatura e corrente ID21/31; `imu_correction.reason`; tempo WALK STOP → STANDING | perdita d'appoggio, piede che trascina > 5 mm, temperatura > 60 °C, fault |
| H3 | STOP operatore in WALK 100 a metà swing anteriore, a metà swing posteriore e in finestra a quattro appoggi | piano alla release successiva, `motion_verified` ≈ 2 s, nessun SAFE_OFF; il robot resta in piedi su 3 zampe | SAFE_OFF o caduta |
| H4 | STAND dopo STOP (recupero) e WALK diretto dopo STOP | completamento del WALK STOP del gait fermato, nessun enable/OFF | fault in recupero |
| H5 | Joystick (legge invariata): avanti, laterale, rotazione 1 ciclo | come dev.17 | qualunque regressione |

Accettazione: H0–H5 senza fault; overrun WALK ≤ dev.17 (9,4 %); nessuno strisciamento significativo; robot stabile a 20 mm/s. Rifiuto → rollback a `1.0.1-dev.17`. Un esito positivo qualifica WALK STANDARD v2 con il drive attuale; **non** qualifica WALK FAST (servirebbe la prova separata del profilo `{1500,100}`: tracking, corrente, termica a 1,9 rad/s).
