# Candidato `1314bc411c58` contro riferimento 18/11b `30c894a0d430` (generato da `MATDOG_REF=1811b cand_vs_ref.py 0623`)

Criteri di `HW_TEST_PLAN.md` (fissati il 17/11), codificati in questo script il 18/11 prima della prova. Stessa catena ABBA, stessi profili HTTP A e B.

## 1. Per prova

| Prova | Build | Passi | Release | ≥15 ms | ≥20 ms | Max µs | Overrun | Perse comando / riposo | heap_min_free | Esito P0 |
|---|---|---|---:|---:|---:|---:|---:|---|---:|---|
| settle_A | 1314bc4 | 12/12 | 3166 | 0 | 0 | 10928 | 0 | 0 / 0 | 44592 | PASS |
| settle_B | 1314bc4 | 12/12 | 4835 | 0 | 0 | 9936 | 0 | 0 / 0 | 44592 | PASS |
| ab_B | 1314bc4 | 18/18 | 6522 | 0 | 0 | 10822 | 0 | 0 / 0 | 44592 | PASS |
| ab_A | 1314bc4 | 18/18 | 6386 | 0 | 0 | 10548 | 0 | 0 / 0 | 44592 | PASS |
| efg_A | 1314bc4 | 20/20 | 6066 | 0 | 0 | 11012 | 0 | 0 / 0 | 44592 | PASS |
| efg_B | 1314bc4 | 20/20 | 6033 | 0 | 0 | 10979 | 0 | 0 / 0 | 44592 | PASS |
| h_B | 1314bc4 | 20/20 | 4548 | 0 | 0 | 9883 | 0 | 0 / 0 | 44592 | PASS |
| h_A | 1314bc4 | 20/20 | 4564 | 0 | 0 | 10829 | 0 | 0 / 0 | 44592 | PASS |
| settle_A | 30c894a | 12/12 | 3169 | 52 | 0 | 18873 | 0 | 0 / 0 | 48464 | FAIL |
| settle_B | 30c894a | 12/12 | 4926 | 8 | 0 | 19482 | 0 | 0 / 0 | 48464 | FAIL |
| ab_B | 30c894a | 18/18 | 6537 | 12 | 0 | 17640 | 0 | 0 / 0 | 48464 | FAIL |
| ab_A | 30c894a | 18/18 | 6410 | 66 | 2 | 20761 | 2 | 0 / 0 | 48464 | FAIL |
| efg_A | 30c894a | 20/20 | 6269 | 54 | 0 | 18902 | 0 | 0 / 0 | 48200 | FAIL |
| efg_B | 30c894a | 20/20 | 6271 | 8 | 0 | 18851 | 0 | 0 / 0 | 48200 | FAIL |
| h_B | 30c894a | 20/20 | 4581 | 1 | 0 | 15447 | 0 | 0 / 0 | 48200 | PASS |
| h_A | 30c894a | 20/20 | 4571 | 16 | 0 | 18919 | 0 | 0 / 0 | 48200 | FAIL |

## 2. Effetto dello scrittore (W1, W2)

| Profilo | ≥15 ms candidato | IC 95 % | ≥15 ms riferimento | IC 95 % | Rapporto | Release (esposizione) cand. / rif. | Intervalli disgiunti |
|---|---:|---|---:|---|---:|---|---|
| A | 0 | [0.0, 3.7] | 188 | [162.1, 216.9] | 0.00 | 20182 / 20419 | sì |
| B | 0 | [0.0, 3.7] | 29 | [19.4, 41.6] | 0.00 | 21938 / 22315 | sì |

W1 (profilo A): riduzione **DIMOSTRATA**.
W2 (profilo B): riduzione dimostrata.
Il confronto è fra build diverse; le release di avvio F6/H18 sono più leggere nel candidato (PlanSplit V2), quindi W1/W2 misurano lo scrittore insieme a V2 e `validDefinition`.

## 3. Prima STAND dopo il boot (`settle_A`, prima finestra)

- Finestra da release 44024, stato iniziale READY/IDLE; massimo 9270 µs (STAND_DEPLOY); overrun 0; perse 0.
- Release perse a comando in `settle_A`: 0 (nessuna).

## 4. Prove con la dashboard

Solo le prove con un registratore di stato HTTP (`http_watch.py`) hanno finestre; quella con la sola dashboard ha i contatori dal boot letti via USB a riposo (`dash_counters.py`).

| Prova | Release | ≥15 ms | Max µs | Overrun | Perse comando / riposo | heap_min_free | Esito P0 |
|---|---:|---:|---:|---:|---|---:|---|
| dash2 | 5982 | 0 | 10670 | 0 | 0 / 0 | 43368 | PASS |

## 5. Verdetti

- **W1 scrittore:** DIMOSTRATO.
- **P0:** CHIUSO (catena 8/8 prove PASS, dashboard con registratore 1/1; sola dashboard D1 PASS).
- **Sicurezza:** PASS (passi falliti o abort: 0; heap_min_free minimo 44592 B, soglia 24576 B).
