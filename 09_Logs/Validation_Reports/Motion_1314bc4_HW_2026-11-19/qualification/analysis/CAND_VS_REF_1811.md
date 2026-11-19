# Candidato `1314bc411c58` contro riferimento 18/11 `85bad0e324da` (generato da `MATDOG_REF=1811 cand_vs_ref.py 0623`)

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
| settle_A | 85bad0e | 12/12 | 3256 | 36 | 0 | 19025 | 0 | 5 / 0 | 48852 | FAIL |
| settle_B | 85bad0e | 12/12 | 4862 | 8 | 0 | 18479 | 0 | 0 / 0 | 41652 | FAIL |
| ab_B | 85bad0e | 18/18 | 6432 | 10 | 0 | 18785 | 0 | 0 / 0 | 41652 | FAIL |
| ab_A | 85bad0e | 18/18 | 6400 | 63 | 1 | 20289 | 1 | 0 / 0 | 34344 | FAIL |
| efg_A | 85bad0e | 20/20 | 6216 | 29 | 1 | 20145 | 1 | 0 / 0 | 34344 | FAIL |
| efg_B | 85bad0e | 20/20 | 6231 | 5 | 0 | 17265 | 0 | 0 / 0 | 34344 | FAIL |
| h_B | 85bad0e | 20/20 | 4559 | 7 | 0 | 18424 | 0 | 0 / 0 | 34344 | FAIL |
| h_A | 85bad0e | 20/20 | 4396 | 19 | 1 | 20604 | 1 | 0 / 0 | 34344 | FAIL |

## 2. Effetto dello scrittore (W1, W2)

| Profilo | ≥15 ms candidato | IC 95 % | ≥15 ms riferimento | IC 95 % | Rapporto | Release (esposizione) cand. / rif. | Intervalli disgiunti |
|---|---:|---|---:|---|---:|---|---|
| A | 0 | [0.0, 3.7] | 147 | [124.2, 172.8] | 0.00 | 20182 / 20268 | sì |
| B | 0 | [0.0, 3.7] | 30 | [20.2, 42.8] | 0.00 | 21938 / 22084 | sì |

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
