# Tabelle A/B (generate da `ab_compare.py 0623`)

Profilo A = carico HTTP del 16/11 (poller 0,5 s + watcher 1 Hz); B = poller 1,0 s, nessun watcher. Stessa build `30c894a0d430`, stessa sessione, stessi comandi.

## 1. Per strumento

| Strumento | Prof. | Passi | Richieste/s (stato/s) a coppie ON | Release nelle finestre | 10–15 ms | ≥15 ms | ≥20 ms | Finestre max ≥15 / ≥16 ms | Max µs | Overrun | Perse (comando / riposo) |
|---|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| settle (D8-D11) | A | 12/12 | 3.54 (2.88) | 3166 | 13 | 0 | 0 | 0 / 0 | 10928 | 0 | 0 / 0 |
| settle (D8-D11) | B | 12/12 | 1.22 (0.91) | 4835 | 0 | 0 | 0 | 0 / 0 | 9936 | 0 | 0 / 0 |
| blocks A+B | A | 18/18 | 3.54 (2.86) | 6386 | 5 | 0 | 0 | 0 / 0 | 10548 | 0 | 0 / 0 |
| blocks A+B | B | 18/18 | 1.22 (0.92) | 6522 | 3 | 0 | 0 | 0 / 0 | 10822 | 0 | 0 / 0 |
| blocks E+F+G (FAST) | A | 20/20 | 3.52 (2.86) | 6066 | 20 | 0 | 0 | 0 / 0 | 11012 | 0 | 0 / 0 |
| blocks E+F+G (FAST) | B | 20/20 | 1.23 (0.92) | 6033 | 2 | 0 | 0 | 0 / 0 | 10979 | 0 | 0 / 0 |
| block H (RECOVERY) | A | 20/20 | 3.55 (2.90) | 4564 | 3 | 0 | 0 | 0 / 0 | 10829 | 0 | 0 / 0 |
| block H (RECOVERY) | B | 20/20 | 1.24 (0.94) | 4548 | 0 | 0 | 0 | 0 / 0 | 9883 | 0 | 0 / 0 |

## 2. Totali

| Misura | A | B | B/A |
|---|---:|---:|---:|
| Richieste HTTP/s a coppie ON (tutte) | 3.54 | 1.23 | 0.35 |
| Letture di stato Motion/s a coppie ON | 2.87 | 0.92 | 0.32 |
| Secondi a coppie ON | 320.20 | 325.50 | 1.02 |
| Release nelle finestre armate | 20182 | 21938 | 1.09 |
| Release 10–15 ms | 41 | 5 | 0.12 |
| Release ≥15 ms | 0 | 0 | – |
| Release ≥20 ms | 0 | 0 | – |
| Finestre con massimo ≥15 ms | 0 | 0 | – |
| Finestre con massimo ≥16 ms | 0 | 0 | – |
| Massimo assoluto (µs) | 11012 | 10979 | 1.00 |
| Overrun (>20 ms) | 0 | 0 | – |
| Release perse durante un comando | 0 | 0 | – |
| Release perse a riposo (release IDLE) | 0 | 0 | – |

Release ≥15 ms: 0 contro 0. A esposizione uguale (stessa sequenza; release nelle finestre 20182 contro 21938), la probabilità di vederne 0 o meno in B per caso, se il profilo non contasse (binomiale, p = 0,5), è 1.0e+00.

## 3. PMU per classe (somma delle prove di ciascun profilo)

COMPENSATE e NOMINAL+COMPENSATE: istruzioni per ciclo (IPC) e quota di cicli fermi sulla memoria istruzioni (`istall_mem`). Ogni release conta un solo gruppo di contatori (release mod 7). OTHER comprende anche le release a riposo armate.

| Classe | Prof. | Release | ms/release | COMPENSATE ms/release | Istruzioni/release (N+C) | IPC N+C | IPC COMP | istall_mem N+C | istall_mem COMP | dstall_mem N+C | Interrupt/release | Altri task sul core |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## 4. Release più pesanti con numero di release e letture di stato

Record `MAX_WORK` dei dump e `MOTION_PMU_HEAVIEST` (contano solo le release dentro la prova). «+k» = release dopo l'ultima lettura di stato di qualunque client (il campo `releases` della risposta). A caso, +1 ha probabilità 1/intervallo.

| Prof. | Prova | Release | Record | Fase | Lavoro µs | COMPENSATE µs | Soluzioni | Dopo la lettura | Intervallo fra letture |
|---|---|---:|---|---|---:|---:|---:|---:|---:|
| A | settle_A_0623 | 48166 | MAX_WORK | PRONE_HEIGHT | 8516 | 2506 | 1 | +1 | 17 |
| B | settle_B_0623 | 53394 | MAX_WORK | PRONE_LIFT | 9922 | 1118 | 2 | +52 | 57 |
| A | ab_A_0623 | 63540 | MAX_WORK | PRONE_LIFT | 8899 | 2091 | 2 | +1 | 27 |
| A | ab_A_0623 | 66576 | MAX_WORK | PRONE_LIFT | 8909 | 2235 | 2 | +1 | 1 |
| B | ab_B_0623 | 56952 | MAX_WORK | PRONE_LIFT | 9844 | 1144 | 2 | +16 | 57 |
| B | ab_B_0623 | 60234 | MAX_WORK | PRONE_DEPLOY | 9202 | 2286 | 2 | +1 | 57 |
| A | efg_A_0623 | 72768 | MAX_WORK | PRONE_HEIGHT | 8762 | 2216 | 2 | +2 | 28 |
| B | efg_B_0623 | 79214 | MAX_WORK | PRONE_LIFT | 8844 | 1182 | 2 | +11 | 56 |
| A | h_A_0623 | 88728 | MAX_WORK | PRONE_LIFT | 9104 | 2217 | 2 | +1 | 28 |
| B | h_B_0623 | 84061 | MAX_WORK | PRONE_LIFT | 8803 | 1220 | 2 | +25 | 57 |

A: 4 release su 5 sono la prima dopo una lettura di stato; a caso se ne attenderebbero 1.17.

B: 1 release su 5 sono la prima dopo una lettura di stato; a caso se ne attenderebbero 0.09.

## 5. Finestre appaiate (stesso strumento, stessa finestra armata)

Massimo di ogni finestra STAND/PRONE (con i WALK, joystick e STOP che la seguono). Sezioni del massimo: [READ, NOMINAL, COMPENSATE, CONVERT, FRAME, REST] µs; lavoro: [soluzioni, IK, Newton, FK].

| Strumento | # | Inizio (A) | Max A µs | fase A | sezioni A | lavoro A | ≥15 A | Max B µs | fase B | sezioni B | lavoro B | ≥15 B |
|---|---:|---|---:|---|---|---|---:|---:|---|---|---|---:|
| settle (D8-D11) | 1 | READY | 9270 | STAND_DEPLOY | [1768, 5433, 1367, 536, 0, 166] | [1, 4, 6, 10] | 0 | 8247 | HOLD | [3333, 0, 3078, 407, 1260, 169] | [1, 4, 4, 11] | 0 |
| settle (D8-D11) | 2 | HEIGHT_CHANGE | 10786 | WALK_STANDARD | [4029, 3242, 1469, 495, 1284, 267] | [0, 0, 0, 0] | 0 | 9879 | WALK_STANDARD | [3924, 2733, 1249, 443, 1272, 258] | [0, 0, 0, 0] | 0 |
| settle (D8-D11) | 3 | HEIGHT_CHANGE | 10928 | WALK_STANDARD | [3967, 3274, 1635, 477, 1307, 268] | [0, 0, 0, 0] | 0 | 9936 | WALK_STANDARD | [3822, 1069, 3200, 431, 1192, 222] | [1, 4, 4, 11] | 0 |
| settle (D8-D11) | 4 | PRONE_DESCENT_NOT_VALIDATED | 8516 | PRONE_HEIGHT | [2117, 0, 2506, 494, 1277, 2122] | [1, 4, 4, 8] | 0 | 9922 | PRONE_LIFT | [2743, 4150, 1118, 575, 1177, 159] | [2, 8, 10, 18] | 0 |
| blocks A+B | 1 | ACQUIRING_REAL_POSE | 7980 | STAND_DEPLOY | [3738, 903, 1350, 393, 1267, 329] | [1, 4, 6, 10] | 0 | 8199 | STAND_LIFT | [3666, 997, 1467, 454, 1317, 298] | [1, 4, 6, 10] | 0 |
| blocks A+B | 2 | HEIGHT_CHANGE | 8333 | HEIGHT | [3981, 764, 1535, 412, 1305, 336] | [1, 4, 6, 10] | 0 | 6353 | HEIGHT | [2868, 0, 1992, 212, 1175, 106] | [1, 4, 4, 8] | 0 |
| blocks A+B | 3 | HEIGHT_CHANGE | 8158 | HEIGHT | [3969, 795, 1470, 358, 1233, 333] | [1, 4, 6, 10] | 0 | 6848 | HEIGHT | [2823, 842, 1590, 245, 1176, 172] | [1, 4, 6, 10] | 0 |
| blocks A+B | 4 | HEIGHT_CHANGE | 7601 | HOLD | [3603, 0, 2281, 305, 1190, 222] | [1, 4, 4, 11] | 0 | 7379 | HEIGHT | [3673, 702, 1169, 343, 1177, 315] | [1, 4, 6, 10] | 0 |
| blocks A+B | 5 | PRONE_DESCENT_NOT_VALIDATED | 8899 | PRONE_LIFT | [3917, 808, 2091, 509, 1253, 321] | [2, 8, 10, 18] | 0 | 9844 | PRONE_LIFT | [2752, 4056, 1144, 556, 1175, 161] | [2, 8, 10, 18] | 0 |
| blocks A+B | 6 | ACQUIRING_REAL_POSE | 8682 | STAND_DEPLOY | [3871, 796, 2005, 408, 1286, 316] | [2, 8, 10, 18] | 0 | 8135 | ALIGN | [1537, 4041, 808, 457, 1175, 117] | [1, 4, 6, 10] | 0 |
| blocks A+B | 7 | HEIGHT_CHANGE | 6876 | HEIGHT | [3394, 0, 1792, 303, 1177, 210] | [1, 4, 4, 8] | 0 | 6598 | HOLD | [2036, 0, 2702, 359, 1273, 228] | [1, 4, 4, 11] | 0 |
| blocks A+B | 8 | HEIGHT_CHANGE | 10548 | WALK_STANDARD | [3982, 1216, 3345, 468, 1268, 269] | [1, 4, 4, 11] | 0 | 10822 | WALK_STANDARD | [3939, 3256, 1677, 467, 1248, 235] | [0, 0, 0, 0] | 0 |
| blocks A+B | 9 | STAND_FROM_STOPPED_WALK | 10268 | WALK_STANDARD | [3587, 2603, 2031, 583, 1254, 210] | [1, 4, 4, 11] | 0 | 9656 | WALK_STANDARD | [3892, 2218, 1740, 417, 1176, 213] | [1, 4, 4, 11] | 0 |
| blocks A+B | 10 | PRONE_DESCENT_NOT_VALIDATED | 8909 | PRONE_LIFT | [3850, 726, 2235, 470, 1280, 348] | [2, 8, 9, 17] | 0 | 9202 | PRONE_DEPLOY | [3933, 889, 2286, 474, 1297, 323] | [2, 8, 9, 17] | 0 |
| blocks E+F+G (FAST) | 1 | ACQUIRING_REAL_POSE | 8777 | STAND_DEPLOY | [3765, 772, 2196, 420, 1313, 311] | [2, 8, 9, 17] | 0 | 8175 | STAND_DEPLOY | [3726, 894, 1563, 411, 1282, 299] | [1, 4, 6, 10] | 0 |
| blocks E+F+G (FAST) | 2 | HEIGHT_CHANGE | 11012 | WALK_STANDARD | [4041, 3192, 1725, 472, 1301, 281] | [0, 0, 0, 0] | 0 | 10979 | WALK_STANDARD | [3886, 3036, 1946, 508, 1349, 254] | [0, 0, 0, 0] | 0 |
| blocks E+F+G (FAST) | 3 | HEIGHT_CHANGE | 10815 | WALK_STANDARD | [3971, 3051, 1842, 429, 1284, 238] | [0, 0, 0, 0] | 0 | 9637 | WALK_STANDARD | [3765, 910, 3114, 403, 1210, 235] | [1, 4, 4, 11] | 0 |
| blocks E+F+G (FAST) | 4 | STAND_FROM_STOPPED_WALK | 9147 | WALK_STANDARD | [3342, 1917, 1942, 481, 1285, 180] | [1, 4, 4, 11] | 0 | 8546 | WALK_STANDARD | [3735, 1750, 1298, 361, 1179, 223] | [1, 4, 4, 11] | 0 |
| blocks E+F+G (FAST) | 5 | HEIGHT_CHANGE | 10967 | WALK_STANDARD | [3970, 3162, 1808, 490, 1287, 250] | [0, 0, 0, 0] | 0 | 9187 | WALK_STANDARD | [3691, 578, 2923, 448, 1269, 278] | [1, 4, 4, 11] | 0 |
| blocks E+F+G (FAST) | 6 | PRONE_DESCENT_NOT_VALIDATED | 8762 | PRONE_HEIGHT | [3930, 588, 2216, 440, 1248, 340] | [2, 8, 9, 17] | 0 | 8844 | PRONE_LIFT | [1559, 4047, 1182, 594, 1277, 185] | [2, 8, 10, 18] | 0 |
| block H (RECOVERY) | 1 | ACQUIRING_REAL_POSE | 8170 | STAND_DEPLOY | [3892, 893, 1477, 419, 1192, 297] | [1, 4, 6, 10] | 0 | 9360 | ALIGN | [2103, 5020, 1391, 617, 0, 229] | [1, 4, 6, 10] | 0 |
| block H (RECOVERY) | 2 | HEIGHT_CHANGE | 8983 | WALK_STANDARD | [3435, 898, 2832, 384, 1252, 182] | [1, 4, 4, 11] | 0 | 9593 | WALK_STANDARD | [3725, 2298, 1715, 426, 1217, 212] | [0, 0, 0, 0] | 0 |
| block H (RECOVERY) | 3 | STAND_FROM_STOPPED_WALK | 9798 | WALK_STANDARD | [4041, 1820, 1841, 552, 1281, 263] | [1, 4, 1, 8] | 0 | 7959 | WALK_STANDARD | [2130, 2143, 1845, 410, 1192, 239] | [1, 4, 4, 11] | 0 |
| block H (RECOVERY) | 4 | STAND_FROM_STOPPED_WALK | 9961 | WALK_STANDARD | [3842, 3883, 437, 387, 1180, 232] | [0, 0, 0, 0] | 0 | 9883 | WALK_STANDARD | [3912, 3493, 445, 481, 1289, 263] | [0, 0, 0, 0] | 0 |
| block H (RECOVERY) | 5 | HEIGHT_CHANGE | 10829 | WALK_STANDARD | [3878, 2332, 2627, 440, 1312, 240] | [1, 4, 4, 11] | 0 | 9810 | WALK_STANDARD | [3993, 2434, 1426, 413, 1262, 282] | [0, 0, 0, 0] | 0 |
| block H (RECOVERY) | 6 | STAND_FROM_STOPPED_WALK | 8490 | WALK_STANDARD | [2203, 3872, 426, 467, 1284, 238] | [0, 0, 0, 0] | 0 | 7295 | WALK_STANDARD | [3957, 2423, 313, 375, 0, 227] | [0, 0, 0, 0] | 0 |
| block H (RECOVERY) | 7 | STAND_FROM_STOPPED_WALK | 10203 | WALK_STANDARD | [3978, 2294, 1877, 516, 1266, 272] | [1, 4, 4, 11] | 0 | 8934 | WALK_STANDARD | [3874, 1542, 1587, 421, 1224, 286] | [1, 4, 4, 11] | 0 |
| block H (RECOVERY) | 8 | PRONE_DESCENT_NOT_VALIDATED | 9104 | PRONE_LIFT | [3893, 918, 2217, 499, 1257, 320] | [2, 8, 9, 17] | 0 | 8803 | PRONE_LIFT | [1524, 4109, 1220, 552, 1251, 147] | [2, 8, 10, 18] | 0 |

## 6. Release perse

| Prova | Perse | Release | Stato prima → dopo | Durante un comando | Prima perdita del comando |
|---|---:|---|---|---|---|

## 7. Memoria, stack, BMS per prova

| Prova | heap_min_free (min dal boot) | loop stack min libero | httpd stack min libero | status_service_max_us | Campioni /status | BMS non OK | fail_streak max | SOC inizio→fine |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| settle_A_0623 | 44592 | 8352 | 9972 | 2448 | 65 | 0 | 0 | 56.7→56.6 |
| settle_B_0623 | 44592 | 8352 | 9972 | 2448 | 30 | 0 | 0 | 56.6→56.4 |
| ab_A_0623 | 44592 | 7920 | 9972 | 2851 | 86 | 0 | 0 | 56.2→56.0 |
| ab_B_0623 | 44592 | 7920 | 9972 | 2448 | 41 | 0 | 0 | 56.4→56.2 |
| efg_A_0623 | 44592 | 7920 | 9972 | 2851 | 82 | 0 | 0 | 56.0→55.9 |
| efg_B_0623 | 44592 | 7920 | 9972 | 2851 | 38 | 0 | 0 | 55.9→55.7 |
| h_A_0623 | 44592 | 7788 | 9972 | 2851 | 63 | 0 | 0 | 55.5→55.4 |
| h_B_0623 | 44592 | 7788 | 9972 | 2851 | 30 | 0 | 0 | 55.7→55.5 |
