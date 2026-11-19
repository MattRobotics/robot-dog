# ADR-008 — Motion Timing P0: chiusura sul robot e congelamento

Data: 2026-11-19. Stato: **Accepted**, con la Stable `v1.0.2` (`1314bc4`).

## Contesto

**Il criterio P0**, fissato il 17/11 e mai modificato, vale in ogni prova:
- massimo per release ≤ 16 000 µs;
- 0 overrun (lavoro > 20 ms);
- 0 release operative perse, prima STAND compresa.

**Dal 10 al 18/11 P0 è rimasto aperto in ogni build:** sull'ultima, `30c894a`, il massimo era 20 761 µs con 2 overrun.

**L'RCA del 18/11** ha mostrato che il residuo oltre 16 ms era calcolo deterministico:
- doppie soluzioni di compensazione;
- IK nominale sterzata;
- nominale più compensazione nella stessa release.

Il carico HTTP di stato lo amplificava. Il nucleo FP32 (`05e1214`) e la P1 minima (`eb1ba28`), integrati in `1314bc4`, attaccano quel calcolo.

## Decisione

1. **Motion Timing P0 CLOSED — VERIFIED HARDWARE nel perimetro delle prove del 19/11/2026** ([evidenze](../Validation_Reports/Motion_1314bc4_HW_2026-11-19/README.md)).
   - **Prove:** catena A/B sulla fixture 8/8 (profili HTTP A e B), dashboard D1 e D2, blocco D 27/27, S9 15/15.
   - **Massimo osservato:** 12 729 µs, contro un budget di 16 000 µs: margine misurato **3 271 µs**.
   - **Overrun e release perse:** zero in ogni prova eseguita. STOP USB e HTTP verificati.
   - **Conformi nelle prove:** NVS, calibrazione, SAFE_OFF, memoria e comunicazione DALY.
2. **Congelamento.**
   - Nessun altro lavoro di ottimizzazione P0 in questa fase.
   - P0 si riapre solo con una **nuova evidenza di regressione**: una prova sul robot, con lo stesso criterio, che mostri un massimo sopra 16 ms, un overrun o una release operativa persa su firmware qualificato.
3. **Ogni modifica futura alla legge Motion** ripete la misura sul robot con lo stesso criterio prima di diventare baseline: catena A/B e blocchi pertinenti. Valgono le regole di sessione dell'[ADR-007](ADR-007_Motion_Timing_Integration_and_Heap_Margin.md): al massimo 2 client HTTP di prova, controllo M0, processi dal registro.

## Che cosa la chiusura non dichiara

- **Nessun WCET matematico universale.** Il massimo è quello osservato nelle sequenze e con i carichi HTTP provati.
- **Nessuna locomozione a terra qualificata.** Tutte le prove sono sulla fixture; STANDARD e FAST hanno `hardware_qualified=false`.
- **Nessuna verifica di ogni condizione IMU.** Ad esempio la condizione `random_3` di `kSteer` (beccheggio ≈ +55 mrad) è certificata solo offline.

## Inventario

**Integrate nella linea della Stable (`main` dopo `v1.0.2`):**

| Data | Voce |
|---|---|
| 14/11 | Patch A: correzione IMU nel frame di WALK_SETTLE dopo una camminata sterzata |
| 14/11 | Joystick FAST: cambio di passo su 4 cicli |
| 14/11 | Riarmo STAND dopo STOP/SAFE_OFF a coppie OFF |
| 14/11 | Conversioni servo risolte una volta per revisione di calibrazione |
| 14/11 | Telemetria DALY D1 |
| 15–16/11 | IRAM G1 e G2s |
| 15–16/11 | T2R1M |
| 15–16/11 | E1/E1b |
| 15–16/11 | P3a/P3b |
| 15–16/11 | P5 |
| 15–16/11 | F3: FAST 29,6 mm/s |
| 17–18/11 | `validDefinition` |
| 17–18/11 | PlanSplit V2 |
| 17–18/11 | Ancora di STOP prima del primo frame |
| 17–18/11 | Writer HTTP di stato senza printf |
| 18/11 | ACQUIRE v2: registro 85 a riposo |
| 18/11 | P1 minima: ricalcolo dell'alzata del raccordo a mezzo tick |
| 18/11 | Nucleo cinematico FP32 |
| 18–19/11 | `kSteer` SLOW ricertificato: correttezza del box, non timing |

**Sperimentate e scartate o non integrate:**
- **P4:** nessun guadagno misurato.
- **IRAM G2 completo:** MEMORY_STOP, sostituito da G2s.
- **Riuso trigonometrico nel kernel:** guadagno insufficiente, solo copie.
- **Split per funzione di REST:** nessuna patch.
- **Opzioni WALK che cambiano i frame, W1 e W2/B:** non integrate.
- **P1 completa (`TiltedContactIk`)**, varianti della regola del 2 % e cambio della tolleranza di Newton.
- **Tuning HTTP.**
- **Strumentazione diagnostica dev.8** (PMU, D2, D3): resta come patch, fuori dalle build operative.

**Idee future, non decise:**
- buffer lwIP e Wi-Fi in PSRAM (ADR-007);
- piano dei segmenti e `compensationSafe` in FP32, oggi tenuti in double per mandato;
- riduzione del costo di I/O sul bus servo.

**Costi residui noti, non blocker per il budget attuale:**
- **READ:** 3,4–4,0 ms nelle release più pesanti.
- **FRAME:** circa 1,2–1,3 ms; 2,5 ms nella release di 12 729 µs.
- **Pianificazione STAND/PRONE:** REST 2,1 ms in una release del piano PRONE.

## Conseguenze

- `main` e la Stable `v1.0.2` sono la baseline Motion. Il prossimo lavoro è la milestone **Ground Locomotion & Gait Performance** ([OPEN_ITEMS](../OPEN_ITEMS.md)).
- Una variazione di READ o FRAME, ad esempio per un cambio del bus o del numero di letture per release, va rimisurata contro questo margine.
- Riferimenti:
  - [rapporto HW](../Validation_Reports/Motion_1314bc4_HW_2026-11-19/qualification/REPORT_HW.md);
  - [completamento](../Validation_Reports/Motion_1314bc4_HW_2026-11-19/completion/REPORT_HW_COMPLETION.md);
  - [manifest della release](../Validation_Reports/Motion_1314bc4_HW_2026-11-19/RELEASE_MANIFEST.md);
  - [FP32 e P1](../Validation_Reports/Motion_FP32_2026-11-18/REPORT.md);
  - [campagna P0 del 17/11](../Validation_Reports/MATDOG_1_0_2_P0_2026-11-17/REPORT.md).
