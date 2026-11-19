# Patch del 17/11/2026 (copie NON applicate)

Copie byte per byte dei diff originali conservati localmente; gli SHA256 sono in [`SHA256SUMS`](SHA256SUMS) e nel [manifest](../EVIDENCE_MANIFEST.md).
- **Nessuna è applicata al branch:** il branch resta al sorgente `202ecbc` più questa documentazione.
- **Formato:** due tipi di diff, con radici diverse:
  - `git diff`: si applica dalla radice del repository con `git apply`;
  - `diff -ru`: si applica con `patch` e il livello `-p` indicato sotto.

| File | Contenuto | Applicazione su `202ecbc` | Stato |
|---|---|---|---|
| `FINAL.diff` | `validDefinition()` e pin, PlanSplit V2, correzione STOP → STAND (4 file) | radice: `git apply patches/FINAL.diff` | candidato consolidato |
| `HTTP_CANDIDATE.diff` | scrittore di stato in `MotionService.cpp` | radice, dopo `FINAL.diff` (o da solo: tocca un file diverso): `patch -p1 < patches/HTTP_CANDIDATE.diff` | isolata, VERIFIED OFFLINE |
| `COMPOSED_vs_202ecbc.diff` | i due precedenti insieme (5 file, +167 −20) | radice: `patch -p0 < patches/COMPOSED_vs_202ecbc.diff` | composizione proposta per la prossima APP |
| `component_validDefinition_202ecbc.diff` | solo `validDefinition()` e pin | radice: `git apply` | componente di `FINAL.diff` |
| `component_PlanSplit_V2.diff` | solo PlanSplit V2 | in `05_Firmware/MATDOG_Controller`, dopo `validDefinition`: `patch -p1` | componente di `FINAL.diff` |
| `component_STOP_STAND_fix.diff` | solo la correzione di `beginStop()` | in `05_Firmware/MATDOG_Controller`, dopo V2: `patch -p1` | componente di `FINAL.diff` |
| `NOT_INTEGRATED_A_kernel_trig_reuse.diff` | riuso di sin/cos dalla FK nel Jacobiano di contatto (5 sorgenti + pin) | in `05_Firmware/MATDOG_Controller`: `patch -p1` | **NON INTEGRATO** (DEFER); esatto bit per bit |
| `NOT_INTEGRATED_B_jacobian_xyy.diff` | Jacobiano di contatto ridotto per la geometria X/Y/Y (`TiltedContactIk.cpp` + pin) | in `05_Firmware/MATDOG_Controller`: `patch -p1` | **NON INTEGRATO** (DEFER); non bit per bit; **esclude A** |

**Verificato il 17/11 sera** su copie di `git archive 202ecbc`:
- `FINAL.diff` + `HTTP_CANDIDATE.diff` e `COMPOSED_vs_202ecbc.diff` producono lo stesso albero del candidato sigillato;
- i tre componenti in sequenza producono lo stesso albero di `FINAL.diff`;
- A e B si applicano sopra `FINAL.diff` (solo prova a vuoto).

`COMPOSED_vs_202ecbc.diff` contiene anche una riga `Only in …: build` (cartella di build locale), ignorata da `patch`.
