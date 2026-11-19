# Locomozione vNext — patch di ricerca (13/11/2026)

**Non applicate al firmware.** Sono candidati di ricerca sul Motion dev.7 `b5610620d0ce`, verificati solo offline (gemello MuJoCo e gate offline del firmware). Nessuna è qualificata sull'hardware.

| Patch | Ruolo |
|---|---|
| `F40T_JCB66.patch` | Base FAST 40 (legge, transizione di modo, joystick FAST), 12/11/2026 |
| `V7.patch` | Candidato principale vNext, VERIFIED OFFLINE |
| `V7L60.patch` | Variante diagnostica: sollevamento posteriore FAST 6,0 mm |
| `C2_crawl.patch` | Prototipo CRAWL nello slot SLOW |
| `T3_trot.patch` | Prototipo TROT nello slot FAST |

- **Rigenerazione:** `../make_vnext_patch.py <albero dev.7> F40T_JCB66.patch <out.patch> [opzioni]`; il risultato è byte per byte identico a questi file. Le opzioni di ogni variante sono in `REPORT_VNEXT_LOCO.md`, `REPORT_VNEXT_F2.md` e nella `PROVENANCE.txt` degli artefatti.
- **Integrità:** `sha256sum -c SHA256SUMS`.
- **Evidenze:** `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/` (`HANDOFF_VNEXT_F2.md`).
