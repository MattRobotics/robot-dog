"""Final labelled report = frozen anonymous report + reveal + batch-effect section."""
import csv, hashlib, json, re, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

anon = (q.AUDIT / "blind_report_ANONYMOUS.md").read_text()
frozen = (q.AUDIT / "blind_report_ANONYMOUS.sha256").read_text().split()[0]
assert hashlib.sha256(anon.encode()).hexdigest() == frozen
mp = q.load_mapping()["mapping"]
batch = json.loads((q.AUDIT / "batch_effect_check.json").read_text())
lab = {sc: Path(mp[sc]["bin"]).name.split("__")[0] for sc in q.scodes()}

head = f"""# MATDOG — Final Labelled QC Audit Report, 18 × Feetech ST-3215-C018

This is the frozen anonymous report with labels substituted. **Nothing in the analysis, the
thresholds or the verdicts changed after the reveal** — the anonymous version is fixed by

```
{frozen}  blind_report_ANONYMOUS.md
```

and `sha256sum -c blind_report_ANONYMOUS.sha256` verifies it. Every S-code below has simply been
replaced by its label; the two extra sections at the end (batch-effect check and the labelled
action list) were added afterwards and are marked as such.

## Headline

| Label | Verdict |
|---|---|
| **M13** | Confirmed focal mechanical anomaly at encoder 2144–2271 (188°–200°) **and** a 50 % output cap |
| **M23** | 50 % output cap; systematic minimum-speed collapse in one direction — cause not separable from the cap |
| **M12** | Confirmed focal mechanical anomaly at encoder 2560–2751 (225°–242°), **no** configuration confound |
| the other 15 | No metric outside the robust population envelope; statistically indistinguishable from each other |

The two capped units (M13, M23) are both in the M-series. All six NEW units are clean.

---

"""

body = anon.split("\n", 1)[1]
for sc in sorted(q.scodes(), reverse=True):
    body = re.sub(rf"\b{sc}\b", f"**{lab[sc]}**", body)
body = body.replace("****", "**")

tail = f"""

---

## 15. Batch-effect check (added after the reveal, as authorized)

Twelve units were run on distinct servo IDs (M-series); six were run one at a time on ID 1
(NEW-series). Comparing the two acquisition groups, excluding the two capped units:

| Metric | M-series median | NEW-series median | Δ |
|---|---|---|---|
"""
for c in batch["comparison"]:
    tail += f"| {c['metric']} | {c['groupA']:.2f} | {c['groupB']:.2f} | {c['delta']:+.2f} |\n"

tail += """
**No batch effect on any performance metric.** MAX speed differs by 2.3 tick/s (0.08 %),
precision MAE by 0.03 ticks, hysteresis and bus voltage by zero. The only real difference is run
duration (+12.9 s for the NEW group), which is a consequence of the adaptive test sequence, not a
servo property. Start temperature differs by 1.5 °C, well inside the population spread.

This check can only *reduce* confidence in a finding, never create one, and it reduces none: the
three flagged units are all M-series, but so are 9 of the 15 clean units, and the group medians
are indistinguishable.

## 16. Action list

**Do not install M13 or M12** in a leg position without further work. Both have a repeatable
localized mechanical signature at a fixed output angle — for a quadruped this matters most if the
affected angle falls inside a joint's working range.

- **M13** — encoder 2144–2271 = **188.4°–199.7°** of output travel.
- **M12** — encoder 2560–2751 = **225.0°–241.9°** of output travel.

Note these are *encoder* angles; the mapping to joint angle depends on Position Offset
(register 31–32), which is not in the RAW. Read it before assuming the defect lies inside a
joint's used range.

**M23 is a recovery candidate, not a reject.** Everything anomalous about it is explainable by
the 50 % output cap. Read registers 48–49 and 16; if either is 500, restore 1000 and re-run QC
before judging the unit.

**M13 needs the same register check** — but even with the cap restored, its focal load anomaly
would remain, because its co-capped twin M23 is flat through the same region.

**Highest-value next step (read-only, needs authorization):** read Torque Limit (48–49) and Max
Torque (16) on all 18. It is a handful of register reads and it converts the largest open
question in this audit from an inference into a fact.

## 17. What this audit could not determine

- Whether the 50 % ceiling on M13/M23 is Torque Limit, Max Torque, or something else — no
  configuration register is captured in the RAW.
- Whether any unit ever reported a hardware fault: the firmware's status check reads register 65,
  an address the installed SCServo driver does not define (§1.3 B2).
- Whether M13's focal anomaly would persist at full torque — that needs a write and a re-run.
- Gear backlash as a separate quantity: only combined directional hysteresis is measurable here.
- Anything about long-term durability; this is a ~150 s characterization, not a life test.
"""

(q.AUDIT / "final_report_labeled.md").write_text(head + body + tail)
print(f"wrote final_report_labeled.md ({len((head+body+tail).splitlines())} lines)")
