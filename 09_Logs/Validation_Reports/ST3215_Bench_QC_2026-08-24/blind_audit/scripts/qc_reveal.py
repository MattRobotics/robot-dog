"""Post-freeze: reveal labels, write label_mapping_revealed.md, run the batch-effect check."""
import csv, hashlib, json, subprocess, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

frozen = (q.AUDIT / "blind_report_ANONYMOUS.sha256").read_text().split()[0]
actual = hashlib.sha256((q.AUDIT / "blind_report_ANONYMOUS.md").read_bytes()).hexdigest()
assert frozen == actual, "anonymous report changed after freeze - reveal aborted"

mp = q.load_mapping()
met = {r["scode"]: r for r in csv.DictReader(open(q.AUDIT / "blind_metrics.csv"))}
integ = {r["scode"]: r for r in csv.DictReader(open(q.AUDIT / "blind_integrity.csv"))}
S = q.scodes()

lab = {}
for sc in S:
    name = Path(mp["mapping"][sc]["bin"]).name
    label, sid, stamp = name.replace(".bin", "").split("__")
    lab[sc] = dict(label=label, sid=int(sid[2:]), stamp=stamp)

o = ["# Label mapping — revealed only after the anonymous report was frozen", "",
     f"Frozen `blind_report_ANONYMOUS.md` SHA256: `{frozen}`  ",
     f"Verified identical at reveal time.  ",
     f"Campaign key: `{mp['campaign_key']}`  ",
     f"Scheme: {mp['scheme']}", "",
     "| S-code | label | servo ID | run timestamp | RAW SHA256 |", "|---|---|---|---|---|"]
for sc in S:
    o.append(f"| {sc} | **{lab[sc]['label']}** | {lab[sc]['sid']} | {lab[sc]['stamp']} | "
             f"`{mp['mapping'][sc]['sha256'][:16]}…` |")
o += ["", "## Reverse (label → S-code)", "",
      "| label | S-code |", "|---|---|"]
for sc in sorted(S, key=lambda s: lab[s]["label"]):
    o.append(f"| {lab[sc]['label']} | {sc} |")
(q.AUDIT / "label_mapping_revealed.md").write_text("\n".join(o) + "\n")

# ---- batch-effect check (authorized post-freeze) ----
grpA = [s for s in S if lab[s]["sid"] != 1]
grpB = [s for s in S if lab[s]["sid"] == 1]
print(f"group A (distinct servo IDs): {len(grpA)} -> {sorted(lab[s]['label'] for s in grpA)}")
print(f"group B (all on servo ID 1):  {len(grpB)} -> {sorted(lab[s]['label'] for s in grpB)}")
print()
rows = []
for name, cols in (("MAX speed (tick/s)", "max_tps"), ("precision MAE", "prec_mae"),
                   ("hysteresis median", "hyst_med"), ("min travel median", "min_travel_med"),
                   ("bus voltage median", "volt_med_all"), ("start temperature", "static_pre_temp_med"),
                   ("temp rise", "temp_rise"), ("elapsed s", None)):
    def val(s):
        return float(integ[s]["elapsed_s"]) if cols is None else float(met[s][cols])
    a = [val(s) for s in grpA if s not in ("S04", "S05")]
    b = [val(s) for s in grpB if s not in ("S04", "S05")]
    rows.append((name, np.median(a), np.median(b), np.median(b) - np.median(a)))
    print(f"{name:24s} groupA_med={np.median(a):9.2f}  groupB_med={np.median(b):9.2f}  "
          f"delta={np.median(b)-np.median(a):+9.2f}")
print()
print("capped units by group:",
      {"A": [lab[s]['label'] for s in grpA if float(met[s]['max_load']) <= 600],
       "B": [lab[s]['label'] for s in grpB if float(met[s]['max_load']) <= 600]})
json.dump(dict(groupA=[lab[s]["label"] for s in grpA], groupB=[lab[s]["label"] for s in grpB],
               comparison=[dict(metric=n, groupA=a, groupB=b, delta=d) for n, a, b, d in rows]),
          open(q.AUDIT / "batch_effect_check.json", "w"), indent=2)
print("\nwrote label_mapping_revealed.md, batch_effect_check.json")
