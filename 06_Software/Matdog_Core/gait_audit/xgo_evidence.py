"""Inventory existing static evidence from the pinned, read-only XGO archive."""
import csv,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'09_Logs/Validation_Reports/G4_Gait_Envelope'
def main():
 root=Path('/tmp/matdog-g35-xgo')
 patterns=['docs/reverse/milestone_g*.md','docs/reverse/milestone_h1_clean_room_interface_specification.md','docs/reverse/milestone_h2_matdog_transfer_boundary.md','evidence/gait/milestone_g*.csv','evidence/runtime/milestone_g2*.csv','evidence/vendor/yahboom_gait_mode_map.csv']
 files=sorted({p for pattern in patterns for p in root.glob(pattern)})
 inventory=[]
 for p in files:
  data=p.read_bytes();item=dict(path=str(p.relative_to(root)),sha256=hashlib.sha256(data).hexdigest())
  if p.suffix=='.csv':
   rows=list(csv.DictReader(data.decode().splitlines()));item['rows']=len(rows)
   item['classifications']=sorted({r.get('confidence',r.get('status','')) for r in rows})
  inventory.append(item)
 findings=[
  ['phase','VERIFIED','milestone_g_phase_state_map.csv PHASE-G-001..009','Four phase slots, shared increment, strict >P reset; MATDOG uses normalized modulo and supplied time.'],
  ['walk','VERIFIED local / CORROBORATED physical','milestone_g_leg_phase_order_map.csv LEGPH-G-002; milestone_g21_semantic_reconciliation.md','Quarter-spaced logical slots. MATDOG chooses canonical rear/front alternating sequence and studies support.'],
  ['trot','VERIFIED local / CORROBORATED physical','LEGPH-G-001,005,006; G2.1','Alternating slot pairs; physical mode and leg mapping remain corroborated.'],
  ['writers','VERIFIED / mode 2 CORROBORATED','milestone_g2_writer_formulas.csv MG2-F-001..102','Six explicit writers including mark-time/high-walk candidate; geometry/signs/units unbound.'],
  ['continuity','VERIFIED local','milestone_g2_writer_continuity.csv','Piecewise endpoints and integer-floor residuals; MATDOG instead uses explicit C2 swing.'],
  ['commands','VERIFIED local','milestone_g_command_state_map.csv','Translation/yaw/body command writers, mode dispatch, phase reset. No numeric transfer.'],
  ['interpolation','VERIFIED','milestone_g_trajectory_equation_map.csv; milestone_g_cartesian_target_writer_map.csv','Linear previous-to-target interpolation; MATDOG quintic.'],
  ['body before IK','VERIFIED call order / CORROBORATED role','milestone_g_ik_convergence_map.csv','Body correction then Cartesian controller/interpolation then IK component. No IMU transfer.'],
  ['zero and stop','VERIFIED local / UNKNOWN complete physical contract','milestone_g_gait_mode_dispatch.csv MODE-G-007','Zero components clear phase initialization; smoothing exists. No proof of world-locked phase-consistent terminal stand.'],
  ['watchdog','UNKNOWN locomotion expiry','G/G2/G2.1 bounded evidence search; H2 host trace','Host response timeout is not command expiry. MATDOG freshness contract independently specified.'],
  ['timing','VERIFIED requested delay / UNKNOWN actual cadence','milestone_g2_runtime_binding.md; runtime tick/task maps','G2 linked mapping and 2 ms requested delay supersede G historical uncertainty; no timing value transferred.'],
  ['crawl and mark-time','UNKNOWN physical crawl / VERIFIED mark-time local writer','MG2-F-001..012; mode dispatch','No recovered crawl physical pose assumed; zero MATDOG command requests stop, not mark-time.']]
 out=dict(source_commit='a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298',view='read-only git archive; original checkout untouched',firmware_sha256='71032255ffac656c75234c2dcf6a40b307aefc753a92a04c1d0f707d67db6b0e',inventory=inventory,findings=[dict(zip(('topic','confidence','evidence','decision'),r)) for r in findings],excluded_transfer=['geometry','joint angles','zeros','signs','joint limits','stride','lift','duty','period','gains','body height','velocity limits','acceleration limits','IMU tuning'])
 (OUT/'xgo_architecture.json').write_text(json.dumps(out,indent=2)+'\n')
 print('XGO evidence indexed:',len(files),'files')
if __name__=='__main__':main()
