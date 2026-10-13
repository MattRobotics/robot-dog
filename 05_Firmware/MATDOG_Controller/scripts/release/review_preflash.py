#!/usr/bin/env python3
"""Offline final review of a completed read-only capture. Never opens USB.
Review JSON is private, written after inspecting the named current snapshots and
recovering an independently stored/decrypted full backup. This is not flash GO.
"""
import argparse,datetime,hashlib,json
from pathlib import Path
from motion_remote_preflight import check,candidate,SIZE
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def review_physical_conditions(report,r):
 check(r.get('operator_physically_present_with_emergency_stop') is True,'physical operator supervision/emergency stop not verified')
 if report.get('diagnostic_evidence',{}).get('origin')=='PREVIOUS_PRE_ROM_CAPTURE_NOT_FRESH':
  check(r.get('hardware_continuity_since_diagnostics_verified') is True,'prior readbacks cannot be fresh evidence after unverified power/configuration changes')
def review(report_path,review_path):
 report=json.loads(report_path.read_text());r=json.loads(review_path.read_text())
 check(report.get('mode')=='EXECUTED_READONLY' and 'error' not in report,'capture incomplete')
 check(r.get('preflight_report_sha256')==sha(report_path),'review must name the exact capture report')
 check(isinstance(r.get('reviewer'),str) and len(r['reviewer'].strip())>=3,'named reviewer required')
 stamp=datetime.datetime.fromisoformat(r['reviewed_at_utc'].replace('Z','+00:00'))
 check(stamp.tzinfo is not None,'review timestamp must have a time zone')
 check(0<=(datetime.datetime.now(datetime.timezone.utc)-stamp).total_seconds()<3600,'fresh review required')
 root=Path(report['session_directory']);check(sha(root/'usb_readonly.log')==report['usb_diagnostics_sha256'],'diagnostic snapshots changed')
 check(sha(root/'fullflash_A.bin')==sha(root/'fullflash_B.bin')==report['backup_sha256'],'backup copies changed')
 check((root/'fullflash_A.bin').stat().st_size==SIZE,'backup size changed')
 recovered=Path(r['recovered_backup_path']);check(recovered.stat().st_size==SIZE and sha(recovered)==report['backup_sha256'],'independent recovered/decrypted image mismatch')
 check(recovered.resolve() not in ((root/'fullflash_A.bin').resolve(),(root/'fullflash_B.bin').resolve()),'local A/B cannot prove independent recovery')
 for key in ('independent_physical_storage_verified','recovery_download_or_medium_reread_verified','asus_link_independent_of_robot_radio','current_mcu_identity_matches','current_gen1_identity_mapping_matches','13_torque_off_readbacks_reviewed','stable_flash_power_and_physical_servo_cut_reviewed','current_bno085_snapshot_reviewed','current_daly_fault_voltage_thermal_snapshot_reviewed','layout_nvs_and_installed_slot_reviewed','candidate_public_manifest_and_toolchain_reviewed'):
  check(r.get(key) is True,'required review missing: '+key)
 review_physical_conditions(report,r)
 gates=report['gates']
 for key in ('servo_persistent_profile','13_servo_readbacks_off','layout_preserves_nvs','gen1_semantic_validation','active_application_identity','candidate_file_identity','installed_source_identity'):
  check(gates.get(key) is True,'capture gate incomplete: '+key)
 designation=Path(r['designation_path']);d,m=candidate(designation);check(d.get('release_ready')=='PASS','candidate review not PASS')
 check(d['application_sha256']==report['candidate']['application_sha256'],'review of different firmware')
 from verify_designated_candidate import verify
 verify(designation)
 return dict(result='PRE_FLASH_TECHNICAL_GO_AWAIT_EXPLICIT_OWNER_AUTHORIZATION',application_sha256=d['application_sha256'],compiled_source_commit=d['source_commit'],preflight_report_sha256=sha(report_path),review_sha256=sha(review_path),flash_authorized=False,flash_executed=False,motion_executed=False)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--report',type=Path,required=True);p.add_argument('--review',type=Path,required=True);a=p.parse_args();print(json.dumps(review(a.report,a.review),indent=2))
if __name__=='__main__':main()
