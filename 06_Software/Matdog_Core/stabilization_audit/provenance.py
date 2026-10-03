"""G5-A geometry and calibration provenance (read-only).

Nothing is written outside the G5-A evidence directory. Calibration material is read with `git show <ref>:<path>` from the
shared repository (no checkout, no access to the calibration worktree). Calibration-persistence work is an INDEPENDENT dependency
and is recorded, not assumed.
"""
import hashlib
import json
import re
import subprocess
import sys
from core import ROOT, OUT

REPORT = '09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md'
PERSISTENCE_BRANCH = 'feat/calibration-persistence-record-store-v1'


def git(*args, check=True):
    return subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True, check=check).stdout


def blob(ref, path):
    p = subprocess.run(['git', '-C', str(ROOT), 'show', f'{ref}:{path}'], capture_output=True)
    return p.stdout if p.returncode == 0 else None


def sha(data):
    return hashlib.sha256(data).hexdigest() if data is not None else None


def parse_contacts(text):
    rows = []
    for line in text.splitlines():
        c = [x.strip() for x in line.strip().strip('|').split('|')]
        if len(c) >= 13 and c[0] in ('LF', 'RF', 'RH', 'LH') and c[1] in ('HIP', 'UPPER', 'LOWER') and c[2] in ('MIN', 'MAX') and '/' in c[3]:
            unit, bus = c[3].split('/')
            rows.append(dict(leg=c[0].lower(), joint=c[1].lower(), side=c[2], unit=unit, bus_id=int(bus), direction=int(c[4]), q0_ticks=int(c[5]), contact_ticks=int(c[9]),
                             measured_contact_deg=float(c[11]), urdf_limit_deg=float(c[12])))
    return rows


def main():
    result = {}
    # ---- geometry --------------------------------------------------------------------------------------------------
    urdf = 'matt_robodog_rev00.urdf'
    wd = (ROOT / '03_CAD/URDF/matt_robodog_rev00' / urdf).read_bytes()
    main_urdf = blob('main', '03_CAD/URDF/matt_robodog_rev00/' + urdf)
    check = subprocess.run([sys.executable, str(ROOT / '06_Software/Matdog_Core/contact_audit/geometry_provenance.py'), '--check'], capture_output=True, text=True)
    cal_text = (blob('main', REPORT) or b'').decode()
    sums = {}
    for line in (ROOT / '03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt').read_text().splitlines():
        if line.strip():
            d, n = line.split(None, 1); sums[n.strip().lstrip('*')] = d
    report_urdf = re.search(r'URDF SHA256 `([0-9a-f]{4,8})…([0-9a-f]{4})`', cal_text)
    result['geometry'] = dict(
        urdf_sha256=sha(wd), urdf_matches_main=sha(wd) == sha(main_urdf), urdf_matches_sha256sums=sha(wd) == sums[urdf],
        full_calibration_report_urdf_prefix_suffix=None if not report_urdf else [report_urdf.group(1), report_urdf.group(2)],
        full_calibration_report_urdf_consistent=bool(report_urdf and sha(wd).startswith(report_urdf.group(1)) and sha(wd).endswith(report_urdf.group(2))),
        geometry_v5_bundle_in_report='2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4' in cal_text,
        g41_geometry_provenance_check_returncode=check.returncode,
        foot_contact_yaml_matches_main=sha((ROOT / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml').read_bytes()) == sha(blob('main', '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml')),
        generated_motion_geometry_header_sha_tag=re.search(r'kUrdfSha256\[\]\s*=\s*"([0-9a-f]+)"', (ROOT / '05_Firmware/MATDOG_Controller/src/motion/LegGeometryData.h').read_text()).group(1) if re.search(r'kUrdfSha256\[\]\s*=\s*"([0-9a-f]+)"', (ROOT / '05_Firmware/MATDOG_Controller/src/motion/LegGeometryData.h').read_text()) else None)
    # ---- calibration (hardware-validated, RAM-only) ------------------------------------------------------------------
    contacts = parse_contacts(cal_text)
    q0 = re.search(r'Fresh q0[^|]*\|\s*([^|]+)\|', cal_text)
    result['full_calibration'] = dict(
        source_ref='main', report_path=REPORT, report_sha256=sha(cal_text.encode()), contacts_parsed=len(contacts), contacts=contacts,
        statements=[l.strip() for l in cal_text.splitlines() if any(k in l for k in ('RAM-only', 'legs_envelope_accepted', 'TRUE FULL CALIBRATION', 'No EEPROM'))][:6],
        q0_line=None if not q0 else q0.group(1).strip()[:400],
        read_only_interface=['MATDOG_SERVO_ALLOCATION.yaml (installation record)', 'q0 per joint, contacts, encoder direction (calibration evidence export)'])
    # ---- independent persistence dependency ------------------------------------------------------------------------------
    readme_main = (blob('main', 'README.md') or b'').decode()
    branch_log = git('log', '--oneline', '-6', PERSISTENCE_BRANCH, check=False).strip().splitlines()
    merged = git('merge-base', '--is-ancestor', PERSISTENCE_BRANCH, 'main', check=False)
    ancestor = subprocess.run(['git', '-C', str(ROOT), 'merge-base', '--is-ancestor', PERSISTENCE_BRANCH, 'main']).returncode == 0
    result['calibration_persistence'] = dict(
        branch=PERSISTENCE_BRANCH, branch_tip=git('rev-parse', PERSISTENCE_BRANCH, check=False).strip(), recent_commits=branch_log, merged_into_main=ancestor,
        main_readme_statements=[l.strip() for l in readme_main.splitlines() if any(k in l for k in ('Calibration is RAM-only', 'Persistence V1', 'stand/gait are not authorized', 'Stand and gait motion on hardware'))][:6],
        conclusion='INDEPENDENT DEPENDENCY, NOT COMPLETE: persistence is not merged into main and its documented status is TO_DESIGN/unrestored on main. Having performed calibration does not provide boot-time restoration. Not merged, cherry-picked or modified by G5-A.')
    result['hardware_authorization'] = dict(stand_or_gait_authorized=False, evidence='Full Calibration report: legs_envelope_accepted=0; README: operational envelopes not approved, stand/gait not authorized')
    result['verdict'] = dict(
        geometry_consistent=result['geometry']['urdf_matches_main'] and result['geometry']['urdf_matches_sha256sums'] and result['geometry']['full_calibration_report_urdf_consistent'] and check.returncode == 0,
        calibration_contacts_available=len(contacts) == 24)
    (OUT / 'provenance.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['verdict']), len(contacts), 'contacts;', 'persistence merged:', ancestor)
    return 0


if __name__ == '__main__':
    sys.exit(main())
