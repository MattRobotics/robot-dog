#!/usr/bin/env bash
# Controller::SimpleServoPort conversions against the per-call lookup they replace
# (test_servo_port_conversion.cpp), on the gen-1 golden tables. Offline only.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="$(mktemp -d /tmp/matdog-port-conversion.XXXXXX)";trap 'rm -rf "$out"' EXIT
# The production slot binding and conversions, verbatim.
python3 -I - "$sketch/src/core/SimpleMotionIntegration.cpp" "$sketch/src/core/Controller.h" "$out" <<'PY'
import re,sys;src,hdr,out=sys.argv[1:];t=open(src).read();h=open(hdr).read()
s="const Controller::SimpleServoPort::Slot* Controller::SimpleServoPort::slot(";e="bool Controller::SimpleServoPort::readDeferred("
r="bool Controller::SimpleServoPort::resolve(";f="bool Controller::SimpleServoPort::frame("
assert t.count(s)==1 and t.count(e)==1 and t.count(r)==1 and t.count(f)==1
open(out+"/production_simple_servo_port_conversion.inc","w").write(s+t.split(s,1)[1].split(e,1)[0]+r+t.split(r,1)[1].split(f,1)[0])
m=re.search(r"\n(\s*struct Slot\{.*?\};\n\s*Slot slots\[12\]\{\};\n\s*const Slot\* slot\(unsigned\);)",h,re.S);assert m
open(out+"/production_simple_servo_port_slot_decl.inc","w").write(m.group(1)+"\n")
PY
files=()
for s in CalibrationRecord CalibrationRecordStore CalibrationSaveMarker CalibrationPersistenceState CalibrationPersistenceService CalibrationRuntimeRestore \
         FullLegCalibrationFinalizer FullLegCalibrationPlan FullLegCalibrationExecutor ContactProbeEngine CalibrationExecutionEngine CalibrationManager \
         CalibrationMotionPermit CalibrationPopulationEvidence CalibrationDomain StartupRecoveryReference RestoreAttestationPolicy; do files+=("$sketch/src/calibration/$s.cpp"); done
for s in OperationalEnvelope MotionDeadman ActuatorRuntime ActuatorWritePolicy CalibrationSequencePlan CalibrationTargetResolver CalibrationGeometryProfile; do files+=("$sketch/src/actuator/$s.cpp"); done
files+=("$sketch/src/servo/ServoPopulation.cpp");for s in SystemState ActuatorAuthority OperatingMode; do files+=("$sketch/src/core/$s.cpp"); done
"${CXX:-g++}" -std=c++17 -O1 -Wall -Wextra -Werror -Wno-misleading-indentation -DDISABLED=0x00 -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$out" -I "$sketch/src/calibration" -I "$sketch/src/actuator" -I "$sketch/src/servo" -I "$sketch/src/core" -I "$here" \
  "$here/test_servo_port_conversion.cpp" "${files[@]}" -o "$out/test"
"$out/test"
