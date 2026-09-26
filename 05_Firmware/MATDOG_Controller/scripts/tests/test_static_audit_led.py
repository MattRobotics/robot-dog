#!/usr/bin/env python3
"""LED V2 mutation checks against real production files and linked host tests.

Static mutations exist only in memory. Executable policy mutations compile
in a temporary directory against the real host suite, then disappear. No
hardware, device access, credentials, or production files are modified.
"""
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
SKETCH_DIR = SCRIPTS_DIR.parent
spec = importlib.util.spec_from_file_location("led_static_audit", SCRIPTS_DIR / "static_audit.py")
audit = importlib.util.module_from_spec(spec)
saved_args = sys.argv
sys.argv = [str(SCRIPTS_DIR / "static_audit.py"), str(SKETCH_DIR)]
try:
    spec.loader.exec_module(audit)
finally:
    sys.argv = saved_args
BASE = [(p, audit.strip_comments(p.read_text(encoding="utf-8"))) for p in audit.iter_source_files()]


def checked_replace(code, old, new):
    count = code.count(old)
    if count != 1:
        raise AssertionError(f"mutation anchor must occur once; got {count}: {old!r}")
    return code.replace(old, new, 1)


def mutate(filename, old, new):
    matches = 0
    result = []
    for path, code in BASE:
        if path.name == filename:
            matches += 1
            code = checked_replace(code, old, new)
        result.append((path, code))
    if matches != 1:
        raise AssertionError(f"expected one production {filename}, got {matches}")
    return result


def findings(files):
    audit.failures.clear()
    audit.check_led_anti_back_power(files)
    audit.check_led_status_boundaries(files)
    return list(audit.failures)


STATIC_MUTATIONS = [
    ("policy Arduino dependency", "LedStatusPolicy.cpp", '#include "LedStatusPolicy.h"',
     '#include "LedStatusPolicy.h"\n#include <Arduino.h>'),
    ("policy hardware clock", "LedStatusPolicy.cpp", "snapshot_ = factsFor(inputs);",
     "snapshot_ = factsFor(inputs); millis();"),
    ("policy GPIO read/write", "LedStatusPolicy.cpp", "snapshot_ = factsFor(inputs);",
     "snapshot_ = factsFor(inputs); digitalWrite(47, 1);"),
    ("status DALY transaction", "LedStatusManager.cpp", "ring_->setFrame(frame);",
     "ring_->setFrame(frame); bms_uart_.write(0);"),
    ("status actuator side effect", "LedStatusManager.cpp", "ring_->setFrame(frame);",
     "ring_->setFrame(frame); ServoBus bus;"),
    ("blocking presentation", "LedStatusManager.cpp", "ring_->setFrame(frame);",
     "ring_->setFrame(frame); delay(600);"),
    ("second frame owner", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "led_status_.update(led_now_ms, led_inputs); led_.setFrame(frame);"),
    ("second solid owner", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "led_status_.update(led_now_ms, led_inputs); led_.setSolid(0, 255, 0, 20);"),
    ("second WS2812 driver", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "led_status_.update(led_now_ms, led_inputs); Adafruit_NeoPixel rogue;"),
    ("unreviewed full producer", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "led_inputs.charge_complete_verified = battery.soc_percent == 100;\n"
     "    led_status_.update(led_now_ms, led_inputs);"),
    ("full input defaults true", "LedStatusPolicy.h", "bool charge_complete_verified = false;\n};\n\n\nstruct LedEffect",
     "bool charge_complete_verified = true;\n};\n\n\nstruct LedEffect"),
    ("full input loses false default", "LedStatusPolicy.h", "struct LedStatusInputs {",
     "struct LedStatusInputs {\n  LedStatusInputs() : charge_complete_verified(true) {}"),
    ("diagnostic ownership bypass", "LedStatusManager.cpp", "ring_ == nullptr || ring_->testRunning()",
     "ring_ == nullptr"),
    ("second manager render", "LedStatusManager.cpp", "ring_->setFrame(frame);",
     "ring_->setFrame(frame); ring_->setFrame(frame);"),
    ("duplicate DALY scheduler call", "Controller.cpp", "daly_.update(now_ms, operating_mode_.mode());",
     "daly_.update(now_ms, operating_mode_.mode()); daly_.update(now_ms, operating_mode_.mode());"),
    ("new DALY query from controller", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "daly_.requestKeyConfigRead(); led_status_.update(led_now_ms, led_inputs);"),
    ("SOC accepts invalid sample", "Controller.cpp", "led_inputs.sample_valid = battery.valid;",
     "led_inputs.sample_valid = true;"),
    ("SOC ignores last comm failure", "Controller.cpp",
     "led_inputs.daly_comm_ok = daly_.lastCommResult() == power::DalyCommResult::OK;",
     "led_inputs.daly_comm_ok = true;"),
    ("SOC fabricates freshness", "Controller.cpp",
     "led_inputs.telemetry_age_ms = led_now_ms - battery.sampled_at_ms;",
     "led_inputs.telemetry_age_ms = 0;"),
    ("freshness uses earlier controller tick", "Controller.cpp", "const uint32_t led_now_ms = millis();",
     "const uint32_t led_now_ms = now_ms;"),
    ("charging alarm word lost", "Controller.cpp", "battery.alarms[3] != 0;", "false;"),
    ("freshness contract silently extended", "DalyProtocol.h", "kDalyTelemetryFreshnessMs = 5000;",
     "kDalyTelemetryFreshnessMs = 6000;"),
    ("KEY age bound diverges", "DalyProtocol.h",
     "kDalyKeyWriteMaxTelemetryAgeMs = kDalyTelemetryFreshnessMs;",
     "kDalyKeyWriteMaxTelemetryAgeMs = 6000;"),
    ("USB_ONLY GPIO becomes output", "LedRing.cpp", "pinMode(pins::kLedRingDin, INPUT);",
     "pinMode(pins::kLedRingDin, OUTPUT);"),
    ("USB_ONLY begins transport early", "LedRing.cpp", "bool LedRing::begin() {",
     "bool LedRing::begin() { pixels_.begin();"),
    ("USB_ONLY guard loses return", "LedRing.cpp", "init_ = core::InitializationState::DEFERRED;\n    return true;",
     "init_ = core::InitializationState::DEFERRED;"),
    ("USB_ONLY frame guard disabled", "LedRing.cpp",
     "void LedRing::renderFrame(const LedFrame& frame) {\n  if (!build::kLedRailPowered) return;",
     "void LedRing::renderFrame(const LedFrame& frame) {\n  if (false) return;"),
    ("USB_ONLY SOC diagnostic enabled", "LedRing.cpp",
     "bool LedRing::startSocTest() {\n  if (!build::kLedRailPowered) return false;",
     "bool LedRing::startSocTest() {\n  if (false) return false;"),
]

# Compile-valid semantic changes prove the suite checks behavior, rather
# than merely grepping matching production expressions.
POLICY_MUTATIONS = [
    ("upward rounding", "LedStatusPolicy.cpp",
     "static_cast<double>(reported_soc) * kSocPixelCount / 100.0",
     "std::ceil(static_cast<double>(reported_soc) * kSocPixelCount / 100.0)"),
    ("float intermediate rounds boundary upward", "LedStatusPolicy.cpp",
     "static_cast<double>(reported_soc) * kSocPixelCount / 100.0", "reported_soc * kSocPixelCount / 100.0f"),
    ("SOC physical start drifts", "LedStatusPolicy.h",
     "{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0}", "{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}"),
    ("100 percent fabricates full", "LedStatusPolicy.cpp",
     "fresh && !in.battery_alarm && in.charge_complete_verified;",
     "fresh && !in.battery_alarm && (in.charge_complete_verified || in.soc_percent == 100);"),
    ("exclusive freshness boundary", "LedStatusPolicy.cpp",
     "in.telemetry_age_ms <= power::kDalyTelemetryFreshnessMs", "in.telemetry_age_ms < power::kDalyTelemetryFreshnessMs"),
    ("comm failure ignored", "LedStatusPolicy.cpp", "in.sample_valid && in.daly_comm_ok &&", "in.sample_valid &&"),
    ("boot fixed white returns", "LedStatusPolicy.cpp",
     "{255, 255, 255, subtleBrightness(now_ms, max_brightness)}",
     "{255, 255, 255, static_cast<uint8_t>(max_brightness / 3)}"),
    ("charging pulse moves to wrong pixel", "LedStatusPolicy.cpp",
     "frame.pixels[kSocPixelOrder[next]]", "frame.pixels[next]"),
    ("charging full tail stops pulsing", "LedStatusPolicy.cpp",
     "if (state() == LedPresentationState::CHARGING) {",
     "if (state() == LedPresentationState::CHARGING && snapshot_.soc_segments < kSocPixelCount) {"),
    ("charging fault suppressed", "LedStatusPolicy.cpp",
     "facts.charging_fault = facts.charging && in.battery_alarm;", "facts.charging_fault = false;"),
]


def run_policy(directory, filename=None, old=None, new=None):
    status_dir = SKETCH_DIR / "src" / "status"
    for name in ("LedStatusPolicy.cpp", "LedStatusPolicy.h"):
        text = (status_dir / name).read_text(encoding="utf-8")
        if name == filename:
            text = checked_replace(text, old, new)
        (directory / name).write_text(text, encoding="utf-8")
    binary = directory / "test_led_policy"
    command = [os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O1",
               "-DDISABLED=0x00", "-I", str(status_dir), "-o", str(binary),
               str(SCRIPTS_DIR / "tests" / "test_led_status_policy.cpp"), str(directory / "LedStatusPolicy.cpp")]
    built = subprocess.run(command, capture_output=True, text=True)
    if built.returncode != 0:
        raise AssertionError(f"policy mutant must compile: {built.stderr}")
    return subprocess.run([str(binary)], capture_output=True, text=True)


def main():
    passed = 0
    failed = []
    baseline = findings(BASE)
    if baseline:
        failed.append(("unmutated static baseline", baseline))
    else:
        passed += 1
    for label, filename, old, new in STATIC_MUTATIONS:
        try:
            if findings(mutate(filename, old, new)):
                passed += 1
            else:
                failed.append((label, "static mutation not detected"))
        except AssertionError as error:
            failed.append((label, str(error)))
    with tempfile.TemporaryDirectory(prefix="matdog-led-mutations-") as tmp:
        directory = Path(tmp)
        baseline = run_policy(directory)
        if baseline.returncode == 0:
            passed += 1
        else:
            failed.append(("unmutated real policy", baseline.stdout))
        for label, filename, old, new in POLICY_MUTATIONS:
            try:
                result = run_policy(directory, filename, old, new)
                if result.returncode != 0 and re.search(r"[1-9][0-9]* failures", result.stdout):
                    passed += 1
                else:
                    failed.append((label, "linked host tests did not reject semantic mutation"))
            except AssertionError as error:
                failed.append((label, str(error)))
    audit.failures.clear()
    total = 2 + len(STATIC_MUTATIONS) + len(POLICY_MUTATIONS)
    print(f"LED mutations: static={len(STATIC_MUTATIONS)} policy={len(POLICY_MUTATIONS)} baselines=2")
    print(f"cases_run={total} passed={passed} failed={len(failed)}")
    for label, detail in failed:
        print(f"  FAIL {label}: {detail}")
    print("LED_AUDIT_MUTATION_TESTS = " + ("PASS" if not failed else "FAIL"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
