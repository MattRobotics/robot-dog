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


def mutate(filename, old, new, struct=None):
    matches = 0
    result = []
    for path, code in BASE:
        if path.name == filename:
            matches += 1
            if struct is None:
                code = checked_replace(code, old, new)
            else:
                body = re.search(rf"struct\s+{struct}\s*\{{(.*?)\n\}};", code, re.DOTALL)
                if not body:
                    raise AssertionError(f"mutation struct not found: {struct}")
                replacement = checked_replace(body.group(1), old, new)
                code = code[:body.start(1)] + replacement + code[body.end(1):]
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
    ("full input defaults true", "LedStatusPolicy.h", "bool charge_complete_verified = false;",
     "bool charge_complete_verified = true;", "LedStatusInputs"),
    ("full input loses false default", "LedStatusPolicy.h", "struct LedStatusInputs {",
     "struct LedStatusInputs {\n  LedStatusInputs() : charge_complete_verified(true) {}"),
    ("reserved input aggregate initializer", "Controller.cpp", "status::LedStatusInputs led_inputs;",
     "status::LedStatusInputs led_inputs = {core::SystemHealth::READY, false, false, false,\n"
     "      false, false, 0, 0, false, false, false, true, true};"),
    ("reserved input aggregate replacement", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "led_inputs = {core::SystemHealth::READY, false, false, false, false, false,\n"
     "      0, 0, false, false, false, true, true}; led_status_.update(led_now_ms, led_inputs);"),
    ("reserved input whole-object alias", "Controller.cpp", "led_status_.update(led_now_ms, led_inputs);",
     "auto& reserved = led_inputs; reserved = {}; led_status_.update(led_now_ms, led_inputs);"),
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

# Both reserved facts must be protected in every production layer, not only
# at the Controller wiring point. Include aliases and constructor overrides
# to show that the audit is stronger than a plain assignment search.
for fact in ("battery_warning", "battery_critical"):
    STATIC_MUTATIONS.extend([
        (f"{fact} Controller SOC producer", "Controller.cpp",
         "led_status_.update(led_now_ms, led_inputs);",
         f"led_inputs.{fact} = battery.soc_percent < 20;\n"
         "    led_status_.update(led_now_ms, led_inputs);"),
        (f"{fact} Controller alias producer", "Controller.cpp",
         "led_status_.update(led_now_ms, led_inputs);",
         f"auto& reserved = led_inputs.{fact}; reserved = true;\n"
         "    led_status_.update(led_now_ms, led_inputs);"),
        (f"{fact} manager producer", "LedStatusManager.cpp", "ring_->setFrame(frame);",
         f"ring_->setFrame(frame); inputs.{fact} = true;"),
        (f"{fact} policy SOC threshold", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = in.soc_percent < 20;"),
        (f"{fact} policy compound producer", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = in.{fact}; facts.{fact} |= true;"),
        (f"{fact} loses passthrough", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", ""),
        (f"{fact} policy freshness gate", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = fresh && in.{fact};"),
        (f"{fact} input constructor overrides default", "LedStatusPolicy.h",
         "struct LedStatusInputs {", f"struct LedStatusInputs {{\n  LedStatusInputs() : {fact}(true) {{}}"),
        (f"{fact} status observation removed", "CommandRouter.cpp",
         f'led.{fact} ? "YES" : "NO"', 'false ? "YES" : "NO"'),
    ])
    for struct in ("LedStatusInputs", "LedStatusSnapshot"):
        STATIC_MUTATIONS.extend([
            (f"{fact} {struct} defaults true", "LedStatusPolicy.h",
             f"bool {fact} = false;", f"bool {fact} = true;", struct),
            (f"{fact} {struct} loses default", "LedStatusPolicy.h",
             f"bool {fact} = false;", f"bool {fact};", struct),
        ])

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
    ("critical loses priority over degraded", "LedStatusPolicy.cpp",
     "if (facts.battery_critical) return LedPresentationState::BATTERY_CRITICAL;\n"
     "  if (in.system_health == core::SystemHealth::DEGRADED) return LedPresentationState::DEGRADED;",
     "if (in.system_health == core::SystemHealth::DEGRADED) return LedPresentationState::DEGRADED;\n"
     "  if (facts.battery_critical) return LedPresentationState::BATTERY_CRITICAL;"),
    ("critical overrides charging fault", "LedStatusPolicy.cpp",
     "if (facts.charging_fault) return LedPresentationState::CHARGING_FAULT;\n"
     "  if (facts.battery_critical) return LedPresentationState::BATTERY_CRITICAL;",
     "if (facts.battery_critical) return LedPresentationState::BATTERY_CRITICAL;\n"
     "  if (facts.charging_fault) return LedPresentationState::CHARGING_FAULT;"),
    ("warning overrides degraded", "LedStatusPolicy.cpp",
     "if (in.system_health == core::SystemHealth::DEGRADED) return LedPresentationState::DEGRADED;\n"
     "  if (facts.battery_warning) return LedPresentationState::BATTERY_WARNING;",
     "if (facts.battery_warning) return LedPresentationState::BATTERY_WARNING;\n"
     "  if (in.system_health == core::SystemHealth::DEGRADED) return LedPresentationState::DEGRADED;"),
    ("warning loses priority over Wi-Fi", "LedStatusPolicy.cpp",
     "if (facts.battery_warning) return LedPresentationState::BATTERY_WARNING;\n"
     "  if (in.wifi_connecting) return LedPresentationState::WIFI_CONNECTING;",
     "if (in.wifi_connecting) return LedPresentationState::WIFI_CONNECTING;\n"
     "  if (facts.battery_warning) return LedPresentationState::BATTERY_WARNING;"),
    ("warning uses wrong RGB", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_WARNING:\n      return {255, 140, 0,",
     "case LedPresentationState::BATTERY_WARNING:\n      return {255, 0, 0,"),
    ("warning becomes fixed amber", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, subtleBrightness(now_ms, max_brightness)};",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, static_cast<uint8_t>(max_brightness / 3)};"),
    ("warning uses 2 second period", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, subtleBrightness(now_ms, max_brightness)};",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, triangleBrightness(now_ms, kBreathePeriodMs,\n"
     "                                             kBreatheMinBrightness, max_brightness / 3)};"),
    ("warning brightness rises to 30", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, subtleBrightness(now_ms, max_brightness)};",
     "case LedPresentationState::BATTERY_WARNING:\n"
     "      return {255, 140, 0, triangleBrightness(now_ms, kSubtleBreathePeriodMs,\n"
     "                                             kBreatheMinBrightness, max_brightness / 2)};"),
    ("critical uses wrong RGB", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_CRITICAL:\n      return {255, 0, 0,",
     "case LedPresentationState::BATTERY_CRITICAL:\n      return {255, 140, 0,"),
    ("critical uses 2 second period", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_CRITICAL:\n"
     "      return {255, 0, 0, triangleBrightness(now_ms, kSubtleBreathePeriodMs,",
     "case LedPresentationState::BATTERY_CRITICAL:\n"
     "      return {255, 0, 0, triangleBrightness(now_ms, kBreathePeriodMs,"),
    ("critical brightness rises to 60", "LedStatusPolicy.cpp",
     "case LedPresentationState::BATTERY_CRITICAL:\n"
     "      return {255, 0, 0, triangleBrightness(now_ms, kSubtleBreathePeriodMs, kBreatheMinBrightness,\n"
     "                                           max_brightness / 2)};",
     "case LedPresentationState::BATTERY_CRITICAL:\n"
     "      return {255, 0, 0, triangleBrightness(now_ms, kSubtleBreathePeriodMs, kBreatheMinBrightness,\n"
     "                                           max_brightness)};"),
]

for fact, state in (("battery_warning", "BATTERY_WARNING"),
                    ("battery_critical", "BATTERY_CRITICAL")):
    POLICY_MUTATIONS.extend([
        (f"{fact} snapshot fact lost", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = false;"),
        (f"{fact} wrongly gated by DALY freshness", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = fresh && in.{fact};"),
        (f"{fact} fabricates threshold", "LedStatusPolicy.cpp",
         f"facts.{fact} = in.{fact};", f"facts.{fact} = in.soc_percent < 20;"),
        (f"{fact} loses state label", "LedStatusPolicy.cpp",
         f'case LedPresentationState::{state}: return "{state}";',
         f'case LedPresentationState::{state}: return "UNKNOWN";'),
    ])


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
    for label, *mutation in STATIC_MUTATIONS:
        try:
            if findings(mutate(*mutation)):
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
