#!/usr/bin/env python3
"""P3a.2 negative controls: restore each USB framing defect in a temporary TU.

Uses the host runner's actual router compile recipe and the same real-router
regressions. No parser copy, repository source edit or hardware access.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


TESTS = Path(__file__).resolve().parent
SKETCH = TESTS.parent.parent
ROUTER = SKETCH / "src/core/CommandRouter.cpp"


def compile_recipe():
    runner = (TESTS / "run_host_tests.sh").read_text()
    record_array = re.search(r"CALREC_SRCS=\(\n(.*?)\n\)", runner, re.S)
    if record_array is None:
        raise RuntimeError("CALREC_SRCS recipe not found")
    records = [arg.replace("$SKETCH_DIR", str(SKETCH))
               for arg in shlex.split(record_array.group(1))]
    block = next(part for part in runner.split("\n\n")
                 if '"$SCRIPT_DIR/test_command_router_persistence.cpp"' in part)
    block = block[block.index('"$CXX"'):].replace("\\\n", " ")
    args = []
    for arg in shlex.split(block):
        if arg == "${CALREC_SRCS[@]}":
            args.extend(records)
        else:
            args.append(arg.replace("$CXX", os.environ.get("CXX", "g++"))
                        .replace("$SCRIPT_DIR", str(TESTS))
                        .replace("$SKETCH_DIR", str(SKETCH)))
    return args


def main():
    original = ROUTER.read_text()
    overflow = "      line_error_ = LineError::OVERFLOW;\n"
    nul = ("    if (c == '\\0') {\n"
           "      line_error_ = LineError::NUL;\n"
           "      continue;\n"
           "    }\n")
    if original.count(overflow) != 1 or original.count(nul) != 1:
        raise RuntimeError("framing mutation anchors changed")
    variants = {
        "silent_overflow_truncation": original.replace(overflow, "", 1),
        "accept_internal_nul": original.replace(nul, "", 1),
    }
    recipe = compile_recipe()
    with tempfile.TemporaryDirectory(prefix="matdog-usb-framing-mutations-") as directory:
        for name, source in variants.items():
            path = Path(directory) / (name + ".cpp")
            # The copied TU keeps exactly the production includes, resolved from
            # their original directory rather than from the temporary directory.
            source = re.sub(r'^#include "([^"]+)"',
                            lambda match: '#include "' + str((ROUTER.parent / match[1]).resolve()) + '"',
                            source, flags=re.M)
            path.write_text(source)
            executable = Path(directory) / name
            args = recipe.copy()
            args[args.index(str(ROUTER))] = str(path)
            args[args.index("-o") + 1] = str(executable)
            compiled = subprocess.run(args, text=True, capture_output=True)
            if compiled.returncode:
                raise RuntimeError(name + " did not compile:\n" + compiled.stdout + compiled.stderr)
            result = subprocess.run([str(executable)], text=True, capture_output=True)
            report = re.search(r"test_command_router_persistence: (\d+) checks, (\d+) failures", result.stdout)
            # A compilation error or only a diagnostic mismatch is insufficient:
            # the regressions must observe writes through both storage paths.
            if (result.returncode != 1 or report is None or int(report[2]) == 0 or
                    "f.writes() == writes" not in result.stdout or
                    "actual.set_calls == nvs.set_calls" not in result.stdout):
                raise RuntimeError(name + " escaped the framing/write regressions:\n" + result.stdout + result.stderr)
            failures = [line for line in result.stdout.splitlines() if line.startswith("FAIL ")]
            if any(" []:" in line for line in failures):
                raise RuntimeError(name + " also failed a pre-existing P3a.1 regression")
            print(f"USB_FRAMING_MUTATION={name} DETECTED checks={report[1]} failures={report[2]}")
            for line in failures[:3]:
                print("  " + line)
    print("USB_FRAMING_MUTATIONS = PASS (both historical defects detected)")


if __name__ == "__main__":
    main()
