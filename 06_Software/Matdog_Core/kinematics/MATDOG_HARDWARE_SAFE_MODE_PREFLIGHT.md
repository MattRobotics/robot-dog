# MATDOG — Hardware Safe-Mode Preflight

## Purpose

This document records the C4-F hardware safe-mode preflight before the first physical stand.

C4-F does not command the robot. It only prepares the safety boundary for C5.

## Source audit result

The archived C4-F source audit validates:

- Python files scanned: 24;
- live read-only tools found: 10;
- existing command-capable tools found: 4;
- direct serial risks: 0;
- stand command candidates: 0;
- first-stand blacklisted tools: 4.

## First-stand blacklist

The following existing tools are command-capable and must not be used for the first stand sequence:

- `06_Software/Matdog_Core/calibration/matdog_leg_hold_probe.py`;
- `06_Software/Matdog_Core/calibration/matdog_micro_probe.py`;
- `06_Software/Matdog_Core/calibration/matdog_micro_probe_diagnostic.py`;
- `06_Software/Matdog_Core/calibration/matdog_visual_zero_pose_probe.py`.

They may remain useful for controlled calibration/probe work, but they are explicitly forbidden for C5 first-stand execution.

## Allowed before C5 motion

Before any motion in C5, only these categories are allowed:

- offline report inspection;
- trajectory export inspection;
- Station telemetry read-only;
- encoder/FK read-only validation;
- hardware checklist confirmation.

## Still forbidden at the end of C4-F

At the end of C4-F, the following remain forbidden:

- direct serial access;
- pyserial access;
- Station Calibrate;
- torque enable;
- servo target command;
- speed/accel command;
- stand command;
- gait command;
- use of any first-stand blacklisted tool.

Station remains the only serial bus owner.

## Hardware preflight checklist for C5

Before the first physical stand in C5:

1. MATDOG on flat, non-slippery surface.
2. All four TPU feet touching the ground naturally.
3. No cable under or near moving legs.
4. No hand inside leg workspace.
5. Operator ready to cut servo power immediately.
6. Camera/phone recording the test from the side.
7. Station telemetry visible.
8. Encoder/FK live read-only check completed.
9. Initial real pose judged compatible with the planned stand trajectory.
10. Explicit operator approval written in terminal/chat before motion.

## Abort policy

The primary abort for the first physical stand is hardware power cut.

C5 must not start until the operator has a clear and reachable way to remove servo power immediately.

Software torque-off can be used only as a secondary abort path if it is verified before the stand test. It must not replace the physical power-cut abort.

## Command eligibility

C4-F is still not command-eligible.

C5 must start in a new chat and must begin with read-only live validation before any physical command is considered.
