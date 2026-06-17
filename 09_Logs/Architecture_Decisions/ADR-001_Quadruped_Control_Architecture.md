# ADR-001 - Quadruped Control Architecture

Date: 2026-06-17

## Status

Accepted

## Context

A decision was required regarding the introduction of an ESP32 low-level controller before gait implementation.

The robot currently consists of:

- 12 ST3215 servos
- Waveshare Bus Servo Adapter
- NormaCore Station
- Asus Ubuntu host

## Decision

Continue development using the current architecture.

Current architecture:

Station
↓
Waveshare Bus Servo Adapter
↓
ST3215 Servos

Development priorities:

1. Stand pose
2. Leg IK
3. Gait generator
4. Walking tests

Do NOT introduce ESP32 at this stage.

## Reasoning

The current architecture:

- works reliably
- provides immediate servo response
- has no verified bus limitations
- is sufficient for IK and gait development

Introducing an ESP32 now would increase complexity without validating locomotion first.

## Future Architecture

Possible future architecture inspired by Yahboom DOGZILLA:

Jetson / Host
↓
ESP32
↓
ST3215

Host responsibilities:

- AI
- Vision
- Planning
- Voice

ESP32 responsibilities:

- Gait generation
- IK
- IMU
- Battery monitoring
- Watchdog
- Real-time servo control

## Consequences

Software must remain modular:

Gait Generator
↓
IK
↓
Servo Target Writer

This allows migration to ESP32 later without rewriting gait and IK.

## Principle

First make the robot walk.

Then optimize the architecture.
