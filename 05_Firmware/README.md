# MATDOG Firmware

This directory is reserved for low-level firmware that belongs specifically to MATDOG.

## Current Status

No dedicated MATDOG firmware is active in the current development phase.

The current locomotion-development stack is:

    Asus Ubuntu
    → NormaCore Station
    → Waveshare Bus Servo Adapter
    → ST3215 serial-bus servos

The official NormaCore ST3215 driver remains the only owner of the servo serial bus.

## Future Scope

This directory may later contain:

- ESP32 motion-controller firmware
- IMU acquisition and filtering
- battery monitoring
- watchdog logic
- safety interlocks
- low-level ST3215 actuator adapter
- communication protocol between Jetson or host and motion controller

## Architecture Rule

Do not add experimental host-side gait, IK or calibration code here.

Those belong in:

    06_Software/

Firmware will be introduced only after MATDOG has achieved validated stand, inverse kinematics and initial walking with the current Station and Waveshare architecture.
