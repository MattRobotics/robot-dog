# MATDOG / NormaCore Upstream Contract

## Objective

MATDOG must absorb useful NormaCore updates without copying or modifying the official ST3215 driver unnecessarily.

## Repository Roles

| Role | Repository |
|---|---|
| Official upstream | `norma-core/norma-core` |
| Integration fork | `MattRobotics/norma-core` |
| MATDOG project | `MattRobotics/robot-dog` |

## Files and Interfaces to Monitor

```text
protobufs/drivers/st3215/st3215.proto
protobufs/station/drivers.proto
software/drivers/st3215/
software/station/shared/station-iface/
software/station/clients/station-viewer/src/st3215/
software/station/clients/station-viewer/src/yahboom_dogzilla_lite/
software/station/clients/station-viewer/src/usbvideo/
```

## Compatibility Boundary

The only MATDOG component allowed to depend on raw ST3215 command structures is:

```text
MatdogSt3215Adapter
```

The following components must remain independent of Station protobuf and serial details:

- URDF and mesh geometry;
- calibration format;
- semantic joint names;
- leg forward and inverse kinematics;
- gait generator;
- dashboard semantic API.

## Controlled Update Procedure

1. Fetch upstream changes in `MattRobotics/norma-core`.
2. Compare the monitored files against the last validated baseline.
3. Build the Station frontend and Rust backend.
4. Run MATDOG compatibility tests.
5. Update only the adapter when the ST3215 transport contract has changed.
6. Record the validated upstream and fork commits below.

## Current Baseline

| Item | Value |
|---|---|
| Status | Pending first MATDOG integration |
| Validated upstream commit | Pending |
| Validated `MattRobotics/norma-core` commit | Pending |
| MATDOG integration branch | Pending |
