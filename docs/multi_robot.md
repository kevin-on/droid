# EXPO-FT multi-robot integration

This fork combines the existing NUC DROID changes (imported in `cff22c2`) with
EXPO-FT's per-robot routing. It is based on pd-perry/droid `076cecd2`.
Deploy it as a separate checkout; the active shared NUC checkout is
`/home/iliad/khhung/expoft`. Installing this fork does not update that installation.

## Routing

The current workstation/NUC mapping is:

| Arm IP | ZeroRPC | Polymetis arm | Polymetis gripper | Gripper FTDI serial |
|---|---|---|---|---|
| 172.16.0.2 | 4242 | 50053 | 50054 | DA6UJOT5 |
| 172.16.0.3 | 4243 | 50051 | 50052 | DA6UJXZ9 |

On the NUC, the launcher scripts retain
`/home/iliad/Utilities/miniconda3` and its `expoft` environment. The following is
a reference command for an explicitly scheduled deployment, run from the new
checkout after activating that environment:

```bash
python scripts/server/run_server.py \
  --zerorpc-port 4242 --robot-ip 172.16.0.2 --robot-port 50053 \
  --gripper-port 50054 \
  --gripper-comport /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6UJOT5-if00-port0
```

The second arm uses the second row of the table and the corresponding full FTDI
path. `--port` aliases `--zerorpc-port`; `--gripper-device` aliases
`--gripper-comport`. The existing positional constructor order is preserved:
`FrankaRobot(robot_ip, robot_port, gripper_comport, gripper_port)`.
`gripper_device` is also accepted as a keyword.

Creating the ZeroRPC server is inert. Creating a remote `RobotEnv` normally
launches controllers. `launch_controller=False` attaches to **both** existing
Polymetis servers at the configured arm and gripper ports; it does not start a
missing gripper server. This attachment initializes interfaces without a reset.

Each `FrankaRobot` tracks the process groups it starts. Restart/cleanup sends
TERM only to those groups. There is no process-name, port, or device-based kill.
An externally owned controller must be reused explicitly or stopped by its owner.
Launcher failure cleans up the other owned launcher, and a failed stop retains
ownership for a later retry. Normal server unwinding cleans up owned controllers.

Per-port logs (`/tmp/droid_robot_PORT.log`, `/tmp/droid_gripper_PORT.log`),
cooperative startup waits, connection/gripper-metadata retries and the 30-second
ZeroRPC heartbeat are preserved. So are the NUC's 30 Hz environment/IK settings,
gripper speed/force and move-time tuning. The configured frequency does not
establish achievable two-arm performance. NUC sudo authentication remains local
configuration; no credential is committed.

## Workstation camera and RPC selection

`RobotEnv` accepts `robot_server_ip`, `robot_server_port`, `launch_controller`,
`camera_serials`, `wrist_camera_serial` and `camera_kwargs`. The previous
`server_port` name remains supported; conflicting nondefault aliases fail.

Each rollout must select a disjoint camera list that includes its own wrist.
Serials are normalized to strings. Missing selected cameras fail before any ZED
wrapper is constructed. Wrist classification, metadata and wrist calibration
use that environment's wrist serial, rather than the global wrist assignment.
Side settings accept DROID's `varied_camera` and EXPO-FT's `static_camera`
fallback; the canonical DROID key takes precedence.

Robot 0 currently uses wrist `15577469` and side `38651013`. Robot 1's camera
assignment is pending installation. Camera setup, per-arm reset joints, bounds
and SpaceMouse assignment must be supplied before a real rollout.

## Hardware-free tests

In a separate Python 3.11 environment with `pytest` and `numpy`, run:

```bash
python -m pytest -q tests
```

Tests execute the real definitions with RPC, SDK, process and device operations
replaced. They check two-arm routing, camera selection/calibration, old/new
arguments, attach mode, retries, heartbeat and process cleanup failures.
They do not open devices, invoke sudo, start controllers or validate robot motion.
