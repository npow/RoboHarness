# RoboHarness V2: physical bimanual I2RT YAM

This workspace lets a coding agent observe and command a physical two-arm
I2RT YAM. Hardware access is provided by the existing
[`inspect-robots-yam`](https://github.com/robocurve/inspect-robots-yam)
adapter, which wraps the I2RT driver with joint limits, a collision guardrail,
motor fault and temperature checks, operator gates, and safe teardown.

The agent talks to a local persistent session through `tools/robot.py`. The
session holds one `YAMEmbodiment` instance, so successive tool calls share the
same driver connection. It binds to loopback only and starts in an idle state.
No driver connection or motion occurs until the operator explicitly runs the
`start` command.

## Install

Use Linux with Python, `uv`, the I2RT YAM arms, and the configured top,
left-wrist, and right-wrist cameras. `setup.sh` creates a Python 3.11
environment and installs pinned adapter/driver revisions. On a host with a
working YAM environment already installed, reuse that interpreter instead of
installing a second copy of I2RT:

```bash
export YAM_PYTHON="$HOME/code/molmoact2/.venv/bin/python"
```

```bash
cd harness/v2/YAM
./setup.sh
cp robot_config/config.example.json .runtime/config.json
```

Edit `.runtime/config.json` with both CAN channels and exactly one source for
each camera slot. A slot can use a stable `*_cam_device` path (V4L2) or a
`*_depth_serial` (RealSense). The example intentionally contains no hardware
addresses. Keep this file local; it is ignored by Git. Populate `home_pose`
with the rig's measured, known-safe 14-value start pose before starting. The
server refuses to start without it; the adapter's all-zero default is not
assumed safe. `rest_pose: null` returns to the pose measured when the session
starts during an explicit normal `close`.

Before first use, follow the adapter's
[preflight and hardware instructions](https://github.com/robocurve/inspect-robots-yam#preflight-prove-compatibility-before-any-motion).
Run its compatibility preflight first. Run its health check only while the
robot is idle, with both arms supported or at rest and an e-stop in hand;
connecting and closing that check releases motor torque.

## Start a session

Use two terminals in this directory. Terminal 1 starts the local service; it
does not connect to hardware yet:

```bash
"${YAM_PYTHON:-.venv/bin/python}" robot_config/server.py --config .runtime/config.json
```

Terminal 2 explicitly connects, waits for the embodiment's operator readiness
gate, and ramps to its configured start pose:

```bash
"${YAM_PYTHON:-.venv/bin/python}" tools/robot.py start --instruction "Move the object to the tray"
```

The reset/home ramp can move both arms. Clear the workcell, keep the operator
at the e-stop, and confirm the configured home pose is appropriate for this
rig before accepting the readiness prompt. The driver is connected only once
for the session. `close` on a healthy session parks to the captured initial
pose; process shutdown or a fault releases the driver without commanding a
motion.

## Agent tools

```bash
"${YAM_PYTHON:-.venv/bin/python}" tools/robot.py observe
"${YAM_PYTHON:-.venv/bin/python}" tools/robot.py step --action '[0,0,0,0,0,0,1,0,0,0,0,0,0,1]'
"${YAM_PYTHON:-.venv/bin/python}" tools/robot.py close
```

The action is an absolute 14-value vector in this order:
`left[j0..j5, gripper], right[j0..j5, gripper]`. Joint angles are radians;
grippers are normalized, with `0` closed and `1` open. Each per-call change is
limited by `max_joint_step_rad` and `max_gripper_step` in the config. The
embodiment applies its configured joint bounds and safety checks as a second
layer. Returned observations include the current state, a motion report, and
paths to fresh camera images under `.runtime/observations/`.

Use the returned images and measured state to decide what to do next. A tool
response saying a command was sent does not prove contact, grasp, or task
success. When finished, `close` asks the embodiment to park and then releases
the driver. Confirm the arms are stable before entering the workcell.

## Session API

The local HTTP API is intentionally limited to loopback and has no
authentication. Do not bind or forward it to a network interface.

| Method | Path | Effect |
|---|---|---|
| `GET` | `/status` | Read session state; no robot connection |
| `GET` or `POST` | `/observe` | Capture current cameras and read state without motion |
| `POST` | `/start` | Connect, operator gate, and start-pose ramp |
| `POST` | `/step` | Send one clipped absolute joint target |
| `POST` | `/close` | Park and release the embodiment |

This hardware workspace is not part of the BEHAVIOR or RoboDojo simulator
evaluation runners. It does not claim simulator scores or sim-to-real
validation.
