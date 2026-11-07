# Bimanual YAM robot and control contract

This workspace controls the physical I2RT YAM through the
[`inspect-robots-yam`](https://github.com/robocurve/inspect-robots-yam)
`YAMEmbodiment`. Its I2RT driver is the only hardware-control path used here.
The RoboHarness session keeps one embodiment and driver open across commands.

## Packed state and actions

All observations and actions use 14 values, left arm then right arm. Each arm
has six joint angles in radians followed by a normalized gripper value:

| Indices | Values | Units |
|---|---|---|
| `0..5` | Left `j0..j5` | radians |
| `6` | Left gripper | `0` closed, `1` open |
| `7..12` | Right `j0..j5` | radians |
| `13` | Right gripper | `0` closed, `1` open |

Actions are absolute targets. The RoboHarness session clips each request to at
most `max_joint_step_rad` per arm joint and `max_gripper_step` per gripper from
the latest measured state. The YAM embodiment then applies its configured
joint limits and guardrails. Clipping is reported as `step_clipped` together
with both `requested_action` and `sent_action`.

## Cameras and channels

Configure the two CAN channels and all three camera slots in
`.runtime/config.json`. Each slot must have exactly one source:

- `*_cam_device`: V4L2 device or camera-hub stream URL.
- `*_depth_serial`: RealSense serial number.

Use stable device paths. Verify channel-to-arm assignment and camera placement
with the upstream adapter's setup and health commands. The workspace server
does not probe devices until `start` reaches `YAMEmbodiment.reset()`.

## Motion lifecycle

The service process starts idle and does not create the I2RT connection.
`start` explicitly calls the embodiment's `reset`, which connects the arms,
checks the configured cameras, runs the operator readiness gate, and ramps to
the configured home/start pose. The adapter's default home is all-zero joints
with open grippers; use `start_from_current_pose: true` to use the measured
current pose, or configure a rig-appropriate `home_pose`/captured `start_pose`.
The current-pose mode refuses feedback outside configured bounds. Set
`rest_pose: null` to return to the pose captured before homing
when an operator explicitly closes a healthy session. `step` requires an
active session. On a fault or process shutdown, the service releases the
driver without commanding a park movement.

`observe` captures a new camera frame and joint state without commanding
motion. It uses the pinned adapter's private `_observe` helper because the
current public embodiment interface returns observations from `reset` and
`step` only.

The robot's actual joint bounds, base placement, gripper type, collision
geometry, and camera calibration are rig-specific. The supplied JSON is a
configuration template only. In particular, measure and configure the
collision-guardrail base offsets for the installed rig; do not infer them from
the arm model. Keep an operator at the e-stop during initial use and verify
each commanded move from fresh images and measured state.
