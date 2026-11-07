# Physical bimanual YAM workspace

Read `README.md` and `robot_config/README.md` before interacting with the
robot. The supported path uses the `inspect-robots-yam` embodiment and its
I2RT driver; do not call I2RT directly or bypass that embodiment.

The local service binds only to `127.0.0.1`. It does not connect or move the
arms until `tools/robot.py start` is called. Start/reset requires either an
explicit pose or `start_from_current_pose=true`; the latter captures measured
joint feedback as the reset target and refuses startup if it is out of bounds.
Reset still uses the embodiment's operator readiness gate. Keep an operator at
the e-stop and make sure the workspace is clear before starting.

Use `tools/robot.py observe` to inspect the latest camera images and packed
joint state. Before every move, inspect fresh camera evidence and the current
state. Send a complete 14-value absolute joint target; the service clips each
joint change to the per-call limits and the embodiment applies its own
configured bounds, collision checks, and driver fault checks. Inspect the
returned observation before deciding on another move.

Do not use unattended mode, disable the embodiment's bounds checks, or issue
commands through another driver path. Call `tools/robot.py close` when done;
the embodiment parks and tears down the driver. Confirm the arms are stable
before reaching into the workcell.
