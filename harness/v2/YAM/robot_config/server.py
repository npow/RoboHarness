#!/usr/bin/env python3
"""Loopback session server for the inspect-robots-yam hardware embodiment."""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _observation_payload(observation: Any, sequence: int) -> dict[str, Any]:
    import cv2

    out_dir = ROOT / ".runtime" / "observations"
    out_dir.mkdir(parents=True, exist_ok=True)
    images: dict[str, str] = {}
    for name, rgb in observation.images.items():
        filename = f"{sequence:06d}_{name}.png"
        path = out_dir / filename
        frame = np.asarray(rgb)
        if frame.ndim == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError(f"could not write observation image {path}")
        images[name] = str(path.relative_to(ROOT))
    state = {name: _jsonable(value) for name, value in observation.state.items()}
    return {
        "sequence": sequence,
        "observed_at_unix": time.time(),
        "instruction": observation.instruction,
        "state": state,
        "images": images,
        "extra": _jsonable(dict(observation.extra)),
    }


class YamSession:
    def __init__(self, config: dict[str, Any]) -> None:
        self.host = config.pop("host", "127.0.0.1")
        self.port = int(config.pop("port", 18761))
        self.max_joint_step_rad = float(config.pop("max_joint_step_rad", 0.05))
        self.max_gripper_step = float(config.pop("max_gripper_step", 0.1))
        if self.host != "127.0.0.1" and self.host != "localhost":
            raise ValueError("YAM session server must bind to loopback only")
        if not 0 < self.max_joint_step_rad <= 0.2:
            raise ValueError("max_joint_step_rad must be in (0, 0.2]")
        if not 0 < self.max_gripper_step <= 0.2:
            raise ValueError("max_gripper_step must be in (0, 0.2]")
        for slot in ("top", "left", "right"):
            sources = [config.get(f"{slot}_cam_device"), config.get(f"{slot}_depth_serial")]
            if sum(source is not None for source in sources) != 1:
                raise ValueError(
                    f"configure exactly one of {slot}_cam_device or {slot}_depth_serial"
                )

        from inspect_robots_yam import YAMEmbodiment, YamConfig

        self._config = YamConfig.from_kwargs(**config)
        self._embodiment = YAMEmbodiment(self._config)
        self._lock = threading.RLock()
        self._active = False
        self._closed = False
        self._connected = False
        self._fault: str | None = None
        self._last: dict[str, Any] | None = None
        self._sequence = 0
        self._instruction = ""

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": (
                    "closed" if self._closed else
                    "faulted" if self._fault else
                    "active" if self._active else
                    "idle"
                ),
                "fault": self._fault,
                "last_observation": self._last,
            }

    def start(self, instruction: str) -> dict[str, Any]:
        if not instruction.strip():
            raise ValueError("instruction must not be empty")
        if self._config.home_pose is None:
            raise ValueError(
                "set an explicit 14-value home_pose in the local config before start; "
                "the adapter's zero pose is not assumed safe for this rig"
            )
        with self._lock:
            if self._active:
                raise RuntimeError("session is already active")
            if self._fault:
                raise RuntimeError(f"session is faulted; close it and restart: {self._fault}")
            if self._closed:
                raise RuntimeError("session was closed; restart the server to reconnect")
            from inspect_robots.scene import Scene

            # reset() is the embodiment's explicit connection, operator gate,
            # and start-pose ramp boundary. No hardware is touched at import or
            # service startup time.
            try:
                obs = self._embodiment.reset(
                    Scene(id="roboharness-yam-live", instruction=instruction)
                )
                self._connected = True
                self._sequence += 1
                observation = _observation_payload(obs, self._sequence)
            except Exception:
                # A failed reset may have connected one or both arms before a
                # camera/operator/runtime error. Release the driver in place;
                # do not command a park pose while handling a fault.
                try:
                    self._embodiment._init_pose = None
                    self._embodiment.close()
                except Exception:
                    logging.exception("YAM teardown after failed start also failed")
                self._closed = True
                raise
            self._active = True
            self._instruction = instruction
            self._last = observation
            return self._last

    def observe(self) -> dict[str, Any]:
        with self._lock:
            if not self._active:
                raise RuntimeError("session is idle; explicitly start it first")
            # YAMEmbodiment currently exposes observations through reset/step.
            # This pinned adapter method captures cameras and public state
            # without sending a motor command.
            try:
                obs = self._embodiment._observe(self._instruction)
                self._sequence += 1
                self._last = _observation_payload(obs, self._sequence)
                return self._last
            except Exception as exc:
                self._fault = f"observation failed: {type(exc).__name__}: {exc}"
                self._active = False
                raise

    def step(self, action: Any) -> dict[str, Any]:
        target = np.asarray(action, dtype=np.float64)
        if target.shape != (14,) or not np.all(np.isfinite(target)):
            raise ValueError("action must be a finite 14-value vector")
        with self._lock:
            if not self._active or self._last is None:
                raise RuntimeError("session is idle; explicitly start it first")
            state = self._last["state"].get("joint_pos")
            if state is None:
                raise RuntimeError("latest observation has no joint_pos state")
            current = np.asarray(state, dtype=np.float64)
            if current.shape != (14,) or not np.all(np.isfinite(current)):
                raise RuntimeError("latest joint state is invalid")
            limits = np.full(14, self.max_joint_step_rad, dtype=np.float64)
            limits[[6, 13]] = self.max_gripper_step
            sent = current + np.clip(target - current, -limits, limits)

            from inspect_robots.types import Action

            try:
                result = self._embodiment.step(Action(data=sent))
                self._sequence += 1
                self._last = _observation_payload(result.observation, self._sequence)
            except Exception as exc:
                self._fault = f"step failed: {type(exc).__name__}: {exc}"
                self._active = False
                raise
            return {
                **self._last,
                "requested_action": target.tolist(),
                "sent_action": sent.tolist(),
                "step_clipped": not np.allclose(target, sent),
                "terminated": result.terminated,
                "termination_reason": result.termination_reason,
                "info": _jsonable(dict(result.info)),
            }

    def close(self) -> dict[str, Any]:
        with self._lock:
            if self._closed:
                return self.status()
            try:
                if self._connected:
                    if self._fault:
                        self._embodiment._init_pose = None
                    self._embodiment.close()
            finally:
                self._active = False
                self._connected = False
                self._closed = True
            return self.status()

    def shutdown(self) -> None:
        """Release driver handles without commanding motion on process exit."""
        with self._lock:
            try:
                if self._connected:
                    self._embodiment._init_pose = None
                    self._embodiment.close()
            finally:
                self._active = False
                self._connected = False
                self._closed = True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    session = YamSession(config)

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/status":
                self._reply(200, session.status())
            elif self.path == "/observe":
                try:
                    self._reply(200, session.observe())
                except (ValueError, RuntimeError, TypeError) as exc:
                    self._reply(400, {"error": str(exc)})
                except Exception as exc:
                    logging.exception("YAM observation failed")
                    self._reply(500, {"error": f"{type(exc).__name__}: {exc}"})
            else:
                self._reply(404, {"error": "unknown endpoint"})

        def do_POST(self) -> None:  # noqa: N802
            if self.path not in {"/start", "/step", "/close", "/observe"}:
                self._reply(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length > 65536:
                    raise ValueError("request body is too large")
                payload = json.loads(self.rfile.read(length) or b"{}")
                if self.path == "/start":
                    response = session.start(str(payload.get("instruction", "")))
                elif self.path == "/step":
                    response = session.step(payload.get("action"))
                elif self.path == "/observe":
                    response = session.observe()
                else:
                    response = session.close()
                self._reply(200, response)
            except (ValueError, RuntimeError, TypeError) as exc:
                self._reply(400, {"error": str(exc)})
            except Exception as exc:  # hardware faults are visible and stop this action
                logging.exception("YAM request failed")
                self._reply(500, {"error": f"{type(exc).__name__}: {exc}"})

        def log_message(self, fmt: str, *args: Any) -> None:
            logging.info("%s - %s", self.address_string(), fmt % args)

    server = ThreadingHTTPServer((session.host, session.port), Handler)
    server.daemon_threads = True
    logging.info("YAM control session listening on http://%s:%d", session.host, session.port)
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown).start())
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            session.shutdown()
        finally:
            server.server_close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    main()
