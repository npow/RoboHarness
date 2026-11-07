#!/usr/bin/env python3
"""Call the local physical-YAM session service."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:18761"


def request(path: str, payload: dict | None = None) -> dict:
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(
        BASE + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=300) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        result = json.loads(exc.read() or b"{}")
        raise SystemExit(f"robot service error: {result.get('error', exc.reason)}") from None
    except OSError as exc:
        raise SystemExit(f"cannot reach local YAM service at {BASE}: {exc}") from None
    print(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ROBOHARNESS_YAM_PORT", "18761"))
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("observe", help="capture fresh camera images and read current state")
    start = sub.add_parser("start", help="connect and begin the operator-gated session")
    start.add_argument("--instruction", required=True)
    step = sub.add_parser("step", help="send one absolute 14-value joint target")
    step.add_argument("--action", required=True, help="JSON array of 14 numbers")
    sub.add_parser("close", help="park and close the hardware session")
    args = parser.parse_args()
    global BASE
    BASE = f"http://127.0.0.1:{args.port}"

    if args.command == "observe":
        request("/observe", {})
    elif args.command == "start":
        request("/start", {"instruction": args.instruction})
    elif args.command == "step":
        try:
            action = json.loads(args.action)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"--action must be a JSON array: {exc}") from None
        request("/step", {"action": action})
    else:
        request("/close", {})


if __name__ == "__main__":
    main()
