#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

command -v uv >/dev/null || { echo "uv is required" >&2; exit 1; }
uv venv --python 3.11 .venv
mkdir -p .runtime
printf '%s\n' 'scikit-build-core<0.10' > .runtime/build-constraints.txt
uv pip install --python .venv/bin/python \
  --build-constraints .runtime/build-constraints.txt \
  'inspect-robots-yam[collision] @ git+https://github.com/robocurve/inspect-robots-yam@5a017d1b47b731f00cb7f26e59831dca03a15a71' \
  'i2rt @ git+https://github.com/i2rt-robotics/i2rt@ac096928d6899ddf852a71c5e8fbaa6055cd9745'
echo "Installed pinned inspect-robots-yam and I2RT into .venv"
