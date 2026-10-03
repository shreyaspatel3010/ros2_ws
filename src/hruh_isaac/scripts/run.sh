#!/usr/bin/env bash
set -euo pipefail
HRUH_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
HRUH_PYTHON="${ISAAC_PYTHON:-/opt/isaac/venv-6.1/bin/python}"
export HRUH_URDF="${HRUH_URDF:-$HRUH_ROOT/artifacts/hruh/hruh_isaac.urdf}"
cd "$HRUH_ROOT"
# Every Isaac process runs under hard RAM / CPU / GPU caps (see limit.sh).
LIMIT="$HRUH_ROOT/src/hruh_isaac/scripts/limit.sh"
mode="${1:-help}"
if (($#)); then shift; fi
case "$mode" in
  prepare)
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$HRUH_ROOT/install/setup.bash"
    set -u
    exec /usr/bin/python3 src/hruh_isaac/scripts/export_isaac_urdf.py --output "$HRUH_URDF" "$@"
    ;;
  doctor)
    nvidia-smi
    "$HRUH_PYTHON" -c 'import torch, importlib.metadata as m; print({p:m.version(p) for p in ("isaacsim", "isaaclab", "rsl-rl-lib")}); print("CUDA available:", torch.cuda.is_available()); assert torch.cuda.is_available(), "CUDA unavailable"; print(torch.cuda.get_device_name(0))'
    test -f "$HRUH_URDF" || { echo "Run: bash src/hruh_isaac/scripts/run.sh prepare"; exit 1; }
    ;;
  train)
    exec "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/rl.py train --task Hruh-Velocity-Flat-v0 --num_envs 256 --visualizer none "$@"
    ;;
  evaluate)
    exec "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/evaluate.py --visualizer none "$@"
    ;;
  joystick)
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    exec "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/evaluate.py --mode joystick --seconds 600 --visualizer kit "$@"
    ;;
  promote)
    exec /usr/bin/python3 src/hruh_isaac/scripts/promote_policy.py "$@"
    ;;
  play|zero)
    exec "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/rl.py "$mode" --task Hruh-Velocity-Flat-v0 "$@"
    ;;
  *)
    echo "Usage: bash src/hruh_isaac/scripts/run.sh {prepare|doctor|train|evaluate|joystick|play|zero|promote} [options]"
    ;;
esac
