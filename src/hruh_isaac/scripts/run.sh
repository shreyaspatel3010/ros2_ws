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
  watch)
    # Live view of training: a few robots that always run the newest checkpoint of the current
    # offline run (swapped in place as training saves them). Never touches the training files.
    # Small share of the machine next to the training run.
    # (the Isaac GUI peaks above 9 GB RAM while starting: 10 GB was too tight)
    export HRUH_CPU_CORES="${HRUH_CPU_CORES:-4}" HRUH_GPU_MEM_GB="${HRUH_GPU_MEM_GB:-5}" HRUH_MEM_GB="${HRUH_MEM_GB:-14}"
    mkdir -p "$HRUH_ROOT/artifacts/hruh/watch"
    "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/watch_live.py "${1:-motion}" --num_envs 4 --visualizer kit "${@:2}" \
      2>&1 | tee "$HRUH_ROOT/artifacts/hruh/watch/live.log"
    ;;
  promote)
    exec /usr/bin/python3 src/hruh_isaac/scripts/promote_policy.py "$@"
    ;;
  play|zero)
    exec "$LIMIT" "$HRUH_PYTHON" src/hruh_isaac/scripts/rl.py "$mode" --task Hruh-Velocity-Flat-v0 "$@"
    ;;
  *)
    echo "Usage: bash src/hruh_isaac/scripts/run.sh {prepare|doctor|train|evaluate|joystick|play|zero|promote|watch} [options]"
    ;;
esac
