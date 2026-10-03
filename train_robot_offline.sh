#!/usr/bin/env bash
# Local Isaac training -> held-out evaluation -> export -> isolated Gazebo test.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" == --help ]]; then
  cat <<'HELP'
Usage: ./train_robot_offline.sh [--check]
No API calls, downloads, package installation, or cloud logging are used.
Defaults: walking, reaching, lifting; up to 3 rounds of 1000 PPO iterations
per skill, 128 parallel environments, followed by evaluation and Gazebo.
Rerun to resume. Ctrl+C stops; existing checkpoints remain saved.

Environment overrides:
  SKILLS="flat reach lift"   Also supports rough
  ROUNDS=3 ITERATIONS=1000 NUM_ENVS=128 EVAL_ENVS=32
  GAZEBO=1 GAZEBO_SECONDS=20
  ISAAC_PYTHON=/opt/isaac/venv-6.1/bin/python
  RUN_ID=offline_v2          Change to start an independent experiment

Examples:
  ./train_robot_offline.sh --check
  SKILLS=reach ROUNDS=1 ITERATIONS=500 ./train_robot_offline.sh
  NUM_ENVS=64 ./train_robot_offline.sh

Current tasks are experimental. A failed score is never promoted to success.
Gazebo reports measure execution and falls, not full manipulation success.
Tool use, writing, and general maintenance are not implemented tasks.
HELP
  exit 0
fi
[[ $# == 0 || ( $# == 1 && "$1" == --check ) ]] || { echo 'Use --help'; exit 2; }
PY="${ISAAC_PYTHON:-/opt/isaac/venv-6.1/bin/python}"
# offline_v2: retrain after the yaw-wobble / reach-workspace fixes (2026-10-03); offline_v1 used the old task code
RUN_ID="${RUN_ID:-offline_v2}"
[[ "$RUN_ID" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid RUN_ID'; exit 2; }
for key in ROUNDS ITERATIONS NUM_ENVS EVAL_ENVS GAZEBO_SECONDS; do
  case "$key" in
    ROUNDS) default=3;; ITERATIONS) default=1000;; NUM_ENVS) default=128;;
    EVAL_ENVS) default=32;; GAZEBO_SECONDS) default=20;;
  esac
  value="${!key:-$default}"
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || { echo "$key must be a positive integer"; exit 2; }
  printf -v "$key" '%s' "$value"
done
GAZEBO="${GAZEBO:-1}"
[[ "$GAZEBO" == 0 || "$GAZEBO" == 1 ]] || exit 2
read -r -a skills <<< "${SKILLS:-flat reach lift}"
for skill in "${skills[@]}"; do
  [[ " flat rough reach lift " == *" $skill "* ]] || { echo "Unknown skill: $skill"; exit 2; }
done
STATE="$ROOT/artifacts/hruh/offline/$RUN_ID"
mkdir -p "$STATE"
exec 9>"$ROOT/artifacts/hruh/offline/training.lock"
flock -n 9 || { echo 'Another offline trainer is already running.'; exit 1; }
# Only this shell owns the lock. Close descriptor 9 in long-running children so
# an orphaned simulator cannot block the next run after this shell has exited.
trap 'echo "Stopped at line $LINENO. See logs in $STATE. Rerun this script to resume." >&2' ERR
trap 'echo "Stopped. Saved checkpoints are retained."; exit 130' INT TERM
export PYTHONUNBUFFERED=1 WANDB_MODE=disabled HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
# All task ground assets resolve to this local tree, including Isaac terrain imports.
export ISAACSIM_ASSET_ROOT="$ROOT/src/hruh_isaac/assets"
export GZ_FUEL_CACHE_ONLY=1
export HRUH_URDF="$ROOT/artifacts/hruh/hruh_isaac.urdf"
export PYTHONPATH="$ROOT/src/hruh_isaac/hruh_lab${PYTHONPATH:+:$PYTHONPATH}"
[[ -x "$PY" ]] || { echo "Missing installed Isaac Python: $PY"; exit 1; }
[[ -f install/setup.bash ]] || { echo 'Missing ROS workspace install/setup.bash'; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv
"$PY" -c 'import torch; assert torch.cuda.is_available(), "CUDA unavailable"; print("CUDA ready:", torch.cuda.get_device_name(0))'
if [[ ! -f "$HRUH_URDF" ]]; then
  /usr/bin/python3 src/hruh_isaac/scripts/export_isaac_urdf.py --output "$HRUH_URDF"
fi
if [[ "$GAZEBO" == 1 ]]; then
  command -v gz >/dev/null
  ros2 pkg prefix gz_ros2_control >/dev/null
fi
[[ "${1:-}" != --check ]] || { echo 'Local prerequisites found. Full simulation is checked by training.'; exit 0; }
# Refuse to silently resume weights under edited task or robot definitions.
fingerprint="$(/usr/bin/python3 - <<'PY'
from pathlib import Path
import hashlib, os
h = hashlib.sha256()
paths = sorted(Path('src/hruh_isaac/hruh_lab').rglob('*.py')) + [Path(os.environ['HRUH_URDF'])]
for p in paths:
    h.update(str(p).encode()); h.update(p.read_bytes())
print(h.hexdigest())
PY
)"
if [[ -f "$STATE/source.sha256" && "$(cat "$STATE/source.sha256")" != "$fingerprint" ]]; then
  echo 'Robot/task code changed. Set a new RUN_ID to train a compatible experiment.'; exit 1
fi
printf '%s\n' "$fingerprint" > "$STATE/source.sha256"
printf 'Run %s: %s; %s rounds, %s iterations/round. Logs: %s\n' "$RUN_ID" "${skills[*]}" "$ROUNDS" "$ITERATIONS" "$STATE"
latest_checkpoint() {
  /usr/bin/python3 - "$1" <<'PY'
from pathlib import Path
import sys
files = list(Path(sys.argv[1]).glob('*/model_*.pt'))
if files:
    print(max(files, key=lambda p: (p.stat().st_mtime_ns, p.name)).resolve())
PY
}
for skill in "${skills[@]}"; do
  case "$skill" in
    flat) task=Hruh-Velocity-Flat-v0;; rough) task=Hruh-Velocity-Rough-v0;;
    reach) task=Hruh-Reach-Right-v0;; lift) task=Hruh-Lift-Right-v0;;
  esac
  experiment="hruh_${RUN_ID}_${skill}"
  mkdir -p "$STATE/$skill"
  for ((round=1; round<=ROUNDS; round++)); do
    report="$STATE/$skill/evaluation_$round.json"
    checkpoint="$(latest_checkpoint "$ROOT/logs/rsl_rl/$experiment")"
    if [[ ! -f "$STATE/$skill/trained_$round" ]]; then
      resume=()
      [[ -z "$checkpoint" ]] || resume=(--checkpoint "$checkpoint")
      echo "[$skill round $round] Training; progress in $STATE/$skill/train_$round.log"
      "$PY" src/hruh_isaac/scripts/rl.py train --task "$task" --visualizer none \
        --num_envs "$NUM_ENVS" --max_iterations "$ITERATIONS" --logger tensorboard \
        --experiment_name "$experiment" --run_name "round_$round" "${resume[@]}" \
        > "$STATE/$skill/train_$round.log" 2>&1 9>&-
      checkpoint="$(latest_checkpoint "$ROOT/logs/rsl_rl/$experiment")"
      [[ -f "$checkpoint" ]] || { echo 'Training produced no checkpoint'; exit 1; }
      printf '%s\n' "$checkpoint" > "$STATE/$skill/trained_$round"
    fi
    checkpoint="$(cat "$STATE/$skill/trained_$round")"
    if [[ ! -f "$report" || ! -f "$(dirname "$checkpoint")/exported/bundle.json" ]]; then
      echo "[$skill round $round] Evaluating and exporting"
      if [[ "$skill" == flat || "$skill" == rough ]]; then
        evaluator=(src/hruh_isaac/scripts/evaluate.py --terrain "$skill")
      else
        evaluator=(src/hruh_isaac/scripts/evaluate_manipulation.py --skill "$skill")
      fi
      "$PY" "${evaluator[@]}" --checkpoint "$checkpoint" --num_envs "$EVAL_ENVS" \
        --visualizer none --export --output "$report" > "$STATE/$skill/evaluate_$round.log" 2>&1 9>&-
    fi
    printf '%s\n' "$checkpoint" > "$STATE/$skill/latest_checkpoint.txt"
    if /usr/bin/python3 - "$report" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
passed = r.get('passed_benchmark', False)
print('Isaac benchmark:', 'PASS' if passed else 'FAIL', '|', sys.argv[1])
sys.exit(0 if passed else 1)
PY
    then break; fi
  done
  bundle="$(dirname "$checkpoint")/exported"
  if [[ "$GAZEBO" == 1 && ! -f "$STATE/$skill/gazebo.json" ]]; then
    echo "[$skill] Testing exported policy in Gazebo (experimental transfer)"
    export ROS_DOMAIN_ID="${HRUH_ROS_DOMAIN_ID:-73}" GZ_PARTITION="hruh_${RUN_ID}_$$"
    generated="$ROOT/artifacts/hruh/gazebo/$(basename "$(dirname "$checkpoint")")/report.json"
    # A failed retry must not reuse a previous run's report.
    if [[ -f "$generated" ]]; then mv "$generated" "$generated.previous"; fi
    # Launch directly from source, so no package rebuild is required.
    timeout --signal=INT --kill-after=20s "$((GAZEBO_SECONDS * 20 + 150))s" \
      ros2 launch "$ROOT/src/hruh_bringup/launch/policy_gazebo.launch.py" \
      workspace:="$ROOT" bundle:="$bundle" python:="$PY" gui:=false seconds:="$GAZEBO_SECONDS" \
      > "$STATE/$skill/gazebo.log" 2>&1 9>&- || echo 'Gazebo exited unsuccessfully; inspect gazebo.log'
    if [[ -f "$generated" ]]; then
      cp "$generated" "$STATE/$skill/gazebo.json"
    else
      echo 'Gazebo produced no report; transfer is NOT verified.'
    fi
  fi
done
/usr/bin/python3 - "$STATE" <<'PY'
from pathlib import Path
import json, sys
p = Path(sys.argv[1]); rows = []
for s in sorted(p.iterdir()):
    if not s.is_dir(): continue
    reports = sorted(s.glob('evaluation_*.json'), key=lambda f: int(f.stem.split('_')[-1]))
    if not reports: continue
    r = json.loads(reports[-1].read_text())
    rows.append(f"{s.name}: Isaac benchmark {'PASS' if r.get('passed_benchmark') else 'FAIL'}")
    g = s / 'gazebo.json'
    if g.exists():
        d = json.loads(g.read_text())
        rows.append(f"  Gazebo: completed={d.get('completed')}, falls={d.get('falls')}, policy_steps={d.get('policy_steps')}; skill success NOT verified")
    else: rows.append('  Gazebo: no report; transfer NOT verified')
rows += ['', 'Failed benchmarks need further task/reward/controller development.',
         'This script does not implement general tool use or guarantee fall-free behavior.']
text = '\n'.join(rows) + '\n'
(p / 'SUMMARY.txt').write_text(text); print(text)
PY
echo "Finished. Reports and checkpoint paths: $STATE"
