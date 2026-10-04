#!/usr/bin/env bash
# Fully automatic local pipeline, one command, no prompts:
#   build workspace -> Isaac training -> held-out evaluation -> export -> Gazebo transfer test
#   -> promotion of passing policies into the robot's runtime (src/hruh_isaac/policies/).
set -Euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" == --help ]]; then
  cat <<'HELP'
Usage: ./train_robot_offline.sh [--check]
Runs everything automatically and shows live progress in the terminal: a bar per
training round (iteration, reward, task error, ETA), every evaluation scenario,
the Gazebo test and the final summary. No API calls, downloads or cloud logging.

Defaults: whole-body movement, reaching, lifting; rounds of 1000 PPO iterations,
repeated until the skill passes its benchmark; 4096 robots trained in parallel
in one GPU scene (lift: 2048, contact-heavy).
  * Between rounds the training is tuned automatically (scripts/auto_tune.py): the
    evaluation and training log are diagnosed and reward weights / strengths, the
    movement / arm / push curriculum and the exploration noise are adjusted by fixed
    rules (each change is printed with its reason). The next round resumes from the
    best checkpoint so far, so a worse round never replaces progress.
  * A skill is finished only when one policy passes its benchmark REQUIRED_PASSES=5
    times: the round's evaluation plus 4 confirmations on new random seeds. Until then
    the run does not move on to the next skill (it keeps tuning and training). A skill stops early when its benchmark passes.
  * The laptop is kept awake (no suspend) until the run ends.
  * A crashed stage is retried automatically (RETRIES=2) from the latest
    checkpoint; if it ran out of GPU/RAM, with half the parallel environments.
  * A skill that keeps failing is reported and the run continues with the next one.
  * Rerun the same command to resume after Ctrl+C, a reboot or a crash.
  * Follow from another terminal: tail -f artifacts/hruh/offline/progress.txt
  * The experiment is named after the code: unchanged code resumes, changed
    robot/task code automatically starts a fresh experiment (old ones are kept).

A policy that passes its Isaac benchmark is promoted to
src/hruh_isaac/policies/<locomotion|reach|lift>/ (the previous one is backed up
to artifacts/hruh/policy_history/). The runtime launch files load it from there:
  ros2 launch hruh_bringup isaac.launch.py                 learned walking (+ reaching)
  ros2 launch hruh_bringup policy_gazebo.launch.py joystick:=true
Every simulator runs under hard RAM / CPU / GPU caps (src/hruh_isaac/scripts/limit.sh)
so the desktop stays responsive; a run that exceeds them is stopped, not the PC.

Environment overrides:
  SKILLS="motion reach lift"
      motion  walk forward / backward, side-step, turn, stop and start while the
              arms hold, swing with the gait or move to random poses (MoveIt-like)
      flat    walking with the arms fixed (older task), rough  uneven terrain
      reach   right-arm reaching, lift  right-hand cube lifting
  ROUNDS=0                   0 = until the benchmark passes; N = at most N rounds
  REQUIRED_PASSES=5          independent passing evaluations (different seeds) per skill
  PATIENCE=0                 0 = never move on before success; N = give up on a skill
                             after N rounds without improvement (avoids endless runs)
  AUTO_TUNE=1                0 = keep the training settings fixed between rounds
  ITERATIONS=1000 EVAL_ENVS=32 RETRIES=2
  NUM_ENVS=<n>               same count for every skill (default per skill: 4096 / lift 2048;
                             halved automatically if the 8 GB GPU budget is exceeded)
  GAZEBO=1 GAZEBO_SECONDS=30     walking skills walk every movement with sudden stops in Gazebo
  PROMOTE=1                  0 = never touch the runtime policies
  POLICY_DIR=src/hruh_isaac/policies   where promoted policies go (the launch files read it)
  REQUIRE_GAZEBO=0           1 = locomotion must also finish the Gazebo test without a fall
  BUILD=1                    0 = skip the automatic colcon build
  WARM_START=1               a new experiment starts from the previous run's best policy
                             of each skill (0 = train from scratch)
  HRUH_GPU_MEM_GB=10 HRUH_CPU_CORES=12 HRUH_MEM_GB=18   resource caps (defaults scale with the PC)
  ISAAC_PYTHON=/opt/isaac/venv-6.1/bin/python
  RUN_ID=<name>              fixed experiment name (default: auto_<code fingerprint>);
                             a fixed name refuses to resume under changed code

Examples:
  ./train_robot_offline.sh --check
  SKILLS=reach ROUNDS=1 ITERATIONS=500 ./train_robot_offline.sh
  PATIENCE=8 ./train_robot_offline.sh    move on if a skill stops improving for 8 rounds
  nohup ./train_robot_offline.sh > training.out 2>&1 &   (plain progress lines, survives closing the terminal)

Current tasks are experimental. A failed score is never promoted.
Gazebo reports measure execution and falls, not full manipulation success.
Tool use, writing, and general maintenance are not implemented tasks.
HELP
  exit 0
fi
[[ $# == 0 || ( $# == 1 && "$1" == --check ) ]] || { echo 'Use --help'; exit 2; }

# ------------------------------------------------------------------ settings
PY="${ISAAC_PYTHON:-/opt/isaac/venv-6.1/bin/python}"
# empty = automatic: named after the robot/task code fingerprint (see below)
RUN_ID="${RUN_ID:-}"
[[ -z "$RUN_ID" || "$RUN_ID" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid RUN_ID'; exit 2; }
NUM_ENVS="${NUM_ENVS:-}"   # empty: per-skill default (skill_envs)
[[ -z "$NUM_ENVS" || "$NUM_ENVS" =~ ^[1-9][0-9]*$ ]] || { echo 'NUM_ENVS must be a positive integer'; exit 2; }
ROUNDS="${ROUNDS:-0}" PATIENCE="${PATIENCE:-0}" AUTO_TUNE="${AUTO_TUNE:-1}" REQUIRED_PASSES="${REQUIRED_PASSES:-5}"
[[ "$ROUNDS" =~ ^[0-9]+$ && "$PATIENCE" =~ ^[0-9]+$ ]] || { echo 'ROUNDS and PATIENCE must be 0 or positive integers'; exit 2; }
[[ "$REQUIRED_PASSES" =~ ^[1-9][0-9]*$ ]] || { echo 'REQUIRED_PASSES must be a positive integer'; exit 2; }
[[ "$AUTO_TUNE" == 0 || "$AUTO_TUNE" == 1 ]] || { echo 'AUTO_TUNE must be 0 or 1'; exit 2; }
for key in ITERATIONS EVAL_ENVS GAZEBO_SECONDS; do
  case "$key" in
    ITERATIONS) default=1000;;
    EVAL_ENVS) default=32;; GAZEBO_SECONDS) default=30;;
  esac
  value="${!key:-$default}"
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || { echo "$key must be a positive integer"; exit 2; }
  printf -v "$key" '%s' "$value"
done
RETRIES="${RETRIES:-2}"
[[ "$RETRIES" =~ ^[0-9]+$ ]] || { echo 'RETRIES must be 0 or a positive integer'; exit 2; }
GAZEBO="${GAZEBO:-1}" PROMOTE="${PROMOTE:-1}" REQUIRE_GAZEBO="${REQUIRE_GAZEBO:-0}" BUILD="${BUILD:-1}"
WARM_START="${WARM_START:-1}"
for key in GAZEBO PROMOTE REQUIRE_GAZEBO BUILD WARM_START; do
  [[ "${!key}" == 0 || "${!key}" == 1 ]] || { echo "$key must be 0 or 1"; exit 2; }
done
read -r -a skills <<< "${SKILLS:-motion reach lift}"
for skill in "${skills[@]}"; do
  [[ " motion flat rough reach lift " == *" $skill "* ]] || { echo "Unknown skill: $skill"; exit 2; }
done
# Hard RAM / CPU caps per simulator process; GPU budget enforced by hruh_lab/gpu_guard.py.
LIMIT="$ROOT/src/hruh_isaac/scripts/limit.sh"
PROGRESS="$ROOT/src/hruh_isaac/scripts/progress.py"
# 12 GB GPU: 10 GB for training, ~2 GB left for the desktop and the live viewer
export HRUH_GPU_MEM_GB="${HRUH_GPU_MEM_GB:-10}"

OFFLINE="$ROOT/artifacts/hruh/offline"
mkdir -p "$OFFLINE" "$ROOT/logs"
# generated trees (source snapshots include setup.py) must not look like colcon packages
touch "$ROOT/artifacts/COLCON_IGNORE" "$ROOT/logs/COLCON_IGNORE"
exec 9>"$ROOT/artifacts/hruh/offline/training.lock"
flock -n 9 || { echo 'Another offline trainer is already running.'; exit 1; }
# Keep the laptop awake (no suspend) while this script runs: the inhibitor ends with this shell.
if [[ "${1:-}" != --check ]] && command -v systemd-inhibit >/dev/null; then
  systemd-inhibit --what=sleep:idle --who="HRUH training" --why="train_robot_offline.sh is training" \
    --mode=block tail --pid=$$ -f /dev/null > /dev/null 2>&1 9>&- &
fi
# Only this shell owns the lock. Descriptor 9 is closed in long-running children so
# an orphaned simulator cannot block the next run after this shell has exited.

START=$SECONDS
interrupted=""
# The running stage receives Ctrl+C itself and saves; this shell stops after it.
trap 'interrupted=1' INT TERM
elapsed() { local s=$((SECONDS - START)); printf '%d:%02d:%02d' $((s / 3600)) $((s / 60 % 60)) $((s % 60)); }
say() {   # terminal + progress.txt (tail -f it from another terminal)
  printf '%s\n' "$*"
  printf '%s %s\n' "$(date '+%F %T')" "$*" >> "$OFFLINE/progress.txt"
}
heading() { say ""; say "━━━ $* ━━━ (total $(elapsed))"; }
stop_if_interrupted() {
  [[ -z "$interrupted" ]] && return 0
  say "Stopped by Ctrl+C after $(elapsed). Checkpoints are saved: run ./train_robot_offline.sh again to resume."
  exit 130
}
notify() {
  printf '\a'
  if command -v notify-send >/dev/null && [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    notify-send "HRUH training" "$1" 2>/dev/null || true
  fi
}

# run_stage MODE LABEL LOG TOTAL command... : full output -> LOG, progress -> terminal.
# Returns the command's exit status. Always call it from an if / && / || context.
run_stage() {
  local mode="$1" label="$2" log="$3" total="$4"
  shift 4
  printf '\n===== %s %s (%s) =====\n' "$label" "$mode" "$(date '+%F %T')" >> "$log"
  STAGE_OFFSET=$(stat -c %s "$log")
  "$@" 2>&1 9>&- | /usr/bin/python3 "$PROGRESS" --mode "$mode" --label "$label" --log "$log" --total "$total" 9>&-
  return "${PIPESTATUS[0]}"
}
stage_output() { tail -c +"$((STAGE_OFFSET + 1))" "$1"; }   # this attempt's part of the log
# exit 137 = killed by the RAM cap (OOM); gpu_guard / CUDA messages = GPU memory
resource_failure() {
  # (gpu_guard also prints its budget at start-up: only its "stopping" message counts)
  [[ "$2" == 137 ]] || stage_output "$1" | grep -qiE '\[gpu_guard\].*stopping|out of memory|CUDA error: out|cudaErrorMemoryAllocation'
}
show_failure() {
  say "  last lines of $1:"
  stage_output "$1" | grep -avE '^\s*$' | tail -n 8 | sed 's/^/    /'
}

# ------------------------------------------------------------------ prerequisites
heading "Preparing"
export PYTHONUNBUFFERED=1 WANDB_MODE=disabled HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
# All task ground assets resolve to this local tree, including Isaac terrain imports.
export ISAACSIM_ASSET_ROOT="$ROOT/src/hruh_isaac/assets"
export GZ_FUEL_CACHE_ONLY=1
export HRUH_URDF="$ROOT/artifacts/hruh/hruh_isaac.urdf"
export PYTHONPATH="$ROOT/src/hruh_isaac/hruh_lab${PYTHONPATH:+:$PYTHONPATH}"
[[ -x "$PY" ]] || { say "Missing installed Isaac Python: $PY (run src/hruh_isaac/scripts/install_isaac.sh)"; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ "$BUILD" == 1 && "${1:-}" != --check ]]; then
  say "Building the ROS workspace (log: $OFFLINE/build.log)"
  if ! nice -n 10 colcon build > "$OFFLINE/build.log" 2>&1; then
    say "  colcon build failed; continuing with the existing install/ (see build.log)"
  fi
fi
stop_if_interrupted
[[ -f install/setup.bash ]] || { say 'Missing ROS workspace install/setup.bash: run colcon build'; exit 1; }
set +u
source "$ROOT/install/setup.bash"
set -u
gpu="$(nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader 2>/dev/null)" \
  || { say 'nvidia-smi failed: NVIDIA driver not available'; exit 1; }
say "GPU: $gpu"
"$PY" -c 'import torch; assert torch.cuda.is_available(), "CUDA unavailable"' \
  || { say 'PyTorch cannot use CUDA in the Isaac Python'; exit 1; }
if [[ ! -f "$HRUH_URDF" ]]; then
  say "Exporting the robot model for Isaac"
  /usr/bin/python3 src/hruh_isaac/scripts/export_isaac_urdf.py --output "$HRUH_URDF" > "$OFFLINE/urdf_export.log" 2>&1 \
    || { say "URDF export failed: $OFFLINE/urdf_export.log"; exit 1; }
fi
if [[ "$GAZEBO" == 1 ]] && ! { command -v gz >/dev/null && ros2 pkg prefix gz_ros2_control >/dev/null 2>&1; }; then
  say "Gazebo / gz_ros2_control not found: skipping the Gazebo transfer tests"
  GAZEBO=0
fi
if [[ "${1:-}" == --check ]]; then
  say 'Local prerequisites found. Full simulation is checked by training.'
  exit 0
fi
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
if [[ -z "$RUN_ID" ]]; then
  # Automatic: one experiment per code version. Same code -> resume; changed code -> new experiment.
  RUN_ID="auto_${fingerprint:0:10}"
  previous="$(cat "$OFFLINE/last_auto_run" 2>/dev/null || true)"
  if [[ -n "$previous" && "$previous" != "$RUN_ID" ]]; then
    say "Robot/task code changed since run $previous: starting new experiment $RUN_ID (the old one is kept)"
  fi
  printf '%s\n' "$RUN_ID" > "$OFFLINE/last_auto_run"
fi
STATE="$OFFLINE/$RUN_ID"
mkdir -p "$STATE"
if [[ -f "$STATE/source.sha256" && "$(cat "$STATE/source.sha256")" != "$fingerprint" ]]; then
  say "Robot/task code changed since run $RUN_ID started: its weights cannot be resumed."
  say "Run without RUN_ID (automatic naming) or choose a new name: RUN_ID=<new name> $0"
  exit 1
fi
printf '%s\n' "$fingerprint" > "$STATE/source.sha256"

latest_checkpoint() {
  /usr/bin/python3 - "$1" <<'PY'
from pathlib import Path
import sys
files = list(Path(sys.argv[1]).glob('*/model_*.pt'))
if files:
    print(max(files, key=lambda p: (p.stat().st_mtime_ns, p.name)).resolve())
PY
}
# Parallel environments per skill: more samples per iteration is what these tasks lacked
# (128 envs gave ~3 M steps per round, 20-50x fewer than Isaac Lab's reference recipes).
skill_envs() {
  if [[ -n "$NUM_ENVS" ]]; then echo "$NUM_ENVS"; return; fi
  # measured: 512 robots 2.9 GB, 1024 robots 3.4 GB and 2x the speed (CPU-bound, GPU 43% busy);
  # 2048 ~4.4 GB fits the 8 GB budget with room for the desktop
  # measured on the 12 GB GPU (11.5 usable): 2048 robots with estimator + symmetry 5.1 GB; the live
  # viewer (run.sh watch) 4.3 GB. 4096 robots (~6.7+ GB) ran out of GPU memory next to the viewer,
  # so 2048 is the default; NUM_ENVS=4096 when nothing else uses the GPU.
  case "$1" in lift) echo 1024;; *) echo 2048;; esac
}
round_of() { ((ROUNDS)) && echo "/$ROUNDS"; }
warm_start_checkpoint() {   # newest other run's best.json for $skill whose checkpoint still exists
  # (argument "tuning": print that run's tuning.json instead)
  /usr/bin/python3 - "$OFFLINE" "$RUN_ID" "$skill" "${1:-checkpoint}" <<'PY'
import json, sys
from pathlib import Path
offline, run, skill, what = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
for best in sorted(offline.glob(f"*/{skill}/best.json"), key=lambda p: p.stat().st_mtime, reverse=True):
    if best.parent.parent.name == run:
        continue
    checkpoint = json.loads(best.read_text()).get("checkpoint", "")
    if Path(checkpoint).is_file():
        print(best.parent / "tuning.json" if what == "tuning" else checkpoint)
        break
PY
}
best_checkpoint() {   # auto_tune.py's best round so far for $skill
  ((AUTO_TUNE)) || return 0
  /usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["checkpoint"])' \
    "$STATE/$skill/best.json" 2>/dev/null || true
}
benchmark_passed() {
  /usr/bin/python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("passed_benchmark") else 1)' "$1"
}

say "Skills: ${skills[*]} | rounds of $ITERATIONS iterations $( ((ROUNDS)) && echo "(at most $ROUNDS)" || echo "until passed")$( ((PATIENCE)) && echo ", stop after $PATIENCE without improvement") | success = $REQUIRED_PASSES passing evaluations | auto-tune $( ((AUTO_TUNE)) && echo on || echo off) | envs: $(for k in "${skills[@]}"; do printf '%s=%s ' "$k" "$(skill_envs "$k")"; done)| caps: GPU ${HRUH_GPU_MEM_GB} GB"
say "Run $RUN_ID | logs and reports: $STATE"

# ------------------------------------------------------------------ stages
# train_round: one training round with automatic retries. Returns 0 when trained.
train_round() {
  local dir="$STATE/$skill" log="$STATE/$skill/train_$round.log" attempt status envs checkpoint
  for ((attempt = 1; attempt <= RETRIES + 1; attempt++)); do
    envs="$(cat "$dir/num_envs" 2>/dev/null || skill_envs "$skill")"
    # resume the best round so far (auto-tune); otherwise the newest checkpoint
    checkpoint="$(best_checkpoint)"
    [[ -f "$checkpoint" ]] || checkpoint="$(latest_checkpoint "$ROOT/logs/rsl_rl/$experiment")"
    # an interrupted round continues from its *own* newest save with the iterations it still needs
    local iterations="$ITERATIONS" partial start done_its
    partial="$(latest_checkpoint "$ROOT/logs/rsl_rl/$experiment")"
    start="$(cat "$dir/round_${round}_start" 2>/dev/null || true)"
    if [[ -n "$start" && -f "$partial" && "$(basename "$(dirname "$partial")")" == *_round_"$round" ]]; then
      done_its=$(( $(basename "$partial" .pt | tr -dc 0-9) - start + 1 ))
      if (( done_its > 0 )); then
        checkpoint="$partial" iterations=$(( ITERATIONS - done_its ))
        (( iterations >= 1 )) || iterations=1
        say "Continuing interrupted round $round: $done_its of $ITERATIONS iterations done"
      fi
    elif [[ -z "$start" ]]; then
      # first launch of this round: remember its first global iteration (checkpoints are model_<iteration>)
      if [[ -f "$checkpoint" ]]; then start=$(( $(basename "$checkpoint" .pt | tr -dc 0-9) + 1 )); else start=0; fi
      echo "$start" > "$dir/round_${round}_start"
    fi
    # new experiment (code changed): start from the best policy of the previous run of this
    # skill. Only on the first attempt - if the network no longer fits, retry from scratch.
    if [[ ! -f "$checkpoint" && "$WARM_START" == 1 && "$attempt" == 1 ]]; then
      checkpoint="$(warm_start_checkpoint)"
      if [[ -n "$checkpoint" ]]; then
        say "Warm start: continuing from the previous run's best $skill policy"
        # ...with the settings the auto-tuner had reached there (unless this run has its own)
        local previous_tuning
        previous_tuning="$(warm_start_checkpoint tuning)"
        if [[ -f "$previous_tuning" && ! -f "$dir/tuning.json" ]]; then
          cp "$previous_tuning" "$dir/tuning.json"
          say "Warm start: inherited the auto-tuned settings of $(basename "$(dirname "$(dirname "$previous_tuning")")")"
        fi
      fi
    fi
    local resume=()
    [[ -z "$checkpoint" ]] || resume=(--checkpoint "$checkpoint")
    say "Training $skill round $round$(round_of): $iterations iterations, $envs envs$([[ -n "$checkpoint" ]] && echo ", resuming $(basename "$(dirname "$checkpoint")")/$(basename "$checkpoint")")$([[ -s "$dir/tuning.json" && "$(cat "$dir/tuning.json")" != "{}" ]] && echo ", auto-tuned settings")"
    run_stage train "[$skill $round$(round_of)]" "$log" "$iterations" \
      env HRUH_TUNING="$( ((AUTO_TUNE)) && echo "$dir/tuning.json")" \
      "$LIMIT" "$PY" src/hruh_isaac/scripts/rl.py train --task "$task" --visualizer none \
      --num_envs "$envs" --max_iterations "$iterations" --logger tensorboard \
      --experiment_name "$experiment" --run_name "round_$round" "${resume[@]}"
    status=$?
    stop_if_interrupted
    checkpoint="$(latest_checkpoint "$ROOT/logs/rsl_rl/$experiment")"
    # success = clean exit, the trainer reached its end, and a checkpoint exists
    if [[ "$status" == 0 && -f "$checkpoint" ]] && stage_output "$log" | grep -aq "Training time:"; then
      printf '%s\n' "$checkpoint" > "$dir/trained_$round"
      return 0
    fi
    say "Training attempt $attempt/$((RETRIES + 1)) failed (exit $status)"
    show_failure "$log"
    if resource_failure "$log" "$status" && ((envs > 16)); then
      echo $((envs / 2)) > "$dir/num_envs"
      say "  out of GPU/RAM budget: retrying with $((envs / 2)) environments"
    fi
  done
  return 1
}

# evaluate_round: held-out evaluation + export with retries. Returns 0 when the report exists.
evaluator_for_skill() {
  if [[ "$skill" == motion || "$skill" == flat || "$skill" == rough ]]; then
    evaluator=(src/hruh_isaac/scripts/evaluate.py --terrain "$skill")
  else
    evaluator=(src/hruh_isaac/scripts/evaluate_manipulation.py --skill "$skill")
  fi
}

evaluate_round() {
  local log="$STATE/$skill/evaluate_$round.log" attempt status evaluator
  evaluator_for_skill
  for ((attempt = 1; attempt <= RETRIES + 1; attempt++)); do
    say "Evaluating $skill round $round on held-out seeds and exporting the policy"
    rm -f "$report"
    run_stage eval "[$skill $round$(round_of) eval]" "$log" 0 \
      "$LIMIT" "$PY" "${evaluator[@]}" --checkpoint "$checkpoint" --num_envs "$EVAL_ENVS" \
      --visualizer none --export --output "$report"
    status=$?
    stop_if_interrupted
    if [[ "$status" == 0 && -f "$report" && -f "$(dirname "$checkpoint")/exported/bundle.json" ]]; then
      return 0
    fi
    say "Evaluation attempt $attempt/$((RETRIES + 1)) failed (exit $status)"
    show_failure "$log"
  done
  rm -f "$report"
  return 1
}

# confirm_round: the round's checkpoint must also pass REQUIRED_PASSES-1 more evaluations on
# new seeds (4242 + 1000 k). Returns 0 when all pass. On a failing confirmation it returns 1
# with CONFIRM_FAILED=<report>; if an evaluation keeps crashing, CONFIRM_ERROR=1.
confirm_round() {
  local k file attempt status evaluator log="$STATE/$skill/confirm_$round.log"
  CONFIRM_FAILED="" CONFIRM_ERROR="" CONFIRM_PASSES=0
  evaluator_for_skill
  for ((k = 2; k <= REQUIRED_PASSES; k++)); do
    file="$STATE/$skill/confirm_${round}_$k.json"
    attempt=0
    while [[ ! -f "$file" ]] && ((++attempt <= RETRIES + 1)); do
      say "Confirmation $k/$REQUIRED_PASSES for $skill round $round (new seed $((4242 + 1000 * (k - 1))))"
      run_stage eval "[$skill $round check $k/$REQUIRED_PASSES]" "$log" 0 \
        "$LIMIT" "$PY" "${evaluator[@]}" --checkpoint "$checkpoint" --num_envs "$EVAL_ENVS" \
        --seed "$((4242 + 1000 * (k - 1)))" --visualizer none --output "$file"
      status=$?
      stop_if_interrupted
      [[ "$status" == 0 && -f "$file" ]] && break
      rm -f "$file"
      say "Confirmation attempt $attempt/$((RETRIES + 1)) failed (exit $status)"
      show_failure "$log"
    done
    if [[ ! -f "$file" ]]; then
      CONFIRM_ERROR=1
      return 1
    fi
    if benchmark_passed "$file"; then
      say "  confirmation $k/$REQUIRED_PASSES: PASS"
    else
      say "  confirmation $k/$REQUIRED_PASSES: FAIL - $((k - 1))/$REQUIRED_PASSES passed, not reliable yet"
      CONFIRM_FAILED="$file" CONFIRM_PASSES=$((k - 1))
      return 1
    fi
  done
  return 0
}

gazebo_test() {
  local log="$STATE/$skill/gazebo.log" generated
  heading "$skill: Gazebo transfer test (${GAZEBO_SECONDS} s, different physics engine)"
  export ROS_DOMAIN_ID="${HRUH_ROS_DOMAIN_ID:-73}" GZ_PARTITION="hruh_${RUN_ID}_$$"
  generated="$ROOT/artifacts/hruh/gazebo/$(basename "$(dirname "$checkpoint")")/report.json"
  # A failed retry must not reuse a previous run's report.
  if [[ -f "$generated" ]]; then mv "$generated" "$generated.previous"; fi
  rm -f "$STATE/$skill/gazebo.json"
  # --foreground: Ctrl+C reaches ros2 launch. Launched from source: no rebuild required.
  run_stage gazebo "[$skill gazebo]" "$log" 0 \
    timeout --foreground --signal=INT --kill-after=30s "$((GAZEBO_SECONDS * 20 + 150))s" \
    "$LIMIT" ros2 launch "$ROOT/src/hruh_bringup/launch/policy_gazebo.launch.py" \
    workspace:="$ROOT" bundle:="$bundle" python:="$PY" gui:=false seconds:="$GAZEBO_SECONDS" \
    scripted:="$( [[ "$skill" == motion || "$skill" == flat || "$skill" == rough ]] && echo true || echo false)" \
    || say "  Gazebo exited unsuccessfully; see $log"
  # gz sim's server can outlive its launcher
  pkill -f "^gz sim .*artifacts/hruh/gazebo/" 2>/dev/null || true
  stop_if_interrupted
  if [[ -f "$generated" ]]; then
    cp "$generated" "$STATE/$skill/gazebo.json"
  else
    say "  Gazebo produced no report: transfer NOT verified"
  fi
  printf '%s\n' "$checkpoint" > "$STATE/$skill/gazebo_checkpoint.txt"
}

promote_skill() {
  local promote=(src/hruh_isaac/scripts/promote_policy.py --bundle "$bundle" --evaluation "$report" --run-id "$RUN_ID"
                 --dest "${POLICY_DIR:-$ROOT/src/hruh_isaac/policies}")
  [[ ! -f "$STATE/$skill/gazebo.json" ]] || promote+=(--gazebo "$STATE/$skill/gazebo.json")
  [[ "$REQUIRE_GAZEBO" == 0 || ( "$skill" != motion && "$skill" != flat && "$skill" != rough ) ]] || promote+=(--require-gazebo)
  local result
  result="$(/usr/bin/python3 "${promote[@]}" 2>&1)" || true
  printf '%s\n' "$result" > "$STATE/$skill/promotion.txt"
  printf '%s\n' "$checkpoint" > "$STATE/$skill/promotion_checkpoint.txt"
  say "Runtime: $result"
}

# ------------------------------------------------------------------ main loop
failed_skills=()
for index in "${!skills[@]}"; do
  skill="${skills[$index]}"
  case "$skill" in
    motion) task=Hruh-Velocity-Motion-v0;; flat) task=Hruh-Velocity-Flat-v0;; rough) task=Hruh-Velocity-Rough-v0;;
    reach) task=Hruh-Reach-Right-v0;; lift) task=Hruh-Lift-Right-v0;;
  esac
  experiment="hruh_${RUN_ID}_${skill}"
  mkdir -p "$STATE/$skill"
  rm -f "$STATE/$skill/failed.txt"
  heading "Skill $((index + 1))/${#skills[@]}: $skill ($task)"
  report="" checkpoint="" skill_ok=1 confirmed=0
  rm -f "$STATE/$skill/outcome.txt"
  for ((round = 1; ROUNDS == 0 || round <= ROUNDS; round++)); do
    report="$STATE/$skill/evaluation_$round.json"
    if [[ ! -f "$STATE/$skill/trained_$round" ]]; then
      if ! train_round; then
        printf 'training round %s failed after %s attempts\n' "$round" "$((RETRIES + 1))" > "$STATE/$skill/failed.txt"
        skill_ok=0
        break
      fi
    else
      say "Round $round$(round_of) already trained (resuming)"
    fi
    checkpoint="$(cat "$STATE/$skill/trained_$round")"
    if [[ ! -f "$report" || ! -f "$(dirname "$checkpoint")/exported/bundle.json" ]]; then
      if ! evaluate_round; then
        printf 'evaluation of round %s failed after %s attempts\n' "$round" "$((RETRIES + 1))" > "$STATE/$skill/failed.txt"
        skill_ok=0
        break
      fi
    fi
    printf '%s\n' "$checkpoint" > "$STATE/$skill/latest_checkpoint.txt"
    diagnosis="$report" passes=0
    if benchmark_passed "$report"; then
      say "Isaac benchmark: PASS (round $round)$( ((REQUIRED_PASSES > 1)) && echo " - confirming on $((REQUIRED_PASSES - 1)) more evaluations with new seeds")"
      if confirm_round; then
        say "$skill: SUCCESS - passed $REQUIRED_PASSES/$REQUIRED_PASSES evaluations (round $round)"
        echo "passed $REQUIRED_PASSES/$REQUIRED_PASSES evaluations in round $round" > "$STATE/$skill/outcome.txt"
        confirmed=1
        break
      fi
      if [[ -n "$CONFIRM_ERROR" ]]; then
        printf 'confirmation of round %s failed after %s attempts\n' "$round" "$((RETRIES + 1))" > "$STATE/$skill/failed.txt"
        skill_ok=0
        break
      fi
      diagnosis="$CONFIRM_FAILED" passes="$CONFIRM_PASSES"   # tune against the evaluation it failed
    else
      say "Isaac benchmark: FAIL (round $round)"
    fi
    if ((AUTO_TUNE)); then
      # diagnose this round and adjust the next one (idempotent when resuming)
      while IFS= read -r line; do say "$line"; done < <(/usr/bin/python3 src/hruh_isaac/scripts/auto_tune.py \
        --skill "$skill" --round "$round" --evaluation "$diagnosis" --score-evaluation "$report" --passes "$passes" \
        --train-log "$STATE/$skill/train_$round.log" \
        --checkpoint "$checkpoint" --state-dir "$STATE/$skill" 2>&1)
      # rounds without improvement *as of this round* (correct when replaying a resumed run)
      since="$(/usr/bin/python3 -c 'import json,sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
print(next((r["since_improvement"] for r in rows if r["round"] == int(sys.argv[2])), 0))' \
        "$STATE/$skill/tuning_history.jsonl" "$round" 2>/dev/null || echo 0)"
      if ((PATIENCE && since >= PATIENCE)); then
        say "$skill: no improvement for $since rounds even with automatic tuning - moving on (PATIENCE=$PATIENCE)"
        echo "stopped after round $round: no improvement for $since rounds" > "$STATE/$skill/outcome.txt"
        break
      fi
    fi
    if ((ROUNDS && round >= ROUNDS)); then
      echo "stopped at the round limit ROUNDS=$ROUNDS" > "$STATE/$skill/outcome.txt"
    else
      say "Training another round"
    fi
  done
  # a skill that did not pass is tested / reported with its best round, not its last
  if [[ "$skill_ok" == 1 && "$confirmed" != 1 ]]; then
    best="$(best_checkpoint)"
    if [[ -f "$best" && "$best" != "$checkpoint" ]]; then
      for ((r = 1; r <= round; r++)); do
        if [[ "$(cat "$STATE/$skill/trained_$r" 2>/dev/null)" == "$best" ]]; then
          checkpoint="$best" report="$STATE/$skill/evaluation_$r.json"
          say "$skill: using its best round $r for the Gazebo test and summary"
        fi
      done
    fi
  fi
  if [[ "$skill_ok" == 0 ]]; then
    say "$skill FAILED: $(cat "$STATE/$skill/failed.txt") - continuing with the next skill"
    failed_skills+=("$skill")
    continue
  fi
  bundle="$(dirname "$checkpoint")/exported"
  if [[ "$GAZEBO" == 1 && "$(cat "$STATE/$skill/gazebo_checkpoint.txt" 2>/dev/null)" != "$checkpoint" ]]; then
    gazebo_test
  fi
  # Deploy: a passing policy replaces the robot's runtime policy unless it is worse.
  if [[ "$PROMOTE" == 1 && "$confirmed" == 1 \
        && "$(cat "$STATE/$skill/promotion_checkpoint.txt" 2>/dev/null)" != "$checkpoint" ]]; then
    promote_skill
  fi
done

# ------------------------------------------------------------------ summary
heading "Summary"
/usr/bin/python3 - "$STATE" <<'PY' | tee "$STATE/SUMMARY.txt"
from pathlib import Path
import json, sys
p = Path(sys.argv[1]); rows = []
for s in sorted(p.iterdir()):
    if not s.is_dir(): continue
    failed = s / 'failed.txt'
    if failed.exists():
        rows.append(f"{s.name}: ERROR - {failed.read_text().strip()} (rerun the script to retry)")
    reports = sorted(s.glob('evaluation_*.json'), key=lambda f: int(f.stem.split('_')[-1]))
    if not reports: continue
    r = json.loads(reports[-1].read_text())
    outcome = (s / 'outcome.txt').read_text().strip() if (s / 'outcome.txt').exists() else ''
    rows.append(f"{s.name}: {'SUCCESS' if outcome.startswith('passed') else 'NOT PASSED'}"
                f" after {len(reports)} round(s){'; ' + outcome if outcome else ''}")
    history = s / 'tuning_history.jsonl'
    if history.exists():
        lines = [json.loads(l) for l in history.read_text().splitlines() if l.strip()]
        rows.append(f"  Auto-tune: {sum(len(l['changes']) for l in lines)} adjustment(s) over {len(lines)} round(s); "
                    f"best score {lines[-1]['best_score']:.3f} (round {lines[-1]['best_round']}); see {history.name}")
    g = s / 'gazebo.json'
    if g.exists():
        d = json.loads(g.read_text())
        rows.append(f"  Gazebo: completed={d.get('completed')}, falls={d.get('falls')}, policy_steps={d.get('policy_steps')}; skill success NOT verified")
    else: rows.append('  Gazebo: no report; transfer NOT verified')
    promo = s / 'promotion.txt'
    rows.append('  Runtime: ' + (promo.read_text().strip() if promo.exists() else
                                 'not promoted' + ('' if outcome.startswith('passed') else ' (not passed)')))
rows += ['', 'Promoted policies: src/hruh_isaac/policies/<name>/PROMOTED.json',
         '  ros2 launch hruh_bringup isaac.launch.py  |  policy_gazebo.launch.py joystick:=true',

         'This script does not implement general tool use or guarantee fall-free behavior.']
if any('no improvement' in row for row in rows):
    rows.insert(-1, 'Stopped without improvement: rerun with PATIENCE=10 to keep tuning, or the task needs new work.')
print('\n'.join(rows))
PY
say "Finished in $(elapsed). Reports and checkpoint paths: $STATE"
if ((${#failed_skills[@]})); then
  notify "Finished with errors in: ${failed_skills[*]} ($(elapsed))"
  exit 1
fi
notify "Training finished ($(elapsed)). See SUMMARY.txt"
