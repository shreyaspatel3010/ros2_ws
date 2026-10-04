# HRUH learning in Isaac

This workspace now has executable PPO training tasks for **locomotion, right-arm
reaching, and right-hand cube lifting**. These are separate skills. A checkpoint
is not evidence of a reliable robot: use the evaluation commands below.

The code targets the installation at `/opt/isaac/venv-6.1` and
`/opt/isaac/IsaacLab-3.0` (Isaac Sim 6.1, RSL-RL 5.4). The installed Isaac Lab
checkout uses the new `core` task paths, separate actor/critic models, `.torch`
state views, and `--visualizer none` for headless operation. See NVIDIA's
[Isaac Lab migration guide](https://isaac-sim.github.io/IsaacLab/develop/source/migration/migrating_to_isaaclab_3-0.html).

## Prepare and check

From `~/ros2_ws`:

```bash
bash src/hruh_isaac/scripts/run.sh prepare
bash src/hruh_isaac/scripts/run.sh doctor
python3 -m unittest discover -s src/hruh_isaac/tests -v
```

Set `ISAAC_PYTHON` if the virtual environment moves. `HRUH_URDF` overrides the
generated model path. Meshes resolve through the built ROS description package.
Generated URDF/USD files and evaluation reports live in `artifacts/hruh/`;
training runs live in `logs/rsl_rl/`. These large generated files are ignored by Git.
Run GPU commands in a terminal with NVIDIA device access. Inside the coding
sandbox CUDA may appear unavailable even though it works on the host.

## Train balance and joystick locomotion

```bash
bash src/hruh_isaac/scripts/run.sh train --max_iterations 1500 --run_name balance
```

The actor drives 14 leg joints and three waist joints at 50 Hz. Its input is
five frames of angular velocity, projected gravity, commanded velocity, joint
positions/velocities, and previous actions (300 values). The critic additionally
sees simulated base velocity. Joint/action order is explicit. Targets and motor
efforts are bounded; leg effort/speed limits follow the URDF. Arms and hands stay
in their default poses.

Training varies friction, pelvis mass, center of mass, actuator gains, observation
noise, and horizontal pushes. It includes stopping and randomized forward,
backward, sideways, and turning requests. This is a policy trained over a range
of conditions, not online retraining or a guarantee of recovery.

Resume a run with its actual checkpoint path:

```bash
bash src/hruh_isaac/scripts/run.sh train --checkpoint /absolute/path/to/model_1499.pt --max_iterations 1500
```

`--max_iterations` is the number of additional iterations when resuming. Keep
`params/` alongside checkpoints. Playback verifies joint ordering, action scales,
observation layout, timestep, default pose and actuator settings against it.

### Measure before using a checkpoint

```bash
bash src/hruh_isaac/scripts/run.sh evaluate --zero --output artifacts/hruh/baseline.json
bash src/hruh_isaac/scripts/run.sh evaluate \
  --checkpoint /absolute/path/to/model_1499.pt \
  --num_envs 32 --seconds 20 --seed 4242 \
  --output artifacts/hruh/evaluation.json --export
```

Ten scenarios test standing, walking forward at 0.4 m/s (the minimum required speed)
and fast at 0.6 m/s, reverse and side-step at 0.3 m/s, turning at 0.8 rad/s,
stop/reverse transitions, **sudden stops** (full speed forward / sideways / turning /
backward, each cut instantly to zero), and two controlled pushes.

**The robot must not fall.** The gate requires **zero falls** in every scenario. After
every stop command, the robot must be standing still (< 0.1 m/s, < 0.2 rad/s) within 2 s.
In the stop scenarios, the 1 s after each command step counts toward that settle time
rather than tracking error. Training practises this: a robot moving faster than
0.3 m/s or 0.5 rad/s gets an instant stop as its next command 30% of the time. A trial ends at its first
fall. Automatic simulator resets do not count as successful recovery. Reports
include survival, time to fall, and command error while upright. The benchmark
passes only if **every** scenario has no falls, linear velocity MAE
at most 0.15 m/s per axis, and yaw-rate MAE at most 0.25 rad/s. Repeat with other
seeds and longer trials before making stronger claims.

`--export` writes normalized TorchScript/ONNX actors plus `contract.json` next to
the checkpoint. These exports have not been validated for hardware deployment.

### Joystick in Isaac

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hruh_teleop joystick.launch.py use_sim_time:=false
```

Terminal 2:

```bash
bash src/hruh_isaac/scripts/run.sh joystick --checkpoint /absolute/path/to/model_1499.pt
```

Hold **LB**, use the left stick to move and the right stick to turn. Playback
subscribes only to `/cmd_vel`; it does not send actuator commands to ROS. It runs
the same Isaac environment and policy history used in training. Command limits
and acceleration ramps prevent requests outside the training envelope. Commands
older than 0.35 s request zero velocity while the balance policy continues.
After a fall, the simulation resets and motion stays latched until a fresh neutral
command arrives. This reset is not a learned get-up behavior.

Full stick requests the trained range: 0.8 m/s forward, 0.4 m/s backward and sideways,
1.0 rad/s turning. The ZMP walker clamps the same commands to its own limit (0.2 m/s in
Gazebo and Isaac). Arm/MoveIt buttons are not connected to the locomotion policy. Do not
run the old walker or a separate Isaac ROS bridge as a second actuator source.

### Learning methods (walking skills)

| Method | Kind | Where |
|---|---|---|
| PPO, asymmetric actor-critic: the critic also sees simulator truth (pelvis velocity) | Deep RL | `tasks/locomotion/agents.py` |
| 2048 robots simulated in parallel on the GPU, domain randomization (friction, mass, centre of mass, motor gains, sensor noise, pushes) | RL at scale | `train_robot_offline.sh`, `env_cfg.py` |
| **Left–right symmetry augmentation**: every sample is also learned mirrored. This doubles the data and gives a symmetric, human-like gait. The mirror is derived from the URDF joint axes and verified by forward kinematics (`tests/test_symmetry.py`) | Data augmentation (Mittal et al. 2024) | `hruh_lab/symmetry.py` |
| **Concurrent velocity estimator**: a network inside the actor learns, by supervised regression on simulator truth, to predict the pelvis velocity from the IMU and joint history. The policy uses the estimate | Supervised DL (Ji et al. 2022) | `hruh_lab/estimator.py` |
| Curricula (movement modes, sudden stops, arm motion, pushes) and automatic tuning between rounds | Training curriculum | `commands.py`, `scripts/auto_tune.py` |

Actor and critic are [512, 256, 128] ELU networks. The exported `policy.pt` / `policy.onnx`
contain the estimator, so deployment (`policy_runner.py`, Gazebo) is unchanged. Reach and
lift use plain PPO: one arm, so no left–right symmetry or pelvis velocity applies.
Measured GPU use on this laptop: 512 robots 2.9 GB, 1024 robots 3.4 GB at twice the
speed; 2048 robots (default) is about 4.5 GB, inside the 8 GB budget.

### Whole-body movement (`Hruh-Velocity-Motion-v0`, skill `motion`)

This is the default walking skill of `train_robot_offline.sh`. It trains the same
legs + waist policy on a broader set of movements.

**Movements.** Each episode switches every 2–5 s between:

| Movement | Share |
|---|---|
| Stand | 15% |
| Walk forward | 25% |
| Walk backward | 10% |
| Side-step left/right | 20% |
| Turn in place | 10% |
| Free combinations | 20% |

The switches add many starts, stops and direction changes. A stand-still penalty
makes the legs return to the stand pose when stopped.

**Arms while moving.** Every 1.5–4 s the arms change mode:

| Mode | Share | What the arms do |
|---|---|---|
| Hold | 25% | Stay in the stand pose |
| Swing | 45% | Swing with the gait like a person: opposite arm forward, 0.15–0.45 rad |
| Pose | 30% | Each arm independently moves to random reach / wave / carry poses at up to 2 rad/s, like MoveIt or the gamepad would |

The arms are not policy outputs. The policy **observes** the 10 arm joints, so it learns
to keep its balance whatever the arms do.

**Speeds.** Training covers −0.4…0.8 m/s forward/backward, ±0.4 m/s sideways and
±1.0 rad/s turning. Forward walking is practised at 0.2–0.8 m/s, so the robot is never
limited below 0.4 m/s.

**Evaluation** runs the nine standard scenarios with swinging arms, plus
`stand_arms`, `forward_arms`, `sidestep_arms` and `turn_arms` with arms moving to
random poses. The same pass gate applies to every scenario.

**Deployment.** In `isaac.launch.py` the runner adds the same counter-swing while
walking (`joints.arm_swing_targets`, identical to training) and returns the arms to the
stand pose when stopping. While standing, the arms belong to MoveIt and the gamepad.
`arm_swing:=false` keeps MoveIt in control of the arms while walking too.

```bash
bash src/hruh_isaac/scripts/run.sh train --task Hruh-Velocity-Motion-v0 --num_envs 128 --max_iterations 3000
bash src/hruh_isaac/scripts/run.sh evaluate --terrain motion --checkpoint /abs/path/model_2999.pt --export
```

### Rough terrain stage

```bash
bash src/hruh_isaac/scripts/run.sh train --task Hruh-Velocity-Rough-v0 \
  --checkpoint /absolute/path/to/flat/model_1499.pt --max_iterations 3000
bash src/hruh_isaac/scripts/run.sh evaluate --terrain rough \
  --checkpoint /absolute/path/to/rough/model_4499.pt --output artifacts/hruh/rough.json
```

Flat and rough tasks share actor/critic interfaces. The rough curriculum includes
2–10 cm steps, shallow slopes and uneven ground. Its height scanner is disabled:
this is a proprioceptive policy and cannot see upcoming holes or obstacles.

## Train arm and hand skills

```bash
bash src/hruh_isaac/scripts/run.sh train --task Hruh-Reach-Right-v0 \
  --num_envs 128 --max_iterations 1000 --run_name reach
bash src/hruh_isaac/scripts/run.sh train --task Hruh-Lift-Right-v0 \
  --num_envs 128 --max_iterations 2500 --run_name grasp
```

These tasks use a **fixed pelvis**. Reaching controls the five right-arm joints
and samples wrist-position targets in a limited workspace. Success means coming
within 2.5 cm; wrist orientation is unconstrained. Cube lifting controls those
five arm joints plus the three thumb joints and four finger-base joints. Finger
mimic joints follow their parents with independent drives disabled. The 4.5 cm,
80 g cube has randomized initial position/yaw and mass.

Grasp success requires lifting the cube 10 cm while maintaining thumb and finger
contact for 0.5 seconds, keeping it close to the hand, with speed below 0.3 m/s.
The policy receives simulated object pose and velocity. No vision model is
trained by this task.

```bash
/opt/isaac/venv-6.1/bin/python src/hruh_isaac/scripts/evaluate_manipulation.py \
  --skill reach --checkpoint /absolute/path/to/reach/model_999.pt \
  --visualizer none --output artifacts/hruh/reach_evaluation.json
/opt/isaac/venv-6.1/bin/python src/hruh_isaac/scripts/evaluate_manipulation.py \
  --skill lift --checkpoint /absolute/path/to/lift/model_2499.pt \
  --visualizer none --output artifacts/hruh/lift_evaluation.json
```

Use `--zero` in place of `--checkpoint` for baselines. Default evaluation runs
128 trials across four seeds and checks that cloned fixed-pelvis robots stay at
their expected positions. A 90% success fraction passes the initial benchmark.
For a visual playback, use `run.sh play --task Hruh-Reach-Right-v0 --checkpoint ...
--num_envs 1 --visualizer kit` (substitute `Lift` for grasping).

## Scope and remaining work

| Requested capability | Current implementation | Evidence still needed |
|---|---|---|
| Balance during joystick motion | PPO training, watchdog, simulation playback, fall/tracking tests | Passing results across speeds, seeds, long trials and terrain |
| Adaptive walking | Friction/mass/gain/noise/push randomization and rough task | Rough-terrain evaluation, latency and actuator-model variation |
| Reach an object | Right-arm position task | Held-out success and collision-aware reach planning |
| Pick up an object | Right-hand cube lift task with object-contact checks | Reliable grasps over different objects and orientations |
| Use a screwdriver | Not implemented | Tool grasp, shaft alignment, slot contact, torque limits and screw/thread model |
| Write with a pen | Not implemented | Pen grasp/calibration, surface contact-force control and path tracking |
| Change a bulb | Not implemented | Bimanual support, socket/thread contact, insertion and twist validation |
| Maintenance/complex jobs | Not implemented | Perception, task planning, recovery and individually validated skills |
| Work while walking | Not implemented | Coupled locomotion/manipulation training with arm-motion observations |
| Recover after falling | Not implemented | Dedicated get-up task and evaluation |

The arm has five axes and cannot independently control an arbitrary six-component
hand pose. Additional wrist articulation may be needed for the intended physical
design. Current self-collision is disabled, so passing a task does not establish
collision-free full-body motion. The motor gains and hand/arm limits remain
simulation assumptions; hardware transfer requires measured actuators, inertia,
latency, encoders, force/contact feedback, and a separate deployment review.

The existing `tasks/imitation/` files are unfinished older scaffolding and are not
registered training tasks. They do not contain human demonstrations or a pretrained
human-motion policy. No code here supplies general human-level task competence.
## Review fixes (2026-10-03)

Findings from evaluating `balance_v1/model_1499.pt` and the reach task:

- **Pelvis yaw wobble.** Every scenario, even standing, had ~0.66 rad/s yaw-rate MAE while
  the signed mean was only ~+0.03 rad/s and heading drifted ~0.4 rad in 10 s: the policy
  learned a twisting shuffle, not a spin. `evaluate.py` now reports
  `yaw_rate_error_signed_mean` and `heading_change_rad_mean_abs` to tell the two apart
  (the benchmark gate is unchanged). Training now adds a quadratic yaw-rate error
  penalty (`hruh_lab.mdp.yaw_rate_error_l2`, -0.3), lowers the `alive` bonus 1.0 -> 0.25
  (survival alone was enough reward to stop walking) and raises linear velocity
  tracking 1.0 -> 1.5.
- **Short strides.** `clip_actions` 1.0 with action scale 0.5 limited every joint to
  +-0.5 rad around the stand pose (knee <= ~0.86 rad). It is now 2.0 (+-1 rad); targets
  are still clipped to the URDF joint limits.
- **Unreachable reach targets.** Forward-kinematics sampling over the arm limits showed
  only 86% of the old target box was reachable, so the 90% gate was impossible. The box
  is now x 0.22-0.34, y -0.36..-0.22, z 0.16-0.32 m (99.8% reachable within 1.5 cm).

- **Action clip mismatch.** The export wrote `raw_action_clip: 1.0` and the manipulation
  evaluator clipped at 1.0, while training clips at `clip_actions` (now 2.0). Both now use
  the training value.
- **Gazebo transfer test.**
  - It started the world before the effort controller was active, so the robot collapsed
    first. A detachable holder now keeps the pelvis at the reset height until the stand
    pose is held.
  - It detected falls from odometry z, which is relative to the spawn pose. Falls now come
    from IMU tilt plus a kinematic pelvis height.
  - Its Python loop ran the 50 Hz policy at about 23 Hz. It now steps on each joint-state
    message and measures 50 Hz.

### After the first full offline run (auto_5ae95cbb56, 2026-10-03)

All three skills failed their benchmarks. The pipeline itself ran cleanly.

| Skill | Result | Cause | Fix |
|---|---|---|---|
| Motion | Survived, but stood and shuffled: forward error 0.28 of a 0.30 m/s command; turning 0.1 of 0.4 rad/s; the step reward earned 0.001 | Velocity-tracking std 0.5 is meant for H1's 1 m/s commands. At HRUH's ≤ 0.4 m/s, standing still still earned ~70% of the tracking reward | Tracking std 0.25 (walking tasks); `feet_air_time` weight 1.0, threshold 0.35 s |
| Reach | 12–16% success; the hand stalled ~3 cm from the target (gate 2.5 cm) | Exploration noise stayed at ~0.6 (entropy 0.01); `tanh(d / 0.08)` is nearly flat at 2–3 cm | `entropy_coef` 0.001, initial std 0.3; extra `tanh(d / 0.02)` reward (weight 2) |
| Lift | 0%; the hand reached the cube but never touched it with thumb + finger | Noise grew to 1.15 (random fingers) | `entropy_coef` 0.002; shaping rewards `touch` (partial contact) and `close_when_near` (flex the fingers within 6 cm), grasp contact weight 2 |
| All | — | 128 environments gave ~3 M steps per round, 20–50× fewer than Isaac Lab's reference recipes | 512 parallel environments (lift 256). Halved automatically if the 8 GB GPU budget is exceeded |

These change the policy contract, so earlier checkpoints are rejected; the offline
script starts a new experiment automatically when the code changes.

# Offline automation

From the workspace root, run `./train_robot_offline.sh`. Use `--help` for options
or `--check` for installed prerequisites. No API tokens, downloads or cloud logging.
For each skill (whole-body movement `motion`, right-arm reaching, right-hand cube
lifting; `flat` and `rough` are optional) it:

1. **Trains** PPO in Isaac in rounds of 1,000 iterations, with 2048 robots simulated
   in parallel in one GPU scene (lift 1024). Rounds repeat **until the benchmark passes**.
2. **Evaluates** each round on held-out seeds and **exports** it (`policy.pt`/`.onnx`,
   `bundle.json`).
   - **If the round failed, the next round is tuned automatically** (`scripts/auto_tune.py`).
     It diagnoses the evaluation and the end of the training log, then adjusts reward
     weights and strengths, the movement / arm / push curriculum and the exploration noise.
     These are fixed rules with hard limits; each change is printed with its reason and
     written to `<run>/<skill>/tuning_history.jsonl`. The overrides
     (`<run>/<skill>/tuning.json`, read through `HRUH_TUNING`) apply to training only;
     evaluation always measures the task as written in the source.
   - **The next round resumes from the best checkpoint so far,** so a worse round never
     replaces progress.
   - **Success means 5 passes.** A round that passes is re-evaluated 4 more times with new
     random seeds (`confirm_<round>_<k>.json`). Only when the same policy passes all 5 is the
     skill finished and the policy promoted. A failed confirmation goes back to the tuner
     like any other failure, and training continues. **The run does not move on to the next
     skill until this happens.** `REQUIRED_PASSES=N` changes the count; `PATIENCE=N` allows
     moving on after N rounds without improvement (default 0 = never); `ROUNDS=N` caps the
     rounds; `AUTO_TUNE=0` keeps the settings fixed.
   - **What tuning cannot do:** it never changes observations, actions or actuators, which
     would make the saved network unusable. It cannot write new task code either. If every
     relevant setting is at its limit, the history says so, and the skill needs new
     development.
3. **Tests transfer to Gazebo** (effort control, different physics engine). Walking skills
   walk every movement by themselves: forward, side-step, turn and backward, each followed
   by a sudden stop. The test reports falls (and during which movement) and the measured
   speed per movement. It is a diagnostic; `REQUIRE_GAZEBO=1` makes a fall-free Gazebo
   test a promotion requirement.
4. **Promotes** a policy that passed 5/5 evaluations into the robot's runtime (see below).
   A failed or unconfirmed policy is never promoted. With `REQUIRE_GAZEBO=1`, walking must also finish the Gazebo test
   without a fall.

Everything is automatic:

- **Live progress.** The terminal shows a live bar per training round (iteration, reward,
  task error, steps/s, ETA), every evaluation scenario as it finishes, the Gazebo
  result and the summary. Without a terminal (`nohup`) it prints a line every 5%.
  `tail -f artifacts/hruh/offline/progress.txt` follows it from elsewhere.
- **Build.** The ROS workspace is built first (`BUILD=0` skips it).
- **No sleep.** The laptop is kept from suspending until the run ends.
- **Retries.** A crashed stage is retried (`RETRIES=2`) from the latest checkpoint. If it
  ran out of the GPU/RAM budget, the retry uses half the environments.
- **Failures.** A skill that keeps failing is reported, and the run continues with the next skill.

Expect several hours on this laptop. Logs, reports and checkpoint paths go to `artifacts/hruh/offline/<run>/`; read
`SUMMARY.txt` afterwards. The run is named after the robot/task code (`auto_<fingerprint>`):
rerunning with unchanged code resumes it (after Ctrl+C, a crash or a reboot), and any change
to `hruh_lab` or the robot model automatically starts a fresh experiment, so incompatible
weights are never resumed. Old runs are kept. A fixed `RUN_ID=<name>` is still accepted.

Example: `NUM_ENVS=64 SKILLS="flat reach" ./train_robot_offline.sh`.
Optional rough terrain: `SKILLS=rough ./train_robot_offline.sh`.

## Resource caps (laptop safety)

Every Isaac / Gazebo process started by `run.sh`, `train_robot_offline.sh` and
`isaac.launch.py` goes through `scripts/limit.sh`. It runs in its own systemd user cgroup:

| Resource | Limit | Default on this laptop |
|---|---|---|
| CPU | `CPUQuota`, low `CPUWeight`, `nice 10` | 12 of 20 threads (`HRUH_CPU_CORES`) |
| RAM | `MemoryMax`, no swap | 18 GB (`HRUH_MEM_GB`) |
| GPU memory | `hruh_lab/gpu_guard.py` stops the process cleanly | 8 GB (`HRUH_GPU_MEM_GB`) |

If a run exceeds these limits, **only that run is stopped**; the desktop, VS Code and the
browser keep running. `HRUH_LIMITS=0` disables the caps.

## Using trained policies in the robot

Promoted policies live in [`policies/`](policies/README.md) with a `PROMOTED.json`
manifest; earlier ones are backed up to `artifacts/hruh/policy_history/`. The runtime
loads them automatically:

```bash
# Isaac + ros2_control + MoveIt + RViz + gamepad, legs and waist driven by the learned policy
ros2 launch hruh_bringup isaac.launch.py            # controller:=auto -> policy if promoted
ros2 launch hruh_bringup isaac.launch.py controller:=walker   # the ZMP walker instead
# learned reaching (also loaded automatically when promoted): give the right hand a goal
ros2 topic pub --once /hruh/hand_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.28, y: -0.29, z: 0.24}}}"
# Gazebo, effort control, with the gamepad
ros2 launch hruh_bringup policy_gazebo.launch.py joystick:=true
```

How the Isaac policy mode works (`scripts/policy_runner.py`):

- **Balance guard.** If the robot starts to tip (tilt > 20° or pelvis < 0.65 m), it
  stops moving at once and only balances: an instant stop, as practised in training.
  It ignores `/cmd_vel` until it has been upright for 1 s.

- **Gains.** Isaac uses the training actuator gains (`gains:=rl`).
- **Start-up.** The pelvis is held at the training reset height. The runner moves the
  robot into the trained stand pose; the sim releases the pelvis once that pose is
  commanded, and the policy takes over.
- **Walking.** The policy streams leg and waist targets at 50 Hz through
  `legs_controller` and `waist_position_controller`, from `/joint_states`, `/imu` and
  `/cmd_vel`. Hold **LB** and use the sticks to walk.
- **Arms.** MoveIt and the gamepad keep the arms, hands and head.
- **Falls.** A fall (tilt > 1 rad or pelvis < 0.45 m, the training terminations) stops
  the policy. Status messages are published on `/hruh_policy/status`.
- **Reaching.** Goals are clamped to the trained workspace (x 0.22–0.34, y −0.36…−0.22,
  z 0.16–0.32 m, pelvis frame). The policy runs for 4 s, then reports the wrist error.

Known gaps: ros2_control adds about one frame (~17 ms) of sensor/command latency that
training did not model. Arm motion while walking was not trained. The waist belongs to the
policy, so MoveIt's `waist` group and the gamepad's waist jog do nothing in this mode.
Policies are simulation-only; hardware needs measured actuators, latency and a separate
safety review. Screwdriver use, writing, bulb changing, vision and whole-body manipulation
still need dedicated tasks.
