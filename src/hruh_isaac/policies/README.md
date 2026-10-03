# Promoted policies

`train_robot_offline.sh` copies a trained policy here only after it **passed its Isaac
benchmark** and was at least as good as the policy already here. These are the policies the
robot's runtime loads:

| Folder | Used by |
|---|---|
| `locomotion/` | `ros2 launch hruh_bringup isaac.launch.py` (`controller:=auto` picks it up), `policy_gazebo.launch.py` |
| `reach/` | `isaac.launch.py` (`reach:=auto`): send a `geometry_msgs/PoseStamped` (frame `base_link`) to `/hruh/hand_target` |
| `lift/` | simulation only: the policy needs the cube's ground-truth pose |

Each folder holds `policy.pt` / `policy.onnx` (TorchScript / ONNX actor with its
observation normalizer), `bundle.json` (observation / action contract), `robot.urdf`,
the `evaluation.json` that passed, the Gazebo report when one exists, the training
`params/`, and `PROMOTED.json` (run, checksums, score, Gazebo transfer status).
Older policies are kept in `artifacts/hruh/policy_history/<name>/`.

Promote by hand:
`bash src/hruh_isaac/scripts/run.sh promote --bundle <run>/exported --evaluation <report.json>`.
To go back to an older policy, copy its folder from `policy_history` over the current one.
These policies are trained and checked in simulation only; they are not hardware controllers.
