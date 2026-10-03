#!/usr/bin/env python3
"""Promote a trained policy that passed its benchmark into the robot's runtime.

    promote_policy.py --bundle <run>/exported --evaluation evaluation.json
                      [--gazebo gazebo.json] [--require-gazebo] [--run-id ID]

Promotion copies the exported actor to src/hruh_isaac/policies/<name>/, the
folder that the runtime launch files load by default:

    locomotion  isaac.launch.py controller:=policy, policy_gazebo.launch.py
    reach       isaac.launch.py reach:=true  (right-arm reaching on /hruh/hand_target)
    lift        simulation only (needs the cube's ground-truth pose)

A policy is promoted only when
  * its Isaac evaluation passed the benchmark (a FAIL is never promoted),
  * the policy / robot checksums in the bundle are intact and the robot model is
    the current one (artifacts/hruh/hruh_isaac.urdf),
  * with --require-gazebo, the Gazebo transfer test completed without a fall, and
  * it is not worse than the policy already promoted for the same robot.
The previous policy is moved to artifacts/hruh/policy_history/<name>/<time>/.
No Isaac / ROS imports: runs with the system python3.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[3]
POLICIES = ROOT / "src/hruh_isaac/policies"
HISTORY = ROOT / "artifacts/hruh/policy_history"
DEPLOY_NAME = {"locomotion": "locomotion", "reach": "reach", "lift": "lift"}
FILES = ["policy.pt", "policy.onnx", "policy.onnx.data", "bundle.json", "robot.urdf", "contract.json"]
KEEP_HISTORY = 5


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def score(skill, evaluation):
    """Higher is better.  Locomotion: worst-scenario survival, then tracking error."""
    if skill == "locomotion":
        rows = evaluation["results"]
        survival = min(r["survival_fraction"] for r in rows)
        # mean of vx / vy MAE (m/s) and half the yaw-rate MAE (rad/s) over scenarios
        error = sum(r["velocity_mae_while_alive_vx_vy_wz"][0] + r["velocity_mae_while_alive_vx_vy_wz"][1]
                    + 0.5 * r["velocity_mae_while_alive_vx_vy_wz"][2] for r in rows) / len(rows)
        # survival dominates: 1 point of worst-case survival outweighs any passing tracking error
        return round(10.0 * survival - error, 6)
    return round(float(evaluation["success_fraction"]), 6)


def gazebo_status(path):
    if not path or not Path(path).is_file():
        return "untested", None
    report = json.loads(Path(path).read_text())
    ok = report.get("completed") is True and report.get("falls", 1) == 0 and report.get("policy_steps", 0) > 0
    return ("pass" if ok else "fail"), report


def promote(bundle, evaluation_path, gazebo=None, require_gazebo=False, run_id=None, dest_root=POLICIES,
            history_root=HISTORY, current_urdf=None):
    """Returns (promoted: bool, message)."""
    bundle = Path(bundle).resolve()
    contract = json.loads((bundle / "bundle.json").read_text())
    evaluation = json.loads(Path(evaluation_path).read_text())
    skill = contract["skill"]
    if skill not in DEPLOY_NAME:
        return False, f"unknown skill {skill!r}"
    if not evaluation.get("passed_benchmark"):
        return False, f"{skill}: Isaac benchmark FAIL, not promoted"
    if sha256(bundle / "policy.pt") != contract["policy_sha256"]:
        return False, f"{skill}: policy.pt checksum differs from bundle.json"
    if sha256(bundle / "robot.urdf") != contract["urdf_sha256"]:
        return False, f"{skill}: robot.urdf checksum differs from bundle.json"
    current_urdf = Path(current_urdf or os.environ.get("HRUH_URDF", ROOT / "artifacts/hruh/hruh_isaac.urdf"))
    if current_urdf.is_file() and sha256(current_urdf) != contract["urdf_sha256"]:
        return False, f"{skill}: trained on a different robot model than {current_urdf}; retrain"
    checkpoint = Path(contract["checkpoint"]).resolve()
    if evaluation.get("checkpoint") and Path(evaluation["checkpoint"]).resolve() != checkpoint:
        return False, f"{skill}: evaluation is for {evaluation['checkpoint']}, bundle for {checkpoint}"
    status, gazebo_report = gazebo_status(gazebo)
    if require_gazebo and status != "pass":
        return False, f"{skill}: Gazebo transfer {status}, not promoted (--require-gazebo)"

    name = DEPLOY_NAME[skill]
    dest = Path(dest_root) / name
    new_score = score(skill, evaluation)
    manifest_path = dest / "PROMOTED.json"
    if manifest_path.is_file():
        old = json.loads(manifest_path.read_text())
        if old.get("policy_sha256") == contract["policy_sha256"]:
            return False, f"{name}: this policy is already promoted"
        same_robot = old.get("urdf_sha256") == contract["urdf_sha256"]
        old_rank = (old.get("gazebo_transfer") == "pass", old.get("score", -1e9))
        if same_robot and old_rank > (status == "pass", new_score):
            return False, (f"{name}: kept the promoted policy (score {old.get('score')}, Gazebo "
                           f"{old.get('gazebo_transfer')}) over this one (score {new_score}, Gazebo {status})")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    staging = Path(dest_root) / f".{name}.staging"
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    for file in FILES:
        if (bundle / file).is_file():
            shutil.copy2(bundle / file, staging / file)
    shutil.copy2(evaluation_path, staging / "evaluation.json")
    if gazebo_report is not None:
        shutil.copy2(gazebo, staging / "gazebo.json")
    params = checkpoint.parent / "params"
    if params.is_dir():
        shutil.copytree(params, staging / "params")
    manifest = {
        "name": name, "skill": skill, "promoted_utc": stamp, "run_id": run_id,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256(checkpoint) if checkpoint.is_file() else None,
        "policy_sha256": contract["policy_sha256"], "urdf_sha256": contract["urdf_sha256"],
        "isaac_benchmark": "PASS", "score": new_score, "gazebo_transfer": status,
        "gazebo_summary": None if gazebo_report is None else {
            k: gazebo_report.get(k) for k in ("completed", "falls", "policy_steps", "elapsed_sim_s")},
        "simulation_only": True,
        "deploy": {"locomotion": "ros2 launch hruh_bringup isaac.launch.py controller:=policy  |  "
                                 "ros2 launch hruh_bringup policy_gazebo.launch.py joystick:=true",
                   "reach": "ros2 launch hruh_bringup isaac.launch.py reach:=true  (goal on /hruh/hand_target)",
                   "lift": "simulation only: needs ground-truth cube pose"}[skill],
    }
    (staging / "PROMOTED.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if dest.exists():
        backup = Path(history_root) / name / stamp
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(dest), str(backup))
        for stale in sorted(backup.parent.iterdir())[:-KEEP_HISTORY]:
            shutil.rmtree(stale, ignore_errors=True)
    staging.rename(dest)
    return True, f"{name}: PROMOTED (score {new_score}, Gazebo {status}) -> {dest}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bundle", type=Path, required=True, help="exported folder (bundle.json, policy.pt)")
    parser.add_argument("--evaluation", type=Path, required=True, help="evaluation report of that checkpoint")
    parser.add_argument("--gazebo", type=Path, help="Gazebo transfer report")
    parser.add_argument("--require-gazebo", action="store_true", help="also require a fall-free Gazebo test")
    parser.add_argument("--run-id")
    parser.add_argument("--dest", type=Path, default=POLICIES)
    args = parser.parse_args()
    try:
        promoted, message = promote(args.bundle, args.evaluation, args.gazebo, args.require_gazebo, args.run_id,
                                    args.dest)
    except (OSError, ValueError, KeyError, TypeError) as error:   # missing / malformed bundle or report
        promoted, message = False, f"not promoted: unreadable bundle or report ({type(error).__name__}: {error})"
    print(message, flush=True)
    return 0 if promoted else 3


if __name__ == "__main__":
    sys.exit(main())
