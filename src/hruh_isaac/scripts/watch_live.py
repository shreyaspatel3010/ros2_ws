#!/usr/bin/env python3
"""Watch training live: an Isaac window whose robots always run the newest checkpoint.

    bash src/hruh_isaac/scripts/run.sh watch [motion|reach|lift|flat|rough]

A few robots run the policy of the current offline run (artifacts/hruh/offline/last_auto_run)
in real time. Whenever training saves a newer checkpoint (every 100 iterations, also across
rounds), the weights are swapped in place - no restart. Movements, sudden stops, arm motion
and pushes come from the task's own training curriculum. A copy of each checkpoint is loaded,
so the training run and its exports are never touched. Close the window or Ctrl+C to stop.
"""
import argparse
import os
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))
os.environ.setdefault("HRUH_URDF", str(ROOT / "artifacts/hruh/hruh_isaac.urdf"))
TASKS = {"motion": "Hruh-Velocity-Motion-v0", "flat": "Hruh-Velocity-Flat-v0", "rough": "Hruh-Velocity-Rough-v0",
         "reach": "Hruh-Reach-Right-v0", "lift": "Hruh-Lift-Right-v0"}


def newest_checkpoint(skill):
    run_file = ROOT / "artifacts/hruh/offline/last_auto_run"
    run = run_file.read_text().strip() if run_file.is_file() else ""
    files = list((ROOT / "logs/rsl_rl").glob(f"hruh_{run}_{skill}/*/model_*.pt"))
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


def main():
    from hruh_lab import gpu_guard
    gpu_guard.cap_torch()
    gpu_guard.start(name="watch_live")
    import warp as wp
    wp.config.enable_backward = False
    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from importlib.metadata import version
    from rsl_rl.runners import OnPolicyRunner
    import hruh_lab  # noqa: F401  (registers the tasks)
    from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("skill", nargs="?", default="motion", choices=sorted(TASKS))
    parser.add_argument("--num_envs", type=int, default=9)
    parser.add_argument("--poll", type=float, default=10.0, help="seconds between checks for a newer checkpoint")
    add_launcher_args(parser)
    args = parser.parse_args()
    task = TASKS[args.skill]

    first = None
    while first is None:
        first = newest_checkpoint(args.skill)
        if first is None:
            print(f"Waiting for the first {args.skill} checkpoint (saved every 100 iterations)...", flush=True)
            time.sleep(args.poll)

    cfg = load_cfg_from_registry(task, "env_cfg_entry_point")
    agent = load_cfg_from_registry(task, "rsl_rl_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = 2.5
    cfg.observations.policy.enable_corruption = False
    if args.device:
        cfg.sim.device = args.device
    args.device = agent.device = cfg.sim.device
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    copy_dir = ROOT / "artifacts/hruh/watch" / args.skill
    copy_dir.mkdir(parents=True, exist_ok=True)

    with launch_simulation(cfg, args):
        env = RslRlVecEnvWrapper(gym.make(task, cfg=cfg), clip_actions=agent.clip_actions)
        runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=cfg.sim.device)

        def load(checkpoint):
            copy = copy_dir / "model.pt"
            shutil.copyfile(checkpoint, copy)              # never read the file training writes
            runner.load(str(copy), load_cfg={"actor": True}, map_location=cfg.sim.device)   # the policy only
            print(f"Now showing {checkpoint.parent.name}/{checkpoint.name}", flush=True)
            return runner.get_inference_policy(device=cfg.sim.device)

        current = first
        policy = load(current)
        obs = env.get_observations()
        dt = env.unwrapped.step_dt
        last_poll = time.monotonic()
        print("Live view: robots switch to each newer checkpoint automatically. Close the window to stop.", flush=True)
        with torch.inference_mode():
            while env.unwrapped.sim.is_headless_or_exist_active_visualizer():
                started = time.monotonic()
                obs, _, dones, _ = env.step(policy(obs))
                if hasattr(policy, "reset"):
                    policy.reset(dones)
                if started - last_poll > args.poll:
                    last_poll = started
                    newest = newest_checkpoint(args.skill)
                    if newest is not None and newest != current and newest.stat().st_size > 0:
                        time.sleep(1.0)                    # let training finish writing it
                        try:
                            policy, current = load(newest), newest
                        except Exception as error:         # partially written: try again next poll
                            print(f"(could not load {newest.name} yet: {error})", flush=True)
                time.sleep(max(0.0, dt - (time.monotonic() - started)))
        env.close()


if __name__ == "__main__":
    main()
