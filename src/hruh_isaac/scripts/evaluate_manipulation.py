#!/usr/bin/env python3
"""Held-out reach/lift trials; each environment counts only its first episode."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))
os.environ.setdefault("HRUH_URDF", str(ROOT / "artifacts/hruh/hruh_isaac.urdf"))


def main():
    from hruh_lab import gpu_guard   # stay inside HRUH_GPU_MEM_GB (default 8) so the desktop survives
    gpu_guard.cap_torch()
    gpu_guard.start(name=__import__("os").path.basename(__file__))
    import warp as wp
    wp.config.enable_backward = False
    from isaaclab.app import AppLauncher, add_launcher_args, launch_simulation  # noqa: F401
    import gymnasium as gym
    import torch
    from importlib.metadata import version
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner
    from hruh_lab.tasks.reach.env_cfg import HruhReachEnvCfg, HruhLiftEnvCfg
    from hruh_lab.tasks.reach.agents import HruhReachPPORunnerCfg, HruhLiftPPORunnerCfg
    from hruh_lab.contract import validate_checkpoint

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skill", choices=("reach", "lift"), required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--checkpoint", type=Path)
    source.add_argument("--zero", action="store_true")
    parser.add_argument("--num_envs", type=int, default=32)
    parser.add_argument("--rounds", type=int, default=4)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--export", action="store_true")
    add_launcher_args(parser)
    args = parser.parse_args()
    if args.num_envs <= 0 or args.rounds <= 0:
        parser.error("--num_envs and --rounds must be positive")
    cfg = (HruhReachEnvCfg if args.skill == "reach" else HruhLiftEnvCfg)()
    agent = (HruhReachPPORunnerCfg if args.skill == "reach" else HruhLiftPPORunnerCfg)()
    cfg.scene.num_envs, cfg.seed = args.num_envs, args.seed
    cfg.sim.device = args.device or cfg.sim.device
    args.device = agent.device = cfg.sim.device
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    if args.checkpoint:
        validate_checkpoint(args.checkpoint, cfg.to_dict())
    with launch_simulation(cfg, args):
        env = RslRlVecEnvWrapper(gym.make(f"Hruh-{args.skill.title()}-Right-v0", cfg=cfg),
                                  clip_actions=agent.clip_actions)  # same clip as training
        try:
            if args.checkpoint:
                runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=env.device)
                runner.load(str(args.checkpoint.resolve()))
                policy = runner.get_inference_policy(device=env.device)
            else:
                policy = lambda obs: torch.zeros(env.num_envs, env.num_actions, device=env.device)
            results = []
            robot = env.unwrapped.scene["robot"]
            with torch.inference_mode():
                for trial in range(args.rounds):
                    env.seed(args.seed + trial)
                    obs, _ = env.reset()
                    alive = torch.ones(env.num_envs, device=env.device, dtype=torch.bool)
                    succeeded = torch.zeros_like(alive)
                    best_error = torch.full((env.num_envs,), torch.inf, device=env.device)
                    max_height = torch.zeros(env.num_envs, device=env.device)
                    time_elapsed = torch.zeros(env.num_envs, device=env.device)
                    if hasattr(policy, "reset"):
                        policy.reset(torch.ones_like(alive))
                    max_root_error = 0.0
                    for step in range(env.max_episode_length + 1):
                        root_relative = robot.data.root_pos_w.torch - env.unwrapped.scene.env_origins
                        root_error = (root_relative - torch.tensor(cfg.scene.robot.init_state.pos, device=env.device)).abs().max()
                        max_root_error = max(max_root_error, float(root_error))
                        if max_root_error > 0.01:
                            raise RuntimeError(f"Fixed pelvis/cloned environment mismatch: {max_root_error:.3f} m")
                        if args.skill == "reach":
                            from isaaclab.envs.mdp import position_command_error
                            from isaaclab.managers import SceneEntityCfg
                            wrist, _ = robot.find_bodies("right_wrist")
                            error = position_command_error(env.unwrapped, "ee_pose",
                                SceneEntityCfg("robot", body_ids=wrist))
                            best_error = torch.where(alive, torch.minimum(best_error, error), best_error)
                        else:
                            height = env.unwrapped.scene["object"].data.root_pos_w.torch[:, 2] - cfg.scene.object.init_state.pos[2]
                            max_height = torch.where(alive, torch.maximum(max_height, height), max_height)
                        time_elapsed += alive * env.unwrapped.step_dt
                        actions = policy(obs)
                        if not torch.isfinite(actions).all():
                            raise RuntimeError("Non-finite action")
                        obs, _, dones, _ = env.step(actions)
                        succeeded |= alive & env.unwrapped.termination_manager.get_term("success")
                        alive &= ~dones.bool()
                        if hasattr(policy, "reset"):
                            policy.reset(dones)
                        if not alive.any():
                            break
                    result = {"seed": args.seed + trial, "trials": env.num_envs,
                              "successes": int(succeeded.sum()), "success_fraction": float(succeeded.float().mean()),
                              "mean_duration_s": float(time_elapsed.mean()), "max_pelvis_position_error_m": max_root_error}
                    if args.skill == "reach":
                        result["mean_best_position_error_m"] = float(best_error.mean())
                    else:
                        result["mean_max_lift_m"] = float(max_height.mean())
                    results.append(result)
                    print(json.dumps(result), flush=True)
            successes = sum(r["successes"] for r in results)
            trials = sum(r["trials"] for r in results)
            report = {"skill": args.skill, "checkpoint": str(args.checkpoint) if args.checkpoint else "zero baseline",
                      "fixed_pelvis": True, "ground_truth_object_state": args.skill == "lift",
                      "success_fraction": successes / trials, "trials": trials,
                      "passed_benchmark": successes / trials >= 0.9,
                      "success_definition": "wrist within 2.5 cm" if args.skill == "reach" else
                          "cube lifted 10 cm, thumb and finger contact, held 0.5 s with speed <0.3 m/s",
                      "rounds": results}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2), flush=True)
            if args.export and args.checkpoint:
                from hruh_lab.export import export_bundle
                export_bundle(env, runner, args.checkpoint, args.skill)
        finally:
            env.close()


if __name__ == "__main__":
    main()
