#!/usr/bin/env python3
"""Evaluate HRUH checkpoints or drive one simulated HRUH from ROS /cmd_vel.

Uses the training environment, action ordering, PD drives and observation
history directly. Does not publish actuator commands to ROS or real hardware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))
os.environ.setdefault("HRUH_URDF", str(ROOT / "artifacts/hruh/hruh_isaac.urdf"))


ARM_SCENARIOS = ("stand_arms", "forward_arms", "sidestep_arms", "turn_arms")


def scenario_command(name, seconds):
    name = name.removesuffix("_arms")   # same command; the arms move to random poses
    if name in ("stand", "push_stand"):
        return (0.0, 0.0, 0.0)
    if name in ("forward", "push_walk"):
        return (0.3, 0.0, 0.0)
    if name == "reverse":
        return (-0.15, 0.0, 0.0)
    if name == "sidestep":
        return (0.0, 0.15, 0.0)
    if name == "turn":
        return (0.0, 0.0, 0.4)
    # Abrupt joystick changes, including a full stop and reversing direction.
    return ((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), (-0.15, 0.0, 0.0),
            (0.0, -0.15, -0.4))[int(seconds / 5) % 4]


def main():
    from hruh_lab import gpu_guard   # stay inside HRUH_GPU_MEM_GB (default 8) so the desktop survives
    gpu_guard.cap_torch()
    gpu_guard.start(name=__import__("os").path.basename(__file__))
    import warp as wp
    wp.config.enable_backward = False
    import gymnasium as gym
    import torch
    # Import the launcher before task configs so Isaac's USD bindings are selected.
    from isaaclab.app import AppLauncher, add_launcher_args, launch_simulation  # noqa: F401
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner
    from importlib.metadata import version
    import hruh_lab  # noqa: F401
    from hruh_lab.tasks.locomotion.env_cfg import HruhFlatEnvCfg, HruhMotionEnvCfg, HruhRoughEnvCfg, POLICY_JOINTS
    from hruh_lab.tasks.locomotion.agents import HruhFlatPPORunnerCfg, HruhMotionPPORunnerCfg

    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--checkpoint", type=Path)
    group.add_argument("--zero", action="store_true", help="Untrained default-pose baseline")
    parser.add_argument("--mode", choices=("evaluate", "joystick"), default="evaluate")
    parser.add_argument("--terrain", choices=("flat", "rough", "motion"), default="flat",
                        help="motion = Hruh-Velocity-Motion-v0 (flat ground, arms moving while walking)")
    parser.add_argument("--num_envs", type=int, default=32)
    parser.add_argument("--seconds", type=float, default=20.0, help="Seconds per evaluation trial or joystick session")
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/hruh/evaluation.json")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--export", action="store_true", help="Also export normalized actor and policy contract")
    add_launcher_args(parser)
    args = parser.parse_args()
    if args.num_envs < 1 or args.seconds <= 0:
        parser.error("--num_envs and --seconds must be positive")
    if args.mode == "evaluate" and args.seconds < 10:
        parser.error("Evaluation requires at least 10 seconds per trial (push occurs at 5 seconds)")
    if args.checkpoint and not args.checkpoint.is_file():
        parser.error(f"Checkpoint does not exist: {args.checkpoint}")
    if args.mode == "joystick" and args.zero:
        parser.error("Joystick playback requires a trained checkpoint")

    cfg = {"flat": HruhFlatEnvCfg, "rough": HruhRoughEnvCfg, "motion": HruhMotionEnvCfg}[args.terrain]()
    cfg.seed = args.seed
    cfg.scene.num_envs = 1 if args.mode == "joystick" else args.num_envs
    cfg.episode_length_s = args.seconds + 2.0
    cfg.events.push_robot = None  # evaluate controlled, repeatable pushes below
    cfg.events.base_external_force_torque = None
    cfg.curriculum.terrain_levels = None
    if cfg.scene.terrain.terrain_generator:
        cfg.scene.terrain.terrain_generator.curriculum = False
    # Keep training's startup randomization and sensor noise for honest evaluation.
    if args.device:
        cfg.sim.device = args.device
    args.device = cfg.sim.device
    agent = handle_deprecated_rsl_rl_cfg((HruhMotionPPORunnerCfg if args.terrain == "motion"
                                          else HruhFlatPPORunnerCfg)(), version("rsl-rl-lib"))
    agent.device = cfg.sim.device
    task = f"Hruh-Velocity-{args.terrain.title()}-v0"
    if args.checkpoint:
        from hruh_lab.contract import validate_checkpoint
        validate_checkpoint(args.checkpoint, cfg.to_dict())

    with launch_simulation(cfg, args):
        env = RslRlVecEnvWrapper(gym.make(task, cfg=cfg), clip_actions=agent.clip_actions)
        try:
            runner = None
            if args.checkpoint:
                runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=cfg.sim.device)
                runner.load(str(args.checkpoint.resolve()))
                policy = runner.get_inference_policy(device=cfg.sim.device)
            else:
                policy = lambda obs: torch.zeros(env.num_envs, env.num_actions, device=env.device)
            if args.mode == "joystick":
                set_arm_mode(env, "joystick")   # motion task: arms swing with the gait
                report = play_joystick(env, policy, args)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, indent=2) + "\n")
            else:
                report = evaluate(env, policy, args)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, indent=2) + "\n")
                print(json.dumps(report, indent=2), flush=True)
            if args.export and runner:
                from hruh_lab.export import export_bundle
                export_bundle(env, runner, args.checkpoint, "locomotion")
                export_dir = args.checkpoint.resolve().parent / "exported"
                contract = {
                    "simulation_only": True, "joint_names": POLICY_JOINTS,
                    "physics_dt": cfg.sim.dt, "control_dt": env.unwrapped.step_dt,
                    "action_scale": cfg.actions.joint_pos.scale,
                    "raw_action_clip": agent.clip_actions,
                    "target_limits": cfg.actions.joint_pos.clip,
                    "standing_pose": cfg.scene.robot.init_state.joint_pos,
                    "actuators": cfg.scene.robot.to_dict()["actuators"],
                    "observation_history": 5,
                    "history_layout": "term-major, each term oldest to newest",
                    "observation_terms": ["base_ang_vel", "projected_gravity", "velocity_commands",
                                          "joint_pos_relative_to_default", "joint_vel", "previous_clipped_action"],
                    "observation_joints": json.loads((export_dir / "bundle.json").read_text())["observation_joints"],
                    "observation_size": int(env.get_observations()["policy"].shape[-1]),
                    "normalization_in_export": True,
                    "urdf_sha256": hashlib.sha256(Path(os.environ["HRUH_URDF"]).read_bytes()).hexdigest(),
                    "checkpoint": str(args.checkpoint.resolve()),
                    "checkpoint_sha256": hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                }
                (export_dir / "contract.json").write_text(json.dumps(contract, indent=2) + "\n")
        finally:
            env.close()


def set_command(env, command):
    import torch
    term = env.unwrapped.command_manager.get_term("base_velocity")
    term.external_command = torch.as_tensor(command, device=env.device, dtype=torch.float32)
    term._update_command()


def set_arm_mode(env, name):
    """Motion task: natural arm swing in the standard scenarios, random arm poses in *_arms."""
    manager = env.unwrapped.command_manager
    if "arm_motion" in manager.active_terms:
        manager.get_term("arm_motion").forced_mode = "pose" if name.endswith("_arms") else "swing"


def evaluate(env, policy, args):
    import torch
    names = ("stand", "forward", "reverse", "sidestep", "turn", "stop_reverse", "push_stand", "push_walk")
    if "arm_motion" in env.unwrapped.command_manager.active_terms:
        names += ARM_SCENARIOS
    dt = env.unwrapped.step_dt
    steps = round(args.seconds / dt)
    results = []
    robot = env.unwrapped.scene["robot"]
    with torch.inference_mode():
        for name in names:
            set_command(env, scenario_command(name, 0.0))
            set_arm_mode(env, name)
            env.seed(args.seed)  # same initial states for each scenario and checkpoint
            obs, _ = env.reset()
            if hasattr(policy, "reset"):
                policy.reset(torch.ones(env.num_envs, dtype=torch.bool, device=env.device))
            alive = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
            lifetime = torch.zeros(env.num_envs, device=env.device)
            error_sum = torch.zeros(3, device=env.device)
            yaw_rate_sum = torch.zeros((), device=env.device)    # signed: spin vs. wobble diagnostic
            start_heading = None
            samples = 0
            push_applied_trials = 0
            for step in range(steps):
                command = scenario_command(name, step * dt)
                set_command(env, command)
                if name.startswith("push_") and step == round(5.0 / dt):
                    push_applied_trials = int(alive.sum())
                    velocity = robot.data.root_vel_w.torch.clone()
                    velocity[:, 0] += 0.2
                    velocity[:, 1] += 0.3
                    robot.write_root_velocity_to_sim(velocity)
                measured = torch.cat((robot.data.root_lin_vel_b.torch[:, :2],
                                      robot.data.root_ang_vel_b.torch[:, 2:3]), dim=-1)
                error = (measured - torch.tensor(command, device=env.device)).abs()
                error_sum += error[alive].sum(dim=0)
                yaw_rate_sum += (measured[:, 2] - command[2])[alive].sum()
                if start_heading is None:
                    start_heading = robot.data.heading_w.torch.clone()
                samples += int(alive.sum())
                lifetime += alive * dt
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    raise RuntimeError("Policy produced non-finite actions; evaluation aborted")
                obs, _, dones, _ = env.step(actions)
                alive &= ~dones.bool()
                if hasattr(policy, "reset"):
                    policy.reset(dones)
                if not alive.any():
                    break  # reset episodes never count as successful trials
            result = {
                "scenario": name, "trials": env.num_envs, "duration_s": steps * dt,
                "falls": int((~alive).sum()), "survival_fraction": float(alive.float().mean()),
                "mean_time_to_fall_or_end_s": float(lifetime.mean()),
                "velocity_mae_while_alive_vx_vy_wz": (error_sum / max(samples, 1)).tolist(),
                # Diagnostics (not part of the gate): a large MAE with a small signed mean
                # means the pelvis wobbles; a signed mean close to the MAE means it spins.
                "yaw_rate_error_signed_mean": float(yaw_rate_sum / max(samples, 1)),
                "heading_change_rad_mean_abs": float(torch.atan2(
                    torch.sin(robot.data.heading_w.torch - start_heading),
                    torch.cos(robot.data.heading_w.torch - start_heading)).abs().mean())
                    if start_heading is not None else None,
                "push_delta_velocity_world_xy": [0.2, 0.3] if name.startswith("push_") else None,
                "push_applied_trials": push_applied_trials,
            }
            results.append(result)
            print(json.dumps(result), flush=True)
    # An explicit benchmark gate, not a claim of universal or hardware safety.
    passed = all(r["survival_fraction"] >= 0.95
                 and max(r["velocity_mae_while_alive_vx_vy_wz"][:2]) <= 0.15
                 and r["velocity_mae_while_alive_vx_vy_wz"][2] <= 0.25 for r in results)
    return {"checkpoint": str(args.checkpoint) if args.checkpoint else "zero-action baseline",
            "seed": args.seed, "terrain": args.terrain, "sensor_noise": True,
            "startup_randomization": True, "passed_benchmark": passed,
            "gate": "Every scenario: >=95% survival, vx/vy MAE <=0.15 m/s, yaw MAE <=0.25 rad/s",
            "scope": "Simulation locomotion benchmark; no hardware, manipulation or get-up validation"
                     + ("; motion: arms swing (standard scenarios) or move to random poses (*_arms)"
                        if args.terrain == "motion" else ""),
            "results": results}


def play_joystick(env, policy, args):
    import torch
    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ImportError as exc:
        raise RuntimeError("Source /opt/ros/jazzy/setup.bash before joystick playback") from exc
    from hruh_lab.velocity_control import VelocityGuard
    guard = VelocityGuard()
    rclpy.init(args=[])
    node = rclpy.create_node("hruh_policy_simulation")
    report = {"simulation_only": True, "received_commands": 0, "falls": 0,
              "stale_steps": 0, "steps": 0, "max_abs_requested_velocity": [0.0] * 3}
    def on_command(message):
        report["received_commands"] += 1
        guard.update((message.linear.x, message.linear.y, message.angular.z))
    node.create_subscription(Twist, args.topic, on_command, 1)
    set_command(env, (0.0, 0.0, 0.0))
    obs, _ = env.reset()
    dt = env.unwrapped.step_dt
    print(f"SIMULATION ONLY: waiting for {args.topic}. On a fall, release the stick/LB before resuming.", flush=True)
    try:
        with torch.inference_mode():
            for _ in range(round(args.seconds / dt)):
                if not env.unwrapped.sim.is_headless_or_exist_active_visualizer():
                    break
                started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.0)
                command = guard.step(dt)
                report["steps"] += 1
                report["stale_steps"] += int(guard.clock() - guard.received > guard.timeout)
                report["max_abs_requested_velocity"] = [max(old, abs(new)) for old, new in
                    zip(report["max_abs_requested_velocity"], command)]
                set_command(env, command)
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    raise RuntimeError("Non-finite policy output; simulation stopped")
                obs, _, dones, _ = env.step(actions)
                if dones.any():
                    report["falls"] += 1
                    guard.fall()
                    set_command(env, (0.0, 0.0, 0.0))
                    policy.reset(dones)
                    print("Fall: simulation reset; motion latched until a fresh neutral command.", flush=True)
                time.sleep(max(0.0, dt - (time.monotonic() - started)))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report), flush=True)
    return report


if __name__ == "__main__":
    main()
