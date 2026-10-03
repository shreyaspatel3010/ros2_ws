"""Export a normalized actor plus the data needed by a separate simulator."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

from .joints import ARM_JOINTS, ARM_SWING_ELBOW, ARM_SWING_GAIN, ARM_SWING_HIP_SPAN


def resolve(value, name):
    if isinstance(value, dict):
        matches = [v for pattern, v in value.items() if re.fullmatch(pattern, name)]
        if len(matches) != 1:
            raise ValueError(f"Expected one parameter match for {name}, got {matches}")
        return float(matches[0])
    return float(value)


def export_bundle(env, runner, checkpoint, skill):
    destination = Path(checkpoint).resolve().parent / "exported"
    destination.mkdir(exist_ok=True)
    runner.export_policy_to_jit(path=str(destination), filename="policy.pt")
    runner.export_policy_to_onnx(path=str(destination), filename="policy.onnx")
    cfg = env.unwrapped.cfg
    robot = env.unwrapped.scene["robot"]
    defaults = dict(zip(robot.joint_names, robot.data.default_joint_pos.torch[0].cpu().tolist()))
    urdf = Path(os.environ["HRUH_URDF"])
    root = ET.parse(urdf).getroot()
    controlled = [j.get("name") for j in root.findall("joint")
                  if j.get("type") == "revolute" and j.find("mimic") is None]
    dynamics = {}
    for name in controlled:
        groups = [a for a in cfg.scene.robot.actuators.values()
                  if any(re.fullmatch(p, name) for p in a.joint_names_expr)]
        if len(groups) != 1:
            raise ValueError(f"Actuator grouping mismatch: {name}")
        actuator = groups[0]
        dynamics[name] = {"kp": resolve(actuator.stiffness, name), "kd": resolve(actuator.damping, name),
                          "effort": resolve(actuator.effort_limit_sim, name),
                          "velocity": resolve(actuator.velocity_limit_sim, name)}
    action_cfg = next(v for v in vars(cfg.actions).values() if v is not None and hasattr(v, "joint_names"))
    names = action_cfg.joint_names
    policy = cfg.observations.policy
    terms = [name for name, value in vars(policy).items() if hasattr(value, "func")]
    joint_obs = getattr(policy, "joint_pos", None)
    asset = joint_obs.params.get("asset_cfg") if joint_obs is not None else None
    observed = list(asset.joint_names) if asset is not None and isinstance(asset.joint_names, (list, tuple)) else names
    contract = {
        "schema_version": 1, "skill": skill, "simulation_only": True,
        "policy_sha256": hashlib.sha256((destination / "policy.pt").read_bytes()).hexdigest(),
        "checkpoint": str(Path(checkpoint).resolve()), "control_dt": env.unwrapped.step_dt,
        "physics_dt": cfg.sim.dt, "observation_terms": terms, "history_length": policy.history_length or 1,
        "observation_size": env.get_observations()["policy"].shape[-1],
        "policy_joints": names, "effort_joints": controlled, "default_positions": defaults,
        # joints in the joint_pos / joint_vel observations (motion task: + the 10 arm joints)
        "observation_joints": observed,
        # motion task: arms move while walking; runtimes add the trained counter-swing
        "arm_motion": ({"swing_gain": ARM_SWING_GAIN, "hip_span": ARM_SWING_HIP_SPAN,
                        "elbow_gain": ARM_SWING_ELBOW, "joints": ARM_JOINTS["left"] + ARM_JOINTS["right"]}
                       if getattr(getattr(cfg, "commands", None), "arm_motion", None) is not None else None),
        "target_rate_limit": 0.8 if "RateLimited" in str(action_cfg.class_type) else None,
        "scale": [resolve(action_cfg.scale, n) for n in names],
        "offset": [defaults[n] if action_cfg.use_default_offset else resolve(action_cfg.offset, n) for n in names],
        "limits": [action_cfg.clip[n] for n in names], "actuators": dynamics,
        "initial_root_position": list(cfg.scene.robot.init_state.pos),
        "fixed_base": cfg.scene.robot.spawn.fix_base,
        # the RslRlVecEnvWrapper clip used in training / evaluation (agent clip_actions)
        "raw_action_clip": float(env.clip_actions) if env.clip_actions is not None else None,
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "joint_velocity_observation_scale": 1.0,
        # wrist-target box (pelvis frame) the reach / lift policy was trained on
        "target_box": ([list(r.pos_x), list(r.pos_y), list(r.pos_z)]
                       if (r := getattr(getattr(getattr(cfg, "commands", None), "ee_pose", None), "ranges", None))
                       else None),
        "object_position": list(cfg.scene.object.init_state.pos) if skill == "lift" else None,
        "table_position": list(cfg.scene.table.init_state.pos) if skill == "lift" else None,
    }
    shutil.copyfile(urdf, destination / "robot.urdf")
    (destination / "bundle.json").write_text(json.dumps(contract, indent=2) + "\n")
    print(f"Portable simulation bundle: {destination}", flush=True)
    return destination
