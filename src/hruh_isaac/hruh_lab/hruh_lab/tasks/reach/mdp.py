"""Grasp rewards require thumb and finger contact with the object, excluding table contact."""
import torch
from isaaclab.managers import ManagerTermBase
from isaaclab.utils.math import quat_apply


def grasp_position(env):
    robot = env.scene["robot"]
    body, _ = robot.find_bodies("right_wrist")
    offset = torch.tensor([-0.02, -0.03, -0.08], device=env.device).expand(env.num_envs, 3)
    return robot.data.body_pos_w.torch[:, body[0]] + quat_apply(robot.data.body_quat_w.torch[:, body[0]], offset)


def object_state(env):
    obj = env.scene["object"].data
    state = torch.cat((obj.root_pos_w.torch - env.scene.env_origins,
                      obj.root_quat_w.torch, obj.root_vel_w.torch,
                      obj.root_pos_w.torch - grasp_position(env)), dim=-1)
    if not torch.isfinite(state).all():
        bad = (~torch.isfinite(state)).nonzero().cpu().tolist()[:12]
        robot = env.scene["robot"]
        body, _ = robot.find_bodies("right_wrist")
        ids = sorted({row for row, col in bad})
        detail = {"wrist_position": robot.data.body_pos_w.torch[ids, body[0]].cpu().tolist(),
                  "wrist_quaternion": robot.data.body_quat_w.torch[ids, body[0]].cpu().tolist(),
                  "joint_positions_finite": bool(torch.isfinite(robot.data.joint_pos.torch[ids]).all())}
        raise RuntimeError(f"Non-finite cube/hand state at (environment, component): {bad}; {detail}")
    return state


def reach_object(env):
    distance = (env.scene["object"].data.root_pos_w.torch - grasp_position(env)).norm(dim=-1)
    return 1.0 - torch.tanh(distance / 0.1)


def grasp_contact(env):
    def touching(name):
        forces = env.scene[name].data.force_matrix_w.torch
        return forces.norm(dim=-1).reshape(env.num_envs, -1).max(dim=-1).values > 0.1
    fingers = torch.stack([touching(f"finger{i}") for i in range(1, 5)], dim=-1).any(dim=-1)
    return (touching("thumb") & fingers).float()


def touch(env):
    """0.5 for the thumb and 0.5 for any finger touching the cube (partial credit)."""
    def touching(name):
        forces = env.scene[name].data.force_matrix_w.torch
        return (forces.norm(dim=-1).reshape(env.num_envs, -1).max(dim=-1).values > 0.1).float()
    fingers = torch.stack([touching(f"finger{i}") for i in range(1, 5)], dim=-1).max(dim=-1).values
    return 0.5 * touching("thumb") + 0.5 * fingers


FINGER_BASES = ["right_palm_to_thomb"] + [f"right_palm_to_finger{i}_lower" for i in range(1, 5)]


def close_when_near(env):
    """Finger / thumb flexion (0 open .. 1 closed) while the grasp point is within 6 cm of the cube."""
    robot = env.scene["robot"]
    if not hasattr(env, "_hruh_finger_ids"):
        ids, _ = robot.find_joints(FINGER_BASES, preserve_order=True)
        limits = robot.data.joint_pos_limits.torch[0, ids]
        env._hruh_finger_ids, env._hruh_finger_limits = ids, limits
    ids, limits = env._hruh_finger_ids, env._hruh_finger_limits
    q = robot.data.joint_pos.torch[:, ids]
    closure = ((q - limits[:, 0]) / (limits[:, 1] - limits[:, 0]).clamp_min(1e-3)).clamp(0.0, 1.0).mean(dim=-1)
    near = (env.scene["object"].data.root_pos_w.torch - grasp_position(env)).norm(dim=-1) < 0.06
    return closure * near.float()


def lift_height(env):
    height = env.scene["object"].data.root_pos_w.torch[:, 2] - env.scene.env_origins[:, 2] - 1.165
    return height.clamp(0.0, 0.15) / 0.15 * grasp_contact(env)


class SustainedGrasp(ManagerTermBase):
    """Succeed after lifting 10 cm with opposed contacts and holding for 0.5 seconds."""
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.steps = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)

    def reset(self, env_ids=None):
        self.steps[slice(None) if env_ids is None else env_ids] = 0

    def __call__(self, env):
        obj = env.scene["object"].data
        height = obj.root_pos_w.torch[:, 2] - env.scene.env_origins[:, 2]
        stable = obj.root_lin_vel_w.torch.norm(dim=-1) < 0.3
        close = (obj.root_pos_w.torch - grasp_position(env)).norm(dim=-1) < 0.12
        good = (height > 1.265) & stable & close & grasp_contact(env).bool()
        self.steps = torch.where(good, self.steps + 1, 0)
        return self.steps >= round(0.5 / env.step_dt)
