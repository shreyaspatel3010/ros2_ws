"""Left-right mirror symmetry of HRUH for PPO data augmentation (Mittal et al. 2024).

Every training sample is also used mirrored through the sagittal (x-z) plane: the left
leg's state becomes the right leg's, sideways velocities and yaw rates change sign. This
doubles the data per iteration and pushes the policy toward a symmetric, human-like gait.

The joint mirror is derived from the URDF instead of being written by hand: joints pair by
swapping "left" / "right" in their names (waist / neck joints pair with themselves), and the
sign follows from the joint axes at the zero pose. For a reflection M = diag(1, -1, 1),
M R(a, q) M = R(M a, -q), so a partner axis b = M a needs q_partner = -q and b = -M a needs
q_partner = +q:  sign = -b . (M a).  The stand pose must be mirror-consistent (checked).

`mirror_plan` is pure Python + numpy (unit-tested without Isaac);
`compute_symmetric_states` is the RSL-RL `data_augmentation_func`.
"""
import os
import re

import numpy as np

REFLECTION = np.diag([1.0, -1.0, 1.0])
# per-component signs of 3-vectors in the pelvis frame under the mirror
VECTOR_SIGNS = {
    "base_lin_vel": (1.0, -1.0, 1.0),        # vectors: y flips
    "projected_gravity": (1.0, -1.0, 1.0),
    "base_ang_vel": (-1.0, 1.0, -1.0),       # pseudo-vector: x and z flip
    "velocity_commands": (1.0, -1.0, -1.0),  # (vx, vy, wz)
}
JOINT_TERMS = ("joint_pos", "joint_vel")


def partner_name(name):
    swapped = re.sub(r"left", "\0", name)
    swapped = re.sub(r"right", "left", swapped)
    return swapped.replace("\0", "right")


def joint_axes(urdf):
    """Each revolute joint's axis in the pelvis frame at the zero pose."""
    from .portable import Kinematics
    kinematics = Kinematics(urdf)
    axes = {}
    for child, (_, name, _, axis, kind) in kinematics.by_child.items():
        if kind in ("revolute", "continuous"):
            a = np.asarray(axis, dtype=float)
            axes[name] = kinematics.transform(child, {})[:3, :3] @ (a / np.linalg.norm(a))
    return axes


def joint_mirror(names, axes):
    """(permutation, signs) with mirrored[i] = signs[i] * q[permutation[i]]."""
    index = {n: i for i, n in enumerate(names)}
    permutation, signs = [], []
    for name in names:
        partner = partner_name(name)
        if partner not in index:
            raise ValueError(f"symmetry: {name} has no mirror partner {partner} in {names}")
        sign = -float(axes[partner] @ (REFLECTION @ axes[name]))
        if abs(abs(sign) - 1.0) > 0.05:
            raise ValueError(f"symmetry: axes of {name} / {partner} are not mirror images ({sign:+.2f})")
        permutation.append(index[partner])
        signs.append(round(sign))
    return permutation, signs


def check_pose(names, permutation, signs, pose, tolerance=1e-3):
    """The pose (e.g. the trained stand pose) must equal its own mirror image."""
    q = np.asarray([pose[n] for n in names])
    bad = [n for n, a, b in zip(names, q, np.asarray(signs) * q[permutation]) if abs(a - b) > tolerance]
    if bad:
        raise ValueError(f"symmetry: stand pose is not mirror-symmetric for {bad}")


def mirror_plan(terms, joint_names, axes, default_pose=None):
    """Plan for one observation group: [(start, end, step, permutation | None, signs)], where
    `terms` is [(name, flat_dim)] in the group's order and joint_names maps a joint term to
    its joint list. Histories are flattened time-major, so `step`-sized blocks are mirrored."""
    plan, start = [], 0
    for name, dim in terms:
        if name in VECTOR_SIGNS:
            plan.append((start, start + dim, 3, None, list(VECTOR_SIGNS[name])))
        elif name in joint_names:
            names = joint_names[name]
            permutation, signs = joint_mirror(names, axes)
            if default_pose is not None and name in JOINT_TERMS:
                check_pose(names, permutation, signs, default_pose)
            plan.append((start, start + dim, len(names), permutation, signs))
        else:
            raise ValueError(f"symmetry: no mirror rule for observation term '{name}'")
        if dim % plan[-1][2]:
            raise ValueError(f"symmetry: term '{name}' size {dim} is not a multiple of {plan[-1][2]}")
        start += dim
    return plan


def apply_plan(x, plan):
    """Mirror a [batch, features] tensor or array with a plan (works for torch and numpy)."""
    out = x.clone() if hasattr(x, "clone") else x.copy()
    for start, end, step, permutation, signs in plan:
        block = x[:, start:end].reshape(x.shape[0], -1, step)
        if permutation is not None:
            block = block[:, :, permutation]
        sign = block.new_tensor(signs) if hasattr(block, "new_tensor") else np.asarray(signs)
        out[:, start:end] = (block * sign).reshape(x.shape[0], end - start)
    return out


# ---------------------------------------------------------------- RSL-RL integration (Isaac)
def _plans(env):
    env = env.unwrapped
    if getattr(env, "_hruh_mirror", None) is None:
        cfg, manager = env.cfg, env.observation_manager
        axes = joint_axes(os.environ["HRUH_URDF"])
        robot = env.scene["robot"]
        default = dict(zip(robot.joint_names, robot.data.default_joint_pos.torch[0].cpu().tolist()))
        action = cfg.actions.joint_pos
        plans = {}
        for group, terms in manager.active_terms.items():
            dims = [int(np.prod(d)) for d in manager.group_obs_term_dim[group]]
            joints = {"actions": list(action.joint_names)}
            group_cfg = getattr(cfg.observations, group)
            for term in JOINT_TERMS:
                term_cfg = getattr(group_cfg, term, None)
                if term_cfg is not None:
                    joints[term] = list(term_cfg.params["asset_cfg"].joint_names)
            plans[group] = mirror_plan(list(zip(terms, dims)), joints, axes, default)
        actions = mirror_plan([("actions", len(action.joint_names))], {"actions": list(action.joint_names)},
                              axes, default)
        env._hruh_mirror = (plans, actions)
    return env._hruh_mirror


def compute_symmetric_states(env, obs=None, actions=None):
    """RSL-RL symmetry augmentation: returns ([original; mirrored] obs, [original; mirrored] actions)."""
    import torch
    with torch.no_grad():
        plans, action_plan = _plans(env)
        obs_aug = actions_aug = None
        if obs is not None:
            n = obs.batch_size[0]
            obs_aug = obs.repeat(2)
            for group in obs.keys():
                obs_aug[group][n:] = apply_plan(obs[group], plans[group])
        if actions is not None:
            actions_aug = torch.cat((actions, apply_plan(actions, action_plan)), dim=0)
        return obs_aug, actions_aug
