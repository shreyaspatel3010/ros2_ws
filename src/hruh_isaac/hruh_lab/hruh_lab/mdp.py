"""HRUH-specific MDP terms shared by the tasks."""
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def hold_default_pose(env: ManagerBasedEnv, env_ids: torch.Tensor | None,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Command the joints in ``asset_cfg`` (that no action term drives) to their
    default positions, e.g. keep arms / hands / head in the standing pose while a
    policy only controls the legs."""
    asset = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    target = asset.data.default_joint_pos.torch[env_ids][:, asset_cfg.joint_ids]
    asset.set_joint_position_target(target, joint_ids=asset_cfg.joint_ids, env_ids=env_ids)


def yaw_rate_error_l2(env: ManagerBasedEnv, command_name: str = "base_velocity",
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Squared error between pelvis yaw rate and the commanded yaw rate.

    The exponential tracking reward saturates, so a policy can trade it for a
    twisting gait; this quadratic term keeps penalising pelvis yaw oscillation."""
    asset = env.scene[asset_cfg.name]
    wz = asset.data.root_ang_vel_b.torch[:, 2]
    wz_cmd = env.command_manager.get_command(command_name)[:, 2]
    return torch.square(wz - wz_cmd)
