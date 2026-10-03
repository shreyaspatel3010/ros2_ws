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
    target = asset.data.default_joint_pos[env_ids][:, asset_cfg.joint_ids]
    asset.set_joint_position_target(target, joint_ids=asset_cfg.joint_ids, env_ids=env_ids)
