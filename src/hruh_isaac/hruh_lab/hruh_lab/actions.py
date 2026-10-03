"""Rate-limited hand targets prevent instantaneous finger closure at reset."""
import torch
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction


class RateLimitedJointPositionAction(JointPositionAction):
    rate_limit = 0.8  # rad/s, below the URDF hand/arm speed limit

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.previous_target = self._asset.data.default_joint_pos.torch[:, self._joint_ids].clone()

    def process_actions(self, actions):
        super().process_actions(actions)
        delta = self.rate_limit * self._env.step_dt
        self._processed_actions = torch.clamp(self._processed_actions,
            min=self.previous_target - delta, max=self.previous_target + delta)
        self.previous_target.copy_(self._processed_actions)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self.previous_target[ids] = self._asset.data.default_joint_pos.torch[ids][:, self._joint_ids]
