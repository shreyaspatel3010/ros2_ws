"""Motion-tracking command: holds each environment's reference clip and phase.

On reset an environment draws a random clip and start time and the robot is
put exactly into that reference state (reference state initialisation), so it
learns every part of every clip.  The command handed to the policy is the next
reference frame (joint targets relative to the standing pose, root velocity in
the heading frame, root height, gravity direction).
"""
from __future__ import annotations

from dataclasses import MISSING
from typing import TYPE_CHECKING, Sequence

import torch

from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

from .motion_lib import MotionLib, quat_rotate_inv, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class MotionCommand(CommandTerm):
    cfg: "MotionCommandCfg"

    def __init__(self, cfg: "MotionCommandCfg", env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self.robot = env.scene[cfg.asset_name]
        self.joint_ids, self.joint_names = self.robot.find_joints(cfg.joint_names, preserve_order=True)
        self.body_ids, self.body_names = self.robot.find_bodies(cfg.key_bodies, preserve_order=True)
        self.lib = MotionLib(cfg.motion_dir, self.robot.joint_names, self.body_names,
                             self.robot.data.default_joint_pos[0], self.device)
        n = self.num_envs
        self.clip = torch.zeros(n, dtype=torch.long, device=self.device)
        self.t0 = torch.zeros(n, device=self.device)
        self.origin = torch.zeros(n, 3, device=self.device)        # world offset of each reference
        self.ref = self.lib.state(self.clip, self.t0)
        self._command = torch.zeros(n, len(self.joint_ids) + 7, device=self.device)
        for m in ("joint_pos_error", "body_pos_error", "root_height_error"):
            self.metrics[m] = torch.zeros(n, device=self.device)

    # --------------------------------------------------------------- helpers
    @property
    def time(self) -> torch.Tensor:
        return self.t0 + self._env.episode_length_buf * self._env.step_dt

    @property
    def finished(self) -> torch.Tensor:
        return self.time >= self.lib.duration[self.clip] - self._env.step_dt

    def ref_body_pos(self):
        return self.ref["body_pos"] + self.origin[:, None, :]

    def ref_root_pos(self):
        return self.ref["root_pos"] + self.origin

    # --------------------------------------------------------------- CommandTerm API
    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        q = self.robot.data.joint_pos[:, self.joint_ids]
        self.metrics["joint_pos_error"] = (q - self.ref["joint_pos"][:, self.joint_ids]).abs().mean(-1)
        body = self.robot.data.body_pos_w[:, self.body_ids]
        self.metrics["body_pos_error"] = (body - self.ref_body_pos()).norm(dim=-1).mean(-1)
        self.metrics["root_height_error"] = (self.robot.data.root_pos_w[:, 2] - self.ref_root_pos()[:, 2]).abs()

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        clip, t = self.lib.sample(len(env_ids), self.cfg.min_remaining_s)
        self.clip[env_ids], self.t0[env_ids] = clip, t
        st = self.lib.state(clip, t)
        # place the clip so its pelvis starts above this environment's origin
        origin = self._env.scene.env_origins[env_ids].clone()
        origin[:, :2] -= st["root_pos"][:, :2]
        origin[:, 2] = 0.0
        self.origin[env_ids] = origin
        # reference state initialisation
        root = torch.cat([st["root_pos"] + origin, st["root_quat"]], -1)
        vel = torch.cat([st["root_lin_vel"], st["root_ang_vel"]], -1)
        self.robot.write_root_pose_to_sim(root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim(vel, env_ids=env_ids)
        self.robot.write_joint_state_to_sim(st["joint_pos"], st["joint_vel"], env_ids=env_ids)
        for k in self.ref:
            self.ref[k][env_ids] = st[k]
        self._fill_command(env_ids)

    def _update_command(self):
        nxt = self.time + self._env.step_dt
        self.ref = self.lib.state(self.clip, self.time)
        self._next = self.lib.state(self.clip, nxt)
        self._fill_command(slice(None))

    def _fill_command(self, ids):
        nxt = getattr(self, "_next", self.ref)
        default = self.robot.data.default_joint_pos[ids][:, self.joint_ids]
        heading = yaw_quat(nxt["root_quat"][ids])
        lin_vel_h = quat_rotate_inv(heading, nxt["root_lin_vel"][ids])
        g = torch.tensor([0.0, 0.0, -1.0], device=self.device).expand(lin_vel_h.shape[0], 3)
        grav_b = quat_rotate_inv(nxt["root_quat"][ids], g)
        self._command[ids] = torch.cat([
            nxt["joint_pos"][ids][:, self.joint_ids] - default,     # joint targets
            lin_vel_h,                                              # root velocity, heading frame
            nxt["root_ang_vel"][ids][:, 2:3],                       # yaw rate
            nxt["root_pos"][ids][:, 2:3],                           # pelvis height
            grav_b[:, :2],                                          # tilt (gravity x/y in the pelvis frame)
        ], -1)


@configclass
class MotionCommandCfg(CommandTermCfg):
    class_type: type = MotionCommand
    asset_name: str = "robot"
    motion_dir: str = MISSING
    joint_names: list[str] = MISSING
    key_bodies: list[str] = MISSING
    min_remaining_s: float = 1.0
    resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)   # only on reset
