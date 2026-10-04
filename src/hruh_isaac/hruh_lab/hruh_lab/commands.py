"""Velocity sampling during training, external commands during evaluation/play,
and arm motion while walking (Hruh-Velocity-Motion-v0)."""
from isaaclab.envs.mdp import UniformVelocityCommand
from isaaclab.managers import CommandTerm


class ControllableVelocityCommand(UniformVelocityCommand):
    external_command = None

    def _update_command(self):
        if self.external_command is None:
            super()._update_command()
        else:
            self.vel_command_b[:] = self.external_command


class MotionVelocityCommand(ControllableVelocityCommand):
    """Velocity commands drawn from explicit movements, so each is trained often:
    stand, walk forward, walk backward, side-step left/right, turn in place and free
    combinations. Short resampling times add many starts, stops and direction changes;
    a robot moving fast is often told to stop instantly (cfg.stop_after_moving)."""

    def _resample_command(self, env_ids):
        import torch
        cfg, n = self.cfg, len(env_ids)
        if n == 0:
            return
        # who is moving right now: their next command may be a sudden stop
        previous = self.vel_command_b[env_ids].clone()
        was_moving = (previous[:, :2].norm(dim=1) > 0.3) | (previous[:, 2].abs() > 0.5)
        super()._resample_command(env_ids)
        probs = torch.tensor([cfg.mode_probabilities[m] for m in self.MODES], device=self.device)
        mode = torch.multinomial(probs / probs.sum(), n, replacement=True)
        u = lambda lo, hi: torch.empty(n, device=self.device).uniform_(lo, hi)  # noqa: E731
        sign = torch.where(torch.rand(n, device=self.device) < 0.5, -1.0, 1.0)
        cmd = self.vel_command_b[env_ids].clone()            # "mixed": keep the uniform sample
        vx_hi, vy_hi, wz_hi = cfg.ranges.lin_vel_x[1], cfg.ranges.lin_vel_y[1], cfg.ranges.ang_vel_z[1]
        small = lambda: u(-0.1, 0.1)  # noqa: E731  (a little turning while walking straight)
        rows = {
            1: (u(0.2, vx_hi), torch.zeros(n, device=self.device), small()),               # forward
            2: (u(cfg.ranges.lin_vel_x[0], -0.05), torch.zeros(n, device=self.device), small()),  # backward
            3: (torch.zeros(n, device=self.device), sign * u(0.05, vy_hi), small()),      # side-step
            4: (torch.zeros(n, device=self.device), torch.zeros(n, device=self.device), sign * u(0.2, wz_hi)),  # turn
        }
        for index, (vx, vy, wz) in rows.items():
            pick = mode == index
            cmd[pick] = torch.stack((vx, vy, wz), dim=1)[pick]
        # sudden stops: from walking fast / turning, instantly to standing still
        sudden = was_moving & (torch.rand(n, device=self.device) < cfg.stop_after_moving)
        mode = torch.where(sudden, torch.zeros_like(mode), mode)
        cmd[sudden] = 0.0
        self.vel_command_b[env_ids] = cmd
        self.is_standing_env[env_ids] = mode == 0                                           # stop / stand

    MODES = ("stand", "forward", "backward", "side", "turn", "mixed")


class ArmMotionCommand(CommandTerm):
    """Moves both arms while the legs walk, like a person or MoveIt would:

      hold   arms in the stand pose
      swing  natural counter-swing driven by the hips (joints.arm_swing_targets)
      pose   each arm (independently) moves to random reach / wave / carry poses

    The arm joints are not policy actions: the walking policy observes them and must
    keep its balance whatever they do. Targets are rate limited (cfg.max_rate)."""

    forced_mode = None   # evaluation: "hold" | "swing" | "pose"
    MODES = ("hold", "swing", "pose")

    def __init__(self, cfg, env):
        import torch
        from .joints import ARM_JOINTS, ARM_SWING_ELBOW, ARM_SWING_HIP_SPAN, arm_pose_range
        super().__init__(cfg, env)
        self.robot = env.scene[cfg.asset_name]
        names = ARM_JOINTS["left"] + ARM_JOINTS["right"]
        self.joint_ids, _ = self.robot.find_joints(names, preserve_order=True)
        self.hip_ids, _ = self.robot.find_joints(["left_hip_pitch_joint", "right_hip_pitch_joint"],
                                                 preserve_order=True)
        self.index = {name: i for i, name in enumerate(names)}
        ranges = torch.tensor([arm_pose_range(n) for n in names], device=self.device)
        self.low, self.high = ranges[:, 0], ranges[:, 1]
        self.default = self.robot.data.default_joint_pos.torch[:, self.joint_ids].clone()
        self.target = self.default.clone()
        self.goal = self.default.clone()
        self.mode = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.gain = torch.zeros(self.num_envs, device=self.device)
        self.hip_span, self.elbow_gain = ARM_SWING_HIP_SPAN, ARM_SWING_ELBOW

    @property
    def command(self):
        return self.target

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.target[ids] = self.default[ids]
        return super().reset(env_ids)

    def _update_metrics(self):
        pass

    def _resample_command(self, env_ids):
        import torch
        n = len(env_ids)
        probs = torch.tensor([self.cfg.mode_probabilities[m] for m in self.MODES], device=self.device)
        self.mode[env_ids] = torch.multinomial(probs / probs.sum(), n, replacement=True)
        self.gain[env_ids] = torch.empty(n, device=self.device).uniform_(*self.cfg.swing_gain_range)
        goal = self.low + torch.rand(n, len(self.low), device=self.device) * (self.high - self.low)
        # each arm moves on its own half of the time (one-handed tasks while walking)
        moving = torch.rand(n, 2, device=self.device) < 0.75
        per_joint = moving.repeat_interleave(len(self.low) // 2, dim=1)
        self.goal[env_ids] = torch.where(per_joint, goal, self.default[env_ids])

    def _update_command(self):
        import torch
        q = self.robot.data.joint_pos.torch
        phase = torch.clamp((q[:, self.hip_ids[1]] - q[:, self.hip_ids[0]]) / self.hip_span, -1.0, 1.0)
        swing = self.default.clone()
        i = self.index
        swing[:, i["chest_to_left_shoulder"]] += self.gain * phase
        swing[:, i["chest_to_right_shoulder"]] -= self.gain * phase
        swing[:, i["left_elbow_inword_to_midle"]] += self.elbow_gain * self.gain * torch.relu(-phase)
        swing[:, i["right_elbow_inword_to_midle"]] += self.elbow_gain * self.gain * torch.relu(phase)
        mode = self.mode
        if self.forced_mode is not None:
            mode = torch.full_like(self.mode, self.MODES.index(self.forced_mode))
        desired = torch.where((mode == 0)[:, None], self.default,
                              torch.where((mode == 1)[:, None], swing, self.goal))
        step = self.cfg.max_rate * self._env.step_dt
        self.target += torch.clamp(desired - self.target, -step, step)
        self.robot.set_joint_position_target(self.target, joint_ids=self.joint_ids)
