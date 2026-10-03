"""Reference-motion library for imitation learning.

Clip format (.npz, one file per motion):
    fps          scalar
    joint_names  [J] str        joints the clip animates (others keep the standing pose)
    joint_pos    [T, J] rad
    root_pos     [T, 3] m       pelvis (base_link) position, world
    root_quat    [T, 4]         pelvis orientation, w x y z
    body_names   [B] str        optional key bodies (e.g. feet, wrists)
    body_pos     [T, B, 3] m    optional key-body positions, world
Velocities are computed here by finite differences.  Generate clips with
hruh_isaac/scripts/make_motion_clips.py or convert your own motion capture.
"""
from __future__ import annotations

import glob
import os

import numpy as np
import torch


def _quat_mul(a, b):
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                        aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], -1)


def _quat_conj(q):
    return torch.cat([q[..., :1], -q[..., 1:]], -1)


def quat_rotate_inv(q, v):
    """Rotate v by the inverse of q (w x y z)."""
    qv = torch.cat([torch.zeros_like(v[..., :1]), v], -1)
    return _quat_mul(_quat_mul(_quat_conj(q), qv), q)[..., 1:]


def yaw_quat(q):
    w, x, y, z = q.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    zero = torch.zeros_like(yaw)
    return torch.stack([torch.cos(yaw / 2), zero, zero, torch.sin(yaw / 2)], -1)


class MotionLib:
    def __init__(self, motion_files: list[str] | str, joint_names: list[str], body_names: list[str],
                 default_joint_pos: torch.Tensor, device: str):
        if isinstance(motion_files, str):
            motion_files = sorted(glob.glob(os.path.join(os.path.expanduser(motion_files), "*.npz")))
        if not motion_files:
            raise FileNotFoundError("no motion clips found (run make_motion_clips.py)")
        self.device = device
        self.names, self.fps, self.length, starts = [], [], [], []
        jp, rp, rq, bp = [], [], [], []
        start = 0
        for f in motion_files:
            d = np.load(f, allow_pickle=False)
            T = d["joint_pos"].shape[0]
            q = np.tile(default_joint_pos.cpu().numpy()[None], (T, 1)).astype(np.float32)
            clip_joints = [str(n) for n in d["joint_names"]]
            for i, n in enumerate(clip_joints):
                if n in joint_names:
                    q[:, joint_names.index(n)] = d["joint_pos"][:, i]
            b = np.zeros((T, len(body_names), 3), np.float32)
            if "body_pos" in d:
                clip_bodies = [str(n) for n in d["body_names"]]
                for i, n in enumerate(body_names):
                    if n in clip_bodies:
                        b[:, i] = d["body_pos"][:, clip_bodies.index(n)]
            jp.append(q); rp.append(d["root_pos"].astype(np.float32)); rq.append(d["root_quat"].astype(np.float32))
            bp.append(b)
            self.names.append(os.path.splitext(os.path.basename(f))[0])
            self.fps.append(float(d["fps"])); self.length.append(T); starts.append(start)
            start += T
        t = lambda x: torch.tensor(np.concatenate(x), device=device)
        self.joint_pos, self.root_pos, self.root_quat, self.body_pos = t(jp), t(rp), t(rq), t(bp)
        self.root_quat = self.root_quat / self.root_quat.norm(dim=-1, keepdim=True)
        self.start = torch.tensor(starts, device=device, dtype=torch.long)
        self.num_frames = torch.tensor(self.length, device=device, dtype=torch.long)
        self.dt = 1.0 / torch.tensor(self.fps, device=device)
        self.duration = (self.num_frames - 1).float() * self.dt
        # finite-difference velocities (per clip, last frame repeats)
        self.joint_vel = torch.zeros_like(self.joint_pos)
        self.root_lin_vel = torch.zeros_like(self.root_pos)
        self.root_ang_vel = torch.zeros_like(self.root_pos)
        for c in range(len(self.names)):
            s, n, dt = starts[c], self.length[c], float(self.dt[c])
            sl = slice(s, s + n - 1)
            nx = slice(s + 1, s + n)
            self.joint_vel[sl] = (self.joint_pos[nx] - self.joint_pos[sl]) / dt
            self.root_lin_vel[sl] = (self.root_pos[nx] - self.root_pos[sl]) / dt
            dq = _quat_mul(self.root_quat[nx], _quat_conj(self.root_quat[sl]))
            dq = torch.where(dq[:, :1] < 0, -dq, dq)
            self.root_ang_vel[sl] = 2.0 * dq[:, 1:] / dt          # small-angle, world frame
            for buf in (self.joint_vel, self.root_lin_vel, self.root_ang_vel):
                buf[s + n - 1] = buf[s + n - 2]
        print(f"[MotionLib] {len(self.names)} clips: " + ", ".join(
            f"{n} ({d:.1f}s)" for n, d in zip(self.names, self.duration.tolist())))

    @property
    def num_clips(self):
        return len(self.names)

    def sample(self, n: int, min_remaining: float = 1.0):
        clip = torch.randint(0, self.num_clips, (n,), device=self.device)
        max_t = (self.duration[clip] - min_remaining).clamp(min=0.0)
        return clip, torch.rand(n, device=self.device) * max_t

    def state(self, clip: torch.Tensor, t: torch.Tensor) -> dict[str, torch.Tensor]:
        """Interpolated reference state at time t (s) of each clip."""
        t = torch.minimum(t, self.duration[clip])
        f = t / self.dt[clip]
        f0 = f.floor().long().clamp(max=self.num_frames[clip] - 1)
        f1 = (f0 + 1).clamp(max=self.num_frames[clip] - 1)
        a = (f - f0.float()).unsqueeze(-1)
        i0, i1 = self.start[clip] + f0, self.start[clip] + f1
        lerp = lambda x: x[i0] * (1 - a) + x[i1] * a
        q0, q1 = self.root_quat[i0], self.root_quat[i1]
        q1 = torch.where((q0 * q1).sum(-1, keepdim=True) < 0, -q1, q1)
        quat = q0 * (1 - a) + q1 * a
        return {
            "joint_pos": lerp(self.joint_pos), "joint_vel": lerp(self.joint_vel),
            "root_pos": lerp(self.root_pos), "root_quat": quat / quat.norm(dim=-1, keepdim=True),
            "root_lin_vel": lerp(self.root_lin_vel), "root_ang_vel": lerp(self.root_ang_vel),
            "body_pos": self.body_pos[i0] * (1 - a.unsqueeze(-1)) + self.body_pos[i1] * a.unsqueeze(-1),
        }
