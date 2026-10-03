"""Imitation rewards / terminations (DeepMimic-style exponential tracking)."""
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .motion_lib import quat_rotate_inv, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _cmd(env: ManagerBasedRLEnv, name="motion"):
    return env.command_manager.get_term(name)


def track_joint_pos(env, std: float, command_name="motion"):
    c = _cmd(env, command_name)
    e = (c.robot.data.joint_pos[:, c.joint_ids] - c.ref["joint_pos"][:, c.joint_ids]).square().sum(-1)
    return torch.exp(-e / std**2)


def track_joint_vel(env, std: float, command_name="motion"):
    c = _cmd(env, command_name)
    e = (c.robot.data.joint_vel[:, c.joint_ids] - c.ref["joint_vel"][:, c.joint_ids]).square().sum(-1)
    return torch.exp(-e / std**2)


def track_body_pos(env, std: float, command_name="motion"):
    """Key bodies (feet, hands) relative to the pelvis, compared in the heading frame."""
    c = _cmd(env, command_name)
    d = c.robot.data
    rel = d.body_pos_w[:, c.body_ids] - d.root_pos_w[:, None]
    ref_rel = c.ref_body_pos() - c.ref_root_pos()[:, None]
    h, href = yaw_quat(d.root_quat_w), yaw_quat(c.ref["root_quat"])
    nb = rel.shape[1]
    rel = quat_rotate_inv(h[:, None].expand(-1, nb, -1), rel)
    ref_rel = quat_rotate_inv(href[:, None].expand(-1, nb, -1), ref_rel)
    return torch.exp(-(rel - ref_rel).square().sum(-1).mean(-1) / std**2)


def track_root_height(env, std: float, command_name="motion"):
    c = _cmd(env, command_name)
    return torch.exp(-(c.robot.data.root_pos_w[:, 2] - c.ref_root_pos()[:, 2]).square() / std**2)


def track_root_orientation(env, std: float, command_name="motion"):
    """Tilt only (gravity direction in the pelvis frame); heading is free."""
    c = _cmd(env, command_name)
    g = torch.tensor([0.0, 0.0, -1.0], device=env.device).expand(env.num_envs, 3)
    e = (quat_rotate_inv(c.robot.data.root_quat_w, g) - quat_rotate_inv(c.ref["root_quat"], g)).square().sum(-1)
    return torch.exp(-e / std**2)


def track_root_lin_vel(env, std: float, command_name="motion"):
    c = _cmd(env, command_name)
    d = c.robot.data
    v = quat_rotate_inv(yaw_quat(d.root_quat_w), d.root_lin_vel_w)[:, :2]
    vr = quat_rotate_inv(yaw_quat(c.ref["root_quat"]), c.ref["root_lin_vel"])[:, :2]
    return torch.exp(-(v - vr).square().sum(-1) / std**2)


def track_root_yaw_rate(env, std: float, command_name="motion"):
    c = _cmd(env, command_name)
    return torch.exp(-(c.robot.data.root_ang_vel_w[:, 2] - c.ref["root_ang_vel"][:, 2]).square() / std**2)


def motion_finished(env, command_name="motion"):
    return _cmd(env, command_name).finished


def tracking_failed(env, max_body_error: float, max_height_error: float, command_name="motion"):
    c = _cmd(env, command_name)
    d = c.robot.data
    body = (d.body_pos_w[:, c.body_ids] - c.ref_body_pos()).norm(dim=-1).max(-1).values
    height = (d.root_pos_w[:, 2] - c.ref_root_pos()[:, 2]).abs()
    return (body > max_body_error) | (height > max_height_error)
