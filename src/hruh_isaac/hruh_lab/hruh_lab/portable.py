"""Observation/history, kinematics and bounded PD control shared with Gazebo.

No Isaac or ROS imports. Quaternions are xyzw, matching ROS and Isaac Lab 3.
"""
from collections import deque
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError("Invalid xyzw quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


class Kinematics:
    def __init__(self, urdf):
        self.by_child = {}
        for joint in ET.parse(urdf).getroot().findall("joint"):
            origin = joint.find("origin")
            xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()] if origin is not None else [0]*3
            rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()] if origin is not None else [0]*3
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = axis_rotation([0,0,1], rpy[2]) @ axis_rotation([0,1,0], rpy[1]) @ axis_rotation([1,0,0], rpy[0])
            axis = joint.find("axis")
            self.by_child[joint.find("child").get("link")] = (
                joint.find("parent").get("link"), joint.get("name"), transform,
                [float(x) for x in axis.get("xyz").split()] if axis is not None else [1,0,0], joint.get("type"))

    def transform(self, body, positions):
        chain = []
        while body in self.by_child:
            parent, name, origin, axis, kind = self.by_child[body]
            local = origin.copy()
            if kind in ("revolute", "continuous"):
                local[:3, :3] = local[:3, :3] @ axis_rotation(axis, positions.get(name, 0.0))
            chain.append(local)
            body = parent
        result = np.eye(4)
        for local in reversed(chain):
            result = result @ local
        return result


class PolicyIO:
    def __init__(self, contract, urdf):
        self.cfg = contract
        self.names = contract["policy_joints"]
        # joints in the joint_pos / joint_vel observations (older bundles: the policy joints)
        self.observed = contract.get("observation_joints") or self.names
        self.kinematics = Kinematics(urdf)
        self.reset()

    def reset(self):
        self.last_action = np.zeros(len(self.names))
        self.previous_target = np.array([self.cfg["default_positions"][n] for n in self.names])
        self.history = {}

    def observation(self, positions, velocities, quaternion, angular_velocity, command=(0,0,0),
                    hand_target=(0.33,-0.31,0.23), object_state=None):
        q = np.array([positions[n] - self.cfg["default_positions"][n] for n in self.observed])
        v = np.array([velocities[n] for n in self.observed])
        terms = {"joint_pos": q, "joint_vel": v, "actions": self.last_action,
                 "base_ang_vel": np.asarray(angular_velocity),
                 "projected_gravity": rotation(quaternion).T @ [0,0,-1],
                 "velocity_commands": np.asarray(command),
                 "pose_command": np.r_[hand_target, 0.0, 0.0, 0.0, 1.0]}
        if self.cfg["skill"] == "lift":
            if object_state is None or np.asarray(object_state).shape != (13,):
                raise ValueError("Lift requires object position, xyzw quaternion and world-frame velocity")
            wrist = self.kinematics.transform("right_wrist", positions)
            grasp = np.asarray(self.cfg["initial_root_position"]) + (wrist @ [-0.02,-0.03,-0.08,1])[:3]
            terms["object_state"] = np.r_[object_state, np.asarray(object_state)[:3] - grasp]
        flattened = []
        for name in self.cfg["observation_terms"]:
            value = np.asarray(terms[name], dtype=np.float32).copy()
            if not np.isfinite(value).all():
                raise ValueError(f"Non-finite observation term: {name}")
            if name not in self.history:
                self.history[name] = deque([value.copy() for _ in range(self.cfg["history_length"])],
                                           maxlen=self.cfg["history_length"])
            else:
                self.history[name].append(value)
            flattened.extend(self.history[name])
        obs = np.concatenate(flattened)
        if obs.size != self.cfg["observation_size"]:
            raise ValueError(f"Observation contract mismatch: {obs.size} != {self.cfg['observation_size']}")
        return obs

    FEET = ("left_foot_link", "right_foot_link")

    def pelvis_height(self, positions, quaternion):
        """Pelvis height above the ground from joint angles + IMU orientation, assuming the lower
        foot is on flat ground (no odometry needed). Calibrated so the trained stand pose gives
        the training spawn height."""
        if not hasattr(self, "_foot_offset"):
            stand = self.cfg["default_positions"]
            self._foot_offset = self.cfg["initial_root_position"][2] - max(
                -self.kinematics.transform(f, stand)[2, 3] for f in self.FEET)
        world = rotation(quaternion)
        return max(-(world @ self.kinematics.transform(f, positions)[:3, 3])[2] for f in self.FEET) + self._foot_offset

    def targets(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (len(self.names),) or not np.isfinite(action).all():
            raise ValueError("Invalid policy action")
        clip = self.cfg.get("raw_action_clip")
        self.last_action = action if clip is None else np.clip(action, -clip, clip)
        limits = np.asarray(self.cfg["limits"])
        values = np.clip(self.last_action * self.cfg["scale"] + self.cfg["offset"], limits[:,0], limits[:,1])
        if self.cfg.get("target_rate_limit"):
            delta = self.cfg["target_rate_limit"] * self.cfg["control_dt"]
            values = np.clip(values, self.previous_target - delta, self.previous_target + delta)
        self.previous_target = values.copy()
        targets = dict(self.cfg["default_positions"])
        targets.update(zip(self.names, values.tolist()))
        return targets

    def efforts(self, targets, positions, velocities):
        output = []
        for name in self.cfg["effort_joints"]:
            cfg = self.cfg["actuators"][name]
            torque = cfg["kp"] * (targets[name] - positions[name]) - cfg["kd"] * velocities[name]
            # Do not accelerate a joint further beyond its specified speed limit.
            if abs(velocities[name]) > cfg["velocity"] and torque * velocities[name] > 0:
                torque = 0.0
            output.append(float(np.clip(torque, -cfg["effort"], cfg["effort"])))
        if not np.isfinite(output).all():
            raise ValueError("Non-finite PD effort")
        return output
