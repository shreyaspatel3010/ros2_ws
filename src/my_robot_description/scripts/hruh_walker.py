#!/usr/bin/env python3
"""
HRUH human-like walking pattern generator + ROS 2 node.

Gait
  * ZMP preview control on a linear inverted pendulum (Kajita 2003) for the
    centre of mass, with the ZMP rolling from heel to toe under each stance foot.
  * Footsteps planned online from /cmd_vel (vx, vy, wz).
  * Swing foot: heel-strike (toes up) at landing, toe-off (heel up, rolling
    over the toe joint) at push-off, minimum-jerk swing with ground clearance.
  * Slight vertical centre-of-mass bob (lowest in double support), pelvis
    rotation with counter-rotating waist, and arms swinging opposite the legs.
  * Closed-form 6-DOF leg inverse kinematics (hip yaw-roll-pitch, knee,
    ankle pitch-roll), toe joint keeps the toes flat during push-off.

Modes (parameter `mode`)
  kinematic    : publishes /joint_states and odom->base_link TF itself
                 (RViz only, no ros2_control, no physics)
  ros2_control : streams the legs to /legs_controller/commands and, while
                 walking, the waist / arm swing to their JointTrajectoryControllers
                 (so MoveIt and the joystick own the arms when standing).  With
                 Gazebo the IMU stabilizer closes the balance loop; with mock
                 hardware set publish_odom_tf:=true.  ("gazebo" is an alias.)

The pattern generator (everything above `class WalkerNode`) has no ROS
dependency, so blender/render_walk.py reuses it to animate the robot.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------------------
# small math helpers
# ---------------------------------------------------------------------------


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_matrix(r, p, y):
    return Rz(y) @ Ry(p) @ Rx(r)


def axis_angle(axis, a):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


# ---------------------------------------------------------------------------
# URDF model: forward kinematics + centre of mass
# ---------------------------------------------------------------------------


class UrdfModel:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.links = {}
        for l in root.findall("link"):
            inert = l.find("inertial")
            m, c = 0.0, np.zeros(3)
            if inert is not None and inert.find("mass") is not None:
                m = float(inert.find("mass").get("value"))
                o = inert.find("origin")
                if o is not None:
                    c = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            self.links[l.get("name")] = (m, c)
        self.joints = {}
        self.children = {}
        child_set = set()
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1.0, 0, 0])
            lim = j.find("limit")
            lo = float(lim.get("lower", "-inf")) if lim is not None else -math.inf
            hi = float(lim.get("upper", "inf")) if lim is not None else math.inf
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            self.joints[j.get("name")] = dict(type=j.get("type"), parent=parent, child=child, xyz=xyz,
                                              R=rpy_matrix(*rpy), axis=axis, lower=lo, upper=hi)
            self.children.setdefault(parent, []).append(j.get("name"))
            child_set.add(child)
        self.root = [n for n in self.links if n not in child_set][0]
        self.movable = [n for n, j in self.joints.items() if j["type"] in ("revolute", "continuous", "prismatic")]

    def fk(self, q, root_R=np.eye(3), root_p=np.zeros(3)):
        out = {self.root: (root_R, root_p)}
        stack = [self.root]
        while stack:
            l = stack.pop()
            R, p = out[l]
            for jn in self.children.get(l, []):
                j = self.joints[jn]
                Rj = R @ j["R"]
                pj = p + R @ j["xyz"]
                if j["type"] in ("revolute", "continuous"):
                    Rj = Rj @ axis_angle(j["axis"], q.get(jn, 0.0))
                out[j["child"]] = (Rj, pj)
                stack.append(j["child"])
        return out

    def com(self, q, root_R=np.eye(3), root_p=np.zeros(3)):
        frames = self.fk(q, root_R, root_p)
        M, s = 0.0, np.zeros(3)
        for name, (m, c) in self.links.items():
            if m > 0 and name in frames:
                R, p = frames[name]
                s += m * (p + R @ c)
                M += m
        return s / M, M


# ---------------------------------------------------------------------------
# Leg geometry + closed-form inverse kinematics
# ---------------------------------------------------------------------------


class LegGeometry:
    def __init__(self, model):
        J = model.joints
        self.hip = {}
        for side in ("left", "right"):
            self.hip[side] = J[side + "_hip_yaw_joint"]["xyz"] + J[side + "_hip_roll_joint"]["xyz"]
        self.thigh = -J["left_knee_joint"]["xyz"][2]
        self.shin = -J["left_ankle_pitch_joint"]["xyz"][2]
        self.toe = J["left_toe_joint"]["xyz"].copy()          # toe hinge in the foot frame
        self.ankle_h = 0.065
        self.heel_x = -0.072

    @staticmethod
    def from_urdf(model, urdf_xml):
        """Sole height and heel position come from the foot collision box."""
        g = LegGeometry(model)
        root = ET.fromstring(urdf_xml)
        for l in root.findall("link"):
            if l.get("name") == "left_foot_link":
                c = l.find("collision")
                if c is not None and c.find("geometry/box") is not None:
                    z = float(c.find("origin").get("xyz").split()[2])
                    sz = [float(v) for v in c.find("geometry/box").get("size").split()]
                    x = float(c.find("origin").get("xyz").split()[0])
                    g.ankle_h = -(z - sz[2] / 2)
                    g.heel_x = x - sz[0] / 2
        return g


def leg_ik(geom, side, p_body, R_body, p_ankle, R_foot):
    """Kajita closed-form IK.  Returns (hip_yaw, hip_roll, hip_pitch, knee,
    ankle_pitch, ankle_roll) for joint axes z, x, y, y, y, x."""
    A, B = geom.thigh, geom.shin
    p_hip = p_body + R_body @ geom.hip[side]
    r = R_foot.T @ (p_hip - p_ankle)                 # ankle -> hip, in the foot frame
    C = float(np.linalg.norm(r))
    C = min(C, (A + B) * 0.9995)
    c5 = (C * C - A * A - B * B) / (2.0 * A * B)
    knee = math.acos(max(-1.0, min(1.0, c5)))
    alpha = math.asin(max(-1.0, min(1.0, A * math.sin(math.pi - knee) / C)))
    ankle_pitch = -math.atan2(r[0], math.copysign(math.sqrt(r[1] ** 2 + r[2] ** 2), r[2])) - alpha
    ankle_roll = math.atan2(r[1], r[2])
    R = R_body.T @ R_foot @ Rx(ankle_roll).T @ Ry(knee + ankle_pitch).T
    hip_yaw = math.atan2(-R[0, 1], R[1, 1])
    cz, sz = math.cos(hip_yaw), math.sin(hip_yaw)
    hip_roll = math.atan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)
    hip_pitch = math.atan2(-R[2, 0], R[2, 2])
    return hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll


# ---------------------------------------------------------------------------
# ZMP preview controller (one instance drives both x and y)
# ---------------------------------------------------------------------------


class PreviewController:
    def __init__(self, zc, dt, horizon, q_e=1.0, r=1e-6, g=9.81):
        self.dt, self.N = dt, int(round(horizon / dt))
        A = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
        B = np.array([[dt ** 3 / 6], [dt * dt / 2], [dt]])
        C = np.array([[1, 0, -zc / g]])
        At = np.block([[np.eye(1), C @ A], [np.zeros((3, 1)), A]])
        Bt = np.vstack([C @ B, B])
        It = np.array([[1.0], [0], [0], [0]])
        Q = np.zeros((4, 4)); Q[0, 0] = q_e
        P = Q.copy()
        for _ in range(5000):                         # discrete Riccati iteration
            S = r + Bt.T @ P @ Bt
            Pn = Q + At.T @ P @ At - At.T @ P @ Bt @ np.linalg.solve(S, Bt.T @ P @ At)
            if np.max(np.abs(Pn - P)) < 1e-12:
                P = Pn
                break
            P = Pn
        S = r + Bt.T @ P @ Bt
        K = np.linalg.solve(S, Bt.T @ P @ At)
        self.Gi, self.Gx = float(K[0, 0]), K[0, 1:]
        Ac = At - Bt @ K
        X = -Ac.T @ P @ It
        Gd = np.zeros(self.N)
        Gd[0] = -self.Gi
        for i in range(1, self.N):
            Gd[i] = np.linalg.solve(S, Bt.T @ X)[0, 0]
            X = Ac.T @ X
        self.Gd = Gd
        self.A, self.B, self.C = A, B, C

    def step(self, x, err_sum, p_future):
        """x: (3,2) state [pos, vel, acc] for x/y; p_future: (N,2) reference ZMP."""
        p = (self.C @ x)[0]
        u = -self.Gi * err_sum - self.Gx @ x - self.Gd[1:] @ p_future[1:self.N]
        x_next = self.A @ x + self.B @ u[None, :]
        return x_next, err_sum + (p - p_future[0])


# ---------------------------------------------------------------------------
# Walking pattern generator
# ---------------------------------------------------------------------------


class Footstep:
    __slots__ = ("x", "y", "yaw", "side", "cx", "cy", "cyaw")

    def __init__(self, x, y, yaw, side, cx, cy, cyaw):
        self.x, self.y, self.yaw, self.side = x, y, yaw, side
        self.cx, self.cy, self.cyaw = cx, cy, cyaw

    def pos(self):
        return np.array([self.x, self.y])


class GaitParams:
    def __init__(self, **kw):
        self.dt = 0.01
        self.step_time = 0.6          # s per step (human ~0.5-0.6 s)
        self.ds_ratio = 0.2           # double-support fraction (human ~20 %)
        self.step_height = 0.055      # swing-foot clearance
        self.max_vx, self.max_vy, self.max_wz = 0.35, 0.12, 0.5
        self.knee_drop = 0.012        # pelvis lowered from straight-leg height
        self.bob = 0.006              # vertical CoM oscillation (human ~2-4 cm peak-to-peak)
        self.toe_off = 0.30           # rad, heel-up push-off at full stride
        self.heel_strike = 0.20       # rad, toes-up landing at full stride
        self.stride_ref = 0.25        # stride at which toe_off / heel_strike reach full value
        self.zmp_heel, self.zmp_toe = 0.0, 0.06   # ZMP travel under the stance foot (ankle frame x)
        self.zmp_inset = 0.015        # ZMP inside the foot centre-line (less lateral sway)
        self.pelvis_yaw = 0.07        # rad, pelvis rotation at full stride
        self.arm_swing = 0.32         # rad, shoulder flexion amplitude at full stride
        self.elbow = 0.30             # rad, relaxed elbow bend
        self.arm_abduction = 0.10     # rad, keep hands clear of the hips
        self.preview = 1.6            # s
        self.start_delay = 0.8        # s from command to first lift-off
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(k)
            setattr(self, k, v)


class WalkingPatternGenerator:
    LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")

    def __init__(self, urdf_xml, params=None):
        self.p = params or GaitParams()
        self.model = UrdfModel(urdf_xml)
        self.geom = LegGeometry.from_urdf(self.model, urdf_xml)
        g, p = self.geom, self.p
        self.hip_w = abs(g.hip["left"][1])
        self.hip_z = -g.hip["left"][2]
        self.pelvis_h = g.ankle_h + g.thigh + g.shin + self.hip_z - p.knee_drop
        # whole-body CoM relative to the pelvis in the nominal pose
        q0 = self._arm_pose(0.0)
        com, self.mass = self.model.com(q0, np.eye(3), np.array([0, 0, self.pelvis_h]))
        self.com_offset = com - np.array([0, 0, self.pelvis_h])
        self.zc = com[2]
        self.ctrl = PreviewController(self.zc, p.dt, p.preview)
        self.t = 0.0
        self.cmd = np.zeros(3)
        w = self.hip_w
        self.steps = [Footstep(0, -w, 0, "right", 0, 0, 0), Footstep(0, w, 0, "left", 0, 0, 0)]
        self.t_walk0 = None          # time the first planned step begins
        self.stopping = False
        mid = np.array([0.0, 0.0])
        self.state = np.zeros((3, 2)); self.state[0] = mid
        self.err = np.zeros(2)
        self.joint_names = [s + "_" + j + "_joint" for s in ("left", "right") for j in self.LEG_JOINTS]

    # ----- footstep planning ------------------------------------------------
    @property
    def walking(self):
        return self.t_walk0 is not None

    def set_command(self, vx, vy, wz):
        p = self.p
        c = np.array([np.clip(vx, -p.max_vx, p.max_vx), np.clip(vy, -p.max_vy, p.max_vy),
                      np.clip(wz, -p.max_wz, p.max_wz)])
        moving = bool(np.any(np.abs(c) > 1e-3))
        if moving and not self.walking:
            last2 = self.steps[-2:]
            self.steps = list(last2)
            self.t_walk0 = self.t + p.start_delay
            self.stopping = False
        if np.any(np.abs(c - self.cmd) > 1e-4) and self.walking:
            # keep steps that already started (plus the next one), re-plan the rest
            keep = self._step_index(self.t + p.step_time) + 1
            self.steps = self.steps[:max(keep, 2)]
            self.stopping = not moving
        self.cmd = c

    def _step_start(self, j):
        return self.t_walk0 + (j - 2) * self.p.step_time

    def _step_index(self, t):
        if not self.walking or t < self.t_walk0:
            return 1
        return int((t - self.t_walk0) // self.p.step_time) + 2

    def _plan(self):
        if not self.walking:
            return
        p = self.p
        horizon_end = self.t + p.preview + 2 * p.step_time
        while self._step_start(len(self.steps)) < horizon_end:
            prev = self.steps[-1]
            side = "left" if prev.side == "right" else "right"
            s = 1.0 if side == "left" else -1.0
            if self.stopping:
                # two zero-advance steps bring the feet side by side, then stop
                if len(self.steps) >= 4 and self.steps[-1].cx == self.steps[-2].cx and \
                        self.steps[-1].cy == self.steps[-2].cy and self._last_closed():
                    break
                vx = vy = wz = 0.0
            else:
                vx, vy, wz = self.cmd
            T = p.step_time
            # lateral steps and turns are led by the leg on that side (no leg crossing)
            dyaw = 2 * wz * T if wz * s > 0 else 0.0
            dy = 2 * vy * T if vy * s > 0 else 0.0
            cyaw = prev.cyaw + dyaw
            ca = prev.cyaw
            cx = prev.cx + math.cos(ca) * vx * T - math.sin(ca) * dy
            cy = prev.cy + math.sin(ca) * vx * T + math.cos(ca) * dy
            fx = cx - math.sin(cyaw) * s * self.hip_w
            fy = cy + math.cos(cyaw) * s * self.hip_w
            self.steps.append(Footstep(fx, fy, cyaw, side, cx, cy, cyaw))

    def _last_closed(self):
        a, b = self.steps[-1], self.steps[-2]
        return abs(wrap(a.yaw - b.yaw)) < 1e-6 and abs(math.hypot(a.x - b.x, a.y - b.y) - 2 * self.hip_w) < 1e-6

    # ----- references ---------------------------------------------------------
    def _zmp_point(self, f, frac):
        p = self.p
        lx = p.zmp_heel + (p.zmp_toe - p.zmp_heel) * frac
        ly = -p.zmp_inset if f.side == "left" else p.zmp_inset
        c, s = math.cos(f.yaw), math.sin(f.yaw)
        return np.array([f.x + c * lx - s * ly, f.y + s * lx + c * ly])

    def _mid(self, a, b):
        return (a.pos() + b.pos()) / 2

    def zmp_ref(self, t):
        p = self.p
        T, Tds = p.step_time, p.step_time * p.ds_ratio
        n = len(self.steps)
        if not self.walking or t < self.t_walk0:
            return self._mid(self.steps[0], self.steps[1])
        j = self._step_index(t)
        if j >= n:                                   # after the last step: back to centre
            tau = t - self._step_start(n)
            a = self._zmp_point(self.steps[-2], 1.0)
            return a + (self._mid(self.steps[-1], self.steps[-2]) - a) * min_jerk(tau / (T * 0.5))
        tau = t - self._step_start(j)
        sup = self.steps[j - 1]
        prev = self._mid(self.steps[0], self.steps[1]) if j == 2 else self._zmp_point(self.steps[j - 2], 1.0)
        if tau < Tds:
            return prev + (self._zmp_point(sup, 0.0) - prev) * (tau / Tds)
        return self._zmp_point(sup, (tau - Tds) / (T - Tds))

    def _stride_scale(self, a, b):
        d = math.hypot(a.x - b.x, a.y - b.y)
        lateral = 2 * self.hip_w
        return min(1.0, max(0.0, (d - lateral) / self.p.stride_ref)) if d > lateral else \
            min(1.0, abs(a.x - b.x) / self.p.stride_ref)

    def _foot_flat(self, f):
        return np.array([f.x, f.y, self.geom.ankle_h]), f.yaw, 0.0

    def _foot_toe_pivot(self, f, th):
        g = self.geom
        Rf = Rz(f.yaw)
        hinge = np.array([f.x, f.y, 0.0]) + Rf @ np.array([g.toe[0], 0, 0]) + np.array([0, 0, g.ankle_h + g.toe[2]])
        ankle = hinge + Rf @ Ry(th) @ np.array([-g.toe[0], 0, -g.toe[2]])
        return ankle, f.yaw, th

    def _foot_heel_pivot(self, f, th):
        g = self.geom
        Rf = Rz(f.yaw)
        heel = np.array([f.x, f.y, 0.0]) + Rf @ np.array([g.heel_x, 0, 0])
        ankle = heel + Rf @ Ry(th) @ np.array([-g.heel_x, 0, g.ankle_h])
        return ankle, f.yaw, th

    def foot_pose(self, side, t):
        """-> (ankle position, yaw, pitch); pitch > 0 = heel up."""
        p = self.p
        T, Tds = p.step_time, p.step_time * p.ds_ratio
        n = len(self.steps)
        last = [f for f in self.steps if f.side == side][-1]
        if not self.walking or t < self.t_walk0:
            first = [f for f in self.steps[:2] if f.side == side][0]
            return self._foot_flat(first)
        j = self._step_index(t)
        if j >= n:
            tau = t - self._step_start(n)
            if last is self.steps[-1] and tau < Tds and n >= 3:
                th = -p.heel_strike * self._stride_scale(self.steps[-3], last)
                return self._foot_heel_pivot(last, th * (1 - min_jerk(tau / Tds)))
            return self._foot_flat(last)
        tau = t - self._step_start(j)
        swing, sup = self.steps[j], self.steps[j - 1]
        if side == swing.side:
            start = self.steps[j - 2]
            k = self._stride_scale(start, swing)
            th_off, th_hs = p.toe_off * k, p.heel_strike * k
            if tau < Tds:
                return self._foot_toe_pivot(start, th_off * min_jerk(tau / Tds))
            s = (tau - Tds) / (T - Tds)
            a0, y0, p0 = self._foot_toe_pivot(start, th_off)
            a1, y1, p1 = self._foot_heel_pivot(swing, -th_hs)
            m = min_jerk(s)
            pos = a0 + (a1 - a0) * m
            # clearance bump (half height when stepping in place)
            pos[2] += p.step_height * (0.5 + 0.5 * k) * math.sin(math.pi * min(1.0, s * 1.08))
            return pos, y0 + wrap(y1 - y0) * m, p0 + (p1 - p0) * m
        # stance foot: rolls from heel-strike to flat during double support
        if j > 2 and tau < Tds:
            k = self._stride_scale(self.steps[j - 3], sup)
            return self._foot_heel_pivot(sup, -p.heel_strike * k * (1 - min_jerk(tau / Tds)))
        return self._foot_flat(sup)

    def _arm_pose(self, phase):
        p = self.p
        return {
            "chest_to_left_shoulder": p.arm_swing * phase,
            "chest_to_right_shoulder": -p.arm_swing * phase,
            "left_shoulder_to_bisecp": -p.arm_abduction,
            "right_shoulder_to_bisecp": p.arm_abduction,
            "left_elbow_inword_to_midle": p.elbow + 0.15 * max(0.0, phase),
            "right_elbow_inword_to_midle": p.elbow + 0.15 * max(0.0, -phase),
        }

    # ----- main update --------------------------------------------------------
    def update(self):
        """Advance by dt.  Returns (joint positions dict, pelvis R, pelvis p)."""
        p = self.p
        self._plan()
        ts = self.t + np.arange(self.ctrl.N) * p.dt
        ref = np.array([self.zmp_ref(t) for t in ts])
        self.state, self.err = self.ctrl.step(self.state, self.err, ref)
        self.t += p.dt
        t = self.t
        if self.walking and self._step_index(t) >= len(self.steps) + 1 and self.stopping and \
                t > self._step_start(len(self.steps)) + 1.5:
            self.t_walk0 = None                       # back to idle standing
            self.steps = self.steps[-2:]
            self.stopping = False

        L = self.foot_pose("left", t)
        R = self.foot_pose("right", t)
        # pelvis orientation: mean foot heading + human pelvis rotation
        yaw_mean = L[1] + wrap(R[1] - L[1]) / 2
        Rl, Rr = Rz(-yaw_mean), Rz(-yaw_mean)
        dx = float((Rl @ L[0])[0] - (Rr @ R[0])[0])  # + when the left foot is ahead
        phase = max(-1.0, min(1.0, dx / p.stride_ref))
        pelvis_yaw = yaw_mean - p.pelvis_yaw * phase
        Rb = Rz(pelvis_yaw)
        # vertical bob: lowest in double support, highest at mid-stance
        bob = 0.0
        if self.walking and self.t_walk0 <= t < self._step_start(len(self.steps)):
            T, Tds = p.step_time, p.step_time * p.ds_ratio
            tau = (t - self.t_walk0) % T
            bob = p.bob * math.cos(2 * math.pi * (tau - Tds / 2 - T / 2) / T)
        com_xy = self.state[0]
        off = Rb @ self.com_offset
        pb = np.array([com_xy[0] - off[0], com_xy[1] - off[1], self.pelvis_h + bob])

        q = {}
        q["waist_yaw_joint"] = p.pelvis_yaw * phase      # chest keeps facing forward
        q["waist_roll_joint"] = 0.0
        q["waist_pitch_joint"] = 0.03 * abs(self.cmd[0]) / max(p.max_vx, 1e-6)
        q.update(self._arm_pose(-phase))                 # left arm forward with the right leg
        # whole-body CoM correction: shift the pelvis until the real CoM (swinging
        # legs and arms included) sits on the planned CoM
        for it in range(3):
            for side, (pa, fyaw, fpitch) in (("left", L), ("right", R)):
                Rf = Rz(fyaw) @ Ry(fpitch)
                sol = leg_ik(self.geom, side, pb, Rb, pa, Rf)
                for name, v in zip(self.LEG_JOINTS, sol):
                    q["%s_%s_joint" % (side, name)] = v
                q[side + "_toe_joint"] = -max(0.0, fpitch)   # toes stay flat while the heel is up
            if it == 2:
                break
            com, _ = self.model.com(q, Rb, pb)
            pb[:2] -= com[:2] - com_xy
        for n, v in list(q.items()):
            j = self.model.joints.get(n)
            if j is not None:
                q[n] = min(max(v, j["lower"]), j["upper"])
        return q, Rb, pb


class Stabilizer:
    """IMU feedback for position-controlled joints (used with Gazebo / hardware).

    Ankle strategy: body pitch / roll errors and rates are fed back to the ankle
    pitch / roll joints (lean back -> dorsiflex, lean right -> roll the shins
    left), like the ankle sway humans use to keep balance.  Hip strategy: body
    pitch error also bends the hips so the torso is pulled back upright."""

    def __init__(self, k_ankle=0.8, d_ankle=0.08, k_hip=0.5, d_hip=0.04,
                 k_roll=0.8, d_roll=0.06, limit=0.25):
        self.k_ankle, self.d_ankle, self.k_hip, self.d_hip = k_ankle, d_ankle, k_hip, d_hip
        self.k_roll, self.d_roll, self.limit = k_roll, d_roll, limit

    def apply(self, q, roll, pitch, rate_x, rate_y):
        c = lambda v: max(-self.limit, min(self.limit, v))
        da = c(self.k_ankle * pitch + self.d_ankle * rate_y)
        dh = c(self.k_hip * pitch + self.d_hip * rate_y)
        dr = c(self.k_roll * roll + self.d_roll * rate_x)
        out = dict(q)
        for side in ("left", "right"):
            out[side + "_ankle_pitch_joint"] = q[side + "_ankle_pitch_joint"] + da
            out[side + "_hip_pitch_joint"] = q[side + "_hip_pitch_joint"] + dh
            out[side + "_ankle_roll_joint"] = q[side + "_ankle_roll_joint"] + dr
        return out


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import TransformBroadcaster
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    LEGS = [s + "_" + j + "_joint" for s in ("left", "right")
            for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll", "toe")]
    WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
    ARMS = {side: ["chest_to_%s_shoulder" % side, "%s_shoulder_to_bisecp" % side, "%s_elbow_inword_to_midle" % side]
            for side in ("left", "right")}

    class WalkerNode(Node):
        def __init__(self):
            super().__init__("hruh_walker")
            self.declare_parameter("robot_description", "")
            self.declare_parameter("mode", "kinematic")          # kinematic | ros2_control (gazebo)
            self.declare_parameter("swing_arms", True)           # arm swing while walking (ros2_control mode)
            self.declare_parameter("publish_odom_tf", False)     # odom->base_link from the plan (mock hardware)
            self.declare_parameter("auto_walk", False)           # walk forward without /cmd_vel
            self.declare_parameter("auto_vx", 0.2)
            self.declare_parameter("step_time", 0.6)
            self.declare_parameter("step_height", 0.055)
            self.declare_parameter("toe_off", 0.30)
            self.declare_parameter("heel_strike", 0.20)
            self.declare_parameter("arm_swing", 0.32)
            self.declare_parameter("pelvis_yaw", 0.07)
            self.declare_parameter("cmd_timeout", 0.5)
            self.declare_parameter("max_vx", 0.35)               # forward speed limit for /cmd_vel
            self.declare_parameter("balance", True)              # IMU stabilizer (gazebo mode)
            self.declare_parameter("heading_gain", 0.6)          # foot-slip heading correction (ros2_control mode)
            urdf = self.get_parameter("robot_description").value
            if not urdf:
                raise RuntimeError("parameter robot_description is empty")
            self.mode = self.get_parameter("mode").value
            gp = GaitParams(step_time=self.get_parameter("step_time").value,
                            step_height=self.get_parameter("step_height").value,
                            toe_off=self.get_parameter("toe_off").value,
                            heel_strike=self.get_parameter("heel_strike").value,
                            arm_swing=self.get_parameter("arm_swing").value,
                            pelvis_yaw=self.get_parameter("pelvis_yaw").value,
                            max_vx=self.get_parameter("max_vx").value)
            self.gen = WalkingPatternGenerator(urdf, gp)
            self.get_logger().info("HRUH walker: mass %.1f kg, CoM height %.3f m, pelvis %.3f m, mode=%s"
                                   % (self.gen.mass, self.gen.zc, self.gen.pelvis_h, self.mode))
            self.last_cmd = None
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            if self.mode == "gazebo":
                self.mode = "ros2_control"
            self.tick_n = 0
            self.was_walking = False
            self.plan_yaw = 0.0
            self.drift_f = 0.0
            self.fix_yaw = 0.0
            if self.mode == "ros2_control":
                self.legs_pub = self.create_publisher(Float64MultiArray, "/legs_controller/commands", 10)
                self.waist_pub = self.create_publisher(JointTrajectory, "/waist_controller/joint_trajectory", 10)
                self.arm_pubs = {side: self.create_publisher(JointTrajectory, "/%s_arm_controller/joint_trajectory" % side, 10)
                                 for side in ("left", "right")}
                if self.get_parameter("publish_odom_tf").value:
                    self.tfb = TransformBroadcaster(self)
                self.stab = Stabilizer()
                self.imu = None
                self.yaw = None
                self.yaw0 = None
                self.create_subscription(Imu, "/imu", self.on_imu, 50)
                self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            else:
                self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
                self.tfb = TransformBroadcaster(self)
                self.all_joints = self.gen.model.movable
            self.create_timer(gp.dt, self.tick)

        def on_imu(self, msg):
            o = msg.orientation
            roll = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x * o.x + o.y * o.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (o.w * o.y - o.z * o.x))))
            yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            # body-frame rates -> roll / pitch rates in the heading frame
            wx, wy = msg.angular_velocity.x, msg.angular_velocity.y
            self.imu = (roll, pitch, wx, wy, yaw)

        def on_odom(self, msg):
            o = msg.pose.pose.orientation
            self.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            if self.yaw0 is None:
                self.yaw0 = self.yaw

        def on_cmd(self, msg):
            self.last_cmd = (self.get_clock().now(), (msg.linear.x, msg.linear.y, msg.angular.z))

        def tick(self):
            cmd = (0.0, 0.0, 0.0)
            if self.get_parameter("auto_walk").value:
                cmd = (self.get_parameter("auto_vx").value, 0.0, 0.0)
            if self.last_cmd is not None:
                age = (self.get_clock().now() - self.last_cmd[0]).nanoseconds * 1e-9
                if age < self.get_parameter("cmd_timeout").value:
                    cmd = self.last_cmd[1]
            if self.mode == "ros2_control" and self.yaw is not None and self.gen.walking:
                # heading drift = measured heading - planned heading (the plan already
                # contains the deliberate pelvis rotation and any commanded turn), so
                # only foot slip is corrected, not the gait's own yaw rhythm
                # (the turns added here are subtracted again, otherwise correcting the
                # plan would never reduce the measured drift)
                drift = wrap((self.yaw - self.yaw0) - (self.plan_yaw - self.fix_yaw))
                self.drift_f += (drift - self.drift_f) * 0.02          # ~0.5 s low-pass at 100 Hz
                wz_fix = max(-0.2, min(0.2, -self.get_parameter("heading_gain").value * self.drift_f))
                self.fix_yaw += wz_fix * self.gen.p.dt
                cmd = (cmd[0], cmd[1], cmd[2] + wz_fix)
            self.gen.set_command(*cmd)
            q, Rb, pb = self.gen.update()
            self.plan_yaw = math.atan2(Rb[1, 0], Rb[0, 0])
            now = self.get_clock().now().to_msg()
            if self.mode == "ros2_control":
                if self.get_parameter("balance").value and self.imu is not None:
                    q = self.stab.apply(q, *self.imu[:4])
                self.legs_pub.publish(Float64MultiArray(data=[float(q[n]) for n in LEGS]))
                self.tick_n += 1
                walking = self.gen.walking
                if self.tick_n % 4 == 0 and (walking or self.was_walking):
                    # 25 Hz short trajectories; one last one when the robot stops
                    self.waist_pub.publish(self.traj(WAIST, q))
                    if self.get_parameter("swing_arms").value:
                        for side, pub in self.arm_pubs.items():
                            pub.publish(self.traj(ARMS[side], q))
                    self.was_walking = walking
                if self.get_parameter("publish_odom_tf").value:
                    self.send_tf(now, Rb, pb)
                return
            js = JointState()
            js.header.stamp = now
            js.name = list(self.all_joints)
            js.position = [float(q.get(n, 0.0)) for n in js.name]
            self.js_pub.publish(js)
            self.send_tf(now, Rb, pb)

        def traj(self, names, q):
            msg = JointTrajectory()
            msg.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(q[n]) for n in names]
            pt.time_from_start = Duration(sec=0, nanosec=60_000_000)
            msg.points = [pt]
            return msg

        def send_tf(self, now, Rb, pb):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pb)
            yaw = math.atan2(Rb[1, 0], Rb[0, 0])
            tf.transform.rotation.z = math.sin(yaw / 2)
            tf.transform.rotation.w = math.cos(yaw / 2)
            self.tfb.sendTransform(tf)

    rclpy.init()
    node = WalkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
