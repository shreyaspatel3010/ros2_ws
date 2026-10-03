#!/usr/bin/env python3
"""
Whole-robot gamepad control for HRUH (same conventions as aries_teleop).

Input is the canonical Xbox layout from joy_layout_normalizer:
  axes    0/1 left stick X/Y, 2 LT (0 released -> 1 pressed), 3/4 right stick X/Y,
          5 RT, 6/7 D-pad X/Y (+1 = left / up)
  buttons 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 BACK, 7 START

Mapping (config/joystick.yaml):
  Hold LB              WALK: left stick = forward/back + sidestep, right stick X = turn  -> /cmd_vel
                       (LB blocks every arm / head output, like the rover drive on aries)
  Hold RB              ARM CARTESIAN jog of the selected hand (base_link frame):
                       left stick = forward/back + left/right, D-pad up/down = up/down,
                       right stick X = wrist rotation, right stick Y = upper-arm rotation.
                       The arms are 5-DOF, which MoveIt Servo (Jazzy) does not support -
                       its singularity check reads a 6th singular value - so the hand
                       position is solved here with damped least squares on the live
                       joint states.
  Hold RT              JOINT jog of the selected chain:
                       arm  : left stick Y shoulder flex, X shoulder abduction,
                              right stick X upper-arm rotation, Y elbow, D-pad X wrist
                       head : left stick X neck turn, Y nod; right stick X waist turn,
                              Y waist bend, D-pad X waist side-bend
  RB or RT held + X    open the selected hand (both hands for the head chain)
  RB or RT held + B    close (grasp)
  Press BACK           select the next chain: right arm -> left arm -> head/waist
  Hold LT + Y/A/B/X    planned MoveIt move to a named pose (SRDF group states):
                       Y home, A wave (selected arm), B hands_up, X reach_forward
  Hold LT + D-pad      up = head center, down = head look_down
  /joy silent for joy_timeout_sec -> everything stops.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = {s: ["chest_to_%s_shoulder" % s, "%s_shoulder_to_bisecp" % s, "%s_bisecp_to_elbow_inword" % s,
           "%s_elbow_inword_to_midle" % s, "%s_forarm_to_wrist" % s] for s in ("left", "right")}
HEAD = ["chest_to_neck", "neck_to_head"]
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
CHAINS = ["right_arm", "left_arm", "head"]


def _rot(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class Kinematics:
    """Forward kinematics / position Jacobian straight from the URDF."""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.joints, self.children, kids = {}, {}, set()
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            r, pch, y = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            R = _rot((0, 0, 1), y) @ _rot((0, 1, 0), pch) @ _rot((1, 0, 0), r)
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1.0, 0, 0])
            par, ch = j.find("parent").get("link"), j.find("child").get("link")
            self.joints[j.get("name")] = (j.get("type"), par, ch, xyz, R, axis)
            self.children.setdefault(par, []).append(j.get("name"))
            kids.add(ch)
        self.root = [l.get("name") for l in root.findall("link") if l.get("name") not in kids][0]

    def fk(self, q):
        out = {self.root: (np.eye(3), np.zeros(3))}
        stack = [self.root]
        while stack:
            l = stack.pop()
            R, p = out[l]
            for jn in self.children.get(l, []):
                t, _, ch, xyz, Rj, axis = self.joints[jn]
                Rc = R @ Rj
                if t in ("revolute", "continuous"):
                    Rc = Rc @ _rot(axis, q.get(jn, 0.0))
                out[ch] = (Rc, p + R @ xyz)
                stack.append(ch)
        return out

    def position_jacobian(self, q, tip, joints):
        f = self.fk(q)
        pt = f[tip][1]
        cols = []
        for jn in joints:
            _, _, ch, _, _, axis = self.joints[jn]
            Rc, pc = f[ch]
            a = Rc @ axis
            cols.append(np.cross(a, pt - pc))
        return np.array(cols).T


class HruhJoystick(Node):
    def __init__(self):
        super().__init__("hruh_joystick")
        p = self.declare_parameter
        p("joy_topic", "/joy")
        p("cmd_vel_topic", "/cmd_vel")
        p("joy_timeout_sec", 0.35)
        p("rate_hz", 50.0)
        p("deadzone", 0.08)
        # buttons / axes (canonical layout)
        p("button_walk", 4); p("button_cartesian", 5); p("button_select", 6)
        p("button_a", 0); p("button_b", 1); p("button_x", 2); p("button_y", 3)
        p("axis_lt", 2); p("axis_rt", 5); p("trigger_press", 0.5); p("trigger_release", 0.35)
        p("axis_left_x", 0); p("axis_left_y", 1); p("axis_right_x", 3); p("axis_right_y", 4)
        p("axis_dpad_x", 6); p("axis_dpad_y", 7)
        # speeds
        p("walk_max_vx", 0.2); p("walk_max_vy", 0.08); p("walk_max_wz", 0.4)
        p("cartesian_linear", 0.15); p("cartesian_damping", 0.03)
        p("joint_speed", 0.7)                 # rad/s at full stick
        p("jog_horizon_sec", 0.12)
        # presets (SRDF group states) under LT
        p("preset_home", ["both_arms", "home"])
        p("preset_hands_up", ["both_arms", "hands_up"])
        p("preset_reach", ["both_arms", "reach_forward"])
        p("preset_velocity_scale", 0.4)
        p("hand_close_duration", 0.6)
        p("use_sim_time_hint", False)

        g = lambda n: self.get_parameter(n).value
        self.g = g
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Joy, g("joy_topic"), self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_subscription(String, "/robot_description", self.on_urdf, latched)
        self.create_subscription(String, "/robot_description_semantic", self.on_srdf, latched)
        self.cmd_vel = self.create_publisher(Twist, g("cmd_vel_topic"), 10)
        self.status = self.create_publisher(String, "~/status", 10)
        self.kin = None
        self.traj = {n: self.create_publisher(JointTrajectory, "/%s_controller/joint_trajectory" % n, 10)
                     for n in ("left_arm", "right_arm", "head", "waist", "left_hand", "right_hand")}
        self.move_group = ActionClient(self, MoveGroup, "/move_action")

        self.joy = None
        self.joy_time = None
        self.prev_buttons = []
        self.q = {}
        self.limits = {}
        self.states = {}               # (group, name) -> {joint: value}
        self.chain = 0
        self.rt_on = self.lt_on = False
        self.jog = None                # chain targets while RT is held
        self.walking = False
        self.preset_busy = False
        self.create_timer(1.0 / g("rate_hz"), self.tick)
        self.say("ready: chain = %s (BACK to change)" % CHAINS[self.chain])

    # ------------------------------------------------------------------ inputs
    def on_joy(self, msg):
        self.joy, self.joy_time = msg, self.get_clock().now()

    def on_js(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def on_urdf(self, msg):
        self.kin = Kinematics(msg.data)
        for j in ET.fromstring(msg.data).findall("joint"):
            lim = j.find("limit")
            if lim is not None and j.get("type") in ("revolute", "prismatic"):
                self.limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))

    def on_srdf(self, msg):
        for gs in ET.fromstring(msg.data).findall("group_state"):
            self.states[(gs.get("group"), gs.get("name"))] = {
                j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
        self.get_logger().info("loaded %d named poses from the SRDF" % len(self.states))

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def axis(self, name):
        i = self.g(name)
        a = self.joy.axes
        v = a[i] if 0 <= i < len(a) else 0.0
        dz = self.g("deadzone")
        return 0.0 if abs(v) < dz else math.copysign((abs(v) - dz) / (1 - dz), v)

    def button(self, name):
        i = self.g(name)
        return 0 <= i < len(self.joy.buttons) and self.joy.buttons[i] == 1

    def pressed(self, name):
        i = self.g(name)
        b = self.joy.buttons
        was = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        return 0 <= i < len(b) and b[i] == 1 and was == 0

    def trigger(self, name, on):
        v = self.joy.axes[self.g(name)] if self.g(name) < len(self.joy.axes) else 0.0
        return v > self.g("trigger_release") if on else v > self.g("trigger_press")

    def clamp(self, joint, v):
        lo, hi = self.limits.get(joint, (-math.pi, math.pi))
        return min(max(v, lo + 0.02), hi - 0.02)

    def send_traj(self, ctrl, positions, horizon):
        msg = JointTrajectory()
        msg.joint_names = list(positions.keys())
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions.values()]
        pt.time_from_start = Duration(sec=int(horizon), nanosec=int((horizon % 1) * 1e9))
        msg.points = [pt]
        self.traj[ctrl].publish(msg)

    def stop_all(self):
        if self.walking:
            self.cmd_vel.publish(Twist())
            self.walking = False
        self.jog = None

    def cartesian_step(self, side, v, wrist_rate, rotation_rate, dt):
        """Move the palm with linear velocity v (base_link frame), position-only
        damped least squares over the 5 arm joints."""
        joints = ARM[side]
        q = dict(self.q)
        q.update(self.jog)
        J = self.kin.position_jacobian(q, "%s_palm" % side, joints)
        lam = self.g("cartesian_damping")
        dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(3), np.asarray(v, float))
        dq[4] += wrist_rate
        dq[2] += rotation_rate
        top = np.max(np.abs(dq))
        limit = self.g("joint_speed") * 1.5
        if top > limit:                       # keep the motion direction, cap joint speed
            dq *= limit / top
        for j, d in zip(joints, dq):
            self.jog[j] = self.clamp(j, self.jog[j] + d * dt)

    def selected_side(self):
        return "left" if CHAINS[self.chain] == "left_arm" else "right"

    def preset(self, group, name):
        if self.preset_busy:
            return
        target = self.states.get((group, name))
        if target is None:
            self.say("no SRDF pose %s/%s (is move_group running?)" % (group, name))
            return
        if not self.move_group.server_is_ready():
            self.say("MoveIt /move_action not available")
            return
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = group
        r.num_planning_attempts = 3
        r.allowed_planning_time = 3.0
        r.max_velocity_scaling_factor = self.g("preset_velocity_scale")
        r.max_acceleration_scaling_factor = self.g("preset_velocity_scale")
        c = Constraints()
        for j, v in target.items():
            c.joint_constraints.append(JointConstraint(joint_name=j, position=v, tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        r.goal_constraints = [c]
        goal.planning_options.plan_only = False
        self.preset_busy = True
        self.say("MoveIt: %s -> %s" % (group, name))

        def done(fut):
            gh = fut.result()
            if not gh.accepted:
                self.preset_busy = False
                self.say("MoveIt rejected %s/%s" % (group, name))
                return
            gh.get_result_async().add_done_callback(lambda f: self.preset_done(group, name, f))
        self.move_group.send_goal_async(goal).add_done_callback(done)

    def preset_done(self, group, name, fut):
        self.preset_busy = False
        code = fut.result().result.error_code.val
        self.say("MoveIt %s/%s %s" % (group, name, "done" if code == 1 else "failed (code %d)" % code))

    def hand(self, side, close):
        state = self.states.get(("%s_hand" % side, "close" if close else "open"))
        if state is None:     # fall back without MoveIt: fingers only
            state = {"%s_palm_to_finger%d_lower" % (side, i): (1.1 if close else 0.0) for i in range(1, 5)}
        self.send_traj("%s_hand" % side, state, self.g("hand_close_duration"))

    # ------------------------------------------------------------------ main loop
    def tick(self):
        if self.joy is None:
            return
        age = (self.get_clock().now() - self.joy_time).nanoseconds * 1e-9
        if age > self.g("joy_timeout_sec"):
            self.stop_all()
            return
        lb, rb = self.button("button_walk"), self.button("button_cartesian")
        self.rt_on = self.trigger("axis_rt", self.rt_on)
        self.lt_on = self.trigger("axis_lt", self.lt_on)

        if self.pressed("button_select"):
            self.chain = (self.chain + 1) % len(CHAINS)
            self.jog = None
            self.say("chain = %s" % CHAINS[self.chain])

        # ---- walking (LB) blocks everything else
        if lb:
            t = Twist()
            t.linear.x = float(self.axis("axis_left_y") * self.g("walk_max_vx"))
            t.linear.y = float(self.axis("axis_left_x") * self.g("walk_max_vy"))
            t.angular.z = float(self.axis("axis_right_x") * self.g("walk_max_wz"))
            self.cmd_vel.publish(t)
            self.walking = True
            self.jog = None
            self.prev_buttons = list(self.joy.buttons)
            return
        if self.walking:
            self.cmd_vel.publish(Twist())
            self.walking = False

        chain = CHAINS[self.chain]
        side = self.selected_side()

        dt = 1.0 / self.g("rate_hz")
        # ---- Cartesian hand jog (RB)
        if rb and chain != "head" and self.kin is not None:
            if self.jog is None or set(self.jog) != set(ARM[side]):
                if not all(j in self.q for j in ARM[side]):
                    self.prev_buttons = list(self.joy.buttons)
                    return
                self.jog = {j: self.q[j] for j in ARM[side]}
            lin = self.g("cartesian_linear")
            v = (self.axis("axis_left_y") * lin, self.axis("axis_left_x") * lin, self.axis("axis_dpad_y") * lin)
            spd = self.g("joint_speed")
            self.cartesian_step(side, v, self.axis("axis_right_x") * spd, self.axis("axis_right_y") * spd, dt)
            self.send_traj("%s_arm" % side, dict(self.jog), self.g("jog_horizon_sec"))

        # ---- joint jog (RT)
        elif self.rt_on:
            speed = self.g("joint_speed")
            if chain == "head":
                joints = {"chest_to_neck": "axis_left_x", "neck_to_head": "axis_left_y",
                          "waist_yaw_joint": "axis_right_x", "waist_pitch_joint": "axis_right_y",
                          "waist_roll_joint": "axis_dpad_x"}
            else:
                a = ARM[side]
                joints = {a[0]: "axis_left_y", a[1]: "axis_left_x", a[2]: "axis_right_x",
                          a[3]: "axis_right_y", a[4]: "axis_dpad_x"}
            if self.jog is None or set(self.jog) != set(joints):
                if not all(j in self.q for j in joints):
                    self.prev_buttons = list(self.joy.buttons)
                    return
                self.jog = {j: self.q[j] for j in joints}
            sign = {a: 1.0 for a in joints.values()}
            if chain != "head":
                # mirror abduction so "stick away from the body" opens either arm
                sign["axis_left_x"] = -1.0 if side == "right" else 1.0
            for j, ax in joints.items():
                self.jog[j] = self.clamp(j, self.jog[j] + sign[ax] * self.axis(ax) * speed * dt)
            if chain == "head":
                self.send_traj("head", {j: self.jog[j] for j in HEAD}, self.g("jog_horizon_sec"))
                self.send_traj("waist", {j: self.jog[j] for j in WAIST}, self.g("jog_horizon_sec"))
            else:
                self.send_traj("%s_arm" % side, dict(self.jog), self.g("jog_horizon_sec"))
        else:
            self.jog = None

        # ---- hands while an arm mode is held
        if rb or self.rt_on:
            sides = ("left", "right") if chain == "head" else (side,)
            if self.pressed("button_x"):
                for s in sides:
                    self.hand(s, close=False)
                self.say("open %s hand" % "/".join(sides))
            if self.pressed("button_b"):
                for s in sides:
                    self.hand(s, close=True)
                self.say("close %s hand" % "/".join(sides))

        # ---- planned presets (LT + face button / D-pad)
        elif self.lt_on:
            if self.pressed("button_y"):
                self.preset(*self.g("preset_home"))
            elif self.pressed("button_a"):
                self.preset("%s_arm" % side, "wave")
            elif self.pressed("button_b"):
                self.preset(*self.g("preset_hands_up"))
            elif self.pressed("button_x"):
                self.preset(*self.g("preset_reach"))
            dy = self.joy.axes[self.g("axis_dpad_y")] if self.g("axis_dpad_y") < len(self.joy.axes) else 0.0
            if dy > 0.5 and not getattr(self, "_dpad_latch", False):
                self.preset("head", "center"); self._dpad_latch = True
            elif dy < -0.5 and not getattr(self, "_dpad_latch", False):
                self.preset("head", "look_down"); self._dpad_latch = True
            elif abs(dy) < 0.2:
                self._dpad_latch = False

        self.prev_buttons = list(self.joy.buttons)


def main():
    rclpy.init()
    node = HruhJoystick()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
