#!/usr/bin/env python3
"""
Keyboard teleop for LEFT ARM (including fingers) — ROS 2 (rclpy)
Publishes to a joint_trajectory_controller.

Usage:
  python3 teleop_whole_robot.py \
    --controller /left_arm_controller/joint_trajectory \
    --joints chest_to_left_shoulder left_shoulder_to_bisecp left_bisecp_to_elbow_inword \
             left_forarm_lower_1_to_internal_support left_forarm_to_wrist left_forarm_upper_to_lower_1 \
             left_wrist_to_palm left_bisecp_uper_to_lower left_internal_support_to_cover \
             left_palm_to_finger1_lower left_palm_to_finger2_lower left_palm_to_finger3_lower \
             left_palm_to_finger4_lower left_palm_to_thomb left_S_to_bisecp_upper

Hotkeys (first 10 joints):
  q/a  w/s  e/d  r/f  t/g  y/h  u/j  i/k  o/l  p/;

Selection (any joint):
  [  prev   ]  next   ,  -sel   .  +sel

Other:
  p print  c zero  + step*1.5  - step/1.5  ? help  ESC quit
"""
import sys, time, select, termios, tty, argparse
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEFAULT_CONTROLLER = "/left_arm_controller/joint_trajectory"
DEFAULT_JOINTS = [
    "chest_to_left_shoulder",
    "left_shoulder_to_bisecp",
    "left_bisecp_to_elbow_inword",
    "left_forarm_lower_1_to_internal_support",
    "left_forarm_to_wrist",
    "left_forarm_upper_to_lower_1",
    "left_wrist_to_palm",
    "left_bisecp_uper_to_lower",
    "left_internal_support_to_cover",
    "left_palm_to_finger1_lower",
    "left_palm_to_finger2_lower",
    "left_palm_to_finger3_lower",
    "left_palm_to_finger4_lower",
    "left_palm_to_thomb",
    "left_S_to_bisecp_upper",
]
DEFAULT_STEP = 0.05
DEFAULT_RATE_HZ = 10.0

HELP = """
Controls (hotkeys for first 10 joints)
--------------------------------------
  q/a : joint 1 + / -
  w/s : joint 2 + / -
  e/d : joint 3 + / -
  r/f : joint 4 + / -
  t/g : joint 5 + / -
  y/h : joint 6 + / -
  u/j : joint 7 + / -
  i/k : joint 8 + / -
  o/l : joint 9 + / -
  p/; : joint 10 + / -

Selection mode (any number of joints)
-------------------------------------
  [   : select previous joint
  ]   : select next joint
  ,   : decrease selected joint
  .   : increase selected joint

Other
-----
  p   : print current targets
  c   : zero all targets
  +   : increase step (x1.5)
  -   : decrease step (/1.5)
  ?   : help
  ESC : quit
--------------------------------------
"""

PAIR_KEYS = [
    ('q','a'), ('w','s'), ('e','d'), ('r','f'), ('t','g'),
    ('y','h'), ('u','j'), ('i','k'), ('o','l'), ('p',';')
]

def getch_nonblock(timeout=0.0):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return sys.stdin.read(1)
    return None

class ArmTeleop(Node):
    def __init__(self, controller_topic: str, joints: List[str], step: float, rate_hz: float):
        super().__init__('arm_keyboard_left_with_fingers')
        if not joints:
            raise ValueError("No joints provided. Use --joints j1 j2 ... or edit DEFAULT_JOINTS.")
        self.joint_names = list(joints)
        self.target = [0.0]*len(self.joint_names)
        self.step = float(step)
        self.period = 1.0/float(rate_hz)
        self.selected = 0

        # Publisher
        self.pub_traj = self.create_publisher(JointTrajectory, controller_topic, 10)
        self.get_logger().info(f"Publishing JointTrajectory to {controller_topic}")

        # Seed from joint_states
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        self.get_logger().info("Ready. Press ? for help.")
        self.get_logger().info(f"Joints: {self.joint_names}")

    def _on_joint_states(self, msg: JointState):
        idx = {n:i for i,n in enumerate(self.joint_names)}
        for n, p in zip(msg.name, msg.position):
            i = idx.get(n)
            if i is not None:
                try:
                    self.target[i] = float(p)
                except Exception:
                    pass

    def _publish(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(self.target)
        pt.time_from_start = Duration(sec=0, nanosec=int(0.5*1e9))
        msg.points = [pt]
        self.pub_traj.publish(msg)

    def _bump(self, i: int, delta: float):
        if 0 <= i < len(self.target):
            self.target[i] += delta
            self._publish()

    def _bump_selected(self, delta: float):
        self._bump(self.selected, delta)
        self.get_logger().info(f"[{self.selected+1}/{len(self.joint_names)}] {self.joint_names[self.selected]} -> {self.target[self.selected]:.3f}")

    def handle_key(self, ch: str):
        if ch == '\x1b':
            raise KeyboardInterrupt
        if ch == '?':
            self.get_logger().info(HELP); return
        if ch == 'p':
            self.get_logger().info(f"Targets: {list(zip(self.joint_names, [round(v,3) for v in self.target]))}"); return
        if ch == 'c':
            self.target = [0.0]*len(self.target); self._publish(); return
        if ch == '+':
            self.step *= 1.5; self.get_logger().info(f"step -> {self.step:.4f}"); return
        if ch == '-':
            self.step /= 1.5; self.get_logger().info(f"step -> {self.step:.4f}"); return
        if ch == '[':
            self.selected = (self.selected - 1) % len(self.joint_names); self.get_logger().info(f"Selected: {self.selected+1}/{len(self.joint_names)} {self.joint_names[self.selected]}"); return
        if ch == ']':
            self.selected = (self.selected + 1) % len(self.joint_names); self.get_logger().info(f"Selected: {self.selected+1}/{len(self.joint_names)} {self.joint_names[self.selected]}"); return
        if ch == ',':
            self._bump_selected(-self.step); return
        if ch == '.':
            self._bump_selected(+self.step); return

        for i, (inc, dec) in enumerate(PAIR_KEYS):
            if i >= len(self.target): break
            if ch == inc:
                self._bump(i, +self.step); return
            if ch == dec:
                self._bump(i, -self.step); return

def main():
    parser = argparse.ArgumentParser(description="Keyboard controller for LEFT ARM + fingers (ROS 2)")
    parser.add_argument('--controller', default=DEFAULT_CONTROLLER, help='joint_trajectory_controller topic')
    parser.add_argument('--joints', nargs='*', default=DEFAULT_JOINTS, help='Joint names in order')
    parser.add_argument('--step', type=float, default=DEFAULT_STEP, help='Increment size (rad/m)')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ, help='Publish rate (Hz)')
    args = parser.parse_args()

    rclpy.init()
    node = ArmTeleop(controller_topic=args.controller, joints=args.joints, step=args.step, rate_hz=args.rate)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        last = 0.0
        while rclpy.ok():
            ch = getch_nonblock(0.01)
            if ch:
                node.handle_key(ch)
            now = time.time()
            if now - last >= node.period:
                node._publish()
                last = now
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()