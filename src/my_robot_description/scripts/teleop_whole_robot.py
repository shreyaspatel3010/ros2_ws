#!/usr/bin/env python3
import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

HELP = """
Controls:
  Base (DiffDrive):
    w/s : linear x +/-
    a/d : angular z +/-
    space: stop base

  Arm (JointPositionController on arm_base_forearm_joint):
    j/k : joint +/-
    r   : joint -> 0.0 rad

  h   : print this help
  q   : quit
"""

class WholeRobotTeleop(Node):
    def __init__(self):
        super().__init__('whole_robot_teleop')
        self.pub_twist = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)
        self.pub_joint = self.create_publisher(Float64, '/arm_base_forearm_joint/cmd_pos', 10)

        self.lin = 0.0
        self.ang = 0.0
        self.jpos = 0.0

        self.lin_step = 0.05
        self.ang_step = 0.05
        self.j_step   = 0.05

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.get_logger().info(HELP)

    def tick(self):
        # publish current base cmd
        t = Twist()
        t.linear.x = self.lin
        t.angular.z = self.ang
        self.pub_twist.publish(t)

        # periodically re-publish joint target so late subscribers catch up
        j = Float64()
        j.data = self.jpos
        self.pub_joint.publish(j)

    def handle_key(self, ch):
        if ch in ('h', 'H'):
            self.get_logger().info(HELP)
        elif ch == ' ':
            self.lin = 0.0; self.ang = 0.0
        elif ch in ('w', 'W'):
            self.lin += self.lin_step
        elif ch in ('s', 'S'):
            self.lin -= self.lin_step
        elif ch in ('a', 'A'):
            self.ang += self.ang_step
        elif ch in ('d', 'D'):
            self.ang -= self.ang_step
        elif ch in ('j', 'J'):
            self.jpos += self.j_step
        elif ch in ('k', 'K'):
            self.jpos -= self.j_step
        elif ch in ('r', 'R'):
            self.jpos = 0.0
        elif ch in ('q', 'Q'):
            rclpy.shutdown()

def getch_nonblock():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def run():
    rclpy.init()
    node = WholeRobotTeleop()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ch = getch_nonblock()
            if ch:
                node.handle_key(ch)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == '__main__':
    run()
