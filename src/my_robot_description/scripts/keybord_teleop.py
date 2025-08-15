#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

JOINT_NAMES = [
    'right_shoulder_to_bisecp',
    'right_bisecp_to_elbow_inword',
    'right_forarm_to_wrist',
    'right_palm_to_thomb',
    'right_thomb_middle_to_thomb_upper',
    'right_palm_to_finger1_lower',
    'right_finger1_lower_to_finger1_middle',
    'right_finger1_middle_to_finger1_upper'
]

KEY_BINDINGS = {
    'q': 0,  # shoulder
    'w': 1,  # elbow
    'e': 2,  # wrist
    'r': 3,  # thumb base
    't': 4,  # thumb upper
    'y': 5,  # finger1 base
    'u': 6,  # finger1 mid
    'i': 7,  # finger1 tip
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.positions = [0.0] * len(JOINT_NAMES)
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Use keys q-i to control joints. Ctrl+C to exit.")

    def run(self):
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() in KEY_BINDINGS:
                        idx = KEY_BINDINGS[key.lower()]
                        delta = 0.1 if key.islower() else -0.1
                        self.positions[idx] += delta
                        msg = Float64MultiArray()
                        msg.data = self.positions
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Updated {JOINT_NAMES[idx]} to {self.positions[idx]:.2f}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()