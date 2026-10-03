#!/usr/bin/env python3
"""Stand-in for hruh_isaac_sim.py with the same ROS 2 interface, for testing the
ros2_control / MoveIt / RViz side without a GPU:

    /isaac_joint_commands -> first-order joint tracking -> /isaac_joint_states (+ mimic joints)
    /clock (wall time), /imu (always upright, for the policy runner's plumbing)

No physics, no odometry: run the walker with publish_odom_tf:=true.
"""
import argparse
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState


class FakeIsaac(Node):
    def __init__(self, urdf, tau, rate):
        super().__init__("fake_isaac")
        root = ET.parse(urdf).getroot()
        self.mimic = {j.get("name"): (j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", "1")),
                                      float(j.find("mimic").get("offset", "0")))
                      for j in root.findall("joint") if j.find("mimic") is not None}
        self.names = [j.get("name") for j in root.findall("joint") if j.get("type") in ("revolute", "continuous")]
        self.q = {n: 0.0 for n in self.names}
        self.v = {n: 0.0 for n in self.names}
        self.target = {}
        self.alpha = min(1.0, 1.0 / (tau * rate))
        self.dt = 1.0 / rate
        self.pub = self.create_publisher(JointState, "/isaac_joint_states", qos_profile_sensor_data)
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.imu = self.create_publisher(Imu, "/imu", 10)   # reliable, like Isaac (the walker needs it)
        self.create_subscription(JointState, "/isaac_joint_commands", self.on_cmd, 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"fake Isaac: {len(self.names)} joints ({len(self.mimic)} mimic)")

    def on_cmd(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.target[n] = p

    def tick(self):
        for n in self.names:
            if n in self.mimic:
                ref, mult, off = self.mimic[n]
                new = self.q[ref] * mult + off
            else:
                new = self.q[n] + self.alpha * (self.target.get(n, self.q[n]) - self.q[n])
            self.v[n] = (new - self.q[n]) / self.dt
            self.q[n] = new
        now = time.time()
        stamp = rclpy.time.Time(seconds=now).to_msg()
        self.clock.publish(Clock(clock=stamp))
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        msg.position = [self.q[n] for n in self.names]
        msg.velocity = [self.v[n] for n in self.names]
        self.pub.publish(msg)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu_frame"
        imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.81
        self.imu.publish(imu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--tau", type=float, default=0.05, help="joint tracking time constant (s)")
    ap.add_argument("--rate", type=float, default=200.0)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeIsaac(a.urdf, a.tau, a.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
