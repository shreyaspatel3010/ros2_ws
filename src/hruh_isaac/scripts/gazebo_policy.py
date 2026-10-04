#!/usr/bin/env python3
"""Run an exported HRUH actor through a dedicated Gazebo effort controller."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
# --scripted: every movement, each followed by a sudden stop (seconds, name, vx, vy, wz); 30 s
MOVEMENTS = [(3, "stand", 0, 0, 0), (4, "forward 0.4 m/s", 0.4, 0, 0), (3, "sudden stop", 0, 0, 0),
             (4, "side-step 0.3 m/s", 0, 0.3, 0), (3, "sudden stop", 0, 0, 0),
             (4, "turn 0.8 rad/s", 0, 0, 0.8), (3, "sudden stop", 0, 0, 0),
             (3, "backward 0.3 m/s", -0.3, 0, 0), (3, "sudden stop", 0, 0, 0)]


def movement_at(seconds):
    for duration, name, *command in MOVEMENTS:
        if seconds < duration:
            return name, tuple(command)
        seconds -= duration
    return MOVEMENTS[-1][1], tuple(MOVEMENTS[-1][2:])
# reach training box (pelvis frame, m) for bundles exported before bundle.json carried "target_box"
TARGET_BOX = [[0.22, 0.34], [-0.36, -0.22], [0.16, 0.32]]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))


def main():
    import numpy as np
    import torch
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    # newest sample only: a slow Python step must never work through a backlog of old states
    latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    from sensor_msgs.msg import Imu, JointState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import Empty, Float64MultiArray
    from hruh_lab.portable import PolicyIO
    from hruh_lab.velocity_control import VelocityGuard

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=ROOT / "src/hruh_isaac/policies/locomotion",
                        help="Exported policy folder (default: the promoted locomotion policy)")
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="Measured simulator time after receiving sensors; 0 = run until stopped")
    parser.add_argument("--scripted", action="store_true",
                        help="drive every movement with sudden stops (no /cmd_vel needed); report falls per movement")
    parser.add_argument("--hold", type=float, default=1.0,
                        help="Simulator seconds to hold the stand pose (pelvis held) before the policy starts")
    parser.add_argument("--report", type=Path, default=ROOT / "artifacts/hruh/gazebo_report.json")
    args, ros_args = parser.parse_known_args()
    contract = json.loads((args.bundle / "bundle.json").read_text())
    policy_path = args.bundle / "policy.pt"
    if not contract.get("simulation_only") or contract.get("schema_version") != 1:
        raise ValueError("Unsupported simulation policy bundle")
    if hashlib.sha256(policy_path.read_bytes()).hexdigest() != contract["policy_sha256"]:
        raise ValueError("Policy checksum does not match bundle")
    if hashlib.sha256((args.bundle / "robot.urdf").read_bytes()).hexdigest() != contract["urdf_sha256"]:
        raise ValueError("Robot checksum does not match bundle")
    torch.set_num_threads(1)
    model = torch.jit.load(str(policy_path), map_location="cpu").eval()
    io = PolicyIO(contract, args.bundle / "robot.urdf")
    rclpy.init(args=ros_args)

    class PolicyNode(Node):
        def __init__(self):
            # The control step runs on every /joint_states message (500 Hz in sim time) and uses the
            # message stamps as the clock: no /clock subscription or sim-time timer to keep up with.
            super().__init__("hruh_gazebo_policy", parameter_overrides=[Parameter("use_sim_time", value=False)])
            self.guard = VelocityGuard()
            self.positions, self.velocities = {}, {}
            self.quaternion, self.angular_velocity = [0,0,0,1], [0,0,0]
            self.joint_time = self.imu_time = self.object_time = -float("inf")
            self.object = None
            self.target = [0.28, -0.29, 0.24]   # centre of the reach training box
            self.targets = dict(contract["default_positions"])
            self.start = self.last = self.last_policy = None
            self.hold_start, self.released = None, False
            self.fallen = False
            self.done = False
            self.report = {"skill": contract["skill"], "simulator": "Gazebo Harmonic", "policy_steps": 0,
                           "falls": 0, "stale_sensor_steps": 0, "received_commands": 0,
                           "peak_abs_effort": 0.0, "min_pelvis_height": None, "elapsed_sim_s": 0.0}
            self.publisher = self.create_publisher(Float64MultiArray, "/policy_effort_controller/commands", 1)
            self.release = self.create_publisher(Empty, "/hruh/release", 1)   # detaches the pelvis holder
            self.create_subscription(JointState, "/joint_states", self.joints, latest)
            self.create_subscription(Imu, "/imu", self.imu, latest)
            self.create_subscription(Odometry, "/hruh/object_odom", self.object_odom, latest)
            self.velocity = None   # measured (vx, vy, wz) in the pelvis frame
            self.create_subscription(Odometry, "/odom", self.odom, latest)
            self.create_subscription(PoseStamped, "/hruh/hand_target", self.hand_target, 1)
            self.create_subscription(Twist, "/cmd_vel", self.command, 1)
            self.get_logger().info("Simulation effort controller ready; waiting for /clock and fresh sensors")
            print("HRUH_POLICY_READY", flush=True)   # policy_gazebo.launch.py unpauses the world on this

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        def joints(self, message):
            if len(message.name) != len(message.position) or len(message.name) != len(message.velocity):
                return
            if not np.isfinite(message.position).all() or not np.isfinite(message.velocity).all():
                return
            self.positions = dict(zip(message.name, message.position))
            self.velocities = dict(zip(message.name, message.velocity))
            self.joint_time = self.stamp(message)
            self.tick(self.joint_time)

        def imu(self, message):
            q, w = message.orientation, message.angular_velocity
            values = [q.x,q.y,q.z,q.w,w.x,w.y,w.z]
            if np.isfinite(values).all() and np.linalg.norm(values[:4]) > 0.5:
                self.quaternion, self.angular_velocity = values[:4], values[4:]
                self.imu_time = self.stamp(message)

        def odom(self, message):
            t = message.twist.twist   # gz odometry twist is in the pelvis (child) frame
            self.velocity = (t.linear.x, t.linear.y, t.angular.z)

        def object_odom(self, message):
            p, q, v, w = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear, message.twist.twist.angular
            from hruh_lab.portable import rotation
            # nav_msgs/Odometry twist is in child_frame_id; the policy expects world velocity.
            rot = rotation([q.x,q.y,q.z,q.w])
            self.object = [p.x,p.y,p.z,q.x,q.y,q.z,q.w, *(rot @ [v.x,v.y,v.z]), *(rot @ [w.x,w.y,w.z])]
            self.object_time = self.stamp(message)

        def hand_target(self, message):
            if message.header.frame_id != "base_link":
                self.get_logger().warning("Hand target must use base_link frame")
                return
            p = message.pose.position
            value = np.array([p.x,p.y,p.z])
            if np.isfinite(value).all():
                box = np.asarray(contract.get("target_box") or TARGET_BOX)
                self.target = np.clip(value, box[:, 0], box[:, 1]).tolist()

        def command(self, message):
            self.report["received_commands"] += 1
            self.guard.update((message.linear.x, message.linear.y, message.angular.z))

        def mark_fall(self):
            if not self.fallen:
                self.fallen = True
                self.report["falls"] += 1
                if args.scripted:
                    self.report["fell_during"] = movement_at(self.report["elapsed_sim_s"])[0]
                self.guard.fall()
                self.get_logger().warning("Fall detected; policy output stopped. Reset the Gazebo world to restart.")

        def publish(self, values):
            self.publisher.publish(Float64MultiArray(data=values))

        def tick(self, now):
            if now <= 0 or self.done:
                return
            if self.last is not None and now < self.last:
                io.reset()
                self.fallen = False
                self.start = self.last_policy = self.hold_start = None
                self.released = False
                self.guard.fall()
            self.last = now
            if self.start is not None:
                self.report["elapsed_sim_s"] = now - self.start
                if args.seconds > 0 and self.report["elapsed_sim_s"] >= args.seconds:
                    self.done = True
                    self.publish([0.0] * len(contract["effort_joints"]))
                    return
            fresh = now - self.joint_time < 0.06 and now - self.imu_time < 0.10
            complete = all(n in self.positions for n in contract["effort_joints"])
            if contract["skill"] == "lift":
                fresh = fresh and now - self.object_time < 0.10
            if not fresh or not complete:
                self.report["stale_sensor_steps"] += 1
                self.publish([0.0] * len(contract["effort_joints"]))
                return
            if not self.released:
                # Hold the stand pose with the trained PD gains, then hand over to the policy.
                if self.hold_start is None:
                    self.hold_start = now
                self.publish(io.efforts(dict(contract["default_positions"]), self.positions, self.velocities))
                if now - self.hold_start < args.hold:
                    return
                if not contract["fixed_base"]:
                    for _ in range(3):
                        self.release.publish(Empty())
                self.released = True
                io.reset()
                self.get_logger().info("Stand pose held; released, policy in control")
            if self.start is None:
                self.start = now
            self.report["elapsed_sim_s"] = now - self.start
            if args.seconds > 0 and self.report["elapsed_sim_s"] >= args.seconds:
                self.done = True
                self.publish([0.0] * len(contract["effort_joints"]))
                return
            from hruh_lab.portable import rotation
            if contract["skill"] == "locomotion" and not self.fallen:
                # the training terminations: tilt > 1 rad or pelvis below 0.45 m
                height = io.pelvis_height(self.positions, self.quaternion)
                old = self.report["min_pelvis_height"]
                self.report["min_pelvis_height"] = height if old is None else min(old, height)
                if rotation(self.quaternion)[2,2] < np.cos(1.0) or height < 0.45:
                    self.mark_fall()
            if self.fallen:
                self.publish([0.0] * len(contract["effort_joints"]))
                return
            if self.last_policy is None or now - self.last_policy >= contract["control_dt"] - 1e-6:
                if args.scripted:
                    name, command = movement_at(now - self.start)
                    if name != self.report.get("movement"):
                        self.get_logger().info(f"movement: {name}")
                        self.report["movement"] = name
                    self.guard.update(command)
                    if self.velocity is not None and not self.fallen:
                        # mean measured velocity per movement (did it really walk / turn / stop?)
                        stats = self.report.setdefault("measured_velocity", {}).setdefault(
                            name, {"command": list(command), "mean": [0.0, 0.0, 0.0], "samples": 0})
                        n = stats["samples"] = stats["samples"] + 1
                        stats["mean"] = [round(m + (v - m) / n, 3) for m, v in zip(stats["mean"], self.velocity)]
                if self.count_publishers("/policy_effort_controller/commands") > 1:
                    raise RuntimeError("Another effort-command publisher is active")
                observation = io.observation(self.positions, self.velocities, self.quaternion, self.angular_velocity,
                                             self.guard.step(contract["control_dt"]), self.target, self.object)
                with torch.inference_mode():
                    action = model(torch.from_numpy(observation).unsqueeze(0)).squeeze(0).numpy()
                self.targets = io.targets(action)
                self.last_policy = now
                self.report["policy_steps"] += 1
                if not self.fallen:
                    self.report["policy_rate_hz"] = round(self.report["policy_steps"] / max(now - self.start, 1e-3), 1)
            efforts = io.efforts(self.targets, self.positions, self.velocities)
            self.report["peak_abs_effort"] = max(self.report["peak_abs_effort"], max(abs(x) for x in efforts))
            self.publish(efforts)

    node = PolicyNode()
    wall_start = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.seconds > 0 and time.monotonic() - wall_start > max(120.0, args.seconds * 20):
                node.report["wall_timeout"] = True
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.report["interrupted"] = True
    finally:
        node.report["completed"] = node.done
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(node.report, indent=2) + "\n")
        print(json.dumps(node.report), flush=True)
        if rclpy.ok():
            node.publish([0.0] * len(contract["effort_joints"]))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
