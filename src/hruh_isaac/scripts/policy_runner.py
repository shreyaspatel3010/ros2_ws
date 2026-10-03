#!/usr/bin/env python3
"""Run promoted HRUH policies on the robot through ros2_control.

Started by `ros2 launch hruh_bringup isaac.launch.py controller:=policy` (learned
walking) and/or `reach:=true` (learned right-arm reaching).  It drives the same
controllers as the rest of the stack, so MoveIt, RViz and the gamepad keep working:

  locomotion  /joint_states + /imu + /cmd_vel  ->  policy (50 Hz)
              -> /legs_controller/commands, /waist_position_controller/commands
              The robot first moves to the trained stand pose (--settle s), then the
              policy balances and walks.  Hold LB on the gamepad to walk.
              Motion policies (trained with moving arms) also swing the arms with the
              gait while walking (--no-arm-swing: MoveIt keeps the arms while walking);
              while standing the arms are free for MoveIt / the gamepad.
  reach       goal: geometry_msgs/PoseStamped on /hruh/hand_target (frame base_link,
              clamped to the trained workspace) -> policy -> /right_arm_controller

Status: std_msgs/String on /hruh_policy/status.  A fall (tilt > 1 rad or pelvis
< 0.45 m, the training terminations) latches the walking policy off: the legs hold
their last targets and /cmd_vel is ignored until restart.

The control step runs on each /joint_states message and uses message stamps as the
clock (simulation time), so it needs no /clock subscription of its own.
Policies are simulation-only: they were trained and checked in Isaac Sim.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))
POLICIES = ROOT / "src/hruh_isaac/policies"
TARGET_BOX = [[0.22, 0.34], [-0.36, -0.22], [0.16, 0.32]]   # reach training box, bundles without "target_box"


def load_bundle(folder, skill):
    import torch
    from hruh_lab.portable import PolicyIO
    folder = Path(folder)
    contract = json.loads((folder / "bundle.json").read_text())
    if contract.get("schema_version") != 1 or contract.get("skill") != skill:
        raise ValueError(f"{folder}: not a schema-1 {skill} bundle")
    if hashlib.sha256((folder / "policy.pt").read_bytes()).hexdigest() != contract["policy_sha256"]:
        raise ValueError(f"{folder}: policy.pt checksum differs from bundle.json")
    model = torch.jit.load(str(folder / "policy.pt"), map_location="cpu").eval()
    return contract, model, PolicyIO(contract, folder / "robot.urdf")


def controller_joints(yaml_path):
    import yaml
    config = yaml.safe_load(Path(yaml_path).read_text())
    return {name: value["ros__parameters"]["joints"] for name, value in config.items()
            if isinstance(value, dict) and "joints" in value.get("ros__parameters", {})}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--locomotion", default="", help=f"bundle folder ('' = off; default policy: {POLICIES}/locomotion)")
    parser.add_argument("--reach", default="", help="bundle folder ('' = off)")
    parser.add_argument("--controllers", required=True, help="hruh_control/config/ros2_controllers.yaml")
    parser.add_argument("--settle", type=float, default=2.0, help="seconds to move into the trained stand pose")
    parser.add_argument("--reach-seconds", type=float, default=4.0, help="policy time per reach goal")
    parser.add_argument("--no-arm-swing", action="store_true",
                        help="motion policies: do not swing the arms while walking")
    parser.add_argument("--no-arm-pose", action="store_true",
                        help="do not move arms / head / hands to the pose the walking policy was trained with")
    args, ros_args = parser.parse_known_args()
    if not args.locomotion and not args.reach:
        parser.error("nothing to run: pass --locomotion and/or --reach")

    import numpy as np
    import torch
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped, Twist
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray, String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from hruh_lab.joints import arm_swing_targets
    from hruh_lab.portable import rotation
    from hruh_lab.velocity_control import VelocityGuard

    torch.set_num_threads(1)
    joints_of = controller_joints(args.controllers)
    latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)

    def trajectory(names, values, seconds):
        point = JointTrajectoryPoint(positions=[float(v) for v in values],
                                     time_from_start=Duration(sec=int(seconds), nanosec=int(seconds % 1 * 1e9)))
        return JointTrajectory(joint_names=list(names), points=[point])

    class Runner(Node):
        def __init__(self):
            super().__init__("hruh_policy_runner")
            self.positions, self.velocities = {}, {}
            self.quaternion, self.angular_velocity, self.imu_time = None, None, -float("inf")
            self.status = self.create_publisher(String, "/hruh_policy/status", 10)
            self.create_subscription(JointState, "/joint_states", self.on_joints, latest)
            self.create_subscription(Imu, "/imu", self.on_imu, latest)
            self.walk = self.reach = None
            if args.locomotion:
                contract, model, io = load_bundle(args.locomotion, "locomotion")
                legs, waist = joints_of["legs_controller"], joints_of["waist_position_controller"]
                if sorted(contract["policy_joints"]) != sorted(legs + waist):
                    raise ValueError("walking policy joints differ from legs_controller + waist_position_controller")
                swing = contract.get("arm_motion") if not args.no_arm_swing else None
                self.walk = dict(contract=contract, model=model, io=io, legs=legs, waist=waist, phase="wait",
                                 guard=VelocityGuard(), last=None, start=None, targets=None,
                                 swing=swing, swinging=False)
                self.legs_pub = self.create_publisher(Float64MultiArray, "/legs_controller/commands", 1)
                self.waist_pub = self.create_publisher(Float64MultiArray, "/waist_position_controller/commands", 1)
                self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 1)
                self.pose_pubs = {name: self.create_publisher(JointTrajectory, f"/{name}/joint_trajectory", 1)
                                  for name in ("left_arm_controller", "right_arm_controller", "head_controller",
                                               "left_hand_controller", "right_hand_controller")}
            if args.reach:
                contract, model, io = load_bundle(args.reach, "reach")
                if contract["policy_joints"] != joints_of["right_arm_controller"]:
                    raise ValueError("reach policy joints differ from right_arm_controller")
                self.reach = dict(contract=contract, model=model, io=io, goal=None, start=None, last=None,
                                  box=np.asarray(contract.get("target_box") or TARGET_BOX))
                self.arm_pub = self.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 1)
                self.create_subscription(PoseStamped, "/hruh/hand_target", self.on_hand_target, 1)
            self.say("policy runner ready: " + ", ".join(
                n for n, on in (("walking", self.walk), ("reaching", self.reach)) if on))

        def say(self, text, warn=False):
            if warn:   # rclpy: one severity per logging call site
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self.status.publish(String(data=text))

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        def on_imu(self, message):
            q, w = message.orientation, message.angular_velocity
            values = [q.x, q.y, q.z, q.w, w.x, w.y, w.z]
            if np.isfinite(values).all() and np.linalg.norm(values[:4]) > 0.5:
                self.quaternion, self.angular_velocity = values[:4], values[4:]
                self.imu_time = self.stamp(message)

        def on_cmd_vel(self, message):
            self.walk["guard"].update((message.linear.x, message.linear.y, message.angular.z))

        def on_hand_target(self, message):
            if message.header.frame_id not in ("base_link", ""):
                self.say("hand target must be in base_link", warn=True)
                return
            p = message.pose.position
            goal = np.array([p.x, p.y, p.z])
            if not np.isfinite(goal).all():
                return
            box = self.reach["box"]
            clamped = np.clip(goal, box[:, 0], box[:, 1])
            if np.linalg.norm(clamped - goal) > 1e-3:
                self.say(f"hand target clamped to the trained workspace: {np.round(clamped, 3).tolist()}", warn=True)
            self.reach.update(goal=clamped, start=None, last=None)
            self.reach["io"].reset()

        def on_joints(self, message):
            if len(message.name) != len(message.position) or len(message.velocity) != len(message.name):
                return
            if not (np.isfinite(message.position).all() and np.isfinite(message.velocity).all()):
                return
            self.positions.update(zip(message.name, message.position))
            self.velocities.update(zip(message.name, message.velocity))
            now = self.stamp(message)
            if self.walk:
                self.step_walk(now)
            if self.reach:
                self.step_reach(now)

        # ---------------------------------------------------------------- walking
        def publish_walk(self, targets):
            w = self.walk
            self.legs_pub.publish(Float64MultiArray(data=[float(targets[n]) for n in w["legs"]]))
            self.waist_pub.publish(Float64MultiArray(data=[float(targets[n]) for n in w["waist"]]))

        def step_walk(self, now):
            w, c = self.walk, self.walk["contract"]
            names, defaults = c["policy_joints"], c["default_positions"]
            if self.quaternion is None or abs(now - self.imu_time) > 0.15 \
                    or not all(n in self.positions for n in w["io"].observed):
                return   # stale or missing sensors: controllers hold the last targets
            if w["phase"] == "wait":
                w.update(phase="settle", start=now, origin={n: self.positions[n] for n in names})
                if not args.no_arm_pose:
                    for name, pub in self.pose_pubs.items():
                        js = joints_of[name]
                        pub.publish(trajectory(js, [defaults.get(j, 0.0) for j in js], args.settle))
                self.say(f"moving to the trained stand pose ({args.settle:.1f} s)")
            if w["phase"] == "settle":
                alpha = min(1.0, (now - w["start"]) / max(args.settle, 1e-3))
                w["targets"] = {n: w["origin"][n] + alpha * (defaults[n] - w["origin"][n]) for n in names}
                self.publish_walk(w["targets"])
                # Isaac releases the pelvis ~10 frames (0.17 s) after the stand pose is commanded
                if now - w["start"] >= args.settle + 0.25:
                    w["io"].reset()
                    w.update(phase="run", last=None)
                    self.say("walking policy in control (hold LB + sticks to walk)")
                return
            if w["phase"] != "run":
                return
            height = w["io"].pelvis_height(self.positions, self.quaternion)
            if rotation(self.quaternion)[2, 2] < np.cos(1.0) or height < 0.45:
                w["phase"] = "fallen"
                w["guard"].fall()
                self.say(f"FALL detected (pelvis {height:.2f} m): walking policy stopped; restart to retry", warn=True)
                return
            if w["last"] is not None and now - w["last"] < c["control_dt"] - 1e-4:
                return
            w["last"] = now if w["last"] is None or now - w["last"] > 2 * c["control_dt"] else w["last"] + c["control_dt"]
            obs = w["io"].observation(self.positions, self.velocities, self.quaternion, self.angular_velocity,
                                      w["guard"].step(c["control_dt"]))
            with torch.inference_mode():
                action = w["model"](torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
            w["targets"] = w["io"].targets(action)
            self.publish_walk(w["targets"])
            if w["swing"]:
                self.swing_arms(w, c)

        def swing_arms(self, w, c):
            """Counter-swing while walking (as in training); back to the stand pose on stopping."""
            command = w["guard"].value
            walking = abs(command[0]) > 0.03 or abs(command[1]) > 0.03 or abs(command[2]) > 0.1
            if walking:
                targets = arm_swing_targets(self.positions["left_hip_pitch_joint"],
                                            self.positions["right_hip_pitch_joint"],
                                            gain=w["swing"]["swing_gain"], stand=c["default_positions"])
                seconds = 2 * c["control_dt"]
            elif w["swinging"]:
                targets = {n: c["default_positions"][n] for n in w["swing"]["joints"]}
                seconds = 0.6
            else:
                return   # standing: the arms belong to MoveIt / the gamepad
            for side in ("left", "right"):
                names = joints_of[f"{side}_arm_controller"]
                self.pose_pubs[f"{side}_arm_controller"].publish(
                    trajectory(names, [targets[n] for n in names], seconds))
            w["swinging"] = walking

        # ---------------------------------------------------------------- reaching
        def step_reach(self, now):
            r, c = self.reach, self.reach["contract"]
            names = c["policy_joints"]
            if r["goal"] is None or not all(n in self.positions for n in names):
                return
            if r["start"] is None:
                r["start"] = now
                self.say(f"reaching to {np.round(r['goal'], 3).tolist()} (base_link)")
            if now - r["start"] > args.reach_seconds:
                wrist = r["io"].kinematics.transform("right_wrist", self.positions)[:3, 3]
                error = float(np.linalg.norm(wrist - r["goal"]))
                self.say(f"reach finished: wrist error {error * 100:.1f} cm "
                         f"({'success' if error <= 0.025 else 'missed the 2.5 cm goal'})", warn=error > 0.025)
                r["goal"] = None
                return
            if r["last"] is not None and now - r["last"] < c["control_dt"] - 1e-4:
                return
            r["last"] = now
            obs = r["io"].observation(self.positions, self.velocities, [0, 0, 0, 1], [0, 0, 0],
                                      hand_target=tuple(r["goal"]))
            with torch.inference_mode():
                action = r["model"](torch.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
            targets = r["io"].targets(action)
            self.arm_pub.publish(trajectory(names, [targets[n] for n in names], 2 * c["control_dt"]))

    rclpy.init(args=ros_args)
    node = Runner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
