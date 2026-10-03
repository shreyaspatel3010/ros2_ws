#!/usr/bin/env python3
"""Generate an isolated Gazebo effort-control model/world from a policy bundle.

The existing position-control Gazebo files are left intact. Generated files are
reviewable under artifacts/hruh/gazebo/<skill>/ and are never hardware configs.
"""
import argparse
import copy
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory


def prepare(bundle, output):
    bundle, output = Path(bundle).resolve(), Path(output).resolve()
    config = json.loads((bundle / "bundle.json").read_text())
    output.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory("my_robot_description"))
    xml = subprocess.run(["xacro", str(description / "urdf/my_robot.urdf.xacro"), "hardware:=gz",
        "enable_lidar:=false", "enable_stereo_cameras:=false", "enable_rgbd_camera:=false"],
        capture_output=True, text=True, check=True).stdout
    xml = re.sub(r"package://([^/]+)/([^\"']+)",
                 lambda match: str(Path(get_package_share_directory(match[1])) / match[2]), xml)
    robot = ET.fromstring(xml)
    joints = config["effort_joints"]
    control = robot.find("ros2_control")
    actual = []
    for joint in control.findall("joint"):
        command = joint.find("command_interface")
        if command is not None:
            command.set("name", "effort")
            actual.append(joint.get("name"))
            initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            initial.text = str(config["default_positions"][joint.get("name")])
    if set(actual) != set(joints):
        raise ValueError("Gazebo and trained actuator names do not match")
    controllers = {
        "controller_manager": {"ros__parameters": {"update_rate": 500,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "policy_effort_controller": {"type": "effort_controllers/JointGroupEffortController"}}},
        "policy_effort_controller": {"ros__parameters": {"joints": joints}},
    }
    controller_path = output / "controllers.yaml"
    controller_path.write_text(yaml.safe_dump(controllers, sort_keys=False))
    for plugin in robot.findall("gazebo/plugin"):
        if "GazeboSimROS2ControlPlugin" in plugin.get("name", ""):
            plugin.find("parameters").text = str(controller_path)
    # SDF insertion below supplies the fixed-world joint without changing the source URDF.
    model_urdf = output / "robot.urdf"
    ET.ElementTree(robot).write(model_urdf, encoding="utf-8", xml_declaration=True)
    sdf_text = subprocess.run(["gz", "sdf", "-p", str(model_urdf)], check=True,
                              capture_output=True, text=True).stdout
    sdf = ET.fromstring(sdf_text)
    model = sdf.find("model")
    model.set("name", "my_robot")
    pose = model.find("pose")
    if pose is None:
        pose = ET.SubElement(model, "pose")
    pose.text = " ".join(str(x) for x in config["initial_root_position"]) + " 0 0 0"
    if config["fixed_base"]:
        anchor = ET.SubElement(model, "joint", {"name": "policy_pelvis_anchor", "type": "fixed"})
        ET.SubElement(anchor, "parent").text = "world"
        ET.SubElement(anchor, "child").text = "base_link"
    model_file = output / "robot.sdf"
    ET.ElementTree(sdf).write(model_file, encoding="utf-8", xml_declaration=True)
    world = ET.parse(description / "worlds/test_world.sdf")
    world_node = world.getroot().find("world")
    world_node.set("name", "hruh_policy")
    world_node.find("gravity").text = "0 0 -9.81"
    # Remove unrelated models and rendering sensors from this transfer test.
    for item in list(world_node.findall("model")):
        if item.get("name") != "ground_plane":
            world_node.remove(item)
    for plugin in list(world_node.findall("plugin")):
        if plugin.get("name", "").endswith("::Sensors"):
            world_node.remove(plugin)
    if not config["fixed_base"]:
        # Hold the pelvis at the training reset height while the effort controller starts
        # (otherwise the robot collapses with zero torque before the policy runs).
        # gazebo_policy.py publishes /hruh/release once it holds the stand pose.
        holder = ET.fromstring('''<model name="policy_holder"><static>true</static><pose>0 0 3 0 0 0</pose>
          <link name="holder"/>
          <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
          <parent_link>holder</parent_link><child_model>my_robot</child_model><child_link>base_link</child_link>
          <detach_topic>/hruh/release</detach_topic></plugin></model>''')
        world_node.append(holder)
    if config["skill"] == "lift":
        table = ET.fromstring('''<model name="work_table"><static>true</static><pose>0.55 -0.3 1.16 0 0 0</pose>
          <link name="table"><collision name="surface"><geometry><box><size>0.4 0.45 0.1</size></box></geometry></collision>
          <visual name="surface"><geometry><box><size>0.4 0.45 0.1</size></box></geometry></visual></link></model>''')
        cube = ET.fromstring('''<model name="work_cube"><pose>0.415 -0.29 1.235 0 0 0</pose>
          <link name="cube"><inertial><mass>0.08</mass><inertia><ixx>0.000027</ixx><iyy>0.000027</iyy><izz>0.000027</izz></inertia></inertial>
          <collision name="cube"><geometry><box><size>0.045 0.045 0.045</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>0.8</mu2></ode></friction></surface></collision>
          <visual name="cube"><geometry><box><size>0.045 0.045 0.045</size></box></geometry></visual></link>
          <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
          <odom_frame>odom</odom_frame><robot_base_frame>cube</robot_base_frame><dimensions>3</dimensions>
          <odom_topic>/hruh/object_odom</odom_topic><odom_publish_frequency>100</odom_publish_frequency></plugin></model>''')
        table.find("pose").text = " ".join(map(str, config["table_position"])) + " 0 0 0"
        cube.find("pose").text = " ".join(map(str, config["object_position"])) + " 0 0 0"
        world_node.extend([table, cube])
    world_file = output / "world.sdf"
    world.write(world_file, encoding="utf-8", xml_declaration=True)
    bridge = yaml.safe_load((description / "config/gazebo_bridge.yaml").read_text())
    bridge = [item for item in bridge if item["ros_topic_name"] in ("/clock", "/imu", "/odom", "/tf")]
    bridge.append({"ros_topic_name": "/hruh/release", "gz_topic_name": "/hruh/release",
        "ros_type_name": "std_msgs/msg/Empty", "gz_type_name": "gz.msgs.Empty", "direction": "ROS_TO_GZ"})
    bridge.append({"ros_topic_name": "/hruh/object_odom", "gz_topic_name": "/hruh/object_odom",
        "ros_type_name": "nav_msgs/msg/Odometry", "gz_type_name": "gz.msgs.Odometry", "direction": "GZ_TO_ROS"})
    (output / "bridge.yaml").write_text(yaml.safe_dump(bridge, sort_keys=False))
    print(json.dumps({"model": str(model_file), "world": str(world_file), "controllers": str(controller_path)}))
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    prepare(args.bundle, args.output)
