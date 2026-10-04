"""Run a trained HRUH policy in Gazebo: exported actor -> bounded PD -> effort controller.

    ros2 launch hruh_bringup policy_gazebo.launch.py joystick:=true
        [bundle:=<exported folder>] [seconds:=0] [gui:=true]

Default bundle: src/hruh_isaac/policies/locomotion, the policy that
train_robot_offline.sh promoted after it passed the Isaac benchmark.  The pelvis is
held for --hold seconds while the stand pose settles, then released to the policy.
seconds:=0 runs until stopped (train_robot_offline.sh uses a timed test).
Hold LB on the gamepad and use the sticks to walk (same as the walker bringups).
"""
import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path(LaunchConfiguration("workspace").perform(context))
    bundle = Path(LaunchConfiguration("bundle").perform(context) or root / "src/hruh_isaac/policies/locomotion").resolve()
    if not (bundle / "bundle.json").is_file():
        return [LogInfo(msg=f"[policy_gazebo] no policy bundle at {bundle}. Train one with ./train_robot_offline.sh "
                            "(passing policies are promoted to src/hruh_isaac/policies/) or pass bundle:=<folder>"),
                EmitEvent(event=Shutdown(reason="no policy"))]
    # promoted bundles live in policies/<skill>; exported ones in <run>/exported
    output = root / "artifacts/hruh/gazebo" / (bundle.name if bundle.name != "exported" else bundle.parent.name)
    subprocess.run(["/usr/bin/python3", str(root / "src/hruh_isaac/scripts/prepare_gazebo.py"),
                    "--bundle", str(bundle), "--output", str(output)], check=True)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    contract = json.loads((bundle / "bundle.json").read_text())
    gz_launch = Path(get_package_share_directory("ros_gz_sim")) / "launch/gz_sim.launch.py"
    teleop_launch = Path(get_package_share_directory("hruh_teleop")) / "launch/joystick.launch.py"
    policy = ExecuteProcess(cmd=[LaunchConfiguration("python"), str(root / "src/hruh_isaac/scripts/gazebo_policy.py"),
        "--bundle", str(bundle), "--seconds", LaunchConfiguration("seconds"),
        "--report", str(output / "report.json")]
        + (["--scripted"] if LaunchConfiguration("scripted").perform(context) == "true" else []), output="screen")
    return [
        RegisterEventHandler(OnProcessExit(target_action=policy,
            on_exit=[EmitEvent(event=Shutdown(reason="Policy test finished"))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_launch)),
            launch_arguments={"gz_args": str(output / "world.sdf") + (" -r" if gui else " -r -s")}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": (output / "robot.urdf").read_text(), "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
            # create's -x/-y/-z default to 0 and override the SDF pose: pass the training spawn pose
            arguments=["-world", "hruh_policy", "-file", str(output / "robot.sdf"), "-name", "my_robot",
                       *[arg for axis, value in zip("xyz", contract["initial_root_position"])
                         for arg in (f"-{axis}", f"{value:.4f}")]]),
        Node(package="controller_manager", executable="spawner", output="screen",
            arguments=["joint_state_broadcaster", "policy_effort_controller", "--activate-as-group",
                       "--controller-manager-timeout", "120"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
            parameters=[{"config_file": str(output / "bridge.yaml"), "use_sim_time": True}]),
        policy,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop_launch)),
            launch_arguments={"use_sim_time": "false"}.items(), condition=IfCondition(LaunchConfiguration("joystick"))),
    ]


def generate_launch_description():
    workspace = str(Path(get_package_prefix("hruh_bringup")).parent.parent)
    return LaunchDescription([
        DeclareLaunchArgument("bundle", default_value="",
                              description="Folder with policy.pt + bundle.json (default: promoted locomotion policy)"),
        DeclareLaunchArgument("workspace", default_value=workspace),
        DeclareLaunchArgument("python", default_value="/opt/isaac/venv-6.1/bin/python"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("joystick", default_value="false"),
        DeclareLaunchArgument("seconds", default_value="0", description="Policy run time in sim seconds; 0 = until stopped"),
        DeclareLaunchArgument("scripted", default_value="false",
                              description="true: walk every movement with sudden stops by itself (transfer test)"),
        OpaqueFunction(function=setup),
    ])
