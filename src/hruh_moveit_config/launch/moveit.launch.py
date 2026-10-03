"""MoveIt for HRUH: move_group and RViz.

MoveIt Servo is not used: the HRUH arms are 5-DOF and Servo in Jazzy assumes at
least 6 (its singularity check reads the 6th singular value of the Jacobian and
emergency-stops every command).  Cartesian hand jogging is done by
hruh_teleop/hruh_joystick instead.

Brings up planning only; something else must provide the controllers:
hruh_bringup demo.launch.py (mock hardware) or sim.launch.py (Gazebo) include
this file.

    hardware:=mock|gz|isaac   which ros2_control hardware the robot_description describes
    use_sim_time:=...   true with Gazebo
    rviz:=false         no RViz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def moveit_config(hardware):
    xacro = Path(get_package_share_directory("my_robot_description")) / "urdf" / "my_robot.urdf.xacro"
    return (MoveItConfigsBuilder("my_robot", package_name="hruh_moveit_config")
            .robot_description(file_path=str(xacro), mappings={"hardware": hardware})
            .robot_description_semantic(file_path="config/hruh.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .planning_scene_monitor(publish_robot_description=False,
                                    publish_robot_description_semantic=True)
            .to_moveit_configs())


def setup(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    cfg = moveit_config(hardware)
    share = Path(get_package_share_directory("hruh_moveit_config"))
    nodes = [
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[cfg.to_dict(), {"use_sim_time": sim_time}]),
        Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(share / "config" / "moveit.rviz")],
             parameters=[cfg.robot_description, cfg.robot_description_semantic, cfg.robot_description_kinematics,
                         cfg.planning_pipelines, cfg.joint_limits, {"use_sim_time": sim_time}]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "gz", "isaac"]),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
