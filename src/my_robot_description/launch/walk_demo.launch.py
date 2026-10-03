"""Kinematic walking demo in RViz (no physics, no ros2_control).

The walker publishes /joint_states and odom -> base_link itself.  Drive it with
/cmd_vel (teleop_twist_keyboard, or the joystick from hruh_moveit_config), or
walk:=true to walk forward on its own.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("my_robot_description")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "my_robot.urdf.xacro"]), " hardware:=none"]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("walk", default_value="true", description="Walk forward without /cmd_vel."),
        DeclareLaunchArgument("vx", default_value="0.2", description="Auto-walk speed (m/s)."),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="my_robot_description", executable="hruh_walker.py", output="screen",
             parameters=[{"robot_description": robot_description, "mode": "kinematic",
                          "auto_walk": ParameterValue(LaunchConfiguration("walk"), value_type=bool),
                          "auto_vx": ParameterValue(LaunchConfiguration("vx"), value_type=float)}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "urdf_config.rviz"]), "-f", "odom"]),
    ])
