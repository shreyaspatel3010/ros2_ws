"""RViz model viewer with joint sliders (no simulation, no ros2_control)."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("my_robot_description")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "my_robot.urdf.xacro"]), " hardware:=none"]),
        value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "urdf_config.rviz"])]),
    ])
