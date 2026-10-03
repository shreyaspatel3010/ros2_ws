"""Gamepad for the whole HRUH robot: joy driver -> joy_layout_normalizer (from
aries_teleop) -> hruh_joystick.

    ros2 launch hruh_teleop joystick.launch.py [joy_driver:=joy_node joy_layout:=bluetooth]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("hruh_teleop")) / "config" / "joystick.yaml")
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    joy_dev = LaunchConfiguration("joy_dev")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("joy_driver", default_value="game_controller_node",
                              choices=["game_controller_node", "joy_node"]),
        DeclareLaunchArgument("joy_layout", default_value="auto",
                              choices=["auto", "dongle", "bluetooth", "game_controller", "passthrough"]),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        Node(package="joy", executable=LaunchConfiguration("joy_driver"), name="joy_node",
             parameters=[{"dev": joy_dev}], remappings=[("joy", "joy/raw")], output="screen"),
        Node(package="hruh_teleop", executable="joy_layout_normalizer.py", name="joy_layout_normalizer",
             parameters=[{"input_topic": "joy/raw", "output_topic": "joy",
                          "layout": LaunchConfiguration("joy_layout"), "device": joy_dev}], output="screen"),
        Node(package="hruh_teleop", executable="hruh_joystick.py", name="hruh_joystick",
             parameters=[config, sim], output="screen"),
    ])
