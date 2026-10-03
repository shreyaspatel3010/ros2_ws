"""Kinematic walking in RViz (no physics, no ros2_control, no MoveIt) with the
gamepad: hold LB and use the sticks to walk.  Heel-strike / toe-off roll-off is
on here (it is turned off in Gazebo).

    ros2 launch hruh_bringup walk_rviz.launch.py [walk:=true] [joystick:=false]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    desc = Path(get_package_share_directory("my_robot_description"))
    teleop = Path(get_package_share_directory("hruh_teleop"))
    return LaunchDescription([
        DeclareLaunchArgument("walk", default_value="false", description="Walk forward on its own."),
        DeclareLaunchArgument("joystick", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(desc / "launch" / "walk_demo.launch.py")),
                                 launch_arguments={"walk": LaunchConfiguration("walk")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop / "launch" / "joystick.launch.py")),
                                 condition=IfCondition(LaunchConfiguration("joystick"))),
    ])
