"""Full HRUH simulation: Gazebo + ros2_control + walker (my_robot_description),
MoveIt (hruh_moveit_config) and the gamepad (hruh_teleop).

    ros2 launch hruh_bringup sim.launch.py [joystick:=false] [moveit:=false]
        [sensors:=false] [gui:=false] [rviz:=false] [walk:=true]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc = Path(get_package_share_directory("my_robot_description"))
    moveit = Path(get_package_share_directory("hruh_moveit_config"))
    teleop = Path(get_package_share_directory("hruh_teleop"))
    return LaunchDescription([
        DeclareLaunchArgument("joystick", default_value="true"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("walk", default_value="false"),
        DeclareLaunchArgument("moveit", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(desc / "launch" / "gazebo.launch.py")),
            launch_arguments={"rviz": "false", "sensors": LaunchConfiguration("sensors"),
                              "gui": LaunchConfiguration("gui"), "walk": LaunchConfiguration("walk")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit / "launch" / "moveit.launch.py")),
            launch_arguments={"hardware": "gz", "use_sim_time": "true", "rviz": LaunchConfiguration("rviz")}.items(),
            condition=IfCondition(LaunchConfiguration("moveit"))),
        # without MoveIt, show the plain robot view instead
        Node(package="rviz2", executable="rviz2", output="log",
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration("rviz"), "' == 'true' and '",
                                                     LaunchConfiguration("moveit"), "' != 'true'"])),
             arguments=["-d", str(desc / "rviz" / "urdf_config.rviz"), "-f", "odom"],
             parameters=[{"use_sim_time": True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(teleop / "launch" / "joystick.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items(),
            condition=IfCondition(LaunchConfiguration("joystick"))),
    ])
