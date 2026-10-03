"""HRUH without a simulator: ros2_control mock hardware (echoes every command),
the walker (streams the legs, publishes odom -> base_link), MoveIt and the
gamepad.

    ros2 launch hruh_bringup demo.launch.py [joystick:=false] [rviz:=false] [walk:=true]

Plan with the MotionPlanning panel (groups: left_arm, right_arm, both_arms,
left_hand, right_hand, head, waist), or drive everything from a gamepad
(hruh_teleop).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONTROLLERS = ["joint_state_broadcaster", "legs_controller", "waist_controller", "head_controller",
               "left_arm_controller", "right_arm_controller", "left_hand_controller", "right_hand_controller"]


def generate_launch_description():
    desc = Path(get_package_share_directory("my_robot_description"))
    moveit = Path(get_package_share_directory("hruh_moveit_config"))
    controllers = Path(get_package_share_directory("hruh_control")) / "config" / "ros2_controllers.yaml"
    robot_description = ParameterValue(
        Command(["xacro ", str(desc / "urdf" / "my_robot.urdf.xacro"), " hardware:=mock"]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("joystick", default_value="true", description="Start hruh_teleop (gamepad)."),
        DeclareLaunchArgument("walk", default_value="false", description="Walk forward on its own."),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[str(controllers)],
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=CONTROLLERS + ["--controller-manager-timeout", "60"]),
        # no physics: the walker's own pelvis plan is the odometry
        Node(package="my_robot_description", executable="hruh_walker.py", output="screen",
             parameters=[{"robot_description": robot_description, "mode": "ros2_control",
                          "publish_odom_tf": True, "balance": False,
                          "auto_walk": ParameterValue(LaunchConfiguration("walk"), value_type=bool)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / "launch" / "moveit.launch.py")),
                                 launch_arguments={"hardware": "mock", "rviz": LaunchConfiguration("rviz")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("hruh_teleop")) / "launch" / "joystick.launch.py")),
            condition=IfCondition(LaunchConfiguration("joystick"))),
    ])
