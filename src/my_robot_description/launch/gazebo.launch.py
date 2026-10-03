"""HRUH humanoid in Gazebo Sim (Harmonic) with ros2_control and the walker.

    ros2 launch my_robot_description gazebo.launch.py [walk:=true] [rviz:=false]
        [sensors:=false] [gui:=false] [world:=<sdf>]

walk:=true      walk forward on its own (vx:=0.15 m/s); otherwise stand and follow /cmd_vel
sensors:=false  skip stereo / RGB-D / LiDAR (much faster simulation)
gui:=false      Gazebo server only (headless)

hruh_moveit_config/launch/gazebo.launch.py includes this file and adds MoveIt,
MoveIt Servo and the joystick on top.
"""
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

CONTROLLERS = ["joint_state_broadcaster", "legs_controller", "waist_controller", "head_controller",
               "left_arm_controller", "right_arm_controller", "left_hand_controller", "right_hand_controller"]


def generate_launch_description():
    pkg = FindPackageShare("my_robot_description")
    sensors = LaunchConfiguration("sensors")
    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([pkg, "urdf", "my_robot.urdf.xacro"]), " hardware:=gz",
        " enable_lidar:=", sensors, " enable_stereo_cameras:=", sensors, " enable_rgbd_camera:=", sensors]),
        value_type=str)
    sim = {"use_sim_time": True}
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument("walk", default_value="false"),
        DeclareLaunchArgument("vx", default_value="0.15", description="auto-walk speed (m/s)"),
        # Heel-to-toe roll-off works kinematically (walk_demo, GLB animation) but makes
        # the feet slip and tip on Gazebo's edge contacts, so physics walks flat-footed.
        DeclareLaunchArgument("toe_off", default_value="0.0", description="push-off heel lift (rad)"),
        DeclareLaunchArgument("heel_strike", default_value="0.0", description="landing toe lift (rad)"),
        DeclareLaunchArgument("step_time", default_value="0.6", description="seconds per step"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg, "worlds", "test_world.sdf"])),

        # let Gazebo resolve package://my_robot_description/meshes/...
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                                  PathJoinSubstitution([FindPackagePrefix("my_robot_description"), "share"])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": [world, " -r"]}.items(),
                                 condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": [world, " -r -s --headless-rendering"]}.items(),
                                 condition=UnlessCondition(LaunchConfiguration("gui"))),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim]),
        # spawn standing: pelvis height with straight legs
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "my_robot", "-z", "0.84"]),
        # controller manager runs inside Gazebo (gz_ros2_control); hruh_control/config/ros2_controllers.yaml
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=CONTROLLERS + ["--controller-manager-timeout", "120"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": PathJoinSubstitution([pkg, "config", "gazebo_bridge.yaml"])}, sim]),

        # human-like walking: footsteps from /cmd_vel, IMU-stabilised leg targets
        Node(package="my_robot_description", executable="hruh_walker.py", output="screen",
             parameters=[{"robot_description": robot_description, "mode": "ros2_control",
                          "auto_walk": ParameterValue(LaunchConfiguration("walk"), value_type=bool),
                          "auto_vx": ParameterValue(LaunchConfiguration("vx"), value_type=float),
                          "toe_off": ParameterValue(LaunchConfiguration("toe_off"), value_type=float),
                          "heel_strike": ParameterValue(LaunchConfiguration("heel_strike"), value_type=float),
                          "step_time": ParameterValue(LaunchConfiguration("step_time"), value_type=float),
                          "max_vx": 0.2},  # tested stable up to ~0.2 m/s in Gazebo
                         sim]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("stereo_image_proc"), "launch", "stereo_image_proc.launch.py"])),
            launch_arguments={"namespace": "/stereo"}.items(), condition=IfCondition(sensors)),
        Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "urdf_config.rviz"]), "-f", "odom"],
             parameters=[sim]),
    ])
