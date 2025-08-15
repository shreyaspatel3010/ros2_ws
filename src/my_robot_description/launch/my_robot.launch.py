
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "rviz",
        "urdf_config.rviz"
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "config",
        "ros2_controllers.yaml"
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare("my_robot_description"),
                "worlds",
                "my_world.world"
            ])
        }.items()
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_path])
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "my_robot"],
            output="screen"
        ),

        # Launch Gazebo
        gazebo_launch,

        # Delay ros2_control_node to ensure robot is spawned
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        {"robot_description": Command(["xacro ", urdf_path])},
                        controllers_yaml
                    ],
                    output="screen"
                )
            ]
        ),

        # Spawn joint_state_broadcaster
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen"
                )
            ]
        ),

        # Spawn arm_controller
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                    output="screen"
                )
            ]
        ),

        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    ])