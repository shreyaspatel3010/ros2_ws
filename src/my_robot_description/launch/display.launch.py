import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    pkg_path = get_package_share_path('my_robot_description')

    # Paths
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')
    controllers_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    return LaunchDescription([
        # Start Gazebo with the world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # Wait a few seconds to ensure Gazebo loads
        TimerAction(
            period=3.0,
            actions=[

                # robot_state_publisher
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description}]
                ),

                # Joint State Publisher GUI
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui'
                ),

                # RViz2
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_path],
                    output='screen'
                ),

                # ROS2 Control Node
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    output='screen',
                    parameters=[
                        {'robot_description': robot_description},
                        controllers_yaml
                    ]
                ),

                # Spawn robot into GZ
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'hruh',
                        '-topic', 'robot_description',
                        '-z', '0.5'
                    ],
                    output='screen'
                ),
            ]
        )
    ])
