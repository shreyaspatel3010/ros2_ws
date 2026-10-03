"""HRUH in Isaac Sim 6.1 + ros2_control + MoveIt + RViz (+ walker, + gamepad).

    ros2 launch hruh_bringup isaac.launch.py [controller:=auto|policy|walker|none]
        [reach:=auto|true|false] [headless:=true] [fix_base:=true] [cameras:=true]
        [gains:=auto|ros|rl] [moveit:=false] [rviz:=false] [joystick:=false] [fake:=true]

Isaac Sim runs src/hruh_isaac/isaac/hruh_isaac_sim.py (PhysX, real-time) and talks
to ros2_control through topic_based_ros2_control on /isaac_joint_{commands,states}.
Everything above ros2_control is the same as the Gazebo / mock bringups: the
controllers in hruh_control, MoveIt (planning groups, presets), hruh_walker and
the gamepad.

controller:=     who balances / walks with the legs and waist
    auto         the learned policy if train_robot_offline.sh promoted one
                 (src/hruh_isaac/policies/locomotion), otherwise the walker standing
    policy       learned walking policy (scripts/policy_runner.py); Isaac uses the
                 training actuator gains (gains:=rl); hold LB + sticks to walk
    walker       ZMP walker (stiff gains:=ros). Stands well in Isaac; its walking gait
                 was tuned for Gazebo and falls in Isaac
    none         legs / waist only hold their position
reach:=auto      learned right-arm reaching on /hruh/hand_target (PoseStamped, base_link)
                 when a reach policy is promoted
fix_base:=true   pelvis fixed in the air: arms / hands / head with MoveIt, no walking
fake:=true       no Isaac (scripts/fake_isaac.py echoes commands) to test the ROS side
Isaac runs under hard RAM / CPU / GPU caps (src/hruh_isaac/scripts/limit.sh; HRUH_CPU_CORES,
HRUH_MEM_GB, HRUH_GPU_MEM_GB) so it is stopped before it can freeze the desktop.
Do not run a trained policy (run.sh joystick) at the same time: one actuator source only.
"""
import json
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ["joint_state_broadcaster", "legs_controller", "waist_controller", "head_controller",
               "left_arm_controller", "right_arm_controller", "left_hand_controller", "right_hand_controller"]


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def setup(context):
    root = Path(LaunchConfiguration("workspace").perform(context))
    python = LaunchConfiguration("python").perform(context)
    fake, fix_base = flag(context, "fake"), flag(context, "fix_base")
    policies = Path(LaunchConfiguration("policy_dir").perform(context) or root / "src/hruh_isaac/policies")
    walk_policy, reach_policy = policies / "locomotion", policies / "reach"
    controller = LaunchConfiguration("controller").perform(context)
    if controller == "auto":
        controller = "policy" if (walk_policy / "bundle.json").is_file() else "walker"
    if fix_base:
        controller = "none"
    if controller == "policy" and not (walk_policy / "bundle.json").is_file():
        return [LogInfo(msg=f"[isaac.launch] controller:=policy but no promoted policy in {walk_policy}: run "
                            "./train_robot_offline.sh (passing policies are promoted) or use controller:=walker"),
                EmitEvent(event=Shutdown())]
    reach = LaunchConfiguration("reach").perform(context)
    reach = (reach_policy / "bundle.json").is_file() if reach == "auto" else reach.lower() in ("true", "1")
    gains = LaunchConfiguration("gains").perform(context)
    if gains == "auto":
        # learned policies expect the actuator gains they were trained with
        gains = "rl" if controller == "policy" or reach else "ros"
    actions = [LogInfo(msg=f"[isaac.launch] legs: {controller}, reach policy: {reach}, Isaac gains: {gains}")]
    urdf = root / "artifacts/hruh/hruh_isaac.urdf"
    if not urdf.exists():
        subprocess.run(["/usr/bin/python3", str(root / "src/hruh_isaac/scripts/export_isaac_urdf.py"),
                        "--output", str(urdf)], check=True)
    xacro = Path(get_package_share_directory("my_robot_description")) / "urdf/my_robot.urdf.xacro"
    robot_description = subprocess.run(["xacro", str(xacro), "hardware:=isaac"], check=True,
                                       capture_output=True, text=True).stdout
    controllers = Path(get_package_share_directory("hruh_control")) / "config/ros2_controllers.yaml"
    sim = {"use_sim_time": True}

    if fake:
        sim_proc = ExecuteProcess(cmd=["/usr/bin/python3", str(root / "src/hruh_isaac/scripts/fake_isaac.py"),
                                       "--urdf", str(urdf)], output="screen", name="fake_isaac")
    else:
        if not shutil.which(python) and not Path(python).exists():
            return [LogInfo(msg=f"[isaac.launch] '{python}' not found: install Isaac Sim (src/hruh_isaac/scripts/"
                                "install_isaac.sh) or use fake:=true"), EmitEvent(event=Shutdown())]
        limit = [str(root / "src/hruh_isaac/scripts/limit.sh")] if flag(context, "limits") else []
        cmd = limit + [python, str(root / "src/hruh_isaac/isaac/hruh_isaac_sim.py"), "--urdf", str(urdf),
               "--gains", gains,
               "--threads", LaunchConfiguration("threads").perform(context)]
        if controller == "policy":
            # spawn at the training reset height, held until the runner has the stand pose
            spawn_z = json.loads((walk_policy / "bundle.json").read_text())["initial_root_position"][2]
            cmd += ["--height", f"{spawn_z:.3f}", "--hold-until-stand", str(walk_policy / "bundle.json")]
        cmd += ["--headless"] if flag(context, "headless") else []
        cmd += ["--fix-base"] if fix_base else []
        cmd += ["--cameras"] if flag(context, "cameras") else []
        sim_proc = ExecuteProcess(cmd=cmd, output="screen", name="isaac_sim")
    actions += [
        sim_proc,
        # closing Isaac ends the session
        RegisterEventHandler(OnProcessExit(target_action=sim_proc,
                                           on_exit=[EmitEvent(event=Shutdown(reason="simulator exited"))])),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[str(controllers), sim],
             remappings=[("~/robot_description", "/robot_description")]),
    ]
    # The controller manager runs on the simulator's /clock: controllers can only be
    # activated once Isaac is stepping, so spawn them when it reports ready (first
    # start takes minutes: shader cache, URDF -> USD conversion).
    controllers_list = [("waist_position_controller" if c == "waist_controller" and controller == "policy" else c)
                        for c in CONTROLLERS]
    spawner = Node(package="controller_manager", executable="spawner", output="screen",
                   arguments=controllers_list + ["--controller-manager-timeout", "600",
                                               # Isaac warms up for ~10 s after READY (sim clock crawls)
                                               "--service-call-timeout", "120", "--switch-timeout", "120"])
    if fake:
        actions.append(spawner)
    else:
        started = []

        def on_output(event):
            if not started and b"HRUH_ISAAC_READY" in event.text:
                started.append(True)
                return [LogInfo(msg="[isaac.launch] Isaac ready: spawning controllers"), spawner]
            return None
        actions.append(RegisterEventHandler(OnProcessIO(target_action=sim_proc, on_stdout=on_output)))
    if controller == "policy" or reach:
        runner = [LaunchConfiguration("policy_python").perform(context),
                  str(root / "src/hruh_isaac/scripts/policy_runner.py"), "--controllers", str(controllers)]
        runner += ["--locomotion", str(walk_policy)] if controller == "policy" else []
        runner += [] if flag(context, "arm_swing") else ["--no-arm-swing"]
        runner += ["--reach", str(reach_policy)] if reach else []
        actions.append(ExecuteProcess(cmd=runner, output="screen", name="policy_runner"))
    if controller == "policy" and fake:
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                            arguments=["--frame-id", "odom", "--child-frame-id", "base_link", "--z", "0.833"]))
    if controller == "walker":
        actions.append(Node(package="my_robot_description", executable="hruh_walker.py", output="screen",
                            parameters=[{"robot_description": robot_description, "mode": "ros2_control",
                                         "publish_odom_tf": fake, "balance": not fake,
                                         "auto_walk": flag(context, "walk"),
                                         "toe_off": 0.0, "heel_strike": 0.0, "max_vx": 0.2}, sim]))
    elif controller != "policy" and (fake or fix_base):
        # nothing publishes odom -> base_link: pin it so RViz / MoveIt have the floating base
        z = "1.0" if fix_base else "0.823"
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                            arguments=["--frame-id", "odom", "--child-frame-id", "base_link", "--z", z]))
    if flag(context, "moveit"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("hruh_moveit_config"))
                                              / "launch/moveit.launch.py")),
            launch_arguments={"hardware": "isaac", "use_sim_time": "true",
                              "rviz": LaunchConfiguration("rviz").perform(context)}.items()))
    elif flag(context, "rviz"):
        actions.append(Node(package="rviz2", executable="rviz2", output="log", parameters=[sim],
                            arguments=["-d", str(Path(get_package_share_directory("my_robot_description"))
                                                 / "rviz/urdf_config.rviz"), "-f", "odom"]))
    if flag(context, "joystick"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("hruh_teleop"))
                                              / "launch/joystick.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()))
    return actions


def generate_launch_description():
    workspace = str(Path(get_package_prefix("hruh_bringup")).parent.parent)
    return LaunchDescription([
        DeclareLaunchArgument("workspace", default_value=workspace),
        DeclareLaunchArgument("python", default_value="isaac-python",
                              description="Isaac Sim Python (the isaac-python wrapper sets up its ROS 2 libraries)"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("fix_base", default_value="false"),
        DeclareLaunchArgument("cameras", default_value="false"),
        DeclareLaunchArgument("gains", default_value="auto", choices=["auto", "ros", "rl"],
                              description="Isaac joint gains: rl = training actuators (policies), ros = stiff (walker)"),
        DeclareLaunchArgument("threads", default_value="8", description="Isaac worker threads (laptop responsiveness)"),
        DeclareLaunchArgument("controller", default_value="auto", choices=["auto", "policy", "walker", "none"]),
        DeclareLaunchArgument("reach", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("arm_swing", default_value="true",
                              description="motion policies: swing the arms while walking (false: MoveIt keeps them)"),
        DeclareLaunchArgument("policy_dir", default_value="",
                              description="Promoted policies (default: <workspace>/src/hruh_isaac/policies)"),
        DeclareLaunchArgument("policy_python", default_value="/opt/isaac/venv-6.1/bin/python",
                              description="Python with torch for the policy runner"),
        DeclareLaunchArgument("limits", default_value="true", description="Hard RAM / CPU / GPU caps for Isaac"),
        DeclareLaunchArgument("walk", default_value="false"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("joystick", default_value="true"),
        DeclareLaunchArgument("fake", default_value="false"),
        OpaqueFunction(function=setup),
    ])
