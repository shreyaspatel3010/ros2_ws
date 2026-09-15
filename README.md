# HRUH — Humanoid Mobile Manipulator (ROS 2 Jazzy + Gazebo Harmonic)

A ROS 2 workspace containing **`my_robot_description`**, the URDF/Xacro model and simulation setup for **HRUH**: a humanoid upper body (chest, neck, head, two articulated arms with four-finger-and-thumb hands) mounted on a rotating torso column on top of a four-wheel mobile base, equipped with stereo, RGB-D, LiDAR and IMU sensors.

## Features

- **Full robot model in Xacro**: mobile base → rotating torso column (`arm_base_link` / `forearm_link`) → chest → neck → head, plus left and right arms with wrists, palms, four fingers and a thumb (STL meshes).
- **Gazebo Sim (Harmonic) simulation** with:
  - Differential-drive plugin driven by `/cmd_vel`, publishing `/odom` and `/tf`
  - Per-joint `JointPositionController` plugins for arm, neck, head and torso joints
  - `JointStatePublisher` bridged to `/joint_states`
- **Sensors**
  - Stereo camera pair (`/stereo/left|right/image_raw`, `camera_info`) + `stereo_image_proc`
  - RGB-D camera (`/camera/image`, `/camera/depth_image`, `/camera/points`)

https://github.com/user-attachments/assets/231ffe8d-f9b1-43d1-a618-bcc002551119


  - GPU LiDAR (`/scan`, `/points`)
  - IMU (`/imu`)
- **ros_gz_bridge** config covering clock, TF, odometry, sensors and all joint commands.
- **RViz2** configuration for visualizing the model and sensor data.
- **Keyboard teleop script** for jogging arm and finger joints.

## Workspace Layout

```
ros2_ws/
├── HRUH.webm                          # Demo video
└── src/
    └── my_robot_description/
        ├── config/gazebo_bridge.yaml  # ROS <-> Gazebo topic bridge
        ├── launch/
        │   ├── display.launch.xml     # RViz + joint_state_publisher_gui
        │   └── my_robot_hurh.launch.xml  # Full Gazebo simulation
        ├── meshes/                    # chest, neck, head, left_arm, right_arm STLs
        ├── rviz/urdf_config.rviz
        ├── scripts/teleop_whole_robot.py
        ├── urdf/
        │   ├── my_robot.urdf.xacro    # Top-level robot description
        │   ├── mobile_base*.xacro     # Base, wheels, IMU, diff drive
        │   ├── arm*.xacro             # Rotating torso column
        │   ├── hruh*.xacro            # Chest, neck, head + joint controllers
        │   ├── left_arm*.xacro / right_arm*.xacro  # Arms and hands
        │   └── stereo_camera / rgbd_camera / lidar_gz .xacro
        └── worlds/                    # test_world.sdf, my_world.sdf, harmonic.sdf, ionic.sdf
```

## Requirements

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Gazebo Harmonic (installed with `ros-jazzy-ros-gz`)

Install the ROS dependencies:

```bash
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-image-proc \
  ros-jazzy-stereo-image-proc \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

Or let `rosdep` resolve them:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

## Build

```bash
git clone https://github.com/shreyaspatel3010/ros2_ws.git ~/ros2_ws
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Usage

### Visualize the model in RViz

```bash
ros2 launch my_robot_description display.launch.xml
```

Use the **Joint State Publisher GUI** sliders to move the joints.

### Run the Gazebo simulation

```bash
ros2 launch my_robot_description my_robot_hurh.launch.xml
```

This starts Gazebo with `worlds/test_world.sdf`, spawns the robot, starts the topic bridge, runs stereo image processing and opens RViz.

### Drive the mobile base

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

or publish directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"
```

### Move joints

Every controlled joint accepts a `std_msgs/msg/Float64` position target (radians):

```bash
# Turn the head
ros2 topic pub --once /neck_to_head/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"

# Rotate the neck
ros2 topic pub --once /chest_to_neck/cmd_pos std_msgs/msg/Float64 "{data: -0.8}"

# Raise the right shoulder
ros2 topic pub --once /right_arm/chest_to_right_shoulder/cmd_pos std_msgs/msg/Float64 "{data: 1.0}"

# Bend the left elbow
ros2 topic pub --once /left_arm/left_bisecp_to_elbow_inword/cmd_pos std_msgs/msg/Float64 "{data: 0.7}"
```

| Group | Command topics |
|-------|----------------|
| Head | `/chest_to_neck/cmd_pos`, `/neck_to_head/cmd_pos` |
| Torso rotation | `/arm_base_forearm_joint/cmd_pos` |
| Left arm | `/left_arm/<joint>/cmd_pos` |
| Right arm | `/right_arm/<joint>/cmd_pos` |

Arm joints with a position controller: `chest_to_<side>_shoulder`, `<side>_shoulder_to_bisecp`,
`<side>_bisecp_to_elbow_inword`, `<side>_elbow_inword_to_midle`, `<side>_forarm_to_wrist`.
See [`config/gazebo_bridge.yaml`](src/my_robot_description/config/gazebo_bridge.yaml) for the full list.

### Keyboard teleop for the arm and fingers

```bash
python3 src/my_robot_description/scripts/teleop_whole_robot.py \
  --controller /left_arm_controller/joint_trajectory
```

| Keys | Action |
|------|--------|
| `q/a` `w/s` `e/d` `r/f` `t/g` `y/h` `u/j` `i/k` `o/l` `p/;` | Joint 1–10 + / − |
| `[` / `]` | Select previous / next joint |
| `,` / `.` | Decrease / increase selected joint |
| `+` / `-` | Increase / decrease step size |
| `c` | Zero all targets |
| `?` | Help |
| `Esc` | Quit |

> The script publishes `trajectory_msgs/JointTrajectory`, so it needs a running
> `joint_trajectory_controller` (ros2_control) on the given topic.

## Topics Overview

| Topic | Type | Direction |
|-------|------|-----------|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |
| `/camera/image`, `/camera/depth_image` | `sensor_msgs/Image` | Gazebo → ROS |
| `/camera/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |
| `/stereo/{left,right}/image_raw` | `sensor_msgs/Image` | Gazebo → ROS |
| `/stereo/{left,right}/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS |

## Author

**Shreyas Patel** — [@shreyaspatel3010](https://github.com/shreyaspatel3010)

## Demo

https://github.com/shreyaspatel3010/ros2_ws/raw/main/HRUH.webm

<video src="https://github.com/shreyaspatel3010/ros2_ws/raw/main/HRUH.webm" controls width="100%"></video>

▶️ If the player above doesn't load, [watch the demo video (HRUH.webm)](HRUH.webm).
