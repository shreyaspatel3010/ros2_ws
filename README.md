# HRUH — Walking Humanoid Robot (ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2)

**HRUH** is a 1.6 m, ~61 kg humanoid robot. It has two 7-DOF legs with human
ranges of motion, a 3-DOF waist, a human-like head with stereo-camera eyes and
a LiDAR built into the crown, and two 5-DOF arms with four-finger hands and a
thumb. It walks with a ZMP preview-control gait, plans arm, hand, head and
waist motion with MoveIt 2, and the whole robot can be driven from one gamepad.

## Packages

| Package | What it contains |
|---|---|
| [`my_robot_description`](src/my_robot_description) | URDF/Xacro, textured meshes (OBJ for ROS, GLB for other tools), Gazebo worlds and bridge, the walking pattern generator `hruh_walker.py`, Blender scripts that generate every asset, and the printable 3MF. |
| [`hruh_control`](src/hruh_control) | ros2_control controller configuration. |
| [`hruh_moveit_config`](src/hruh_moveit_config) | MoveIt 2: SRDF with groups, named poses and collision matrix, kinematics, joint limits, controllers, RViz layout and `moveit.launch.py`. |
| [`hruh_teleop`](src/hruh_teleop) | Gamepad control of the whole robot (`hruh_joystick.py`) plus `joy_layout_normalizer.py`, taken from `aries_teleop`. |
| [`hruh_bringup`](src/hruh_bringup) | Top-level launch files: Gazebo simulation, mock-hardware demo, kinematic RViz walking. |

## The robot

| Part | Degrees of freedom (range) |
|---|---|
| Hip (each leg) | yaw ±40–45°, roll (abduction 45° / adduction 25°), pitch (flexion 120° / extension 30°); the three axes meet in one point like a ball joint |
| Knee | 0–140° |
| Ankle | pitch (dorsiflexion 30° / plantarflexion 50°), roll (inversion 35° / eversion 20°) |
| Toe | −60° to +20° (heel-to-toe roll-off) |
| Waist | yaw ±45°, side bend ±20°, flexion 60° / extension 25° |
| Neck / head | turn ±80°, nod (down 50° / up 60°) |
| Arm (each) | shoulder flex, abduction, upper-arm rotation, elbow, wrist rotation |
| Hand (each) | thumb ×3, four fingers (the middle and upper finger joints follow the base joint through URDF mimic) |

Sensors:
- **Stereo camera eyes:** 64 mm baseline, the human eye spacing.
- **LiDAR:** in the head crown, 64 beams from −30° to +15°.
- **RGB-D camera:** on the upper sternum.
- **IMU:** in the pelvis.

The root link is `base_link`, the pelvis; the robot has a floating base.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select my_robot_description hruh_control hruh_moveit_config hruh_teleop hruh_bringup
source install/setup.bash
```

Dependencies are Jazzy's `ros_gz`, `gz_ros2_control`, `ros2_controllers`, MoveIt 2 (`move_group`, OMPL, KDL, `moveit_configs_utils`), `joy` and `stereo_image_proc`.

## Run

| Command | What you get |
|---|---|
| `ros2 launch hruh_bringup sim.launch.py` | Gazebo, ros2_control, the walker, MoveIt and the gamepad |
| `ros2 launch hruh_bringup sim.launch.py sensors:=false walk:=true` | Faster simulation; the robot walks forward on its own |
| `ros2 launch hruh_bringup demo.launch.py` | No simulator: mock hardware, the walker, MoveIt and the gamepad |
| `ros2 launch hruh_bringup walk_rviz.launch.py walk:=true` | Kinematic walking in RViz, with full heel-strike and toe-off |
| `ros2 launch my_robot_description gazebo.launch.py` | Gazebo, controllers and walker only (no MoveIt, no gamepad) |
| `ros2 launch my_robot_description display.launch.py` | Model viewer with joint sliders |

Common arguments:
- `joystick:=false`
- `moveit:=false`
- `rviz:=false`
- `gui:=false` (Gazebo server only)
- `sensors:=false` (no cameras or LiDAR)
- `walk:=true vx:=0.15`

Walk with any `/cmd_vel` source, for example `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

### Gamepad (same layout and conventions as `aries_teleop`)

| Input | Action |
|---|---|
| **Hold LB** | **Walk.** Left stick moves forward/back and sidesteps; right stick turns. LB blocks all arm output. |
| **Hold RB** | **Cartesian hand jog** of the selected arm: left stick moves forward/back and left/right, D-pad moves up/down, right stick turns the wrist and upper arm. |
| **Hold RT** | **Joint jog** of the selected chain. For an arm: shoulder flex/abduction, upper-arm rotation, elbow, wrist. For head/waist: neck turn and nod, waist turn, bend and side-bend. |
| RB or RT held + **X** / **B** | Open / close the hand |
| **BACK** | Select the next chain: right arm → left arm → head/waist |
| **Hold LT** + Y / A / B / X | MoveIt preset: home / wave / hands up / reach forward |
| **Hold LT** + D-pad up / down | Head centre / look down |

If `/joy` goes quiet for 0.35 s, the robot stops. The mapping is in [`hruh_teleop/config/joystick.yaml`](src/hruh_teleop/config/joystick.yaml).

### MoveIt

- **Planning groups:** `left_arm`, `right_arm`, `both_arms`, `left_hand`, `right_hand`, `head`, `waist`.
- **Named poses:** `home`, `hands_up`, `t_pose`, `reach_forward`, `hold_object`, `wave`, hand `open`/`close`, head `center`/`look_down`, waist `upright`.
- **Collision matrix:** generated with `collisions_updater`.
- **Not planned by MoveIt:** the legs. The walker owns them, because they keep the robot balanced.

## Walking

[`hruh_walker.py`](src/my_robot_description/scripts/hruh_walker.py) generates the gait:
- **Balance:** ZMP preview control (Kajita) on a linear inverted pendulum, with the ZMP rolling from heel to toe under each stance foot. Whole-body centre-of-mass correction includes the swinging limbs.
- **Footsteps:** planned online from `/cmd_vel`.
- **Legs:** closed-form 6-DOF leg IK.
- **Human gait features:** heel-strike and toe-off, a centre-of-mass bob that is lowest in double support, pelvis rotation with a counter-rotating waist, and arm swing opposite the legs. The knees stay nearly straight in stance (about 11°) and bend about 52° in swing.
- **In Gazebo:** IMU ankle and hip feedback, plus heading-drift correction from odometry.

Results in Gazebo:
- **Flat-footed at 0.15 m/s:** walked 8 m in a straight line without falling. This is the default.
- **0.2 m/s:** about 7.5 m before falling.
- **Heel-toe roll-off:** makes the feet slip on Gazebo's edge contacts, so it is on only in the kinematic RViz demo and the GLB animation.

## Assets (Blender)

Every new mesh is generated by scripts in [`src/my_robot_description/blender`](src/my_robot_description/blender), so you can edit and regenerate them:

```bash
cd src/my_robot_description
blender -b --factory-startup -P blender/build_parts.py              # legs, pelvis, waist, chest, neck, head
blender -b --factory-startup -P blender/build_arms.py -- <robot.urdf>   # clean + texture the arm/hand meshes
blender -b --factory-startup -P blender/build_arm_collision.py      # light arm collision meshes
blender -b --factory-startup -P blender/export_robot_glb.py -- <robot.urdf>  # whole robot + walking animation
blender -b --factory-startup -P blender/export_3mf.py               # printable parts
```

| Output | Where |
|---|---|
| Textured meshes used by the URDF | `meshes/{legs,chest,neck,head,arms}/*.obj` + `.mtl` + baked `*_albedo.png` |
| Self-contained PBR **GLB** for every part (albedo, roughness/metallic, emission) | `meshes/*/glb/*.glb` |
| Whole robot GLB, standing and with a baked walking animation | `meshes/robot/hruh_robot.glb`, `hruh_robot_walk.glb` |
| **3MF** with all 75 printable parts (watertight, millimetres, per-triangle print colours) | `print/hruh_robot_parts.3mf` |
| Editable Blender file | `blender/humanoid_parts.blend` |

Notes:
- **GLB orientation:** GLB files are Y-up as the glTF spec requires, so they open correctly in Blender, three.js, Unity and Isaac Sim.
- **Arm meshes:** the arm/hand meshes are the original geometry, welded watertight and re-textured. Mean deviation from the original STLs is ≤0.35 mm; the original STLs are still in `meshes/left_arm` and `meshes/right_arm`.
- **3MF parts:** the 3MF is real size. The chest, thighs and shins (~340–370 mm) need a large-format printer, or splitting/scaling in the slicer (`export_3mf.py -- out.3mf 0.2` gives a 1:5 model). These are shells: add your own actuator mounts before printing working parts.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Walking command |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` |
| `/legs_controller/commands` | `std_msgs/Float64MultiArray` | Streamed by the walker |
| `/{left,right}_arm_controller/joint_trajectory`, `/{left,right}_hand_controller/…`, `/head_controller/…`, `/waist_controller/…` | `trajectory_msgs/JointTrajectory` | Plus `follow_joint_trajectory` actions for MoveIt |
| `/odom`, `/tf` | | Ground-truth pelvis pose (Gazebo) |
| `/imu` | `sensor_msgs/Imu` | Pelvis IMU |
| `/stereo/{left,right}/image_raw`, `camera_info` | | Eye cameras (+ `stereo_image_proc`) |
| `/camera/image`, `/camera/depth_image`, `/camera/points` | | Chest RGB-D |
| `/scan`, `/points` | | Head-crown LiDAR |
| `/hruh_joystick/status` | `std_msgs/String` | Gamepad mode and MoveIt result messages |

## Known limitations

- **Simulation speed:** the simulation runs at about 0.3× real time on this machine, even with sensors off. The robot itself is the cost: 59 actuated joints and 16 mimic constraints at a 1 ms physics step.
- **MoveIt Servo:** Servo (Jazzy) does not work with 5-DOF arms; its singularity check reads a 6th singular value. Cartesian jogging is therefore done in `hruh_joystick.py`, with damped least squares on the hand position. It has no collision checking, so watch the arms near the body.
- **Controller config location:** the controller config lives in its own package because ros2_control 4.x drops any controller-manager argument containing `robot_description`. A params-file path under `my_robot_description/` therefore breaks every controller with `Couldn't parse params file: '--params-file -p'`.

## Author

**Shreyas Patel** — [@shreyaspatel3010](https://github.com/shreyaspatel3010)

## Demo

▶️ [Watch the HRUH simulation demo (HRUH_github.mp4)](HRUH_github.mp4) (the earlier wheeled version).
