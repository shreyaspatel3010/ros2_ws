# HRUH — Walking Humanoid Robot (ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2)

**HRUH** is a 1.6 m, ~61 kg humanoid robot. It has two 7-DOF legs with human
ranges of motion, a rigid torso, a human-like head with stereo-camera eyes and
a LiDAR built into the crown, and two 5-DOF arms with four-finger hands and a
thumb. It walks with a ZMP preview-control gait, plans arm, hand, head and
motion with MoveIt 2, and the whole robot can be driven from one gamepad.

## Packages

**Isaac learning:** see [`src/hruh_isaac/README.md`](src/hruh_isaac/README.md)
for GPU training, measured evaluation and right-arm reaching / right-hand
cube-lifting tasks. Policies that pass their benchmark are promoted to
`src/hruh_isaac/policies/` and run by `isaac.launch.py` in place of the ZMP walker
described below. They are trained and checked in simulation only.

| Package | What it contains |
|---|---|
| [`my_robot_description`](src/my_robot_description) | URDF/Xacro, textured meshes (OBJ for ROS, GLB for other tools), Gazebo worlds and bridge, the walking pattern generator `hruh_walker.py`, Blender scripts that generate every asset, and the printable 3MF. |
| [`hruh_control`](src/hruh_control) | ros2_control controller configuration. |
| [`hruh_moveit_config`](src/hruh_moveit_config) | MoveIt 2: SRDF with groups, named poses and collision matrix, kinematics, joint limits, controllers, RViz layout and `moveit.launch.py`. |
| [`hruh_teleop`](src/hruh_teleop) | Gamepad control of the whole robot (`hruh_joystick.py`) plus `joy_layout_normalizer.py`, taken from `aries_teleop`. |
| [`hruh_bringup`](src/hruh_bringup) | Top-level launch files: Gazebo simulation, mock-hardware demo, kinematic RViz walking, Isaac Sim, trained-policy Gazebo. |
| [`hruh_isaac`](src/hruh_isaac) | Isaac Sim 6.1 app, Isaac Lab training tasks, `train_robot_offline.sh` pipeline, promoted policies (`policies/`), policy runner. |

## The robot

| Part | Degrees of freedom (range) |
|---|---|
| Hip (each leg) | yaw 40° in / 43° out, roll (abduction 31° / adduction 24°), pitch (flexion 89° / extension 29°). The three axes meet in one point like a ball joint; each motor sits along its own axis where it fits: yaw motor in the pelvis, roll motor behind the hip, pitch motor outside the thigh. Ranges are the measured collision-free ranges of the meshes. |
| Knee | 0–140° |
| Ankle | pitch (dorsiflexion 30° / plantarflexion 50°), roll (inversion 35° / eversion 20°) |
| Toe | −60° to +20° (heel-to-toe roll-off) |
| Torso | rigid: the chest is bolted to the pelvis through a lumbar block (no waist joints) |
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
| `ros2 launch hruh_bringup isaac.launch.py` | Isaac Sim + ros2_control + MoveIt + RViz + gamepad; the legs use the promoted learned policy if there is one, otherwise the walker stands (`controller:=walker\|policy\|none`, `fake:=true` without Isaac) |
| `ros2 launch hruh_bringup policy_gazebo.launch.py joystick:=true` | The promoted walking policy in Gazebo (effort control) |
| `./train_robot_offline.sh` | Train → evaluate → Gazebo test → promote passing policies into the runtime (hours; resource-capped) |

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
| **Hold RT** | **Joint jog** of the selected chain. For an arm: shoulder flex/abduction, upper-arm rotation, elbow, wrist. For the head: neck turn and nod. |
| RB or RT held + **X** / **B** | Open / close the hand |
| **BACK** | Select the next chain: right arm → left arm → head |
| **Hold LT** + Y / A / B / X | MoveIt preset: home / wave / hands up / reach forward |
| **Hold LT** + D-pad up / down | Head centre / look down |

If `/joy` goes quiet for 0.35 s, the robot stops. The mapping is in [`hruh_teleop/config/joystick.yaml`](src/hruh_teleop/config/joystick.yaml).

### MoveIt

- **Planning groups:** `left_arm`, `right_arm`, `both_arms`, `left_hand`, `right_hand`, `head`.
- **Named poses:** `home`, `hands_up`, `t_pose`, `reach_forward`, `hold_object`, `wave`, hand `open`/`close`, head `center`/`look_down`.
- **Collision matrix:** generated with `collisions_updater`.
- **Not planned by MoveIt:** the legs. The walker owns them, because they keep the robot balanced.

## Walking

[`hruh_walker.py`](src/my_robot_description/scripts/hruh_walker.py) generates the gait:
- **Balance:** ZMP preview control (Kajita) on a linear inverted pendulum, with the ZMP rolling from heel to toe under each stance foot. Whole-body centre-of-mass correction includes the swinging limbs.
- **Footsteps:** planned online from `/cmd_vel`.
- **Legs:** closed-form 6-DOF leg IK.
- **Human gait features:** heel-strike and toe-off, a centre-of-mass bob that is lowest in double support, and arm swing opposite the legs (pelvis rotation is off by default: with a rigid torso it would turn the chest). The knees stay nearly straight in stance (about 11°) and bend about 52° in swing.
- **In Gazebo:** IMU ankle and hip feedback, plus heading-drift correction from odometry.

Results in Gazebo:
- **Flat-footed at 0.15 m/s:** walked 8 m in a straight line without falling. This is the default.
- **0.2 m/s:** about 7.5 m before falling.
- **Heel-toe roll-off:** makes the feet slip on Gazebo's edge contacts, so it is on only in the kinematic RViz demo and the GLB animation.

## Assets (Blender)

Every new mesh is generated by scripts in [`src/my_robot_description/blender`](src/my_robot_description/blender), so you can edit and regenerate them:

```bash
cd src/my_robot_description
blender -b --factory-startup -P blender/build_parts.py              # legs, pelvis, lumbar block, chest, neck, head
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
| `/{left,right}_arm_controller/joint_trajectory`, `/{left,right}_hand_controller/…`, `/head_controller/…` | `trajectory_msgs/JointTrajectory` | Plus `follow_joint_trajectory` actions for MoveIt |
| `/odom`, `/tf` | | Ground-truth pelvis pose (Gazebo) |
| `/imu` | `sensor_msgs/Imu` | Pelvis IMU |
| `/stereo/{left,right}/image_raw`, `camera_info` | | Eye cameras (+ `stereo_image_proc`) |
| `/camera/image`, `/camera/depth_image`, `/camera/points` | | Chest RGB-D |
| `/scan`, `/points` | | Head-crown LiDAR |
| `/hruh_joystick/status` | `std_msgs/String` | Gamepad mode and MoveIt result messages |

## Mechanical design review (2026-10-04)

A check of the meshes for physical feasibility found two parts that could not be built:

- **Hips and thighs.** The hip centres were only ±8.5 cm apart while the thighs are
  ~12 cm wide, so the legs passed through each other beyond ~10° adduction. The hip roll
  housing sat flush against the pelvis (contact at 2° abduction). The thigh went through
  the pelvis beyond ~10° flexion. The roll and pitch motors shared one centre inside an
  11 cm housing.
- **Waist.** Three 150–200 N·m joints sat in one point, joined by placeholder 0.5 kg links,
  with no space for actuators.

What changed:

- **Hips.** Spacing is now ±10.5 cm. Each motor sits along its own axis: the yaw motor
  high in the pelvis on a hub, the roll motor 13 cm behind the hip centre, and the pitch
  motor outside the thigh. The axes still meet in one point, so the walker's closed-form
  leg IK is unchanged.
- **Pelvis and thigh.** The pelvis is narrower with a raised floor, and the thigh top is
  slimmer.
- **Masses.** Link masses and centres of mass follow the motors; the robot is now ~66 kg.
- **Waist.** The torso is rigid: a lumbar block joins the pelvis and chest.
- **Joint limits** are the measured collision-free ranges (`blender/build_parts.py` HIP,
  `urdf/legs.xacro`), checked by voxel overlap of the real meshes:

  | Hip motion | Old collision-free range | New collision-free range |
  |---|---|---|
  | Flexion | 10° | 92° |
  | Extension | 14° | 30° |
  | Abduction | 2° | 33° |
  | Adduction | 7° | 25° |

- **Self-collision.** Leg-to-leg contact (adduction past ~11° next to a straight leg, or
  both feet toed out past ~21°) is real contact. Self-collision is therefore **on** in
  Isaac training and in MoveIt; the collision matrix was regenerated.

## Known limitations

- **Simulation speed:** the simulation runs at about 0.3× real time on this machine, even with sensors off. The robot itself is the cost: 56 actuated joints and 16 mimic constraints at a 1 ms physics step.
- **MoveIt Servo:** Servo (Jazzy) does not work with 5-DOF arms; its singularity check reads a 6th singular value. Cartesian jogging is therefore done in `hruh_joystick.py`, with damped least squares on the hand position. It has no collision checking, so watch the arms near the body.
- **Controller config location:** the controller config lives in its own package because ros2_control 4.x drops any controller-manager argument containing `robot_description`. A params-file path under `my_robot_description/` therefore breaks every controller with `Couldn't parse params file: '--params-file -p'`.

## Author

**Shreyas Patel** — [@shreyaspatel3010](https://github.com/shreyaspatel3010)

## Demo

▶️ [Watch the HRUH simulation demo (HRUH_github.mp4)](HRUH_github.mp4) (the earlier wheeled version).
