"""HRUH in Isaac Sim 5.1 with a ROS 2 bridge (run with Isaac's Python).

    isaac-python hruh_isaac_sim.py [--headless] [--fix-base] [--cameras] [--gains ros|rl]

ROS 2 interface (consumed by topic_based_ros2_control / hruh_bringup isaac.launch.py):
    /isaac_joint_states    sensor_msgs/JointState   all joints (published)
    /isaac_joint_commands  sensor_msgs/JointState   position targets (subscribed)
    /clock /odom /imu, TF odom -> base_link
    --cameras: /stereo/{left,right}/image_raw + camera_info (the eye cameras)

The URDF comes from `ros2 run hruh_isaac export_isaac_urdf.py`.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--urdf", default=os.path.expanduser("~/.cache/hruh/hruh_isaac.urdf"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--fix-base", action="store_true", help="pelvis fixed in the air (arm / MoveIt work without balance)")
ap.add_argument("--height", type=float, default=0.84, help="spawn height of the pelvis (m)")
ap.add_argument("--physics-hz", type=float, default=200.0)
ap.add_argument("--gains", choices=["ros", "rl"], default="ros",
                help="ros: stiff drives for the walker / MoveIt; rl: the Isaac Lab training actuators")
ap.add_argument("--cameras", action="store_true", help="publish the stereo eye cameras")
ap.add_argument("--no-ros", action="store_true", help="no ROS 2 bridge (self-test)")
ap.add_argument("--steps", type=int, default=0, help="quit after N physics steps (0 = run until closed)")
ap.add_argument("--report", action="store_true", help="print the articulation state when quitting")
ap.add_argument("--threads", type=int, default=8,
                help="worker threads for Isaac's task scheduler (keeps the laptop responsive)")
args, _ = ap.parse_known_args()

if not os.path.exists(args.urdf):
    sys.exit(f"[hruh] {args.urdf} not found: run `ros2 run hruh_isaac export_isaac_urdf.py` first")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                     "extra_args": [f"--/plugins/carb.tasking.plugin/threadCount={args.threads}"]})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.sensors.physics")
if not args.no_ros:
    enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

# ------------------------------------------------------------------ joint drive gains
# (stiffness N*m/rad, damping N*m*s/rad, max effort N*m) by joint-name pattern
import re  # noqa: E402

GAINS = {
    "ros": [  # stiff position servos: the walker / MoveIt expect commanded = actual
        (r".*_(hip_yaw|hip_roll|hip_pitch|knee)_joint", 1500.0, 40.0, 300.0),
        (r".*_ankle_(pitch|roll)_joint", 800.0, 20.0, 150.0),
        (r".*_toe_joint", 60.0, 2.0, 40.0),
        (r"waist_.*", 1200.0, 30.0, 150.0),
        (r"(chest_to_.*_shoulder|.*_shoulder_to_bisecp|.*_bisecp_to_elbow_inword|.*_elbow_inword_to_midle|.*_forarm_to_wrist)",
         400.0, 10.0, 80.0),
        (r"(chest_to_neck|neck_to_head)", 100.0, 4.0, 20.0),
        (r".*(thomb|finger).*", 20.0, 0.5, 10.0),
    ],
    "rl": [  # identical to hruh_lab.robots.ACTUATORS (policies trained with these)
        (r".*_hip_yaw_joint", 150.0, 5.0, 250.0), (r".*_(hip_roll|hip_pitch)_joint", 200.0, 5.0, 250.0),
        (r".*_knee_joint", 250.0, 6.0, 250.0), (r".*_ankle_.*", 60.0, 3.0, 120.0), (r".*_toe_joint", 15.0, 0.5, 120.0),
        (r"waist_.*", 200.0, 6.0, 150.0),
        (r"(chest_to_.*_shoulder|.*_shoulder_to_bisecp|.*_bisecp_to_elbow_inword|.*_elbow_inword_to_midle|.*_forarm_to_wrist)",
         60.0, 3.0, 60.0),
        (r"(chest_to_neck|neck_to_head)", 30.0, 2.0, 20.0), (r".*(thomb|finger).*", 5.0, 0.2, 10.0),
    ],
}


def gains_for(name):
    for pat, kp, kd, eff in GAINS[args.gains]:
        if re.fullmatch(pat, name):
            return kp, kd, eff
    return 100.0, 5.0, 50.0


urdf_root = ET.parse(args.urdf).getroot()
MIMIC = {j.get("name") for j in urdf_root.findall("joint") if j.find("mimic") is not None}
ROBOT_NAME = urdf_root.get("name")

# ------------------------------------------------------------------ world
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / 60.0)
stage = omni.usd.get_context().get_stage()
GroundPlane("/World/ground", size=60.0, color=np.array([0.35, 0.37, 0.4]))
light = UsdLux.DomeLight.Define(stage, "/World/dome")
light.CreateIntensityAttr(1200.0)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 20.0, 0.0))

# ------------------------------------------------------------------ robot
_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True          # massless frames fold into their parents (TF comes from ROS)
cfg.fix_base = args.fix_base
cfg.import_inertia_tensor = True
cfg.make_default_prim = False
cfg.create_physics_scene = False
cfg.distance_scale = 1.0
cfg.convex_decomp = False
cfg.parse_mimic = True
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, art_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=args.urdf, import_config=cfg,
                                         get_articulation_root=True)
robot_root = "/" + art_path.strip("/").split("/")[0]
print(f"[hruh] imported {ROBOT_NAME}: root {robot_root}, articulation {art_path}")
UsdGeom.XformCommonAPI(stage.GetPrimAtPath(robot_root)).SetTranslate(Gf.Vec3d(0.0, 0.0, args.height))

base_link = None
n_drives = 0
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(robot_root):
        continue
    if prim.GetName() == "base_link" and prim.IsA(UsdGeom.Xformable) and base_link is None:
        base_link = p
    if prim.IsA(UsdPhysics.RevoluteJoint):
        name = prim.GetName()
        kp, kd, eff = gains_for(name)
        if name in MIMIC:            # driven by its mimic constraint
            kp, kd = 0.0, 0.0
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(kp * math.pi / 180.0)      # USD angular drives are per degree
        drive.CreateDampingAttr(kd * math.pi / 180.0)
        drive.CreateMaxForceAttr(eff)
        drive.CreateTargetPositionAttr(0.0)
        n_drives += 1
print(f"[hruh] {n_drives} joint drives ({args.gains} gains), base link {base_link}")

# pelvis IMU (same place as imu_frame in the URDF)
imu_path = None
if base_link:
    ok, imu_prim = omni.kit.commands.execute("IsaacSensorCreateImuSensor", path="/imu", parent=base_link,
                                             sensor_period=1.0 / args.physics_hz)
    imu_path = str(imu_prim.GetPath()) if ok and imu_prim else None


def find_prim(name):
    for prim in stage.Traverse():
        if prim.GetName() == name and str(prim.GetPath()).startswith(robot_root):
            return str(prim.GetPath())
    return None


# ------------------------------------------------------------------ eye cameras
cams = {}
if args.cameras:
    head = find_prim("head_base")
    if head:
        # head_base faces -x; USD cameras look along -Z with +Y up
        rot = Gf.Rotation(Gf.Matrix3d(0, 1, 0, 0, 0, 1, 1, 0, 0)).GetQuat()   # rows = camera X, Y, Z in head frame
        for side, y in (("left", -0.032), ("right", 0.032)):
            cam = UsdGeom.Camera.Define(stage, f"{head}/{side}_eye_camera")
            cam.CreateFocalLengthAttr(1.4)
            cam.CreateHorizontalApertureAttr(2.35)   # ~80 deg horizontal FOV like the Gazebo cameras
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
            xf = UsdGeom.XformCommonAPI(cam)
            xf.SetTranslate(Gf.Vec3d(-0.080, y, 0.085))
            q = rot
            xf.SetRotate(Gf.Vec3f(*Gf.Rotation(q).Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())))
            cams[side] = str(cam.GetPath())
    else:
        print("[hruh] head_base not found: no cameras")

# ------------------------------------------------------------------ ROS 2 graph
if not args.no_ros:
    import omni.graph.core as og
    import usdrt.Sdf

    K = og.Controller.Keys
    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("JointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("JointCmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("Articulation", "isaacsim.core.nodes.IsaacArticulationController"),
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("OdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("OdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ]
    connect = [
        ("Tick.outputs:tick", "Clock.inputs:execIn"),
        ("Tick.outputs:tick", "JointStates.inputs:execIn"),
        ("Tick.outputs:tick", "JointCmd.inputs:execIn"),
        ("Tick.outputs:tick", "Articulation.inputs:execIn"),
        ("Tick.outputs:tick", "Odom.inputs:execIn"),
        ("Odom.outputs:execOut", "OdomPub.inputs:execIn"),
        ("Odom.outputs:execOut", "OdomTF.inputs:execIn"),
        ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "JointStates.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "OdomTF.inputs:timeStamp"),
        ("JointCmd.outputs:jointNames", "Articulation.inputs:jointNames"),
        ("JointCmd.outputs:positionCommand", "Articulation.inputs:positionCommand"),
        ("JointCmd.outputs:velocityCommand", "Articulation.inputs:velocityCommand"),
        ("JointCmd.outputs:effortCommand", "Articulation.inputs:effortCommand"),
        ("Odom.outputs:position", "OdomPub.inputs:position"),
        ("Odom.outputs:orientation", "OdomPub.inputs:orientation"),
        ("Odom.outputs:linearVelocity", "OdomPub.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "OdomPub.inputs:angularVelocity"),
        ("Odom.outputs:position", "OdomTF.inputs:translation"),
        ("Odom.outputs:orientation", "OdomTF.inputs:rotation"),
    ]
    values = [
        ("JointStates.inputs:targetPrim", [usdrt.Sdf.Path(art_path)]),
        ("JointStates.inputs:topicName", "/isaac_joint_states"),
        ("JointCmd.inputs:topicName", "/isaac_joint_commands"),
        ("Articulation.inputs:targetPrim", [usdrt.Sdf.Path(art_path)]),
        ("Odom.inputs:chassisPrim", [usdrt.Sdf.Path(base_link or art_path)]),
        ("OdomPub.inputs:topicName", "/odom"),
        ("OdomPub.inputs:odomFrameId", "odom"),
        ("OdomPub.inputs:chassisFrameId", "base_link"),
        ("OdomTF.inputs:topicName", "/tf"),
        ("OdomTF.inputs:parentFrameId", "odom"),
        ("OdomTF.inputs:childFrameId", "base_link"),
    ]
    if imu_path:
        nodes += [("ImuRead", "isaacsim.sensors.physics.IsaacReadIMU"), ("ImuPub", "isaacsim.ros2.bridge.ROS2PublishImu")]
        connect += [("Tick.outputs:tick", "ImuRead.inputs:execIn"),
                    ("ImuRead.outputs:execOut", "ImuPub.inputs:execIn"),
                    ("ImuRead.outputs:angVel", "ImuPub.inputs:angularVelocity"),
                    ("ImuRead.outputs:linAcc", "ImuPub.inputs:linearAcceleration"),
                    ("ImuRead.outputs:orientation", "ImuPub.inputs:orientation"),
                    ("SimTime.outputs:simulationTime", "ImuPub.inputs:timeStamp")]
        values += [("ImuRead.inputs:imuPrim", [usdrt.Sdf.Path(imu_path)]), ("ImuRead.inputs:readGravity", True),
                   ("ImuPub.inputs:topicName", "/imu"), ("ImuPub.inputs:frameId", "imu_frame")]
    for side, cam in cams.items():
        rp, img, info = f"RP_{side}", f"Img_{side}", f"Info_{side}"
        nodes += [(rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                  (img, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                  (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
        connect += [("Tick.outputs:tick", f"{rp}.inputs:execIn"),
                    (f"{rp}.outputs:execOut", f"{img}.inputs:execIn"),
                    (f"{rp}.outputs:execOut", f"{info}.inputs:execIn"),
                    (f"{rp}.outputs:renderProductPath", f"{img}.inputs:renderProductPath"),
                    (f"{rp}.outputs:renderProductPath", f"{info}.inputs:renderProductPath")]
        values += [(f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(cam)]),
                   (f"{rp}.inputs:width", 640), (f"{rp}.inputs:height", 480),
                   (f"{img}.inputs:type", "rgb"), (f"{img}.inputs:topicName", f"/stereo/{side}/image_raw"),
                   (f"{img}.inputs:frameId", f"stereo_{side}_optical"),
                   (f"{info}.inputs:topicName", f"/stereo/{side}/camera_info"),
                   (f"{info}.inputs:frameId", f"stereo_{side}_optical")]
    og.Controller.edit({"graph_path": "/HruhROS2", "evaluator_name": "execution"},
                       {K.CREATE_NODES: nodes, K.CONNECT: connect, K.SET_VALUES: values})
    print("[hruh] ROS 2 graph: /clock /isaac_joint_states /isaac_joint_commands /odom /tf"
          + (" /imu" if imu_path else "") + (" /stereo/*" if cams else ""))

# ------------------------------------------------------------------ run
world.reset()
omni.timeline.get_timeline_interface().play()
print("HRUH_ISAAC_READY", flush=True)
render = (not args.headless) or bool(cams)
step = 0
while app.is_running():
    world.step(render=render)
    step += 1
    if args.steps and step >= args.steps:
        break

if args.report:
    from isaacsim.core.prims import SingleArticulation
    art = SingleArticulation(prim_path=art_path)
    art.initialize()
    pos, _ = art.get_world_pose()
    q = art.get_joint_positions()
    names = art.dof_names
    print(f"HRUH_REPORT dofs={len(names)} base_height={float(pos[2]):.3f} "
          f"max_abs_joint={float(np.max(np.abs(q))):.3f}")
    print("HRUH_REPORT joints=" + ",".join(names))
app.close()
