"""HRUH in Isaac Sim 6.1 with a ROS 2 bridge (run with Isaac's Python).

    isaac-python hruh_isaac_sim.py [--headless] [--fix-base] [--cameras] [--gains ros|rl]

ROS 2 interface (used by topic_based_ros2_control in hruh_bringup isaac.launch.py):
    /isaac_joint_states    sensor_msgs/JointState   all joints (published)
    /isaac_joint_commands  sensor_msgs/JointState   position targets (subscribed)
    /clock /odom /imu, TF odom -> base_link
    --cameras: /stereo/{left,right}/image_raw + camera_info (the eye cameras)

Physics is PhysX (the articulation / odometry graph nodes are PhysX based), paced
to real time so ros2_control, MoveIt and the walker run on a consistent clock.
The URDF comes from `ros2 run hruh_isaac export_isaac_urdf.py`; the converted
USD is cached in ~/.cache/hruh/isaac_usd/.
"""
import argparse
import hashlib
import math
import os
import re
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
ap.add_argument("--hold-until-stand", metavar="BUNDLE_JSON", default="",
                help="learned walking: hold the pelvis at --height until /isaac_joint_commands reach the "
                     "policy's trained stand pose (policy_runner.py settles into it), then release it")
ap.add_argument("--no-ros", action="store_true", help="no ROS 2 bridge (self-test)")
ap.add_argument("--steps", type=int, default=0, help="quit after N app updates (0 = run until closed)")
ap.add_argument("--report", action="store_true", help="print the articulation state when quitting")
ap.add_argument("--physics", choices=["cpu", "gpu"], default="cpu",
                help="cpu: PhysX CPU dynamics (one robot, low GPU memory); gpu: PhysX GPU dynamics")
ap.add_argument("--tilt-deg", type=float, default=0.0, help="spawn tilted (falls over): physics robustness test")
ap.add_argument("--threads", type=int, default=8,
                help="worker threads for Isaac's task scheduler (keeps the laptop responsive)")
args, _ = ap.parse_known_args()

if not os.path.exists(args.urdf):
    sys.exit(f"[hruh] {args.urdf} not found: run `ros2 run hruh_isaac export_isaac_urdf.py` first")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "extra_args": [
    f"--/plugins/carb.tasking.plugin/threadCount={args.threads}",
    "--/exts/isaacsim.core.simulation_manager/default_engine=physx",
    "--/exts/isaacsim.physics.newton/auto_switch_on_startup=false",
    "--/app/runLoops/main/rateLimitEnabled=true",         # real time
    "--/app/runLoops/main/rateLimitFrequency=60",
    # keep VRAM low: the desktop (Xorg, VS Code, browser) shares this GPU
    "--/rtx-transient/resourcemanager/texturestreaming/memoryBudget=0.25",
    "--/rtx/post/dlss/execMode=0",
]})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hruh_lab"))
from hruh_lab import gpu_guard  # noqa: E402  (VRAM budget, HRUH_GPU_MEM_GB, default 8)

gpu_guard.start(name="isaac_sim")

for ext in ["isaacsim.asset.importer.urdf", "isaacsim.sensors.experimental.physics", "isaacsim.sensors.physics.nodes"] \
        + ([] if args.no_ros else ["isaacsim.ros2.bridge"]):
    app_utils.enable_extension(ext)
app.update()

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

# ------------------------------------------------------------------ joint gains
# (stiffness N*m/rad, damping N*m*s/rad, max effort N*m) by joint-name regex; first match wins
ARM = r"(chest_to_.*_shoulder|.*_shoulder_to_bisecp|.*_bisecp_to_elbow_inword|.*_elbow_inword_to_midle|.*_forarm_to_wrist)"
MIMIC = r".*_finger[1-4]_(lower_to_finger[1-4]_middle|middle_to_finger[1-4]_upper)"
GAINS = {
    "ros": [  # stiff position servos: the walker / MoveIt expect commanded ~= actual
        (MIMIC, 0.0, 0.0, 10.0),
        (r".*_(hip_yaw|hip_roll|hip_pitch|knee)_joint", 1500.0, 40.0, 300.0),
        (r".*_ankle_(pitch|roll)_joint", 800.0, 20.0, 150.0),
        (r".*_toe_joint", 60.0, 2.0, 40.0),
        (r"waist_.*", 1200.0, 30.0, 150.0),
        (ARM, 400.0, 10.0, 80.0),
        (r"(chest_to_neck|neck_to_head)", 100.0, 4.0, 20.0),
        (r".*(thomb|finger).*", 20.0, 0.5, 10.0),
    ],
    "rl": [  # identical to hruh_lab.robots.ACTUATORS (policies were trained with these)
        (MIMIC, 0.0, 0.0, 10.0),
        (r".*_hip_yaw_joint", 150.0, 5.0, 150.0), (r".*_hip_roll_joint", 200.0, 5.0, 200.0),
        (r".*_hip_pitch_joint", 200.0, 5.0, 250.0), (r".*_knee_joint", 250.0, 6.0, 300.0),
        (r".*_ankle_pitch_joint", 60.0, 3.0, 150.0), (r".*_ankle_roll_joint", 60.0, 3.0, 100.0),
        (r".*_toe_joint", 15.0, 0.5, 40.0), (r"waist_.*", 200.0, 6.0, 150.0),
        (ARM, 60.0, 3.0, 60.0), (r"(chest_to_neck|neck_to_head)", 30.0, 2.0, 20.0),
        (r".*(thomb|finger).*", 5.0, 0.2, 10.0),
    ],
}
profile = GAINS[args.gains]


def gains_for(name):
    for pat, kp, kd, eff in profile:
        if re.fullmatch(pat, name):
            return kp, kd, eff
    return 100.0, 5.0, 50.0


urdf_text = open(args.urdf, "rb").read()
urdf_root = ET.fromstring(urdf_text)
JOINTS = [j.get("name") for j in urdf_root.findall("joint") if j.get("type") in ("revolute", "continuous")]
# URDF <mimic>: joint -> (driving joint, multiplier, offset)
MIMICS = {j.get("name"): (j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", "1")),
                          float(j.find("mimic").get("offset", "0")))
          for j in urdf_root.findall("joint") if j.find("mimic") is not None}

# ------------------------------------------------------------------ URDF -> USD (cached)
key = hashlib.sha256(urdf_text + repr((profile, args.fix_base)).encode()).hexdigest()[:16]
usd_dir = os.path.expanduser(f"~/.cache/hruh/isaac_usd/{args.gains}{'_fixed' if args.fix_base else ''}_{key}")
import glob  # noqa: E402
existing = sorted(glob.glob(os.path.join(usd_dir, "*", "*.usda")))   # <usd_dir>/<robot>/<robot>.usda
if existing:
    usd_path = existing[0]
    print(f"[hruh] cached USD {usd_path}")
else:
    os.makedirs(usd_dir, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=args.urdf, usd_path=usd_dir,
        merge_fixed_joints=True,          # massless frames fold into their parents (TF comes from ROS)
        fix_base=args.fix_base,
        collision_type="Convex Hull",
        allow_self_collision=False,
        joint_drive_type="force",
        joint_target_type="position",
        # exact names in the importer's regex dicts (SI units, converted to USD per-degree internally)
        override_joint_stiffness={re.escape(n): gains_for(n)[0] for n in JOINTS},
        override_joint_damping={re.escape(n): gains_for(n)[1] for n in JOINTS},
    )
    usd_path = URDFImporter(config).import_urdf()
    print(f"[hruh] converted URDF -> {usd_path}")

# ------------------------------------------------------------------ world
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
px.CreateTimeStepsPerSecondAttr(int(args.physics_hz))
px.CreateSolverTypeAttr("TGS")
# one robot: CPU physics is plenty and keeps GPU memory for rendering / the desktop
if args.physics == "cpu":
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr(os.environ.get("HRUH_BROADPHASE", "SAP"))

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.XformCommonAPI(ground)
gx.SetTranslate(Gf.Vec3d(0, 0, -0.05))
gx.SetScale(Gf.Vec3f(60.0, 60.0, 0.1))
ground.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.37, 0.4)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
mat = UsdShade.Material.Define(stage, "/World/ground_material")
pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
pm.CreateStaticFrictionAttr(1.0)
pm.CreateDynamicFrictionAttr(0.9)
pm.CreateRestitutionAttr(0.0)
UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1200.0)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 20.0, 0.0))

# ------------------------------------------------------------------ robot
robot_root = "/World/hruh"
robot = stage.DefinePrim(robot_root, "Xform")
robot.GetReferences().AddReference(usd_path)
# the 6.1 importer authors one variant per physics engine and selects none by default
physics_vs = robot.GetVariantSets().GetVariantSet("Physics")
if physics_vs:
    physics_vs.SetVariantSelection("physx")
UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0.0, 0.0, args.height))
if args.tilt_deg:
    UsdGeom.XformCommonAPI(robot).SetRotate(Gf.Vec3f(args.tilt_deg, 0.0, 0.0))

art_path = base_link = head = None
joint_prims = {}
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(robot_root):
        continue
    if art_path is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        art_path = p
    if prim.GetName() == "base_link" and base_link is None and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        base_link = p
    if prim.GetName() == "head_base" and head is None and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        head = p
    if prim.IsA(UsdPhysics.RevoluteJoint):
        joint_prims[prim.GetName()] = prim
if art_path is None or not joint_prims:
    sys.exit(f"[hruh] no articulation in {usd_path} (physics variant 'physx' missing?)")

# PhysX mimic constraints (the importer only writes Newton mimic joints)
ROT = {"X": UsdPhysics.Tokens.rotX, "Y": UsdPhysics.Tokens.rotY, "Z": UsdPhysics.Tokens.rotZ}
n_mimic = 0
for name, (ref, mult, off) in MIMICS.items():
    if name in joint_prims and ref in joint_prims:
        j, r = UsdPhysics.RevoluteJoint(joint_prims[name]), UsdPhysics.RevoluteJoint(joint_prims[ref])
        api = PhysxSchema.PhysxMimicJointAPI.Apply(joint_prims[name], ROT[j.GetAxisAttr().Get()])
        api.CreateReferenceJointRel().SetTargets([joint_prims[ref].GetPath()])
        api.CreateReferenceJointAxisAttr(ROT[r.GetAxisAttr().Get()])
        api.CreateGearingAttr(-mult)        # q + gearing * q_ref + offset = 0
        api.CreateOffsetAttr(-off)
        n_mimic += 1
print(f"[hruh] robot {robot_root}: articulation {art_path}, base {base_link}, "
      f"{len(joint_prims)} joints ({args.gains} gains), {n_mimic} mimic constraints")

# learned walking: pin the pelvis to the world until the policy runner has the stand pose
# (the training gains alone cannot keep the robot upright while ros2_control starts)
hold_path, stand = None, {}
if args.hold_until_stand and base_link:
    import json
    bundle = json.load(open(args.hold_until_stand))
    stand = {n: bundle["default_positions"][n] for n in bundle["policy_joints"]}
    hold_path = "/World/policy_hold"
    hold = UsdPhysics.FixedJoint.Define(stage, hold_path)
    hold.CreateBody1Rel().SetTargets([base_link])          # body0 empty = the world
    world_tf = UsdGeom.Xformable(stage.GetPrimAtPath(base_link)).ComputeLocalToWorldTransform(0)
    hold.CreateLocalPos0Attr(Gf.Vec3f(world_tf.ExtractTranslation()))
    hold.CreateLocalRot0Attr(Gf.Quatf(world_tf.ExtractRotationQuat()))
    hold.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    hold.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    hold.CreateExcludeFromArticulationAttr(True)
    print(f"[hruh] pelvis held at {args.height:.3f} m until the stand pose of {len(stand)} policy joints is commanded")

# pelvis IMU (same place as imu_frame in the URDF)
imu_path = None
if base_link:
    try:
        from isaacsim.sensors.experimental.physics import IMU
        IMU(f"{base_link}/imu")
        imu_path = f"{base_link}/imu"
    except Exception as e:  # keep the robot usable without an IMU
        print(f"[hruh] IMU not created: {e}")

# ------------------------------------------------------------------ eye cameras
cams = {}
if args.cameras and head:
    # head_base faces -x; USD cameras look along -Z with +Y up (rows = camera X, Y, Z in head frame)
    rot = Gf.Rotation(Gf.Matrix3d(0, 1, 0, 0, 0, 1, 1, 0, 0))
    euler = rot.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    for side, y in (("left", -0.032), ("right", 0.032)):
        cam = UsdGeom.Camera.Define(stage, f"{head}/{side}_eye_camera")
        cam.CreateFocalLengthAttr(1.4)
        cam.CreateHorizontalApertureAttr(2.35)      # ~80 deg horizontal FOV like the Gazebo cameras
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
        xf = UsdGeom.XformCommonAPI(cam)
        xf.SetTranslate(Gf.Vec3d(-0.080, y, 0.085))
        xf.SetRotate(Gf.Vec3f(*euler))
        cams[side] = str(cam.GetPath())

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
app_utils.play()
print("HRUH_ISAAC_READY", flush=True)
step = 0
settled = 0


def stand_commanded():
    """True when the last /isaac_joint_commands put every policy joint at its stand pose."""
    try:
        names = og.Controller.attribute("/HruhROS2/JointCmd.outputs:jointNames").get()
        values = og.Controller.attribute("/HruhROS2/JointCmd.outputs:positionCommand").get()
    except Exception:
        return False
    if names is None or values is None or len(names) != len(values):
        return False
    cmd = dict(zip([str(n) for n in names], values))
    return all(n in cmd and math.isfinite(cmd[n]) and abs(cmd[n] - q) < 0.01 for n, q in stand.items())


try:
    while app.is_running():
        app.update()
        step += 1
        if hold_path and not args.no_ros:
            settled = settled + 1 if stand_commanded() else 0
            if settled >= 10:
                stage.RemovePrim(hold_path)
                hold_path = None
                print("[hruh] stand pose reached: pelvis released to the walking policy", flush=True)
        if args.steps and step >= args.steps:
            break
except KeyboardInterrupt:          # Ctrl+C, launch shutdown or the GPU guard
    print("[hruh] stopping", flush=True)

if args.report:
    from isaacsim.core.experimental.prims import Articulation
    art = Articulation(art_path)
    pos = np.asarray(art.get_world_poses()[0].numpy() if hasattr(art.get_world_poses()[0], "numpy")
                     else art.get_world_poses()[0]).reshape(-1, 3)
    q = art.get_dof_positions()
    q = np.asarray(q.numpy() if hasattr(q, "numpy") else q)
    print(f"HRUH_REPORT dofs={len(art.dof_names)} base_height={float(pos[0, 2]):.3f} "
          f"max_abs_joint={float(np.max(np.abs(q))):.3f}")
app.close()
