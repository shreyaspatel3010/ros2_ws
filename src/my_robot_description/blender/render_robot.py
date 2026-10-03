"""Render the assembled robot from its URDF (stills or a turntable).

    blender -b --factory-startup -P blender/render_robot.py -- <robot.urdf> <out_dir> <frames|0>
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Matrix, Vector
import scene_util as su
import urdf_blender as ub

argv = sys.argv[sys.argv.index("--") + 1:]
urdf, out, frames = argv[0], argv[1], int(argv[2])
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = int(os.environ.get("RES", "720"))

# relaxed human standing pose: soft knees, arms slightly away from the body
POSE = {
    "left_hip_pitch_joint": -0.12, "right_hip_pitch_joint": -0.12,
    "left_knee_joint": 0.24, "right_knee_joint": 0.24,
    "left_ankle_pitch_joint": -0.12, "right_ankle_pitch_joint": -0.12,
    "left_shoulder_to_bisecp": -0.10, "right_shoulder_to_bisecp": 0.10,
    "left_elbow_inword_to_midle": 0.25, "right_elbow_inword_to_midle": 0.25,
}

su.reset()
su.studio(res=(RES, int(RES * 1.25)), samples=48)
r = ub.Robot(urdf, PKG); r.build()
ub.upgrade_baked_materials()
r.set_joints(POSE)
bpy.context.view_layer.update()
mn, mx = r.world_bounds()
r.set_joints(POSE, Matrix.Translation((0, 0, -mn.z)))
bpy.context.view_layer.update()
mn, mx = r.world_bounds()
print("ROBOT HEIGHT %.3f" % (mx.z - mn.z))
tgt = (0, 0, (mx.z - mn.z) * 0.5)
if frames > 0:
    su.orbit_rig(tgt, 4.6, 0.9, 50, frames)
    su.render_frames(out, "f")
else:
    views = os.environ.get("VIEWS", "front:0,side:90,iso:35,back:200").split(",")
    zoom = os.environ.get("ZOOM")   # e.g. "1.2:0.35" -> look at z=1.2 with ortho-like tight framing
    for v in views:
        tag, ang = v.split(":"); a = math.radians(float(ang))
        if zoom:
            zz, half = map(float, zoom.split(":"))
            su.camera((3.0 * math.cos(a), 3.0 * math.sin(a), zz), (0, 0, zz), 50 * 1.5 / half)
        else:
            su.camera((4.4 * math.cos(a), 4.4 * math.sin(a), 1.1), tgt, 75)
        su.render_still(os.path.join(out, "robot_%s.png" % tag))
